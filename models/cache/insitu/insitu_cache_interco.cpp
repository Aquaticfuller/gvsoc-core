// SPDX-FileCopyrightText: 2026 ETH Zurich and University of Bologna
//
// SPDX-License-Identifier: Apache-2.0
//
// InSitu Cache interconnect — hashed N-to-M crossbar.
//
// Routes upstream TCDM ports to cache controllers by address. Pattern follows
// ``pulp/cluster/l1_interleaver_impl.cpp`` (address→bank hash + per-output req_forward).
//
// See prompt/insitu_cache_architecture.md §5.

#include <cstdint>
#include <cstdio>
#include <vector>

#include <vp/vp.hpp>
#include <vp/itf/io.hpp>

namespace {
inline unsigned int ceil_log2(unsigned int n)
{
    if (n <= 1) return 0;
    return 32 - __builtin_clz(n - 1);
}
}

class InsituCacheInterco : public vp::Component
{
public:
    explicit InsituCacheInterco(vp::ComponentConf &conf);

private:
    static vp::IoReqStatus req_handler(vp::Block *__this, vp::IoReq *req, int input_id);

    uint32_t num_inputs_;
    uint32_t num_outputs_;
    uint32_t dynamic_offset_;
    int32_t  interco_latency_cycles_;
    uint32_t output_mask_;
    uint32_t output_bits_;
    bool     enable_input_coalesce_;
    uint32_t line_bits_;            // log2(cache_line_bytes), for same-line grouping
    int64_t  coalesce_max_latency_; // only reads with latency <= this seed the window (-1 = no limit)
    bool     forward_initiator_;    // tag req->initiator with the input-port index (scalar bypass)
    int32_t  output_accept_width_;  // accepts per output per cycle before serialization kicks in
    bool     per_cycle_output_arb_; // true = per-cycle reset (open-loop replay); false = accumulate

    std::vector<vp::IoSlave *>  inputs_;
    std::vector<vp::IoMaster *> outputs_;

    // Scratch request used to split a wide access (one larger than the interleave granularity)
    // into per-owning-controller byte ranges. Single-outstanding (synchronous slave) so one
    // scratch req is safe.
    vp::IoReq split_subreq_;

    // Output accept arbitration — two modes (see req_handler for why):
    //  • accumulate (default): monotonic busy-until cyclestamp. Models sustained 1/cyc output
    //    backpressure across cycles — correct for CLOSED-LOOP (Spatz/microbench), where the core
    //    actually stalls on the returned latency.
    //  • per-cycle (per_cycle_output_arb_): reset the accept counter each cycle, serialize only
    //    same-cycle requests. Correct for OPEN-LOOP trace replay (calib), where t_issue already
    //    encodes the RTL's cross-cycle backpressure and accumulating would double-count it.
    std::vector<int64_t> output_busy_until_;     // accumulate mode
    std::vector<int64_t> out_cycle_stamp_;       // per-cycle mode
    std::vector<int64_t> out_accepts_in_cycle_;  // per-cycle mode

    // Input par-coalescer state, per output. A same-cycle (cycle == hit_win_cycle_) read
    // to the same line (hit_win_line_) that the cycle's first reader HIT inherits its
    // latency without re-forwarding or consuming an accept slot — modelling N narrow VLSU
    // reads collapsing into one wide cache lookup. valid iff hit_win_cycle_[o] == now.
    std::vector<int64_t>  hit_win_cycle_;
    std::vector<uint64_t> hit_win_line_;
    std::vector<int64_t>  hit_win_latency_;

    vp::Trace trace_;
};

InsituCacheInterco::InsituCacheInterco(vp::ComponentConf &conf)
    : vp::Component(conf)
{
    auto *cfg = this->get_js_config();
    num_inputs_             = cfg->get_child_int("num_inputs");
    num_outputs_            = cfg->get_child_int("num_outputs");
    dynamic_offset_         = cfg->get_child_int("dynamic_offset");
    interco_latency_cycles_ = cfg->get_child_int("interco_latency_cycles");
    enable_input_coalesce_  = cfg->get_child_bool("enable_input_coalesce");
    line_bits_              = ceil_log2((unsigned)cfg->get_child_int("cache_line_bytes"));
    coalesce_max_latency_   = cfg->get_child_int("coalesce_max_latency");
    forward_initiator_      = cfg->get_child_bool("forward_initiator");
    output_accept_width_    = cfg->get_child_int("output_accept_width");
    if (output_accept_width_ < 1) output_accept_width_ = 1;
    per_cycle_output_arb_   = cfg->get_child_bool("per_cycle_output_arb");

    output_bits_ = ceil_log2(num_outputs_);
    output_mask_ = (num_outputs_ > 1) ? (num_outputs_ - 1) : 0;

    inputs_.resize(num_inputs_);
    outputs_.resize(num_outputs_);
    output_busy_until_.assign(num_outputs_, -1);
    out_cycle_stamp_.assign(num_outputs_, -1);
    out_accepts_in_cycle_.assign(num_outputs_, 0);
    hit_win_cycle_.assign(num_outputs_, -1);
    hit_win_line_.assign(num_outputs_, 0);
    hit_win_latency_.assign(num_outputs_, 0);

    for (uint32_t i = 0; i < num_inputs_; ++i) {
        inputs_[i] = new vp::IoSlave();
        inputs_[i]->set_req_meth_muxed(&InsituCacheInterco::req_handler, (int)i);
        this->new_slave_port("in_" + std::to_string(i), inputs_[i]);
    }
    for (uint32_t o = 0; o < num_outputs_; ++o) {
        outputs_[o] = new vp::IoMaster();
        this->new_master_port("out_" + std::to_string(o), outputs_[o]);
    }

    this->traces.new_trace("trace", &this->trace_, vp::DEBUG);
    this->trace_.msg(vp::Trace::LEVEL_INFO,
        "InsituCacheInterco N=%u M=%u offset=%u\n",
        num_inputs_, num_outputs_, dynamic_offset_);
}

vp::IoReqStatus InsituCacheInterco::req_handler(vp::Block *__this, vp::IoReq *req,
                                                  int input_id)
{
    InsituCacheInterco *_this = static_cast<InsituCacheInterco *>(__this);
    const uint64_t addr = req->get_addr();

    // Wide-access split: the controller-select bits ([dynamic_offset +: log2(num_outputs)]) sit
    // WITHIN the cache line, so an access larger than the interleave granularity (1<<dynamic_offset)
    // spans multiple controllers. Route each byte range to its OWNING controller — otherwise an
    // 8-byte store routed wholesale to ctrl0 leaves ctrl1's copy of the upper word stale (a later
    // narrow read of that word goes to ctrl1 and reads garbage). Each owning controller fills its
    // slice of the request's data buffer in place. (Accesses within one granule skip this.)
    if (_this->num_outputs_ > 1) {
        const uint32_t gran = 1u << _this->dynamic_offset_;
        const uint32_t size = (uint32_t)req->get_size();
        if (((uint32_t)(addr & (gran - 1)) + size) > gran) {
            uint8_t *data = req->get_data();
            const bool is_wr = req->get_is_write();
            uint64_t a = addr;
            uint32_t rem = size;
            int64_t max_lat = 0;
            while (rem > 0) {
                const uint32_t out = (uint32_t)((a >> _this->dynamic_offset_) & _this->output_mask_);
                const uint32_t to_bound = gran - (uint32_t)(a & (gran - 1));
                const uint32_t chunk = (rem < to_bound) ? rem : to_bound;
                _this->split_subreq_.init();
                _this->split_subreq_.set_addr(a);
                _this->split_subreq_.set_size(chunk);
                _this->split_subreq_.set_is_write(is_wr);
                if (data != nullptr) _this->split_subreq_.set_data(data + (uint32_t)(a - addr));
                if (_this->forward_initiator_) _this->split_subreq_.set_initiator(input_id);
                _this->split_subreq_.inc_latency(_this->interco_latency_cycles_);
                (void)_this->outputs_[out]->req(&_this->split_subreq_);
                const int64_t l = (int64_t)_this->split_subreq_.get_full_latency();
                if (l > max_lat) max_lat = l;
                a += chunk;
                rem -= chunk;
            }
            req->inc_latency(max_lat);
            return vp::IO_REQ_OK;
        }
    }

    const uint32_t out_id =
        (_this->num_outputs_ > 1)
            ? (uint32_t)((addr >> _this->dynamic_offset_) & _this->output_mask_)
            : 0u;

    _this->trace_.msg(vp::Trace::LEVEL_TRACE,
        "forward input=%d → output=%u addr=0x%lx\n",
        input_id, out_id, (unsigned long)addr);

    // Tag the request with its input-port index so the controller can recognise the scalar
    // bypass port. Off by default (Spatz unchanged); the user request is resp'd at the
    // controller and never reaches memory, so this does not collide with the memory models'
    // initiator (LR/SC) use.
    if (_this->forward_initiator_) {
        req->set_initiator(input_id);
    }

    const int64_t now = _this->clock.get_cycles();

    // Input par-coalescer: a same-cycle read to the same line that the cycle's first reader
    // already HIT is served by that one wide lookup — inherit its latency, do NOT re-forward
    // and do NOT consume an accept slot. So N same-line same-cycle reads cost ~one bank
    // access (RTL coal_warm ≈ 4x single-port hit rate). Misses are NOT recorded (the first
    // reader returns PENDING), so cold same-line reads fall through to the controller's MSHR
    // merge (coal_cold mem_rd unchanged).
    if (_this->enable_input_coalesce_ && !req->get_is_write()) {
        const uint64_t line = addr >> _this->line_bits_;
        if (_this->hit_win_cycle_[out_id] == now && _this->hit_win_line_[out_id] == line) {
            req->inc_latency(_this->hit_win_latency_[out_id]);
            return vp::IO_REQ_OK;
        }
    }

    // Output accept arbitration (see member declaration for the two modes).
    int64_t latency;
    if (_this->per_cycle_output_arb_) {
        // Per-cycle: the RTL coalescer + wide datapath absorbs output_accept_width accepts per
        // cycle; only requests beyond that within the SAME cycle pay a serialization cost. This
        // does NOT accumulate across cycles — the replay trace's t_issue already encodes the RTL's
        // cross-cycle backpressure (the core stalled when the cache couldn't accept), so a monotonic
        // busy-until would double-count it and inflate hit latency (~+33 cy on the fft trace; see
        // insitu_cache_realkernel_alignment_2026-06-12.md §9).
        if (now != _this->out_cycle_stamp_[out_id]) {
            _this->out_cycle_stamp_[out_id]      = now;
            _this->out_accepts_in_cycle_[out_id] = 0;
        }
        const int64_t k = _this->out_accepts_in_cycle_[out_id]++;
        latency = _this->interco_latency_cycles_ + (k / _this->output_accept_width_);
    } else {
        // Accumulate (default): monotonic 1/cyc output backpressure, correct for closed-loop.
        latency = _this->interco_latency_cycles_;
        if (_this->output_busy_until_[out_id] > now) {
            latency += (_this->output_busy_until_[out_id] - now);
        }
        _this->output_busy_until_[out_id] = now + latency;
    }

    req->inc_latency(latency);
    vp::IoReqStatus st = _this->outputs_[out_id]->req_forward(req);

    // Record this cycle's first read-HIT to a line so same-cycle same-line followers merge.
    // Gate on coalesce_max_latency so only a TRUE warm hit seeds the window: a cold line
    // refilled inline returns OK but with a refill-sized latency, which must NOT coalesce
    // (its followers belong on the controller's MSHR-merge / drain-paced path).
    if (_this->enable_input_coalesce_ && !req->get_is_write() && st == vp::IO_REQ_OK) {
        const int64_t lat = (int64_t)req->get_full_latency();
        if (_this->coalesce_max_latency_ < 0 || lat <= _this->coalesce_max_latency_) {
            _this->hit_win_cycle_[out_id]   = now;
            _this->hit_win_line_[out_id]    = addr >> _this->line_bits_;
            _this->hit_win_latency_[out_id] = lat;
        }
    }
    return st;
}

extern "C" vp::Component *gv_new(vp::ComponentConf &config)
{
    return new InsituCacheInterco(config);
}
