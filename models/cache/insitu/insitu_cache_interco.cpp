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

    std::vector<vp::IoSlave *>  inputs_;
    std::vector<vp::IoMaster *> outputs_;

    // Per-output busy-until cyclestamps for single-accept-per-cycle arbitration.
    std::vector<int64_t> output_busy_until_;

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

    output_bits_ = ceil_log2(num_outputs_);
    output_mask_ = (num_outputs_ > 1) ? (num_outputs_ - 1) : 0;

    inputs_.resize(num_inputs_);
    outputs_.resize(num_outputs_);
    output_busy_until_.assign(num_outputs_, -1);
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

    // Per-output single-accept-per-cycle arbitration. If another input already drove this
    // output this cycle, delay by interco_latency_cycles (1 cycle in canonical config).
    int64_t latency = _this->interco_latency_cycles_;
    if (_this->output_busy_until_[out_id] > now) {
        latency += (_this->output_busy_until_[out_id] - now);
    }
    _this->output_busy_until_[out_id] = now + latency;

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
