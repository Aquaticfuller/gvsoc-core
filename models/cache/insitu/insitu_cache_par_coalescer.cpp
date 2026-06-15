// SPDX-FileCopyrightText: 2026 ETH Zurich and University of Bologna
//
// SPDX-License-Identifier: Apache-2.0
//
// InSitu Cache parallel coalescer (par_coalescer) — per-controller front-end.
//
// Phase-1 increment 1: a STRUCTURAL extraction of what the interco's
// `enable_input_coalesce` window did inline. One par_coalescer sits between each
// interco output and its cache controller. It performs, on the request stream routed
// to that one controller:
//   (1) same-cycle / same-line READ merge — the cycle's first read to a line that HIT
//       seeds a window; same-cycle same-line followers inherit that latency and return
//       OK without re-forwarding or consuming an accept slot (N narrow VLSU reads → one
//       wide cache lookup, RTL coal_warm ≈ 4× single-port hit rate); and
//   (2) output-accept arbitration (the same two modes as the interco: accumulate for
//       closed-loop, per-cycle for open-loop replay).
//
// The logic is a verbatim relocation of the interco's per-output body
// (insitu_cache_interco.cpp req_handler, the merge-window + arb block) operating on a
// single output, so when the tile switches to the structural path it reproduces the
// calibrated numbers by construction. This is the foundation the later Phase-1
// increments (wide-merge / response-split / miss-coalescing / scalar bypass) build on.
//
// See prompt/insitu_cache_architecture_v2.md §8 (par_coalescer) and
// prompt/insitu_cache_dev_plan_2026-06-15.md Phase 1.

#include <cstdint>
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

class InsituCacheParCoalescer : public vp::Component
{
public:
    explicit InsituCacheParCoalescer(vp::ComponentConf &conf);

private:
    static vp::IoReqStatus req_handler(vp::Block *__this, vp::IoReq *req);

    int32_t  interco_latency_cycles_;
    bool     merge_same_line_reads_;
    uint32_t line_bits_;
    int64_t  coalesce_max_latency_;
    int32_t  output_accept_width_;
    bool     per_cycle_output_arb_;

    vp::IoSlave  in_;
    vp::IoMaster out_;

    // Output-accept arbitration state (scalar — one output). See the interco for the
    // accumulate-vs-per-cycle rationale.
    int64_t output_busy_until_   = -1;   // accumulate mode
    int64_t out_cycle_stamp_     = -1;   // per-cycle mode
    int64_t out_accepts_in_cycle_ = 0;   // per-cycle mode

    // Same-cycle same-line read-merge window (valid iff hit_win_cycle_ == now).
    int64_t  hit_win_cycle_   = -1;
    uint64_t hit_win_line_    = 0;
    int64_t  hit_win_latency_ = 0;

    vp::Trace trace_;
};

InsituCacheParCoalescer::InsituCacheParCoalescer(vp::ComponentConf &conf)
    : vp::Component(conf)
{
    auto *cfg = this->get_js_config();
    interco_latency_cycles_ = cfg->get_child_int("interco_latency_cycles");
    merge_same_line_reads_  = cfg->get_child_bool("merge_same_line_reads");
    line_bits_              = ceil_log2((unsigned)cfg->get_child_int("cache_line_bytes"));
    coalesce_max_latency_   = cfg->get_child_int("coalesce_max_latency");
    output_accept_width_    = cfg->get_child_int("output_accept_width");
    if (output_accept_width_ < 1) output_accept_width_ = 1;
    per_cycle_output_arb_   = cfg->get_child_bool("per_cycle_output_arb");

    this->in_.set_req_meth(&InsituCacheParCoalescer::req_handler);
    this->new_slave_port("in", &this->in_);
    this->new_master_port("out", &this->out_);

    this->traces.new_trace("trace", &this->trace_, vp::DEBUG);
    this->trace_.msg(vp::Trace::LEVEL_INFO,
        "InsituCacheParCoalescer merge=%d per_cycle_arb=%d\n",
        (int)merge_same_line_reads_, (int)per_cycle_output_arb_);
}

vp::IoReqStatus InsituCacheParCoalescer::req_handler(vp::Block *__this, vp::IoReq *req)
{
    InsituCacheParCoalescer *_this = static_cast<InsituCacheParCoalescer *>(__this);
    const uint64_t addr = req->get_addr();
    const int64_t now = _this->clock.get_cycles();

    // (1) same-cycle same-line read merge: a follower inherits the leader's latency and
    // returns OK without re-forwarding or consuming an accept slot.
    if (_this->merge_same_line_reads_ && !req->get_is_write()) {
        const uint64_t line = addr >> _this->line_bits_;
        if (_this->hit_win_cycle_ == now && _this->hit_win_line_ == line) {
            req->inc_latency(_this->hit_win_latency_);
            return vp::IO_REQ_OK;
        }
    }

    // (2) output-accept arbitration (identical to the interco's per-output logic).
    int64_t latency;
    if (_this->per_cycle_output_arb_) {
        if (now != _this->out_cycle_stamp_) {
            _this->out_cycle_stamp_      = now;
            _this->out_accepts_in_cycle_ = 0;
        }
        const int64_t k = _this->out_accepts_in_cycle_++;
        latency = _this->interco_latency_cycles_ + (k / _this->output_accept_width_);
    } else {
        latency = _this->interco_latency_cycles_;
        if (_this->output_busy_until_ > now) {
            latency += (_this->output_busy_until_ - now);
        }
        _this->output_busy_until_ = now + latency;
    }

    req->inc_latency(latency);
    vp::IoReqStatus st = _this->out_.req_forward(req);

    // Seed the read-merge window only for a TRUE warm hit (gate on coalesce_max_latency:
    // a cold line refilled inline returns OK with a refill-sized latency and must NOT
    // coalesce — its followers belong on the controller's MSHR-merge path).
    if (_this->merge_same_line_reads_ && !req->get_is_write() && st == vp::IO_REQ_OK) {
        const int64_t lat = (int64_t)req->get_full_latency();
        if (_this->coalesce_max_latency_ < 0 || lat <= _this->coalesce_max_latency_) {
            _this->hit_win_cycle_   = now;
            _this->hit_win_line_    = addr >> _this->line_bits_;
            _this->hit_win_latency_ = lat;
        }
    }
    return st;
}

extern "C" vp::Component *gv_new(vp::ComponentConf &config)
{
    return new InsituCacheParCoalescer(config);
}
