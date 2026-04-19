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

    std::vector<vp::IoSlave *>  inputs_;
    std::vector<vp::IoMaster *> outputs_;

    // Per-output busy-until cyclestamps for single-accept-per-cycle arbitration.
    std::vector<int64_t> output_busy_until_;

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

    output_bits_ = ceil_log2(num_outputs_);
    output_mask_ = (num_outputs_ > 1) ? (num_outputs_ - 1) : 0;

    inputs_.resize(num_inputs_);
    outputs_.resize(num_outputs_);
    output_busy_until_.assign(num_outputs_, -1);

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

    const int64_t now = _this->clock.get_cycles();

    // Per-output single-accept-per-cycle arbitration. If another input already drove this
    // output this cycle, delay by interco_latency_cycles (1 cycle in canonical config).
    int64_t latency = _this->interco_latency_cycles_;
    if (_this->output_busy_until_[out_id] > now) {
        latency += (_this->output_busy_until_[out_id] - now);
    }
    _this->output_busy_until_[out_id] = now + latency;

    req->inc_latency(latency);
    return _this->outputs_[out_id]->req_forward(req);
}

extern "C" vp::Component *gv_new(vp::ComponentConf &config)
{
    return new InsituCacheInterco(config);
}
