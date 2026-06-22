// SPDX-FileCopyrightText: 2026 ETH Zurich and University of Bologna
// SPDX-License-Identifier: Apache-2.0
//
// InSitu cache — inter-tile remote crossbar (cachepool_remote_xbar), one per port-class.
//
// The group instances NrTCDMPortsPerCore (=5) of these (cachepool_group.sv:399-432), one per port-class.
// Each routes a cross-tile request emitted on a SOURCE tile's remote-out slot to the TARGET tile's
// remote-in slot, by the TileID field of the address — extending the shared L1 across tiles (any core in
// tile A can reach a bank homed in tile B). The GVSoC response auto-routes back along the preserved
// resp-port chain, so the RTL's source-tile-mod-N slot pinning is a timing/contention detail, not needed
// for functional correctness here; this models WHICH target tile a request lands on.
//
// num_tiles inputs (one source-tile remote-out per port-class) × num_tiles outputs (one target-tile
// remote-in). Route: out = addr_tile = addr[dynamic_offset + log2(NumCache) +: TileIDWidth] (route.hpp).

#include <cstdint>
#include <string>
#include <vector>

#include <vp/vp.hpp>
#include <vp/itf/io.hpp>

#include "insitu_cache_route.hpp"

using namespace insitu;

class InsituCacheRemoteXbar : public vp::Component
{
public:
    explicit InsituCacheRemoteXbar(vp::ComponentConf &conf);

private:
    static vp::IoReqStatus req_handler(vp::Block *__this, vp::IoReq *req, int input_id);

    RouteGeom geom_;
    uint32_t  num_tiles_;
    int32_t   hop_latency_cycles_;
    std::vector<vp::IoSlave *>  inputs_;
    std::vector<vp::IoMaster *> outputs_;
    vp::Trace trace_;
};

InsituCacheRemoteXbar::InsituCacheRemoteXbar(vp::ComponentConf &conf) : vp::Component(conf)
{
    auto *cfg = this->get_js_config();
    num_tiles_          = cfg->get_child_int("num_tiles");
    hop_latency_cycles_ = cfg->get_child_int("hop_latency_cycles");

    geom_.init(/*n_cache*/cfg->get_child_int("num_cache"), /*n_remote*/1,
               /*n_cores*/cfg->get_child_int("num_cores"), /*n_tiles*/num_tiles_,
               /*dyn_offset*/cfg->get_child_int("dynamic_offset"), /*addr_w*/cfg->get_child_int("addr_width"),
               /*priv_start*/0);

    inputs_.resize(num_tiles_);
    outputs_.resize(num_tiles_);
    for (uint32_t i = 0; i < num_tiles_; i++) {
        inputs_[i] = new vp::IoSlave();
        inputs_[i]->set_req_meth_muxed(&InsituCacheRemoteXbar::req_handler, (int)i);
        this->new_slave_port("in_" + std::to_string(i), inputs_[i]);
    }
    for (uint32_t o = 0; o < num_tiles_; o++) {
        outputs_[o] = new vp::IoMaster();
        this->new_master_port("out_" + std::to_string(o), outputs_[o]);
    }

    this->traces.new_trace("trace", &this->trace_, vp::DEBUG);
    this->trace_.msg(vp::Trace::LEVEL_INFO, "InsituCacheRemoteXbar tiles=%u\n", num_tiles_);
}

vp::IoReqStatus InsituCacheRemoteXbar::req_handler(vp::Block *__this, vp::IoReq *req, int input_id)
{
    InsituCacheRemoteXbar *_this = static_cast<InsituCacheRemoteXbar *>(__this);
    (void)input_id;
    uint32_t target = _this->geom_.addr_tile(req->get_addr());
    if (target >= _this->num_tiles_) target = _this->num_tiles_ - 1;   // safety clamp
    if (_this->hop_latency_cycles_ > 0) req->inc_latency(_this->hop_latency_cycles_);
    _this->trace_.msg(vp::Trace::LEVEL_TRACE, "remote src=%d addr=0x%lx -> tile=%u\n",
                      input_id, (unsigned long)req->get_addr(), target);
    return _this->outputs_[target]->req_forward(req);
}

extern "C" vp::Component *gv_new(vp::ComponentConf &config)
{
    return new InsituCacheRemoteXbar(config);
}
