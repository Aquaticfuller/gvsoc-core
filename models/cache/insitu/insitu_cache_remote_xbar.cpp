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
#include <cstdlib>
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
    // E3: dyn_offset only (csr-id 2).
    static vp::IoReqStatus config_handler(vp::Block *__this, vp::IoReq *req);

    RouteGeom geom_;
    uint32_t  num_tiles_, nrpc_, n_slots_;
    uint32_t  num_groups_ = 1, tiles_per_group_ = 1, group_id_ = 0, n_local_slots_ = 0;
    int32_t   hop_latency_cycles_;
    std::vector<vp::IoSlave *>  inputs_;
    std::vector<vp::IoMaster *> outputs_;
    vp::IoSlave config_;    // E3: dyn_offset only (csr-id 2)
    vp::Trace trace_;
};

InsituCacheRemoteXbar::InsituCacheRemoteXbar(vp::ComponentConf &conf) : vp::Component(conf)
{
    auto *cfg = this->get_js_config();
    num_tiles_          = cfg->get_child_int("num_tiles");
    nrpc_               = cfg->get_child_int("num_remote_port_core");
    // P1 multi-group: the address TileID field is CLUSTER-GLOBAL, so a "remote" request may target a
    // tile in this group or in another one. tiles_per_group + group_id let this crossbar tell them
    // apart: same group -> a local tile slot; other group -> the NoC egress slots. num_groups==1
    // keeps the single-group behaviour and does not create the NoC ports at all.
    num_groups_       = cfg->get("num_groups")       ? cfg->get_child_int("num_groups")       : 1;
    tiles_per_group_  = cfg->get("tiles_per_group")  ? cfg->get_child_int("tiles_per_group")  : num_tiles_;
    group_id_         = cfg->get("group_id")         ? cfg->get_child_int("group_id")         : 0;
    if (tiles_per_group_ == 0) tiles_per_group_ = num_tiles_;
    if (nrpc_ < 1) nrpc_ = 1;
    n_slots_            = num_tiles_ * nrpc_;    // NumInp = NumOut = NumTiles * NumRemotePortCore
    hop_latency_cycles_ = cfg->get_child_int("hop_latency_cycles");

    geom_.init(/*n_cache*/cfg->get_child_int("num_cache"), /*n_remote*/nrpc_,
               /*n_cores*/cfg->get_child_int("num_cores"), /*n_tiles*/num_tiles_,
               /*dyn_offset*/cfg->get_child_int("dynamic_offset"), /*addr_w*/cfg->get_child_int("addr_width"),
               /*priv_start*/0);

    // Local slots address the tiles OF THIS GROUP; with >1 group, nrpc extra slots per direction
    // carry off-group traffic to/from the L1 NoC.
    n_local_slots_ = tiles_per_group_ * nrpc_;
    const uint32_t n_noc = (num_groups_ > 1) ? nrpc_ : 0;
    const uint32_t n_in  = n_local_slots_ + n_noc;
    const uint32_t n_out = n_local_slots_ + n_noc;
    inputs_.resize(n_in);
    outputs_.resize(n_out);
    for (uint32_t i = 0; i < n_in; i++) {
        inputs_[i] = new vp::IoSlave();
        inputs_[i]->set_req_meth_muxed(&InsituCacheRemoteXbar::req_handler, (int)i);
        if (i < n_local_slots_) this->new_slave_port("in_" + std::to_string(i), inputs_[i]);
        else                    this->new_slave_port("noc_in_" + std::to_string(i - n_local_slots_), inputs_[i]);
    }
    for (uint32_t o = 0; o < n_out; o++) {
        outputs_[o] = new vp::IoMaster();
        if (o < n_local_slots_) this->new_master_port("out_" + std::to_string(o), outputs_[o]);
        else                    this->new_master_port("noc_out_" + std::to_string(o - n_local_slots_), outputs_[o]);
    }

    this->traces.new_trace("trace", &this->trace_, vp::DEBUG);
    this->trace_.msg(vp::Trace::LEVEL_INFO, "InsituCacheRemoteXbar tiles=%u nrpc=%u\n", num_tiles_, nrpc_);

    // E3: dyn_offset only (the remote router never re-partitions banks, it re-extracts addr_tile).
    config_.set_req_meth(&InsituCacheRemoteXbar::config_handler);
    this->new_slave_port("config", &config_);
}

vp::IoReqStatus InsituCacheRemoteXbar::config_handler(vp::Block *__this, vp::IoReq *req)
{
    InsituCacheRemoteXbar *_this = static_cast<InsituCacheRemoteXbar *>(__this);
    if ((uint32_t)req->get_addr() == 2) {   // XBAR_OFFSET only
        uint32_t value = 0;
        if (req->get_data() != nullptr) memcpy(&value, req->get_data(), req->get_size() < 4 ? req->get_size() : 4);
        _this->geom_.dyn_offset = value;
    }
    return vp::IO_REQ_OK;
}

vp::IoReqStatus InsituCacheRemoteXbar::req_handler(vp::Block *__this, vp::IoReq *req, int input_id)
{
    InsituCacheRemoteXbar *_this = static_cast<InsituCacheRemoteXbar *>(__this);
    // Input slot = source_tile*nrpc + r. Route to the TARGET tile (by the address TileID); within the
    // target, pin the slot by source-tile-mod-N (RTL group.sv:285-295). The GVSoC response auto-routes
    // back along the preserved resp-port chain, so the exact slot is a timing/contention choice only.
    const uint32_t source = (uint32_t)input_id / _this->nrpc_;
    uint32_t target = _this->geom_.addr_tile(req->get_addr());
    if (target >= _this->num_tiles_) target = _this->num_tiles_ - 1;   // safety clamp
    // The TileID field is cluster-global: derive the owning group, then either land on one of this
    // group's tiles or leave through the NoC egress.
    const uint32_t tgt_group = target / _this->tiles_per_group_;
    uint32_t out;
    if (_this->num_groups_ > 1 && tgt_group != _this->group_id_) {
        // ONE egress port per port class toward the L1 NoC. The NI is a single injection point with
        // one pending read burst and one pending write burst; giving it two masters (source%nrpc)
        // doubles the contention on that single slot for no modelled benefit, and deviates from the
        // arrangement proven at 256 cores in v2 (exactly one master per NI input).
        out = _this->n_local_slots_;                                     // → L1 NoC
    } else {
        const uint32_t local_tile = target % _this->tiles_per_group_;
        out = local_tile * _this->nrpc_ + (source % _this->nrpc_);
    }
    // INSITU_RXBAR_DEBUG=1: budgeted stderr trace of the routing decision (the component's own
    // per-request message is LEVEL_TRACE, which plain --trace does not emit).
    {
        static const int dbg = [](){ const char *e = getenv("INSITU_RXBAR_DEBUG"); return e ? atoi(e) : 0; }();
        static int budget = 300;
        if (dbg && budget > 0) {
            budget--;
            fprintf(stderr, "[RXBAR %s] cyc=%ld in=%d addr=0x%lx target=%u tgt_grp=%u my_grp=%u out=%u%s\n",
                    _this->get_path().c_str(), (long)_this->clock.get_cycles(), input_id,
                    (unsigned long)req->get_addr(), target, tgt_group, _this->group_id_, out,
                    out == _this->n_local_slots_ ? " ->NOC" : " ->local");
        }
    }
    if (_this->hop_latency_cycles_ > 0) req->inc_latency(_this->hop_latency_cycles_);
    _this->trace_.msg(vp::Trace::LEVEL_TRACE, "remote src=%u addr=0x%lx -> tile=%u slot=%u\n",
                      source, (unsigned long)req->get_addr(), target, out);
    vp::IoReqStatus st = _this->outputs_[out]->req_forward(req);
    {
        static const int dbg = [](){ const char *e = getenv("INSITU_RXBAR_DEBUG"); return e ? atoi(e) : 0; }();
        static int sbudget = 40;
        if (dbg && sbudget > 0) {
            sbudget--;
            fprintf(stderr, "[RXBAR-ST %s] cyc=%ld addr=0x%lx out=%u status=%d (0=OK 1=INVALID 2=DENIED 3=PENDING)\n",
                    _this->get_path().c_str(), (long)_this->clock.get_cycles(),
                    (unsigned long)req->get_addr(), out, (int)st);
        }
    }
    return st;
}

extern "C" vp::Component *gv_new(vp::ComponentConf &config)
{
    return new InsituCacheRemoteXbar(config);
}
