// SPDX-FileCopyrightText: 2026 ETH Zurich and University of Bologna
// SPDX-License-Identifier: Apache-2.0
//
// InSitu cache — E3 partition-config broadcast shim.
//
// A 1-slave / M-master fan-out: the cluster peripheral drives ONE partition-commit write in, and this
// component replays it as a fresh synchronous IoReq ({addr=csr-id, data=value}) to every endpoint
// (lane xbars, remote xbars, cache cores), stamping the incoming request with the max endpoint
// latency (mirrors the F1 flush fan-out in cluster_registers.cpp). The RTL broadcasts
// l1d_private_o/private_start_addr_o/dynamic_offset_o to all tiles from one peripheral output, so a
// single master -> shim -> all endpoints matches the RTL structure and avoids an N-port ordering
// contract. Endpoints that don't answer are tolerated (is_bound-checked by the caller's wiring).

#include <cstdint>
#include <cstring>
#include <vector>

#include <vp/vp.hpp>
#include <vp/itf/io.hpp>

class InsituCacheConfigBroadcast : public vp::Component
{
public:
    explicit InsituCacheConfigBroadcast(vp::ComponentConf &conf);

private:
    static vp::IoReqStatus req_handler(vp::Block *__this, vp::IoReq *req);

    uint32_t nb_masters_;
    std::vector<vp::IoMaster *> outs_;
    vp::IoSlave input_;
    vp::IoReq replay_;
    vp::Trace trace_;
};

InsituCacheConfigBroadcast::InsituCacheConfigBroadcast(vp::ComponentConf &conf) : vp::Component(conf)
{
    auto *cfg = this->get_js_config();
    nb_masters_ = cfg->get_child_int("nb_masters");
    outs_.resize(nb_masters_);
    for (uint32_t i = 0; i < nb_masters_; i++) {
        outs_[i] = new vp::IoMaster();
        this->new_master_port("out_" + std::to_string(i), outs_[i]);
    }
    input_.set_req_meth(&InsituCacheConfigBroadcast::req_handler);
    this->new_slave_port("input", &input_);
    this->traces.new_trace("trace", &this->trace_, vp::DEBUG);
}

vp::IoReqStatus InsituCacheConfigBroadcast::req_handler(vp::Block *__this, vp::IoReq *req)
{
    InsituCacheConfigBroadcast *_this = static_cast<InsituCacheConfigBroadcast *>(__this);
    const uint64_t csr = req->get_addr();
    uint32_t value = 0;
    if (req->get_data() != nullptr) memcpy(&value, req->get_data(), req->get_size() < 4 ? req->get_size() : 4);

    int64_t max_lat = 0;
    for (uint32_t i = 0; i < _this->nb_masters_; i++) {
        if (!_this->outs_[i]->is_bound()) continue;
        _this->replay_.init();
        _this->replay_.set_addr(csr);
        _this->replay_.set_size(4);
        _this->replay_.set_is_write(true);
        _this->replay_.set_data((uint8_t *)&value);
        vp::IoReqStatus st = _this->outs_[i]->req(&_this->replay_);
        if (st == vp::IO_REQ_OK) {
            const int64_t lat = (int64_t)_this->replay_.get_full_latency();
            if (lat > max_lat) max_lat = lat;
        }
    }
    _this->trace_.msg(vp::Trace::LEVEL_INFO, "config broadcast csr=%lu value=0x%x -> %u masters (max_lat=%ld)\n",
                      (unsigned long)csr, value, _this->nb_masters_, (long)max_lat);
    req->inc_latency(max_lat);
    return vp::IO_REQ_OK;
}

extern "C" vp::Component *gv_new(vp::ComponentConf &config)
{
    return new InsituCacheConfigBroadcast(config);
}
