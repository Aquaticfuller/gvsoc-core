// SPDX-FileCopyrightText: 2026 ETH Zurich, University of Bologna and EssilorLuxottica SAS
//
// SPDX-License-Identifier: Apache-2.0
//
// Authors: Germain Haugou (germain.haugou@gmail.com)

/*
 * Width-declaring IoV2SingleReq stub target for IoV2SingleReqWidthAdapter.
 * Models a bank-interleaved fabric input (the GAP9 shared-L2 LogIco): every
 * access MUST fit one aligned width granule — POLICED with traces.assert,
 * which is exactly the guarantee the width adapter has to provide.
 *
 * Each request is answered with a single-beat response: inline IO_REQ_DONE
 * (default, with latency annotation and optional IO_RESP_INVALID) or async
 * IO_REQ_GRANTED + one resp() (async_resp=true). Requests whose 0-based
 * arrival index is listed in deny_at are DENIED once and retried retry_delay
 * cycles later (a re-sent request gets a new index) — this drives both the
 * adapter's stateless first-chunk deny and its held mid-split chunk.
 *
 * Read data is the addr-derived pattern; write payloads are checked against
 * it (POLICES that the adapter's chunks carve the master's buffer at the
 * right offsets).
 *
 * Config keys: width, latency, error(bool), async_resp(bool), deny_at(list),
 * retry_delay, base, size, logname.
 */

#include <vp/vp.hpp>
#include <vp/itf/io_v2.hpp>
#include <cstdio>
#include <deque>
#include <set>
#include <string>

class StubTarget : public vp::Component
{
public:
    StubTarget(vp::ComponentConf &conf);
    void reset(bool active) override;

private:
    static vp::IoReqStatus req_handler(vp::Block *__this, vp::IoReq *req);
    static void resp_event_handler(vp::Block *__this, vp::ClockEvent *event);
    static void retry_event_handler(vp::Block *__this, vp::ClockEvent *event);

    void serve(vp::IoReq *req);

    struct Pending { vp::IoReq *req; int64_t resp_cycle; };

    vp::IoSlave in{&StubTarget::req_handler};
    vp::ClockEvent resp_event;
    vp::ClockEvent retry_event;
    vp::Trace trace;
    std::string logname;
    int width = 4;
    int64_t latency = 0;
    bool error = false;
    bool async_resp = false;
    std::set<int> deny_at_cfg;
    std::set<int> deny_remaining;
    int arrival = 0;
    int retry_delay = 2;
    uint64_t base = 0;
    uint64_t size = 0;
    std::deque<Pending> pending;   // outstanding async responses, FIFO
};

StubTarget::StubTarget(vp::ComponentConf &config)
    : vp::Component(config),
      resp_event(this, &StubTarget::resp_event_handler),
      retry_event(this, &StubTarget::retry_event_handler)
{
    this->traces.new_trace("trace", &this->trace, vp::DEBUG);
    this->new_slave_port("input", &this->in);

    this->logname = this->get_js_config()->get_child_str("logname");
    if (this->logname.empty()) this->logname = this->get_name();
    this->width = (int)this->get_js_config()->get_child_int("width");
    this->latency = this->get_js_config()->get_child_int("latency");
    this->error = this->get_js_config()->get_child_bool("error");
    this->async_resp = this->get_js_config()->get_child_bool("async_resp");
    this->retry_delay = (int)this->get_js_config()->get_child_int("retry_delay");
    if (this->retry_delay <= 0) this->retry_delay = 2;
    this->base = (uint64_t)this->get_js_config()->get_child_int("base");
    this->size = (uint64_t)this->get_js_config()->get_child_int("size");

    js::Config *deny = this->get_js_config()->get("deny_at");
    if (deny != nullptr)
    {
        for (auto &d : deny->get_elems())
        {
            this->deny_at_cfg.insert((int)d->get_int());
        }
    }
}

void StubTarget::reset(bool active)
{
    if (active)
    {
        this->pending.clear();
        this->deny_remaining = this->deny_at_cfg;
        this->arrival = 0;
        this->resp_event.cancel();
        this->retry_event.cancel();
    }
}

void StubTarget::serve(vp::IoReq *req)
{
    uint64_t addr = req->get_addr();
    uint64_t sz = req->get_size();
    if (!req->get_is_write())
    {
        uint8_t *d = req->get_data();
        for (uint64_t i = 0; i < sz; i++) d[i] = (uint8_t)((addr + i) & 0xff);
    }
    else
    {
        uint8_t *d = req->get_data();
        for (uint64_t i = 0; i < sz; i++)
        {
            this->traces.assert(d[i] == (uint8_t)((addr + i) & 0xff),
                "write data mismatch at addr 0x%lx byte %lu", addr, i);
        }
    }
    req->inc_latency(this->latency);
    req->set_resp_status(this->error ? vp::IO_RESP_INVALID : vp::IO_RESP_OK);
}

vp::IoReqStatus StubTarget::req_handler(vp::Block *__this, vp::IoReq *req)
{
    StubTarget *_this = (StubTarget *)__this;
    int64_t now = _this->clock.get_cycles();
    uint64_t addr = req->get_addr();
    uint64_t sz = req->get_size();
    int index = _this->arrival++;

    printf("[%ld] %s REQ addr=0x%lx size=%lu write=%d\n",
        now, _this->logname.c_str(), addr, sz, req->get_is_write() ? 1 : 0);

    // ---- The granule contract this whole testbench exists for ----
    // No access may cross an aligned width granule: the interleaved fabric
    // routes the whole request to one bank, so a straddling access would
    // alias another address. The width adapter must have chopped it.
    _this->traces.assert(
        (addr & ((uint64_t)_this->width - 1)) + sz <= (uint64_t)_this->width,
        "access straddles the width granule (addr=0x%lx, size=%lu, width=%d)",
        addr, sz, _this->width);
    _this->traces.assert(req->is_first && req->is_last,
        "single-req slave must receive single accesses (first=%d last=%d)",
        req->is_first ? 1 : 0, req->is_last ? 1 : 0);
    _this->traces.assert(addr >= _this->base && addr + sz <= _this->base + _this->size,
        "access [0x%lx,+%lu) outside target range [0x%lx,+%lu)",
        addr, sz, _this->base, _this->size);

    // ---- Request-path back-pressure at the listed arrival indices ----
    if (_this->deny_remaining.erase(index))
    {
        printf("[%ld] %s DENY addr=0x%lx (arrival %d, retry in %d)\n",
            now, _this->logname.c_str(), addr, index, _this->retry_delay);
        _this->retry_event.enqueue(_this->retry_delay);
        return vp::IO_REQ_DENIED;
    }

    _this->serve(req);

    if (!_this->async_resp)
    {
        return vp::IO_REQ_DONE;
    }

    int64_t resp_cycle = now + std::max((int64_t)1, _this->latency);
    _this->pending.push_back({req, resp_cycle});
    _this->resp_event.enqueue(resp_cycle - now);
    return vp::IO_REQ_GRANTED;
}

void StubTarget::resp_event_handler(vp::Block *__this, vp::ClockEvent *event)
{
    StubTarget *_this = (StubTarget *)__this;
    int64_t now = _this->clock.get_cycles();

    while (!_this->pending.empty() && _this->pending.front().resp_cycle <= now)
    {
        vp::IoReq *req = _this->pending.front().req;
        _this->pending.pop_front();
        printf("[%ld] %s RESP addr=0x%lx size=%lu\n", now, _this->logname.c_str(),
            req->get_addr(), (unsigned long)req->get_size());
        _this->traces.assert(_this->in.resp(req) == vp::IO_RESP_ACCEPTED,
            "the adapter must accept the single-req slave's resp()");
    }

    if (!_this->pending.empty())
    {
        _this->resp_event.enqueue(_this->pending.front().resp_cycle - now);
    }
}

void StubTarget::retry_event_handler(vp::Block *__this, vp::ClockEvent *event)
{
    StubTarget *_this = (StubTarget *)__this;
    printf("[%ld] %s RETRY\n", _this->clock.get_cycles(), _this->logname.c_str());
    // The master (the adapter) re-sends its held request synchronously inside
    // this call, per the io_v2 contract.
    _this->in.retry();
}

extern "C" vp::Component *gv_new(vp::ComponentConf &config)
{
    return new StubTarget(config);
}
