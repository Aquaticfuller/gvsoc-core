// SPDX-FileCopyrightText: 2026 ETH Zurich, University of Bologna and EssilorLuxottica SAS
//
// SPDX-License-Identifier: Apache-2.0
//
// Authors: Germain Haugou (germain.haugou@gmail.com)

/*
 * IoV2Beat stub target for IoV2BeatWidthAdapter: a beat slave of width
 * beat_width.
 *
 *   READ : the request is a single data-less burst descriptor. Returns
 *          IO_REQ_GRANTED and streams ceil(size / beat_width) DISTINCT
 *          allocator-backed response beats, one per cycle, payload filled
 *          with the addr-derived pattern, initiator copied from the
 *          descriptor (initiator-owned convention: the descriptor itself is
 *          never freed or round-tripped here). A consumer IO_RESP_DENIED
 *          holds the exact beat, re-sent synchronously from our registered
 *          resp_retry handler (logged BHOLD / BRETRY).
 *   WRITE: per-beat requests, each at most beat_width bytes (POLICED — this
 *          is exactly what the width adapter must guarantee). The payload is
 *          checked against the addr-derived pattern (POLICES the adapter's
 *          repacking). Acked inline (IO_REQ_DONE + latency annotation) or,
 *          with async_ack=true, via GRANTED + one resp() later.
 *
 * Request-path back-pressure: the first deny_count requests are DENIED and
 * retried retry_delay cycles later (logged DENY / RETRY).
 *
 * Config keys: beat_width, latency, error(bool), async_ack(bool), deny_count,
 * retry_delay, base, size, logname.
 */

#include <vp/vp.hpp>
#include <vp/itf/io_v2.hpp>
#include <cstdio>
#include <deque>
#include <string>

class StubTarget : public vp::Component
{
public:
    StubTarget(vp::ComponentConf &conf);
    void reset(bool active) override;

private:
    static vp::IoReqStatus req_handler(vp::Block *__this, vp::IoReq *req);
    static void resp_retry_handler(vp::Block *__this, vp::IoRetryChannel channel);
    static void stream_event_handler(vp::Block *__this, vp::ClockEvent *event);
    static void ack_event_handler(vp::Block *__this, vp::ClockEvent *event);
    static void retry_event_handler(vp::Block *__this, vp::ClockEvent *event);

    void pump_stream();

    // One in-flight read burst being streamed back as beats.
    struct Stream {
        void *initiator;
        uint64_t addr;
        uint64_t total;
        uint64_t emitted;
        int64_t burst_id;
    };

    struct PendingAck { vp::IoReq *req; int64_t resp_cycle; };

    vp::IoSlave in;
    vp::ClockEvent stream_event;
    vp::ClockEvent ack_event;
    vp::ClockEvent retry_event;
    vp::Trace trace;
    vp::IoReqAllocator *beat_allocator;
    std::string logname;
    int beat_width = 0;
    int64_t latency = 0;
    bool error = false;
    bool async_ack = false;
    int deny_count_cfg = 0;
    int deny_remaining = 0;
    int retry_delay = 2;
    uint64_t base = 0;
    uint64_t size = 0;
    std::deque<Stream> streams;         // served in order, one beat per cycle
    std::deque<PendingAck> pending_acks; // async write acks
    // Consumer back-pressure: the refused beat, re-sent from resp_retry.
    vp::IoReq *held_beat = nullptr;
    uint64_t held_size = 0;
};

StubTarget::StubTarget(vp::ComponentConf &config)
    : vp::Component(config),
      in(&StubTarget::req_handler, &StubTarget::resp_retry_handler),
      stream_event(this, &StubTarget::stream_event_handler),
      ack_event(this, &StubTarget::ack_event_handler),
      retry_event(this, &StubTarget::retry_event_handler)
{
    this->traces.new_trace("trace", &this->trace, vp::DEBUG);
    this->new_slave_port("input", &this->in);

    this->logname = this->get_js_config()->get_child_str("logname");
    if (this->logname.empty()) this->logname = this->get_name();
    this->beat_width = (int)this->get_js_config()->get_child_int("beat_width");
    this->latency = this->get_js_config()->get_child_int("latency");
    this->error = this->get_js_config()->get_child_bool("error");
    this->async_ack = this->get_js_config()->get_child_bool("async_ack");
    this->deny_count_cfg = (int)this->get_js_config()->get_child_int("deny_count");
    this->retry_delay = (int)this->get_js_config()->get_child_int("retry_delay");
    if (this->retry_delay <= 0) this->retry_delay = 2;
    this->base = (uint64_t)this->get_js_config()->get_child_int("base");
    this->size = (uint64_t)this->get_js_config()->get_child_int("size");

    this->beat_allocator = vp::IoReqAllocator::get(this->beat_width);
}

void StubTarget::reset(bool active)
{
    if (active)
    {
        this->streams.clear();
        this->pending_acks.clear();
        this->deny_remaining = this->deny_count_cfg;
        if (this->held_beat != nullptr)
        {
            this->held_beat->free();
            this->held_beat = nullptr;
        }
        this->stream_event.cancel();
        this->ack_event.cancel();
        this->retry_event.cancel();
    }
}

vp::IoReqStatus StubTarget::req_handler(vp::Block *__this, vp::IoReq *req)
{
    StubTarget *_this = (StubTarget *)__this;
    int64_t now = _this->clock.get_cycles();
    uint64_t addr = req->get_addr();
    uint64_t sz = req->get_size();
    bool is_write = req->get_is_write();

    printf("[%ld] %s REQ addr=0x%lx size=%lu write=%d first=%d last=%d burst_id=%ld\n",
        now, _this->logname.c_str(), addr, sz, is_write ? 1 : 0,
        req->is_first ? 1 : 0, req->is_last ? 1 : 0, (long)req->burst_id);

    _this->traces.assert(addr >= _this->base && addr + sz <= _this->base + _this->size,
        "access [0x%lx,+%lu) outside target range [0x%lx,+%lu)",
        addr, sz, _this->base, _this->size);

    // ---- Request-path back-pressure: deny the first deny_count requests ----
    if (_this->deny_remaining > 0)
    {
        _this->deny_remaining--;
        printf("[%ld] %s DENY addr=0x%lx (retry in %d)\n",
            now, _this->logname.c_str(), addr, _this->retry_delay);
        _this->retry_event.enqueue(_this->retry_delay);
        return vp::IO_REQ_DENIED;
    }

    if (is_write)
    {
        // The width adapter must chop/pack writes to OUR beat width.
        _this->traces.assert(sz <= (uint64_t)_this->beat_width,
            "write beat wider than the slave width (size=%lu, width=%d)",
            sz, _this->beat_width);
        // The payload must carry the addr-derived pattern: checks that the
        // adapter's repacking preserved the byte->address mapping.
        uint8_t *d = req->get_data();
        for (uint64_t i = 0; i < sz; i++)
        {
            _this->traces.assert(d[i] == (uint8_t)((addr + i) & 0xff),
                "write data mismatch at addr 0x%lx byte %lu", addr, i);
        }
        req->set_resp_status(_this->error ? vp::IO_RESP_INVALID : vp::IO_RESP_OK);
        if (!_this->async_ack)
        {
            req->inc_latency(_this->latency);
            return vp::IO_REQ_DONE;
        }
        int64_t resp_cycle = now + std::max((int64_t)1, _this->latency);
        _this->pending_acks.push_back({req, resp_cycle});
        _this->ack_event.enqueue(resp_cycle - now);
        return vp::IO_REQ_GRANTED;
    }

    // READ: a single data-less burst descriptor (io_v2 read burst convention).
    _this->traces.assert(req->is_first && req->is_last,
        "read burst must be a single descriptor (first=%d last=%d)",
        req->is_first ? 1 : 0, req->is_last ? 1 : 0);
    _this->traces.assert(req->get_data() == nullptr,
        "read burst request must be data-less");

    // Inline error completion for an error target: no stream, status inline.
    if (_this->error)
    {
        req->set_resp_status(vp::IO_RESP_INVALID);
        req->inc_latency(_this->latency);
        return vp::IO_REQ_DONE;
    }

    _this->streams.push_back(Stream{req->initiator, addr, sz, 0, req->burst_id});
    _this->stream_event.enqueue(std::max((int64_t)1, _this->latency));
    return vp::IO_REQ_GRANTED;
}

// Emit one response beat of the head stream. Called once per cycle from the
// stream event, and synchronously from resp_retry to re-send a held beat.
void StubTarget::pump_stream()
{
    int64_t now = this->clock.get_cycles();

    if (this->held_beat != nullptr)
    {
        vp::IoReq *beat = this->held_beat;
        if (this->in.resp(beat) == vp::IO_RESP_DENIED)
        {
            return;   // still refused; keep holding
        }
        printf("[%ld] %s BEAT addr=0x%lx size=%lu first=%d last=%d\n",
            now, this->logname.c_str(), beat->get_addr(),
            (unsigned long)beat->get_size(), beat->is_first ? 1 : 0,
            beat->is_last ? 1 : 0);
        this->held_beat = nullptr;
        Stream &s = this->streams.front();
        s.emitted += this->held_size;
        if (s.emitted >= s.total)
        {
            this->streams.pop_front();
        }
        if (!this->streams.empty())
        {
            this->stream_event.enqueue(1);
        }
        return;
    }

    if (this->streams.empty())
    {
        return;
    }

    Stream &s = this->streams.front();
    uint64_t beat_sz = std::min<uint64_t>(s.total - s.emitted,
                                          (uint64_t)this->beat_width);

    vp::IoReq *beat = this->beat_allocator->alloc();
    beat->prepare();
    beat->set_addr(s.addr + s.emitted);
    beat->set_size(beat_sz);
    beat->set_is_write(false);
    beat->is_first = (s.emitted == 0);
    beat->is_last = (s.emitted + beat_sz >= s.total);
    beat->burst_id = s.burst_id;
    beat->initiator = s.initiator;
    for (uint64_t i = 0; i < beat_sz; i++)
    {
        beat->get_data()[i] = (uint8_t)((beat->get_addr() + i) & 0xff);
    }

    if (this->in.resp(beat) == vp::IO_RESP_DENIED)
    {
        // Consumer back-pressure: hold the exact beat, re-send on resp_retry.
        printf("[%ld] %s BHOLD addr=0x%lx\n", now, this->logname.c_str(),
            beat->get_addr());
        this->held_beat = beat;
        this->held_size = beat_sz;
        return;
    }

    printf("[%ld] %s BEAT addr=0x%lx size=%lu first=%d last=%d\n",
        now, this->logname.c_str(), beat->get_addr(), (unsigned long)beat_sz,
        beat->is_first ? 1 : 0, beat->is_last ? 1 : 0);

    s.emitted += beat_sz;
    if (s.emitted >= s.total)
    {
        this->streams.pop_front();
    }
    if (!this->streams.empty())
    {
        this->stream_event.enqueue(1);
    }
}

void StubTarget::stream_event_handler(vp::Block *__this, vp::ClockEvent *event)
{
    StubTarget *_this = (StubTarget *)__this;
    _this->pump_stream();
}

void StubTarget::resp_retry_handler(vp::Block *__this, vp::IoRetryChannel channel)
{
    StubTarget *_this = (StubTarget *)__this;
    int64_t now = _this->clock.get_cycles();
    printf("[%ld] %s BRETRY\n", now, _this->logname.c_str());
    // The consumer can take responses again; re-send the held beat now,
    // synchronously, per the io_v2 contract.
    _this->pump_stream();
}

void StubTarget::ack_event_handler(vp::Block *__this, vp::ClockEvent *event)
{
    StubTarget *_this = (StubTarget *)__this;
    int64_t now = _this->clock.get_cycles();

    while (!_this->pending_acks.empty()
           && _this->pending_acks.front().resp_cycle <= now)
    {
        vp::IoReq *req = _this->pending_acks.front().req;
        _this->pending_acks.pop_front();
        printf("[%ld] %s WACK addr=0x%lx size=%lu\n", now, _this->logname.c_str(),
            req->get_addr(), (unsigned long)req->get_size());
        _this->traces.assert(_this->in.resp(req) == vp::IO_RESP_ACCEPTED,
            "the adapter must accept downstream write acks");
    }

    if (!_this->pending_acks.empty())
    {
        _this->ack_event.enqueue(_this->pending_acks.front().resp_cycle - now);
    }
}

void StubTarget::retry_event_handler(vp::Block *__this, vp::ClockEvent *event)
{
    StubTarget *_this = (StubTarget *)__this;
    int64_t now = _this->clock.get_cycles();
    printf("[%ld] %s RETRY\n", now, _this->logname.c_str());
    // Tell the adapter (our master) it can re-send; per the io_v2 contract it
    // re-issues the held request synchronously inside this call.
    _this->in.retry();
}

extern "C" vp::Component *gv_new(vp::ComponentConf &config)
{
    return new StubTarget(config);
}
