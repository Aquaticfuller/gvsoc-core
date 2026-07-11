// SPDX-FileCopyrightText: 2026 ETH Zurich, University of Bologna and EssilorLuxottica SAS
//
// SPDX-License-Identifier: Apache-2.0
//
// Authors: Germain Haugou (germain.haugou@gmail.com)

/*
 * IoV2Beat stub target for IoV2SingleReqToBeatAdapter: a beat slave of width
 * beat_width.
 *
 *   READ : the request must be data-less (POLICED — the adapter forwards its
 *          own data-less request, never the master's buffered one). Returns
 *          IO_REQ_GRANTED and streams ceil(size / beat_width) DISTINCT
 *          allocator-backed response beats, one per cycle, payload filled
 *          with the addr-derived pattern, initiator copied from the request.
 *          Several requests may be in flight; their streams are served in
 *          FIFO order (this is what lets the pipelined test prove the adapter
 *          keeps multiple accesses outstanding). error=true answers reads
 *          inline with IO_REQ_DONE + IO_RESP_INVALID instead.
 *   WRITE: per-burst acknowledgement (io_v2.hpp "Write acknowledgement").
 *          Each beat is an allocator-backed size-0-pool object whose data
 *          aliases the sender's buffer; the payload is checked against the
 *          addr-derived pattern. A non-last beat is consumed and freed
 *          (GRANTED, no resp ever). The last beat (the adapter sends one-beat
 *          bursts, is_first=is_last) is acked inline (IO_REQ_DONE + status +
 *          latency annotation; the caller keeps its beat) or, with
 *          async_ack=true, consumed via GRANTED and recycled into the burst's
 *          single data-less ack sent by one resp() later (the initiator frees
 *          it). Atomics keep the classic round-trip of the sender's object.
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
    static void stream_event_handler(vp::Block *__this, vp::ClockEvent *event);
    static void ack_event_handler(vp::Block *__this, vp::ClockEvent *event);
    static void retry_event_handler(vp::Block *__this, vp::ClockEvent *event);

    // One in-flight read being streamed back as beats.
    struct Stream {
        void *initiator;
        uint64_t addr;
        uint64_t total;
        uint64_t emitted;
    };

    struct PendingAck { vp::IoReq *req; int64_t resp_cycle; };

    vp::IoSlave in{&StubTarget::req_handler};
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
    std::deque<Stream> streams;          // served in order, one beat per cycle
    std::deque<PendingAck> pending_acks; // async write acks
};

StubTarget::StubTarget(vp::ComponentConf &config)
    : vp::Component(config),
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
        // Recycled pure-write acks in the queue are ours (the consumed
        // beats, waiting to be resp()'d); atomic acks are the sender's own
        // object and must not be freed.
        for (auto &a : this->pending_acks)
        {
            if (a.req->get_opcode() == vp::WRITE)
            {
                a.req->free();
            }
        }
        this->pending_acks.clear();
        this->deny_remaining = this->deny_count_cfg;
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

    printf("[%ld] %s REQ addr=0x%lx size=%lu write=%d\n",
        now, _this->logname.c_str(), addr, sz, is_write ? 1 : 0);

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
        // Consume the payload: it must carry the addr-derived pattern (it
        // aliases the master's buffer — valid while the beat is unfreed).
        uint8_t *d = req->get_data();
        for (uint64_t i = 0; i < sz; i++)
        {
            _this->traces.assert(d[i] == (uint8_t)((addr + i) & 0xff),
                "write data mismatch at addr 0x%lx byte %lu", addr, i);
        }

        if (req->get_opcode() == vp::WRITE)
        {
            // A write beat crosses a consumer-frees boundary: it must be
            // allocator-backed (POLICED — an unported producer would hand us
            // an object we would corrupt the heap by freeing).
            _this->traces.assert(req->allocator != nullptr,
                "write beat is not allocator-backed (req=%p)", req);

            // Per-burst write acknowledgement (io_v2.hpp): a non-last beat is
            // consumed and freed, never resp()'d.
            if (!req->is_last)
            {
                req->free();
                return vp::IO_REQ_GRANTED;
            }

            // Last beat — the adapter sends one-beat bursts, so the burst
            // total equals this beat's size.
            vp::IoRespStatus status =
                _this->error ? vp::IO_RESP_INVALID : vp::IO_RESP_OK;
            if (!_this->async_ack)
            {
                // Inline burst ack: the caller keeps its beat.
                req->set_resp_status(status);
                req->inc_latency(_this->latency);
                return vp::IO_REQ_DONE;
            }
            // Async: consume the beat and recycle it as the burst's single
            // data-less ack (the contract allows recycling the consumed
            // size-0 last beat; the initiator frees the ack after resp()).
            // addr / burst_id / initiator are kept from the beat.
            req->prepare();
            req->set_data(nullptr);
            req->set_size(sz);
            req->is_first = true;
            req->is_last = true;
            req->set_resp_status(status);
            int64_t resp_cycle = now + std::max((int64_t)1, _this->latency);
            _this->pending_acks.push_back({req, resp_cycle});
            _this->ack_event.enqueue(resp_cycle - now);
            return vp::IO_REQ_GRANTED;
        }

        // Atomic: classic round-trip — the sender's own object comes back
        // via resp() (or inline DONE), nobody frees it.
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

    // READ: must be a single data-less descriptor (io_v2 beat-read convention;
    // the adapter's own request, never the master's buffered one).
    _this->traces.assert(req->is_first && req->is_last,
        "read must be a single descriptor (first=%d last=%d)",
        req->is_first ? 1 : 0, req->is_last ? 1 : 0);
    _this->traces.assert(req->get_data() == nullptr,
        "read request must be data-less");

    // Inline error completion: status inline, no stream.
    if (_this->error)
    {
        req->set_resp_status(vp::IO_RESP_INVALID);
        req->inc_latency(_this->latency);
        return vp::IO_REQ_DONE;
    }

    _this->streams.push_back(Stream{req->initiator, addr, sz, 0});
    _this->stream_event.enqueue(std::max((int64_t)1, _this->latency));
    return vp::IO_REQ_GRANTED;
}

// Emit one response beat of the head stream per cycle.
void StubTarget::stream_event_handler(vp::Block *__this, vp::ClockEvent *event)
{
    StubTarget *_this = (StubTarget *)__this;
    int64_t now = _this->clock.get_cycles();

    if (_this->streams.empty())
    {
        return;
    }

    Stream &s = _this->streams.front();
    uint64_t beat_sz = std::min<uint64_t>(s.total - s.emitted,
                                          (uint64_t)_this->beat_width);

    vp::IoReq *beat = _this->beat_allocator->alloc();
    beat->prepare();
    beat->set_addr(s.addr + s.emitted);
    beat->set_size(beat_sz);
    beat->set_is_write(false);
    beat->is_first = (s.emitted == 0);
    beat->is_last = (s.emitted + beat_sz >= s.total);
    beat->burst_id = -1;
    beat->initiator = s.initiator;
    for (uint64_t i = 0; i < beat_sz; i++)
    {
        beat->get_data()[i] = (uint8_t)((beat->get_addr() + i) & 0xff);
    }

    printf("[%ld] %s BEAT addr=0x%lx size=%lu first=%d last=%d\n",
        now, _this->logname.c_str(), beat->get_addr(), (unsigned long)beat_sz,
        beat->is_first ? 1 : 0, beat->is_last ? 1 : 0);

    // The adapter consumes each beat immediately (memcpy into the master's
    // buffer) and never back-pressures the downstream response.
    _this->traces.assert(_this->in.resp(beat) == vp::IO_RESP_ACCEPTED,
        "the adapter must accept downstream read beats");

    s.emitted += beat_sz;
    if (s.emitted >= s.total)
    {
        _this->streams.pop_front();
    }
    if (!_this->streams.empty())
    {
        _this->stream_event.enqueue(1);
    }
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
    printf("[%ld] %s RETRY\n", _this->clock.get_cycles(), _this->logname.c_str());
    // Tell the adapter (our master) it can re-send; it forwards the retry
    // upstream and the master re-issues synchronously inside this call.
    _this->in.retry();
}

extern "C" vp::Component *gv_new(vp::ComponentConf &config)
{
    return new StubTarget(config);
}
