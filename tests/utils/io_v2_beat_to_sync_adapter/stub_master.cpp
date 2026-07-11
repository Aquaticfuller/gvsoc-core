// SPDX-FileCopyrightText: 2026 ETH Zurich, University of Bologna and EssilorLuxottica SAS
//
// SPDX-License-Identifier: Apache-2.0
//
// Authors: Germain Haugou (germain.haugou@gmail.com)

/*
 * Beat-master testbench for IoV2BeatToSyncAdapter. It declares signature
 * IoV2Beat on its output, so binding it to an IoV2Sync stub target makes the
 * framework auto-insert the adapter between them.
 *
 * READS: the master issues ONE whole-burst descriptor per schedule entry
 * (size = the full burst) and expects the adapter to stream back
 * ceil(size / beat_width) per-beat resp() calls — each a distinct
 * allocator-backed object the master frees.
 *
 * WRITES (per-burst ack contract, see io_v2.hpp "Write acknowledgement"): the
 * master submits the burst in beat form — one allocator-backed beat per cycle
 * from the size-0 pool, data aliasing its source buffer, the same initiator on
 * every beat — and forgets each beat on GRANTED (ownership, buffer included,
 * transfers to the consumer; everything still needed is snapshot BEFORE
 * req()). It then expects exactly ONE resp() per burst: the data-less burst
 * ack (is_first = is_last = true, data == NULL), which it frees. An inline
 * IO_REQ_DONE on the last beat (the contract's fast path) is tolerated: the
 * master keeps that beat and recycles it itself. The source buffer is only
 * reclaimed once the ack has arrived.
 *
 * It POLICES the response stream with traces.assert (active only in
 * asserts/debug builds): exactly N beats (reads) / exactly one ack (writes),
 * is_first/is_last placement, burst_id, ≤1 beat/cycle, status, and read data.
 *
 * Schedule entry keys (cycle/addr/size required):
 *   cycle        : issue cycle of this burst (writes: of its first beat)
 *   addr         : base address
 *   size         : whole-burst size in bytes (0 => one zero-size beat/ack)
 *   is_write     : false for reads
 *   burst_id     : tag echoed on the response(s) (default -1)
 *   expect_status: 0 => IO_RESP_OK, 1 => IO_RESP_INVALID (default 0)
 *   deny_beats   : list of 0-based response indices to back-pressure once each
 *                  (writes: only index 0 exists — the single burst ack): the
 *                  master returns IO_RESP_DENIED for that response, then calls
 *                  out.resp_retry() retry_delay cycles later. The adapter must
 *                  hold the exact object and re-send it; the second arrival is
 *                  accepted. Exercises the response-path back-pressure handshake.
 *   retry_delay  : cycles between a denied response and the resp_retry() (default 2)
 *   name         : log label
 */

#include <vp/vp.hpp>
#include <vp/itf/io_v2.hpp>
#include <cstdio>
#include <set>
#include <string>
#include <vector>

class StubMaster : public vp::Component
{
public:
    StubMaster(vp::ComponentConf &conf);
    void reset(bool active) override;

private:
    struct BurstEntry {
        int64_t start_cycle;
        uint64_t base_addr;
        uint64_t size;
        bool is_write;
        int64_t burst_id;
        int expect_status;
        std::vector<int> deny_beats;
        int retry_delay;
        std::string name;
    };

    // Live state for one outstanding burst, reachable from each resp() via
    // req->initiator (every write beat carries it, and the adapter copies it
    // onto the burst ack).
    struct BurstState {
        BurstEntry *entry;
        uint8_t *buffer;        // master-owned data buffer (writes: source,
                                // reclaimable only after the burst ack)
        vp::IoReq *req;         // READ only: the whole-burst descriptor
        int expected_beats;     // responses expected: reads = N beats; writes = 1 ack
        int nb_wr_beats;        // writes: beats to submit
        int wr_submitted;       // writes: beats accepted so far
        int beats_seen;
        int64_t last_resp_cycle;
        std::set<int> deny_remaining;   // response indices still to back-pressure once
    };

    static vp::IoRespAck resp_handler(vp::Block *__this, vp::IoReq *req);
    static void retry_handler(vp::Block *__this, vp::IoRetryChannel);
    static void issue_handler(vp::Block *__this, vp::ClockEvent *event);
    static void quit_handler(vp::Block *__this, vp::ClockEvent *event);
    static void resp_retry_handler(vp::Block *__this, vp::ClockEvent *event);

    void send_burst(BurstEntry *entry);
    // Submit the current write burst's next beat (allocator-backed, beat form).
    void send_write_beat(BurstState *bs);
    // Burst fully acknowledged (ack received or inline DONE): reclaim buffer/state.
    void complete_write_burst(BurstState *bs);
    // Arm the next step: next write beat (1/cycle), next schedule entry, or quit.
    void arm_next();

    vp::IoMaster out;
    vp::ClockEvent issue_event;
    vp::ClockEvent quit_event;
    vp::ClockEvent resp_retry_event;
    vp::Trace trace;
    std::vector<BurstEntry *> schedule;
    size_t next_to_schedule = 0;
    int beat_width = 0;
    int outstanding = 0;
    std::string logname;
    int64_t quit_after_cycles = 100;
    bool quit_armed = false;
    // Size-0 pool serving the write beats (data caller-managed, pointing into
    // the burst's source buffer). The beats are consumed and freed downstream;
    // the burst ack coming back is freed by us.
    vp::IoReqAllocator *wr_allocator;
    // Write burst currently submitting beats (one per cycle); nullptr once its
    // last beat has been accepted (the ack may still be outstanding).
    BurstState *cur_wr = nullptr;
};

StubMaster::StubMaster(vp::ComponentConf &config)
    : vp::Component(config),
      out(&StubMaster::retry_handler, &StubMaster::resp_handler),
      issue_event(this, &StubMaster::issue_handler),
      quit_event(this, &StubMaster::quit_handler),
      resp_retry_event(this, &StubMaster::resp_retry_handler)
{
    this->traces.new_trace("trace", &this->trace, vp::DEBUG);
    this->new_master_port("output", &this->out);

    this->logname = this->get_js_config()->get_child_str("logname");
    if (this->logname.empty()) this->logname = this->get_name();

    this->beat_width = this->get_js_config()->get_child_int("beat_width");

    this->wr_allocator = vp::IoReqAllocator::get(0);

    int qac = this->get_js_config()->get_child_int("quit_after_cycles");
    if (qac > 0) this->quit_after_cycles = qac;

    js::Config *sched = this->get_js_config()->get("schedule");
    if (sched != NULL)
    {
        for (auto &item : sched->get_elems())
        {
            BurstEntry *b = new BurstEntry();
            b->start_cycle = item->get_int("cycle");
            b->base_addr = (uint64_t)item->get_int("addr");
            b->size = (uint64_t)item->get_int("size");
            b->is_write = item->get_child_bool("is_write");
            b->burst_id = (int64_t)item->get_int("burst_id");
            if (item->get("burst_id") == nullptr) b->burst_id = -1;
            b->expect_status = item->get_child_int("expect_status");
            b->retry_delay = (int)item->get_int("retry_delay");
            if (b->retry_delay <= 0) b->retry_delay = 2;
            js::Config *deny = item->get("deny_beats");
            if (deny != nullptr)
            {
                for (auto &d : deny->get_elems())
                    b->deny_beats.push_back((int)d->get_int());
            }
            b->name = item->get_child_str("name");
            if (b->name.empty()) b->name = "b" + std::to_string(this->schedule.size());
            this->schedule.push_back(b);
        }
    }
}

void StubMaster::reset(bool active)
{
    if (!active && !this->schedule.empty() && this->next_to_schedule == 0)
    {
        int64_t first = this->schedule[0]->start_cycle;
        if (first <= 0) first = 1;
        this->issue_event.enqueue(first);
    }
}

void StubMaster::send_burst(BurstEntry *entry)
{
    int64_t now = this->clock.get_cycles();

    BurstState *bs = new BurstState();
    bs->entry = entry;
    int nb_beats = entry->size == 0 ? 1
        : (int)((entry->size + this->beat_width - 1) / this->beat_width);
    // Per-burst write ack: a write burst is answered by exactly ONE resp().
    bs->expected_beats = entry->is_write ? 1 : nb_beats;
    bs->nb_wr_beats = nb_beats;
    bs->wr_submitted = 0;
    bs->beats_seen = 0;
    bs->last_resp_cycle = -1;
    bs->deny_remaining = std::set<int>(entry->deny_beats.begin(),
                                       entry->deny_beats.end());

    printf("[%ld] %s SEND name=%s addr=0x%lx size=%lu write=%d burst_id=%ld expect_beats=%d\n",
        now, this->logname.c_str(), entry->name.c_str(), entry->base_addr,
        entry->size, entry->is_write ? 1 : 0, (long)entry->burst_id,
        bs->expected_beats);

    if (entry->is_write)
    {
        // Beat-form write: allocator-backed beats, data aliasing our buffer.
        // The buffer stays valid as long as any unfreed beat aliases it; we
        // may only reclaim it after the burst ack (see complete_write_burst).
        uint64_t buf_size = entry->size == 0 ? 1 : entry->size;
        bs->buffer = new uint8_t[buf_size];
        for (uint64_t i = 0; i < entry->size; i++)
        {
            bs->buffer[i] = (uint8_t)((entry->base_addr + i) & 0xff);
        }
        bs->req = nullptr;
        this->outstanding++;
        this->cur_wr = bs;
        this->send_write_beat(bs);
        return;
    }

    // READ: one whole-burst, data-less descriptor (beat protocol) — the data
    // comes back inside the allocator-backed response beats.
    bs->buffer = nullptr;
    bs->req = new vp::IoReq(entry->base_addr, nullptr, entry->size, false);
    bs->req->is_first = true;
    bs->req->is_last = true;
    bs->req->burst_id = entry->burst_id;
    bs->req->initiator = bs;

    this->outstanding++;
    vp::IoReqStatus st = this->out.req(bs->req);

    // The adapter always accepts and streams the response back via resp().
    this->traces.assert(st == vp::IO_REQ_GRANTED,
        "adapter must return IO_REQ_GRANTED for a read burst (got %d)", (int)st);
}

void StubMaster::send_write_beat(BurstState *bs)
{
    int64_t now = this->clock.get_cycles();
    BurstEntry *e = bs->entry;
    int i = bs->wr_submitted;
    uint64_t offset = (uint64_t)i * (uint64_t)this->beat_width;
    uint64_t chunk = e->size > offset
        ? std::min<uint64_t>(e->size - offset, (uint64_t)this->beat_width)
        : 0;

    // One allocator-backed beat from the size-0 pool. data is caller-managed
    // on this pool: point it into our source buffer (no copy). Every beat of
    // the burst carries the same initiator (contract requirement).
    vp::IoReq *beat = this->wr_allocator->alloc();
    beat->prepare();
    beat->set_addr(e->base_addr + offset);
    beat->set_data(bs->buffer + offset);
    beat->set_size(chunk);
    beat->set_opcode(vp::WRITE);
    beat->is_first = (i == 0);
    beat->is_last = (i == bs->nb_wr_beats - 1);
    beat->burst_id = e->burst_id;
    beat->initiator = bs;

    // Snapshot anything still needed BEFORE req(): on GRANTED ownership
    // (buffer included) transfers to the consumer and the beat must not be
    // touched again.
    bool last = beat->is_last;

    printf("[%ld] %s SENDB name=%s beat=%d/%d addr=0x%lx size=%lu first=%d last=%d\n",
        now, this->logname.c_str(), e->name.c_str(), i, bs->nb_wr_beats,
        e->base_addr + offset, (unsigned long)chunk,
        i == 0 ? 1 : 0, last ? 1 : 0);

    vp::IoReqStatus st = this->out.req(beat);

    if (st == vp::IO_REQ_GRANTED)
    {
        // Beat consumed (the adapter frees it) — forget it.
        bs->wr_submitted++;
        if (last)
        {
            this->cur_wr = nullptr;   // fully submitted; the ack is outstanding
        }
    }
    else if (st == vp::IO_REQ_DONE)
    {
        // Inline burst ack (fast path, last beat only): status and latency
        // are on the beat, ownership never transferred — we keep the beat
        // and recycle it ourselves.
        this->traces.assert(last,
            "inline DONE on a non-last write beat (beat=%d)", i);
        this->traces.assert(
            (int)beat->get_resp_status() ==
                (e->expect_status ? (int)vp::IO_RESP_INVALID
                                  : (int)vp::IO_RESP_OK),
            "inline write ack status %d != expected",
            (int)beat->get_resp_status());
        printf("[%ld] %s DONE name=%s status=%d\n", now, this->logname.c_str(),
            e->name.c_str(), (int)beat->get_resp_status());
        beat->free();
        this->cur_wr = nullptr;
        this->complete_write_burst(bs);
    }
    else
    {
        // The beat->sync adapter never back-pressures the request channel.
        this->traces.assert(false,
            "unexpected IO_REQ_DENIED from the beat-sync adapter");
    }
}

void StubMaster::complete_write_burst(BurstState *bs)
{
    // The burst ack implies every beat of the burst has been consumed and
    // freed — the source buffer is reclaimable again.
    delete[] bs->buffer;
    delete bs;
    this->outstanding--;
}

void StubMaster::arm_next()
{
    int64_t now = this->clock.get_cycles();
    if (this->cur_wr != nullptr)
    {
        // Next beat of the current write burst, one per cycle.
        this->issue_event.enqueue(1);
        return;
    }
    if (this->next_to_schedule < this->schedule.size())
    {
        int64_t next_cycle = this->schedule[this->next_to_schedule]->start_cycle;
        int64_t delta = next_cycle - now;
        if (delta <= 0) delta = 1;
        this->issue_event.enqueue(delta);
        return;
    }
    if (!this->quit_armed)
    {
        this->quit_armed = true;
        this->quit_event.enqueue(this->quit_after_cycles);
    }
}

void StubMaster::issue_handler(vp::Block *__this, vp::ClockEvent *event)
{
    StubMaster *_this = (StubMaster *)__this;

    if (_this->cur_wr != nullptr)
    {
        // Continue the write burst in flight, one beat per cycle.
        _this->send_write_beat(_this->cur_wr);
    }
    else if (_this->next_to_schedule < _this->schedule.size())
    {
        BurstEntry *entry = _this->schedule[_this->next_to_schedule];
        _this->next_to_schedule++;
        _this->send_burst(entry);
    }

    _this->arm_next();
}

void StubMaster::quit_handler(vp::Block *__this, vp::ClockEvent *event)
{
    StubMaster *_this = (StubMaster *)__this;
    int64_t now = _this->clock.get_cycles();
    // Every burst must have fully drained by quit time.
    _this->traces.assert(_this->outstanding == 0,
        "%d burst(s) never completed", _this->outstanding);
    printf("[%ld] %s QUIT\n", now, _this->logname.c_str());
    _this->time.get_engine()->quit(0);
}

vp::IoRespAck StubMaster::resp_handler(vp::Block *__this, vp::IoReq *req)
{
    StubMaster *_this = (StubMaster *)__this;
    int64_t now = _this->clock.get_cycles();
    BurstState *bs = (BurstState *)req->initiator;
    BurstEntry *e = bs->entry;

    // ---- Response-path back-pressure ----
    // Back-pressure this response once if its index is in deny_beats. We must
    // refuse BEFORE counting / asserting / freeing: the adapter holds this exact
    // object and re-sends it after we call out.resp_retry(), at which point we
    // accept it. beats_seen is the 0-based index of the response being offered
    // right now (for a write burst only index 0 exists — the single ack).
    if (bs->deny_remaining.erase(bs->beats_seen))
    {
        printf("[%ld] %s DENY name=%s beat=%d/%d retry_in=%d\n",
            now, _this->logname.c_str(), e->name.c_str(),
            bs->beats_seen, bs->expected_beats, e->retry_delay);
        // Only one beat can be held by the adapter at a time, so a single event
        // suffices. Re-opening happens in resp_retry_handler.
        _this->resp_retry_event.enqueue(e->retry_delay);
        return vp::IO_RESP_DENIED;
    }

    printf("[%ld] %s RESP name=%s beat=%d/%d addr=0x%lx size=%lu first=%d last=%d status=%d\n",
        now, _this->logname.c_str(), e->name.c_str(),
        bs->beats_seen, bs->expected_beats, req->get_addr(),
        (unsigned long)req->get_size(), req->is_first ? 1 : 0,
        req->is_last ? 1 : 0, (int)req->get_resp_status());

    // ---- Protocol assertions (the adapter / target behave correctly) ----
    _this->traces.assert(req->burst_id == e->burst_id,
        "beat burst_id %ld != expected %ld", (long)req->burst_id, (long)e->burst_id);
    if (bs->beats_seen == 0)
        _this->traces.assert(req->is_first, "first beat must have is_first=1");
    else
        _this->traces.assert(!req->is_first, "non-first beat must have is_first=0");
    _this->traces.assert(now > bs->last_resp_cycle,
        "more than one beat in cycle %ld (beat channel is 1/cycle)", now);
    bs->last_resp_cycle = now;
    _this->traces.assert(
        (int)req->get_resp_status() == (e->expect_status ? (int)vp::IO_RESP_INVALID
                                                         : (int)vp::IO_RESP_OK),
        "beat status %d != expected", (int)req->get_resp_status());
    if (e->is_write)
    {
        // The single per-burst write ack: data-less, single-beat framing.
        _this->traces.assert(req->is_first && req->is_last,
            "write burst ack must have is_first=is_last=1");
        _this->traces.assert(req->get_data() == NULL,
            "write burst ack must be data-less");
    }
    // Read data must carry the target's addr-derived pattern.
    if (!e->is_write && req->get_resp_status() == vp::IO_RESP_OK)
    {
        uint8_t *d = req->get_data();
        for (uint64_t i = 0; i < req->get_size(); i++)
        {
            _this->traces.assert(d[i] == (uint8_t)((req->get_addr() + i) & 0xff),
                "read data mismatch at beat addr 0x%lx byte %lu", req->get_addr(), i);
        }
    }

    bs->beats_seen++;
    bool last = req->is_last;

    // ---- Ownership ----
    // Write burst ack: a distinct data-less allocator-backed object (the
    // adapter recycled one of OUR consumed beats, but we must never correlate
    // by object identity) — the initiator frees it. Read responses are
    // distinct allocator-backed objects too — free each back to its pool. Our
    // read descriptor (bs->req) is ours alone: nothing downstream frees it or
    // round-trips it, and we free it on the last response.
    if (e->is_write)
    {
        req->free();
    }
    else if (req != bs->req)
    {
        req->free();    // a distinct allocator-backed response beat
    }
    else
    {
        _this->traces.assert(false,
            "our descriptor must never be round-tripped as a read beat");
    }

    if (last)
    {
        _this->traces.assert(bs->beats_seen == bs->expected_beats,
            "got %d beats, expected %d", bs->beats_seen, bs->expected_beats);
        if (e->is_write)
        {
            _this->complete_write_burst(bs);
        }
        else
        {
            delete bs->req;   // our descriptor (initiator-owned)
            delete bs;
            _this->outstanding--;
        }
    }

    return vp::IO_RESP_ACCEPTED;
}

void StubMaster::retry_handler(vp::Block *__this, vp::IoRetryChannel)
{
    StubMaster *_this = (StubMaster *)__this;
    // The adapter never denies a beat master's request (it always accepts and
    // streams), so it must never retry the request channel. This is the
    // request-path retry, distinct from the response-path resp_retry() we drive.
    _this->traces.assert(false, "unexpected retry() from the beat-sync adapter");
}

void StubMaster::resp_retry_handler(vp::Block *__this, vp::ClockEvent *event)
{
    StubMaster *_this = (StubMaster *)__this;
    int64_t now = _this->clock.get_cycles();
    printf("[%ld] %s RETRY\n", now, _this->logname.c_str());
    // Tell the adapter we can accept responses again. The adapter re-sends the
    // exact held beat synchronously inside this call (-> resp_handler accepts it).
    _this->out.resp_retry(vp::IO_RETRY_ANY);
}

extern "C" vp::Component *gv_new(vp::ComponentConf &config)
{
    return new StubMaster(config);
}
