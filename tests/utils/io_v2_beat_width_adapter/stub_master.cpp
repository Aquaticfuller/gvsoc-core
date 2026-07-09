// SPDX-FileCopyrightText: 2026 ETH Zurich, University of Bologna and EssilorLuxottica SAS
//
// SPDX-License-Identifier: Apache-2.0
//
// Authors: Germain Haugou (germain.haugou@gmail.com)

/*
 * Beat-master testbench for IoV2BeatWidthAdapter. It declares signature
 * IoV2Beat(beat_width) on its output, so binding it to an IoV2Beat stub target
 * of a different width makes the framework auto-insert the width adapter.
 *
 * Reads and big-packet writes are issued as ONE whole-burst request per
 * schedule entry; the master then expects ceil(size / beat_width) resp() beats
 * at ITS OWN width. With write_beats=true a write burst is instead streamed as
 * per-beat requests, one per cycle, each expecting one ack — the beat-form
 * write of the v2 protocol, which the width adapter packs/chops downstream.
 *
 * The response stream is POLICED with traces.assert (asserts builds): exactly
 * N beats, is_first/is_last placement, burst_id, <=1 beat/cycle, status, read
 * data pattern, and the initiator-owned ownership rules.
 *
 * Schedule entry keys (cycle/addr/size required):
 *   cycle        : issue cycle of this burst
 *   addr         : base address
 *   size         : whole-burst size in bytes (0 => one zero-size beat)
 *   is_write     : false for reads
 *   write_beats  : stream the write as per-beat requests (implies is_write)
 *   burst_id     : tag echoed on every beat (default -1)
 *   expect_status: 0 => IO_RESP_OK, 1 => IO_RESP_INVALID (default 0)
 *   deny_beats   : list of 0-based beat indices to back-pressure once each
 *                  (IO_RESP_DENIED then out.resp_retry() retry_delay later)
 *   retry_delay  : cycles between a denied beat and the resp_retry (default 2)
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
        bool write_beats;
        int64_t burst_id;
        int expect_status;
        std::vector<int> deny_beats;
        int retry_delay;
        std::string name;
    };

    // Live state for one outstanding burst, reachable from each resp() beat via
    // req->initiator.
    struct BurstState {
        BurstEntry *entry;
        uint8_t *buffer;            // master-owned data buffer (writes)
        vp::IoReq *req;             // whole-burst descriptor (null in beat mode)
        std::set<vp::IoReq *> own_beats;  // per-beat write requests in flight
        uint64_t beats_sent_bytes;  // beat mode: bytes already submitted
        int expected_beats;
        int beats_seen;
        std::set<int> deny_remaining;   // beat indices still to back-pressure once
        vp::IoReq *held_beat;       // beat-mode write DENIED, re-sent on retry
    };

    static vp::IoRespAck resp_handler(vp::Block *__this, vp::IoReq *req);
    static void retry_handler(vp::Block *__this, vp::IoRetryChannel);
    static void issue_handler(vp::Block *__this, vp::ClockEvent *event);
    static void beat_pump_handler(vp::Block *__this, vp::ClockEvent *event);
    static void quit_handler(vp::Block *__this, vp::ClockEvent *event);
    static void resp_retry_handler(vp::Block *__this, vp::ClockEvent *event);

    void send_burst(BurstEntry *entry);
    void send_next_write_beat(BurstState *bs);

    vp::IoMaster out;
    vp::ClockEvent issue_event;
    vp::ClockEvent beat_pump_event;
    vp::ClockEvent quit_event;
    vp::ClockEvent resp_retry_event;
    vp::Trace trace;
    std::vector<BurstEntry *> schedule;
    size_t next_to_schedule = 0;
    int beat_width = 0;
    int outstanding = 0;
    std::string logname;
    int64_t quit_after_cycles = 200;
    // Whole-burst request DENIED by the adapter, held and re-sent from
    // retry_handler (synchronously, per the io_v2 contract).
    BurstState *req_held = nullptr;
    // Beat-mode write burst currently streaming its per-beat requests.
    BurstState *streaming = nullptr;
    // Response-path 1-beat/cycle arbiter: cycle of the last accepted beat.
    int64_t last_accept_cycle = -1;
};

StubMaster::StubMaster(vp::ComponentConf &config)
    : vp::Component(config),
      out(&StubMaster::retry_handler, &StubMaster::resp_handler),
      issue_event(this, &StubMaster::issue_handler),
      beat_pump_event(this, &StubMaster::beat_pump_handler),
      quit_event(this, &StubMaster::quit_handler),
      resp_retry_event(this, &StubMaster::resp_retry_handler)
{
    this->traces.new_trace("trace", &this->trace, vp::DEBUG);
    this->new_master_port("output", &this->out);

    this->logname = this->get_js_config()->get_child_str("logname");
    if (this->logname.empty()) this->logname = this->get_name();

    this->beat_width = this->get_js_config()->get_child_int("beat_width");

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
            b->write_beats = item->get_child_bool("write_beats");
            if (b->write_beats) b->is_write = true;
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
    bs->expected_beats = entry->size == 0 ? 1
        : (int)((entry->size + this->beat_width - 1) / this->beat_width);
    bs->beats_seen = 0;
    bs->beats_sent_bytes = 0;
    bs->held_beat = nullptr;
    bs->deny_remaining = std::set<int>(entry->deny_beats.begin(),
                                       entry->deny_beats.end());

    // For a write, preload a buffer with the addr-derived pattern (the target
    // checks it). A read burst is data-less (beat protocol): the data comes
    // back inside the allocator-backed response beats.
    if (entry->is_write)
    {
        uint64_t buf_size = entry->size == 0 ? 1 : entry->size;
        bs->buffer = new uint8_t[buf_size];
        for (uint64_t i = 0; i < entry->size; i++)
        {
            bs->buffer[i] = (uint8_t)((entry->base_addr + i) & 0xff);
        }
    }
    else
    {
        bs->buffer = nullptr;
    }

    printf("[%ld] %s SEND name=%s addr=0x%lx size=%lu write=%d beats_mode=%d burst_id=%ld expect_beats=%d\n",
        now, this->logname.c_str(), entry->name.c_str(), entry->base_addr,
        entry->size, entry->is_write ? 1 : 0, entry->write_beats ? 1 : 0,
        (long)entry->burst_id, bs->expected_beats);

    if (entry->write_beats)
    {
        // Beat-form write: stream per-beat requests, one per cycle.
        this->traces.assert(this->streaming == nullptr,
            "a beat-form write started while another is still streaming");
        bs->req = nullptr;
        this->streaming = bs;
        this->outstanding++;
        this->send_next_write_beat(bs);
        return;
    }

    bs->req = new vp::IoReq(entry->base_addr, bs->buffer, entry->size, entry->is_write);
    bs->req->prepare();
    bs->req->is_first = true;
    bs->req->is_last = true;
    bs->req->burst_id = entry->burst_id;
    bs->req->initiator = bs;

    vp::IoReqStatus st = this->out.req(bs->req);

    if (st == vp::IO_REQ_GRANTED)
    {
        this->outstanding++;
    }
    else if (st == vp::IO_REQ_DENIED)
    {
        this->traces.assert(this->req_held == nullptr,
            "a second request was denied while one is already held");
        printf("[%ld] %s REQHOLD name=%s write=%d\n", now, this->logname.c_str(),
            entry->name.c_str(), entry->is_write ? 1 : 0);
        this->req_held = bs;
    }
    else
    {
        this->traces.assert(false,
            "adapter must return GRANTED or DENIED for a beat master (got %d)",
            (int)st);
    }
}

// Submit the next per-beat write request of the streaming burst. On DENY the
// beat is held (re-sent synchronously from retry_handler); otherwise the pump
// re-arms for the next beat one cycle later.
void StubMaster::send_next_write_beat(BurstState *bs)
{
    int64_t now = this->clock.get_cycles();
    BurstEntry *e = bs->entry;
    uint64_t off = bs->beats_sent_bytes;
    uint64_t beat = std::min<uint64_t>(e->size - off, (uint64_t)this->beat_width);

    vp::IoReq *r = new vp::IoReq(e->base_addr + off, bs->buffer + off, beat, true);
    r->prepare();
    r->is_first = (off == 0);
    r->is_last = (off + beat >= e->size);
    r->burst_id = e->burst_id;
    r->initiator = bs;
    bs->own_beats.insert(r);

    printf("[%ld] %s WBEAT name=%s addr=0x%lx size=%lu first=%d last=%d\n",
        now, this->logname.c_str(), e->name.c_str(), r->get_addr(), beat,
        r->is_first ? 1 : 0, r->is_last ? 1 : 0);

    vp::IoReqStatus st = this->out.req(r);

    if (st == vp::IO_REQ_DENIED)
    {
        printf("[%ld] %s REQHOLD name=%s write=1\n", now, this->logname.c_str(),
            e->name.c_str());
        bs->held_beat = r;
        return;
    }
    this->traces.assert(st == vp::IO_REQ_GRANTED,
        "adapter must return GRANTED or DENIED for a write beat (got %d)", (int)st);

    bs->beats_sent_bytes += beat;
    if (bs->beats_sent_bytes >= e->size)
    {
        this->streaming = nullptr;   // all beats submitted; acks still pending
    }
    else
    {
        this->beat_pump_event.enqueue(1);
    }
}

void StubMaster::beat_pump_handler(vp::Block *__this, vp::ClockEvent *event)
{
    StubMaster *_this = (StubMaster *)__this;
    if (_this->streaming != nullptr && _this->streaming->held_beat == nullptr)
    {
        _this->send_next_write_beat(_this->streaming);
    }
}

void StubMaster::issue_handler(vp::Block *__this, vp::ClockEvent *event)
{
    StubMaster *_this = (StubMaster *)__this;
    if (_this->next_to_schedule >= _this->schedule.size()) return;

    BurstEntry *entry = _this->schedule[_this->next_to_schedule];
    _this->send_burst(entry);
    _this->next_to_schedule++;

    if (_this->next_to_schedule < _this->schedule.size())
    {
        int64_t now = _this->clock.get_cycles();
        int64_t next_cycle = _this->schedule[_this->next_to_schedule]->start_cycle;
        int64_t delta = next_cycle - now;
        if (delta <= 0) delta = 1;
        _this->issue_event.enqueue(delta);
    }
    else
    {
        _this->quit_event.enqueue(_this->quit_after_cycles);
    }
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
    // Back-pressure this beat once if its index is in deny_beats. We must refuse
    // BEFORE counting / asserting / freeing: the adapter holds this exact object
    // and re-sends it after we call out.resp_retry(), at which point we accept it.
    if (bs->deny_remaining.erase(bs->beats_seen))
    {
        printf("[%ld] %s DENY name=%s beat=%d/%d retry_in=%d\n",
            now, _this->logname.c_str(), e->name.c_str(),
            bs->beats_seen, bs->expected_beats, e->retry_delay);
        _this->resp_retry_event.enqueue(e->retry_delay);
        return vp::IO_RESP_DENIED;
    }

    // ---- 1-beat/cycle response arbiter ----
    // A faithful beat consumer accepts at most one beat per cycle: refuse a
    // second same-cycle beat and re-drive it next cycle via resp_retry().
    if (now <= _this->last_accept_cycle)
    {
        printf("[%ld] %s STALL name=%s beat=%d/%d\n",
            now, _this->logname.c_str(), e->name.c_str(),
            bs->beats_seen, bs->expected_beats);
        _this->resp_retry_event.enqueue(1);
        return vp::IO_RESP_DENIED;
    }

    printf("[%ld] %s RESP name=%s beat=%d/%d addr=0x%lx size=%lu first=%d last=%d status=%d\n",
        now, _this->logname.c_str(), e->name.c_str(),
        bs->beats_seen, bs->expected_beats, req->get_addr(),
        (unsigned long)req->get_size(), req->is_first ? 1 : 0,
        req->is_last ? 1 : 0, (int)req->get_resp_status());

    // ---- Protocol assertions ----
    _this->traces.assert(req->burst_id == e->burst_id,
        "beat burst_id %ld != expected %ld", (long)req->burst_id, (long)e->burst_id);
    if (bs->beats_seen == 0)
        _this->traces.assert(req->is_first, "first beat must have is_first=1");
    else
        _this->traces.assert(!req->is_first, "non-first beat must have is_first=0");
    _this->last_accept_cycle = now;
    _this->traces.assert(
        (int)req->get_resp_status() == (e->expect_status ? (int)vp::IO_RESP_INVALID
                                                         : (int)vp::IO_RESP_OK),
        "beat status %d != expected", (int)req->get_resp_status());
    // Read data must carry the target's addr-derived pattern (checks that the
    // width repacking preserved cumulative byte order).
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

    // ---- Ownership (initiator-owned request convention) ----
    // We own our requests and free them ourselves. Read responses are distinct
    // allocator-backed objects the adapter produces — free each to its pool.
    // Write acks round-trip our own object: the whole-burst descriptor (each of
    // its ceil(size/width) acks reuses it), or, in beat mode, the matching
    // per-beat request (freed on its single ack).
    if (!e->is_write)
    {
        _this->traces.assert(req != bs->req,
            "our descriptor must never be round-tripped as a read beat");
        req->free();
    }
    else if (e->write_beats)
    {
        auto it = bs->own_beats.find(req);
        _this->traces.assert(it != bs->own_beats.end(),
            "write ack must round-trip one of our own beat requests");
        bs->own_beats.erase(it);
        delete req;
    }
    else
    {
        _this->traces.assert(req == bs->req,
            "write ack must round-trip our own descriptor");
    }

    if (last)
    {
        _this->traces.assert(bs->beats_seen == bs->expected_beats,
            "got %d beats, expected %d", bs->beats_seen, bs->expected_beats);
        delete bs->req;       // our whole-burst request (null in beat mode)
        delete[] bs->buffer;
        delete bs;
        _this->outstanding--;
    }

    return vp::IO_RESP_ACCEPTED;
}

void StubMaster::retry_handler(vp::Block *__this, vp::IoRetryChannel)
{
    StubMaster *_this = (StubMaster *)__this;
    int64_t now = _this->clock.get_cycles();

    // A held per-beat write must be re-sent synchronously inside retry().
    if (_this->streaming != nullptr && _this->streaming->held_beat != nullptr)
    {
        BurstState *bs = _this->streaming;
        vp::IoReq *r = bs->held_beat;
        vp::IoReqStatus st = _this->out.req(r);
        if (st == vp::IO_REQ_GRANTED)
        {
            printf("[%ld] %s REQRESUME name=%s\n", now, _this->logname.c_str(),
                bs->entry->name.c_str());
            bs->held_beat = nullptr;
            bs->beats_sent_bytes += r->get_size();
            if (bs->beats_sent_bytes >= bs->entry->size)
            {
                _this->streaming = nullptr;
            }
            else
            {
                _this->beat_pump_event.enqueue(1);
            }
        }
        return;
    }

    // A held whole-burst request: re-send it now, synchronously.
    if (_this->req_held == nullptr)
    {
        return;
    }
    BurstState *bs = _this->req_held;
    vp::IoReqStatus st = _this->out.req(bs->req);
    if (st == vp::IO_REQ_GRANTED)
    {
        printf("[%ld] %s REQRESUME name=%s\n", now, _this->logname.c_str(),
            bs->entry->name.c_str());
        _this->req_held = nullptr;
        _this->outstanding++;
    }
    else if (st != vp::IO_REQ_DENIED)
    {
        _this->traces.assert(false,
            "adapter must return GRANTED or DENIED on retry (got %d)", (int)st);
    }
}

void StubMaster::resp_retry_handler(vp::Block *__this, vp::ClockEvent *event)
{
    StubMaster *_this = (StubMaster *)__this;
    int64_t now = _this->clock.get_cycles();
    printf("[%ld] %s RETRY\n", now, _this->logname.c_str());
    // Tell the adapter we can accept responses again. It re-sends the exact
    // held beat synchronously inside this call (-> resp_handler accepts it).
    _this->out.resp_retry(vp::IO_RETRY_ANY);
}

extern "C" vp::Component *gv_new(vp::ComponentConf &config)
{
    return new StubMaster(config);
}
