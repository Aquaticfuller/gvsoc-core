// SPDX-FileCopyrightText: 2026 ETH Zurich, University of Bologna and EssilorLuxottica SAS
//
// SPDX-License-Identifier: Apache-2.0
//
// Authors: Germain Haugou (germain.haugou@gmail.com)

/*
 * CDCTester β€” bidirectional io_v2 clock-domain-bridge testbench driver.
 *
 * Issues a fixed sequence of writes followed by reads against its single
 * io_v2 master port, verifying each read returns the pattern written.
 *
 * `pipeline_burst` controls how many transactions the tester keeps in
 * flight at once. With pipeline_burst=1 (default) each access waits for
 * its response before the next is issued β€” single-in-flight, matches the
 * 35-test calibration matrix unchanged. With pipeline_burst>1 the tester
 * fires up to N requests back-to-back from a pool of slots, demonstrating
 * pipelining when the bridge under test has depth>=N.
 *
 * `burst_beats` selects the beat mode (0 = off, the historic big-packet
 * mode above). When > 0 the tester is a beat-plane master following the
 * io_v2 per-burst write-acknowledgement contract (io_v2.hpp, "Write
 * acknowledgement"):
 *
 *   - Each write access is a burst of `burst_beats` beats of `access_size`
 *     bytes, one beat per cycle, each an allocator-backed (size-0 pool)
 *     request whose data points into the slot buffer. A GRANTED beat is
 *     gone (the target consumes and frees it; the buffer stays valid while
 *     the beat is unfreed); the burst completes with ONE ack: inline DONE
 *     on the last beat (the tester keeps and frees its beat) or a single
 *     data-less ack via resp() that the tester frees. Acks are correlated
 *     by initiator (the slot), never by object identity.
 *   - Each read access is one data-less burst request (freed downstream);
 *     the response arrives as distinct pool-backed beats the tester copies
 *     out, verifies and frees.
 *   - A DENIED beat is held unchanged and re-sent after retry().
 *
 * Cross-tester quit coordination is unchanged: each tester pulses
 * `done_out` when its sequence completes; receiving the partner's pulse
 * on `done_in` plus being locally done causes engine->quit().
 */

#include <vp/vp.hpp>
#include <vp/itf/io_v2.hpp>
#include <vp/itf/wire.hpp>
#include <cstdarg>
#include <cstdio>
#include <cstring>


class CDCTester : public vp::Component
{
public:
    CDCTester(vp::ComponentConf &conf);
    void reset(bool active) override;

private:
    enum Phase
    {
        PHASE_WRITE,
        PHASE_READ,
        PHASE_WAIT_PARTNER,
        PHASE_DONE,
    };

    static constexpr int MAX_BURST = 16;

    struct Slot
    {
        bool     active = false;
        uint64_t index;
        bool     is_write;
        uint8_t  data[64];
        uint8_t  expected[64];
        // Big-packet mode only: the tester's own round-tripped request.
        vp::IoReq req;
        // Beat mode only:
        uint64_t beat_cursor = 0;    // write submission cursor (bytes)
        uint64_t resp_cursor = 0;    // read response fill cursor (bytes)
        vp::IoReq *held = nullptr;   // DENIED beat, held for the retry re-send
    };

    static vp::IoRespAck out_resp(vp::Block *__this, vp::IoReq *req);
    static void out_retry(vp::Block *__this, vp::IoRetryChannel);
    static void done_in_sync(vp::Block *__this, bool value);
    static void step_handler(vp::Block *__this, vp::ClockEvent *event);
    static void timeout_handler(vp::Block *__this, vp::ClockEvent *event);

    uint8_t pattern_byte(uint64_t off) const
    {
        return (uint8_t)((((uint32_t)this->pattern_seed ^ (off & 0xff))
                          + (uint32_t)(off >> 8)) & 0xff);
    }

    void schedule_step(int delay = 1);
    void step();
    bool issue_into_slot(int slot_idx, uint64_t index, bool is_write);
    int  find_free_slot();
    int  find_slot_for_req(vp::IoReq *req);
    bool issue_next_if_possible();
    void finish_local();
    void try_quit(int status);

    // Beat mode:
    bool beat_mode() const { return this->burst_beats > 0; }
    uint64_t burst_bytes() const
    {
        return this->beat_mode() ? this->burst_beats * this->access_size
                                 : this->access_size;
    }
    bool issue_beat_step();
    void start_burst(int slot_idx, uint64_t index, bool is_write);
    void send_current_beat();
    void complete_slot(Slot &s);
    vp::IoRespAck beat_resp(vp::IoReq *req);

    void fail(const char *fmt, ...) __attribute__((format(printf, 2, 3)));
    void pass();

    vp::IoMaster out{&CDCTester::out_retry, &CDCTester::out_resp};
    vp::WireSlave<bool> done_in;
    vp::WireMaster<bool> done_out;
    vp::ClockEvent step_event;
    vp::ClockEvent timeout_event;
    vp::Trace trace;

    // Configuration:
    std::string logname;
    uint64_t base;
    uint64_t access_size;
    uint64_t nb_accesses;
    uint32_t pattern_seed;
    int64_t  quit_after_cycles;
    int      pipeline_burst;
    int      burst_beats;        // 0 = big-packet mode, >0 = beat mode

    // State:
    Phase phase;
    uint64_t cursor;             // next access index to issue
    int64_t start_cycle = 0;
    bool partner_done = false;
    bool local_done = false;
    bool failed = false;
    bool retry_pending = false;  // last issue was DENIED; wait for retry

    Slot slots[MAX_BURST];
    int  in_flight_count = 0;

    // Beat-mode state: slot whose burst is currently being submitted (beats
    // of one burst never interleave with another on the binding), or -1.
    int issuing_slot = -1;
    // Size-0 pool for write beats / data-less read burst requests: they
    // cross a consumer-frees boundary, so they must be allocator-backed.
    vp::IoReqAllocator *beat_pool = nullptr;
};


CDCTester::CDCTester(vp::ComponentConf &config)
    : vp::Component(config),
      step_event(this, &CDCTester::step_handler),
      timeout_event(this, &CDCTester::timeout_handler)
{
    this->traces.new_trace("trace", &this->trace, vp::DEBUG);

    this->new_master_port("output", &this->out);

    this->done_in.set_sync_meth(&CDCTester::done_in_sync);
    this->new_slave_port("done_in", &this->done_in);

    this->new_master_port("done_out", &this->done_out);

    js::Config *cfg = this->get_js_config();
    this->logname     = cfg->get_child_str("logname");
    if (this->logname.empty()) this->logname = this->get_name();
    this->base        = (uint64_t)cfg->get_child_int("base");
    this->access_size = (uint64_t)cfg->get_child_int("access_size");
    if (this->access_size == 0 || this->access_size > sizeof(this->slots[0].data))
    {
        this->trace.fatal("access_size %lu out of range (1..%zu)\n",
                          this->access_size, sizeof(this->slots[0].data));
    }
    this->nb_accesses = (uint64_t)cfg->get_child_int("nb_accesses");
    this->pattern_seed = (uint32_t)cfg->get_child_int("pattern_seed");
    this->quit_after_cycles = cfg->get_child_int("quit_after_cycles");
    if (this->quit_after_cycles <= 0) this->quit_after_cycles = 1000000;

    js::Config *pb = cfg->get("pipeline_burst");
    this->pipeline_burst = pb != nullptr ? pb->get_int() : 1;
    if (this->pipeline_burst < 1) this->pipeline_burst = 1;
    if (this->pipeline_burst > MAX_BURST) this->pipeline_burst = MAX_BURST;

    js::Config *bb = cfg->get("burst_beats");
    this->burst_beats = bb != nullptr ? bb->get_int() : 0;
    if (this->burst_beats < 0) this->burst_beats = 0;
    if (this->beat_mode())
    {
        if (this->burst_bytes() > sizeof(this->slots[0].data))
        {
            this->trace.fatal("burst_beats*access_size %lu exceeds slot buffer (%zu)\n",
                              this->burst_bytes(), sizeof(this->slots[0].data));
        }
        this->beat_pool = vp::IoReqAllocator::get(0);
    }

    for (int i = 0; i < MAX_BURST; i++)
    {
        this->slots[i].req.set_data(this->slots[i].data);
        this->slots[i].req.set_size(this->access_size);
    }
}


void CDCTester::reset(bool active)
{
    if (!active)
    {
        this->phase = PHASE_WRITE;
        this->cursor = 0;
        this->partner_done = false;
        this->local_done = false;
        this->failed = false;
        this->retry_pending = false;
        this->in_flight_count = 0;
        this->issuing_slot = -1;
        for (int i = 0; i < MAX_BURST; i++)
        {
            Slot &s = this->slots[i];
            s.active = false;
            s.beat_cursor = 0;
            s.resp_cursor = 0;
            // A held (DENIED) beat is still ours — return it to its pool.
            if (s.held != nullptr)
            {
                s.held->free();
                s.held = nullptr;
            }
        }
        this->start_cycle = this->clock.get_cycles();
        printf("[%ld] %s START accesses=%lu access_size=%lu base=0x%lx seed=0x%x burst=%d\n",
            this->clock.get_cycles(), this->logname.c_str(),
            this->nb_accesses, this->access_size, this->base, this->pattern_seed,
            this->pipeline_burst);
        this->schedule_step(1);
        this->timeout_event.enqueue(this->quit_after_cycles);
    }
}


void CDCTester::schedule_step(int delay)
{
    if (delay < 1) delay = 1;
    if (!this->step_event.is_enqueued())
        this->step_event.enqueue(delay);
}


int CDCTester::find_free_slot()
{
    for (int i = 0; i < MAX_BURST; i++)
        if (!this->slots[i].active) return i;
    return -1;
}


int CDCTester::find_slot_for_req(vp::IoReq *req)
{
    for (int i = 0; i < MAX_BURST; i++)
        if (this->slots[i].active && &this->slots[i].req == req) return i;
    return -1;
}


bool CDCTester::issue_into_slot(int slot_idx, uint64_t index, bool is_write)
{
    Slot &s = this->slots[slot_idx];
    uint64_t off = index * this->access_size;
    uint64_t addr = this->base + off;

    for (uint64_t i = 0; i < this->access_size; i++)
        s.expected[i] = this->pattern_byte(off + i);

    if (is_write)
    {
        for (uint64_t i = 0; i < this->access_size; i++)
            s.data[i] = s.expected[i];
    }
    else
    {
        for (uint64_t i = 0; i < this->access_size; i++)
            s.data[i] = 0xee;   // poison
    }

    s.req.prepare();
    s.req.set_addr(addr);
    s.req.set_size(this->access_size);
    s.req.set_is_write(is_write);
    s.req.set_data(s.data);
    s.req.is_first = true;
    s.req.is_last  = true;
    s.req.burst_id = -1;
    s.req.set_resp_status(vp::IO_RESP_OK);

    s.active   = true;
    s.index    = index;
    s.is_write = is_write;
    this->in_flight_count++;

    printf("[%ld] %s %-5s idx=%lu addr=0x%lx slot=%d\n",
        this->clock.get_cycles(), this->logname.c_str(),
        is_write ? "WRITE" : "READ",
        index, addr, slot_idx);

    vp::IoReqStatus st = this->out.req(&s.req);
    if (st == vp::IO_REQ_DONE)
    {
        if (s.req.get_resp_status() != vp::IO_RESP_OK)
        {
            this->fail("sync DONE INVALID idx=%lu is_write=%d", index, is_write);
            return false;
        }
        if (!is_write && memcmp(s.data, s.expected, this->access_size) != 0)
        {
            this->fail("sync DONE mismatch idx=%lu", index);
            return false;
        }
        s.active = false;
        this->in_flight_count--;
        return true;
    }
    if (st == vp::IO_REQ_DENIED)
    {
        // Roll back: undo slot allocation.
        s.active = false;
        this->in_flight_count--;
        this->retry_pending = true;
        return false;
    }
    // GRANTED β€” response will arrive via out_resp.
    return true;
}


bool CDCTester::issue_next_if_possible()
{
    if (this->beat_mode()) return this->issue_beat_step();

    if (this->retry_pending) return false;
    if (this->in_flight_count >= this->pipeline_burst) return false;
    if (this->cursor >= this->nb_accesses) return false;
    int slot = this->find_free_slot();
    if (slot < 0) return false;
    bool ok = this->issue_into_slot(slot, this->cursor,
                                    this->phase == PHASE_WRITE);
    if (ok)
    {
        this->cursor++;
    }
    return ok;
}


// ---- Beat mode -------------------------------------------------------------

bool CDCTester::issue_beat_step()
{
    // At most one beat is issued per step (one beat per cycle);
    // send_current_beat() reschedules the step for the next beat, so this
    // always returns false to break the caller's issue loop.
    if (this->retry_pending) return false;

    if (this->issuing_slot < 0)
    {
        if (this->in_flight_count >= this->pipeline_burst) return false;
        if (this->cursor >= this->nb_accesses) return false;
        int slot_idx = this->find_free_slot();
        if (slot_idx < 0) return false;
        this->start_burst(slot_idx, this->cursor, this->phase == PHASE_WRITE);
        this->cursor++;
    }

    this->send_current_beat();
    return false;
}


void CDCTester::start_burst(int slot_idx, uint64_t index, bool is_write)
{
    Slot &s = this->slots[slot_idx];
    uint64_t bytes = this->burst_bytes();
    uint64_t off = index * bytes;

    for (uint64_t i = 0; i < bytes; i++)
        s.expected[i] = this->pattern_byte(off + i);
    for (uint64_t i = 0; i < bytes; i++)
        s.data[i] = is_write ? s.expected[i] : 0xee;   // 0xee = read poison

    s.active      = true;
    s.index       = index;
    s.is_write    = is_write;
    s.beat_cursor = 0;
    s.resp_cursor = 0;
    s.held        = nullptr;
    this->in_flight_count++;
    this->issuing_slot = slot_idx;

    printf("[%ld] %s %-5s idx=%lu addr=0x%lx slot=%d beats=%d\n",
        this->clock.get_cycles(), this->logname.c_str(),
        is_write ? "WRITE" : "READ",
        index, this->base + off, slot_idx, this->burst_beats);
}


void CDCTester::complete_slot(Slot &s)
{
    s.active = false;
    this->in_flight_count--;
    this->schedule_step(1);
}


void CDCTester::send_current_beat()
{
    Slot &s = this->slots[this->issuing_slot];
    uint64_t bytes = this->burst_bytes();
    uint64_t burst_addr = this->base + s.index * bytes;

    vp::IoReq *beat = s.held;
    if (beat == nullptr)
    {
        beat = this->beat_pool->alloc();
        beat->prepare();
        if (s.is_write)
        {
            // Allocator-backed write beat: data points into the slot buffer
            // (valid as long as the beat is unfreed — no copy).
            beat->set_addr(burst_addr + s.beat_cursor);
            beat->set_size(this->access_size);
            beat->set_data(s.data + s.beat_cursor);
            beat->set_opcode(vp::WRITE);
            beat->is_first = s.beat_cursor == 0;
            beat->is_last  = s.beat_cursor + this->access_size >= bytes;
        }
        else
        {
            // Data-less read burst request (beat-plane read convention);
            // freed downstream, the payload comes back as distinct beats.
            beat->set_addr(burst_addr);
            beat->set_size(bytes);
            beat->set_data(nullptr);
            beat->set_opcode(vp::READ);
            beat->is_first = true;
            beat->is_last  = true;
        }
        beat->burst_id  = (int64_t)s.index;
        beat->initiator = &s;
    }

    // Snapshot before req(): GRANTED transfers ownership of a write beat.
    bool was_last = beat->is_last;

    vp::IoReqStatus st = this->out.req(beat);

    if (st == vp::IO_REQ_DENIED)
    {
        // Hold the exact object unchanged, re-send after retry().
        s.held = beat;
        this->retry_pending = true;
        return;
    }
    s.held = nullptr;

    if (s.is_write)
    {
        if (st == vp::IO_REQ_DONE)
        {
            // The inline burst ack: only legal on the last beat (or as the
            // inline-INVALID escape hatch). The tester keeps the beat and,
            // being its allocator, returns it to the pool.
            vp::IoRespStatus status = beat->get_resp_status();
            beat->free();
            if (!was_last)
            {
                this->fail("DONE on non-last write beat idx=%lu (status=%d)",
                           s.index, (int)status);
                return;
            }
            if (status != vp::IO_RESP_OK)
            {
                this->fail("inline write ack INVALID idx=%lu", s.index);
                return;
            }
            this->issuing_slot = -1;
            this->complete_slot(s);
            return;
        }
        // GRANTED: the beat (buffer included) now belongs downstream — never
        // touch it again. Non-last beats get no response at all; the burst's
        // single ack arrives via out_resp once the last beat is consumed.
        s.beat_cursor += this->access_size;
        if (was_last)
        {
            this->issuing_slot = -1;   // submission done; await the ack
        }
        this->schedule_step(1);
    }
    else
    {
        if (st == vp::IO_REQ_DONE)
        {
            vp::IoRespStatus status = beat->get_resp_status();
            beat->free();
            this->fail("inline DONE on a data-less read idx=%lu (status=%d)",
                       s.index, (int)status);
            return;
        }
        // GRANTED: the request is gone (freed downstream); response beats
        // arrive via out_resp, correlated by initiator.
        this->issuing_slot = -1;
        this->schedule_step(1);
    }
}


vp::IoRespAck CDCTester::beat_resp(vp::IoReq *req)
{
    Slot *s = static_cast<Slot *>(req->initiator);
    int slot_idx = (int)(s - this->slots);
    if (slot_idx < 0 || slot_idx >= MAX_BURST || !s->active)
    {
        this->fail("resp with unknown initiator (req=%p)", (void *)req);
        return vp::IO_RESP_ACCEPTED;
    }

    if (req->get_resp_status() != vp::IO_RESP_OK)
    {
        req->free();
        this->fail("async resp INVALID idx=%lu is_write=%d",
                   s->index, s->is_write ? 1 : 0);
        return vp::IO_RESP_ACCEPTED;
    }

    if (req->get_opcode() == vp::WRITE)
    {
        // The burst's single data-less ack (possibly our own recycled last
        // beat — correlation is by initiator, never by object identity).
        // The initiator consumes the status and frees the ack.
        if (req->get_data() != nullptr)
        {
            this->fail("write burst ack carries data idx=%lu", s->index);
        }
        req->free();
        this->complete_slot(*s);
        return vp::IO_RESP_ACCEPTED;
    }

    // Read response beat: copy the payload out at the running offset, free
    // the beat, verify the whole burst on the last one.
    bool last = req->is_last;
    uint64_t size = req->get_size();
    if (s->resp_cursor + size > this->burst_bytes())
    {
        req->free();
        this->fail("read beats overflow burst idx=%lu", s->index);
        return vp::IO_RESP_ACCEPTED;
    }
    if (size > 0)
    {
        memcpy(s->data + s->resp_cursor, req->get_data(), size);
        s->resp_cursor += size;
    }
    req->free();

    if (last)
    {
        if (s->resp_cursor != this->burst_bytes())
        {
            this->fail("short read burst idx=%lu (%lu/%lu bytes)",
                       s->index, s->resp_cursor, this->burst_bytes());
            return vp::IO_RESP_ACCEPTED;
        }
        if (memcmp(s->data, s->expected, this->burst_bytes()) != 0)
        {
            this->fail("read mismatch idx=%lu (beat)", s->index);
            return vp::IO_RESP_ACCEPTED;
        }
        this->complete_slot(*s);
    }
    return vp::IO_RESP_ACCEPTED;
}


void CDCTester::step()
{
    switch (this->phase)
    {
        case PHASE_WRITE:
            // Drain accepted writes; move to read phase when all done.
            while (this->issue_next_if_possible()) {}
            if (this->cursor >= this->nb_accesses
                && this->in_flight_count == 0
                && !this->retry_pending)
            {
                printf("[%ld] %s WRITES_DONE\n",
                    this->clock.get_cycles(), this->logname.c_str());
                this->phase = PHASE_READ;
                this->cursor = 0;
                this->schedule_step(1);
                return;
            }
            break;

        case PHASE_READ:
            while (this->issue_next_if_possible()) {}
            if (this->cursor >= this->nb_accesses
                && this->in_flight_count == 0
                && !this->retry_pending)
            {
                this->pass();
                return;
            }
            break;

        case PHASE_WAIT_PARTNER:
        case PHASE_DONE:
            break;
    }

    // If we still have outstanding requests or pending retry, idle. The
    // resp / retry handlers reschedule the step when state changes.
    if (this->in_flight_count == 0
        && !this->retry_pending
        && this->phase != PHASE_DONE
        && this->phase != PHASE_WAIT_PARTNER)
    {
        this->schedule_step(1);
    }
}


vp::IoRespAck CDCTester::out_resp(vp::Block *__this, vp::IoReq *req)
{
    CDCTester *_this = (CDCTester *)__this;
    if (_this->failed) return vp::IO_RESP_ACCEPTED;

    if (_this->beat_mode()) return _this->beat_resp(req);

    int slot_idx = _this->find_slot_for_req(req);
    if (slot_idx < 0)
    {
        _this->fail("resp for unknown req=%p", (void *)req);
        return vp::IO_RESP_ACCEPTED;
    }
    Slot &s = _this->slots[slot_idx];

    if (req->get_resp_status() != vp::IO_RESP_OK)
    {
        _this->fail("async resp INVALID idx=%lu is_write=%d",
            s.index, s.is_write ? 1 : 0);
        return vp::IO_RESP_ACCEPTED;
    }
    if (!s.is_write
        && memcmp(s.data, s.expected, _this->access_size) != 0)
    {
        _this->fail("read mismatch idx=%lu (async)", s.index);
        return vp::IO_RESP_ACCEPTED;
    }

    s.active = false;
    _this->in_flight_count--;
    _this->schedule_step(1);
    return vp::IO_RESP_ACCEPTED;
}


void CDCTester::out_retry(vp::Block *__this, vp::IoRetryChannel)
{
    CDCTester *_this = (CDCTester *)__this;
    if (_this->retry_pending)
    {
        _this->retry_pending = false;
        _this->schedule_step(1);
    }
}


void CDCTester::done_in_sync(vp::Block *__this, bool value)
{
    CDCTester *_this = (CDCTester *)__this;
    if (!value) return;
    if (_this->partner_done) return;
    _this->partner_done = true;
    printf("[%ld] %s PARTNER_DONE\n",
        _this->clock.get_cycles(), _this->logname.c_str());
    if (_this->local_done)
        _this->try_quit(_this->failed ? 1 : 0);
}


void CDCTester::finish_local()
{
    this->local_done = true;
    if (this->done_out.is_bound())
        this->done_out.sync(true);
    if (this->partner_done)
        this->try_quit(this->failed ? 1 : 0);
    else
        this->phase = PHASE_WAIT_PARTNER;
}


void CDCTester::try_quit(int status)
{
    if (this->phase == PHASE_DONE) return;
    this->phase = PHASE_DONE;
    if (this->timeout_event.is_enqueued())
        this->timeout_event.cancel();
    this->time.get_engine()->quit(status);
}


void CDCTester::step_handler(vp::Block *__this, vp::ClockEvent *event)
{
    CDCTester *_this = (CDCTester *)__this;
    _this->step();
}


void CDCTester::timeout_handler(vp::Block *__this, vp::ClockEvent *event)
{
    CDCTester *_this = (CDCTester *)__this;
    _this->fail("timeout after %ld cycles in phase %d (cursor=%lu in_flight=%d)",
        _this->quit_after_cycles, (int)_this->phase,
        _this->cursor, _this->in_flight_count);
}


void CDCTester::fail(const char *fmt, ...)
{
    char buf[1024];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    printf("[%ld] %s FAIL %s\n", this->clock.get_cycles(), this->logname.c_str(), buf);
    this->failed = true;
    this->try_quit(1);
}


void CDCTester::pass()
{
    int64_t now = this->clock.get_cycles();
    printf("[%ld] %s PASS writes=%lu reads=%lu cycles=%ld\n",
        now, this->logname.c_str(),
        this->nb_accesses, this->nb_accesses,
        (long)(now - this->start_cycle));
    this->finish_local();
}


extern "C" vp::Component *gv_new(vp::ComponentConf &config)
{
    return new CDCTester(config);
}
