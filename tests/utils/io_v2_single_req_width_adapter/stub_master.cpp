// SPDX-FileCopyrightText: 2026 ETH Zurich, University of Bologna and EssilorLuxottica SAS
//
// SPDX-License-Identifier: Apache-2.0
//
// Authors: Germain Haugou (germain.haugou@gmail.com)

/*
 * Single-req testbench master for IoV2SingleReqWidthAdapter. It declares a
 * width-0 (unknown) IoV2SingleReq signature on its output, so binding it to a
 * width-declaring IoV2SingleReq stub target makes the framework auto-insert
 * the width adapter between them.
 *
 * Faithful to the single-req identity contract: each schedule entry is one
 * single-beat access (a read carries its own destination buffer), several may
 * be OUTSTANDING concurrently, and every completion must hand back the very
 * request object that was sent — POLICED with traces.assert (asserts builds),
 * along with the response forms (inline DONE / async GRANTED + one resp() /
 * DENIED + retry), status and read-data pattern.
 *
 * Schedule entry keys (cycle/addr/size required):
 *   cycle        : issue cycle of this access
 *   addr         : address
 *   size         : access size in bytes
 *   is_write     : false for reads
 *   expect_status: 0 => IO_RESP_OK, 1 => IO_RESP_INVALID (default 0)
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
    struct Entry {
        int64_t start_cycle;
        uint64_t addr;
        uint64_t size;
        bool is_write;
        int expect_status;
        std::string name;
        // Live state
        uint8_t *buffer = nullptr;
        vp::IoReq *req = nullptr;
    };

    static vp::IoRespAck resp_handler(vp::Block *__this, vp::IoReq *req);
    static void retry_handler(vp::Block *__this, vp::IoRetryChannel);
    static void issue_handler(vp::Block *__this, vp::ClockEvent *event);
    static void quit_handler(vp::Block *__this, vp::ClockEvent *event);

    void send(Entry *e);
    void complete(Entry *e, const char *how);

    vp::IoMaster out;
    vp::ClockEvent issue_event;
    vp::ClockEvent quit_event;
    vp::Trace trace;
    std::vector<Entry *> schedule;
    size_t next_to_schedule = 0;
    int outstanding = 0;
    std::string logname;
    int64_t quit_after_cycles = 200;
    // Identity contract: the set of request objects currently in flight; a
    // response must be exactly one of them.
    std::set<vp::IoReq *> in_flight;
    // An access the adapter DENIED, held and re-sent from retry_handler
    // (synchronously, per the io_v2 contract).
    Entry *held = nullptr;
};

StubMaster::StubMaster(vp::ComponentConf &config)
    : vp::Component(config),
      out(&StubMaster::retry_handler, &StubMaster::resp_handler),
      issue_event(this, &StubMaster::issue_handler),
      quit_event(this, &StubMaster::quit_handler)
{
    this->traces.new_trace("trace", &this->trace, vp::DEBUG);
    this->new_master_port("output", &this->out);

    this->logname = this->get_js_config()->get_child_str("logname");
    if (this->logname.empty()) this->logname = this->get_name();

    int qac = this->get_js_config()->get_child_int("quit_after_cycles");
    if (qac > 0) this->quit_after_cycles = qac;

    js::Config *sched = this->get_js_config()->get("schedule");
    if (sched != NULL)
    {
        for (auto &item : sched->get_elems())
        {
            Entry *e = new Entry();
            e->start_cycle = item->get_int("cycle");
            e->addr = (uint64_t)item->get_int("addr");
            e->size = (uint64_t)item->get_int("size");
            e->is_write = item->get_child_bool("is_write");
            e->expect_status = item->get_child_int("expect_status");
            e->name = item->get_child_str("name");
            if (e->name.empty()) e->name = "a" + std::to_string(this->schedule.size());
            this->schedule.push_back(e);
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

void StubMaster::send(Entry *e)
{
    int64_t now = this->clock.get_cycles();

    if (e->req == nullptr)
    {
        // A single-req access always carries its own buffer — the destination
        // of a read (unlike a data-less beat burst request) or the write data.
        uint64_t buf_size = e->size == 0 ? 1 : e->size;
        e->buffer = new uint8_t[buf_size];
        if (e->is_write)
        {
            for (uint64_t i = 0; i < e->size; i++)
            {
                e->buffer[i] = (uint8_t)((e->addr + i) & 0xff);
            }
        }
        e->req = new vp::IoReq(e->addr, e->buffer, e->size, e->is_write);
        e->req->prepare();
        e->req->is_first = true;
        e->req->is_last = true;
        e->req->burst_id = -1;
        e->req->initiator = e;
    }

    printf("[%ld] %s SEND name=%s addr=0x%lx size=%lu write=%d\n",
        now, this->logname.c_str(), e->name.c_str(), e->addr, e->size,
        e->is_write ? 1 : 0);

    vp::IoReqStatus st = this->out.req(e->req);

    if (st == vp::IO_REQ_DONE)
    {
        this->complete(e, "DONE");
    }
    else if (st == vp::IO_REQ_GRANTED)
    {
        this->outstanding++;
        this->in_flight.insert(e->req);
    }
    else
    {
        this->traces.assert(this->held == nullptr,
            "a second access was denied while one is already held");
        printf("[%ld] %s REQHOLD name=%s\n", now, this->logname.c_str(),
            e->name.c_str());
        this->held = e;
    }
}

// Common completion checks for inline (DONE) and async (resp) endings.
void StubMaster::complete(Entry *e, const char *how)
{
    int64_t now = this->clock.get_cycles();
    vp::IoReq *req = e->req;

    printf("[%ld] %s %s name=%s status=%d latency=%ld\n",
        now, this->logname.c_str(), how, e->name.c_str(),
        (int)req->get_resp_status(), (long)req->get_full_latency());

    this->traces.assert(
        (int)req->get_resp_status() == (e->expect_status ? (int)vp::IO_RESP_INVALID
                                                         : (int)vp::IO_RESP_OK),
        "status %d != expected", (int)req->get_resp_status());
    // Read data must carry the target's addr-derived pattern, in OUR buffer:
    // the adapter copies the beat payloads into the master's own destination.
    if (!e->is_write && req->get_resp_status() == vp::IO_RESP_OK)
    {
        for (uint64_t i = 0; i < e->size; i++)
        {
            this->traces.assert(
                e->buffer[i] == (uint8_t)((e->addr + i) & 0xff),
                "read data mismatch at addr 0x%lx byte %lu", e->addr, i);
        }
    }

    delete req;
    delete[] e->buffer;
    e->req = nullptr;
    e->buffer = nullptr;
}

void StubMaster::issue_handler(vp::Block *__this, vp::ClockEvent *event)
{
    StubMaster *_this = (StubMaster *)__this;
    if (_this->next_to_schedule >= _this->schedule.size()) return;

    Entry *e = _this->schedule[_this->next_to_schedule];
    _this->send(e);
    _this->next_to_schedule++;

    if (_this->next_to_schedule < _this->schedule.size())
    {
        int64_t now = _this->clock.get_cycles();
        int64_t delta = _this->schedule[_this->next_to_schedule]->start_cycle - now;
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
    _this->traces.assert(_this->outstanding == 0,
        "%d access(es) never completed", _this->outstanding);
    _this->traces.assert(_this->held == nullptr,
        "a denied access was never retried");
    printf("[%ld] %s QUIT\n", now, _this->logname.c_str());
    _this->time.get_engine()->quit(0);
}

vp::IoRespAck StubMaster::resp_handler(vp::Block *__this, vp::IoReq *req)
{
    StubMaster *_this = (StubMaster *)__this;

    // ---- The single-req identity contract ----
    // The async response must hand back the very object we sent: no
    // allocator-backed beat, no reallocated copy, ever reaches this master.
    auto it = _this->in_flight.find(req);
    _this->traces.assert(it != _this->in_flight.end(),
        "response is not one of our own in-flight requests (req=%p)", req);
    _this->in_flight.erase(it);

    Entry *e = (Entry *)req->initiator;
    _this->complete(e, "RESP");
    _this->outstanding--;

    return vp::IO_RESP_ACCEPTED;
}

void StubMaster::retry_handler(vp::Block *__this, vp::IoRetryChannel)
{
    StubMaster *_this = (StubMaster *)__this;
    if (_this->held == nullptr)
    {
        return;
    }
    // Re-send the held access now, synchronously, per the io_v2 contract.
    Entry *e = _this->held;
    _this->held = nullptr;
    printf("[%ld] %s REQRESUME name=%s\n", _this->clock.get_cycles(),
        _this->logname.c_str(), e->name.c_str());
    _this->send(e);
}

extern "C" vp::Component *gv_new(vp::ComponentConf &config)
{
    return new StubMaster(config);
}
