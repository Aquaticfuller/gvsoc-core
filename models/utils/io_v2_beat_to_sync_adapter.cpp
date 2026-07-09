// SPDX-FileCopyrightText: 2026 ETH Zurich, University of Bologna and EssilorLuxottica SAS
//
// SPDX-License-Identifier: Apache-2.0
//
// Authors: Germain Haugou (germain.haugou@gmail.com)

#include "io_v2_beat_to_sync_adapter.hpp"

#include <algorithm>


IoV2BeatToSyncAdapter::IoV2BeatToSyncAdapter(vp::ComponentConf &config)
    : vp::Component(config),
      in(&IoV2BeatToSyncAdapter::req_handler, &IoV2BeatToSyncAdapter::resp_retry_in_handler),
      out(&IoV2BeatToSyncAdapter::retry_handler, &IoV2BeatToSyncAdapter::resp_handler),
      fsm_event(this, &IoV2BeatToSyncAdapter::fsm_handler)
{
    this->traces.new_trace("trace", &this->trace, vp::DEBUG);

    this->beat_width = this->get_js_config()->get_child_int("beat_width");
    if (this->beat_width <= 0)
    {
        this->trace.fatal("IoV2BeatToSyncAdapter requires beat_width > 0 (got %d)\n",
                          this->beat_width);
    }

    this->beat_allocator = vp::IoReqAllocator::get(this->beat_width);

    this->new_slave_port("input", &this->in);
    this->new_master_port("output", &this->out);
}


vp::IoReqStatus IoV2BeatToSyncAdapter::req_handler(vp::Block *__this, vp::IoReq *req)
{
    auto *self = static_cast<IoV2BeatToSyncAdapter *>(__this);

    self->trace.msg(vp::Trace::LEVEL_TRACE,
        "Submit (req=%p, addr=0x%lx, size=%lu, write=%d, burst_id=%ld)\n",
        req, req->get_addr(), req->get_size(), req->get_is_write() ? 1 : 0,
        (long)req->burst_id);

    uint64_t burst_addr = req->get_addr();
    uint64_t total      = req->get_size();
    int64_t  burst_id   = req->burst_id;
    int64_t  latency    = 0;

    if (req->get_is_write())
    {
        // Forward the whole write burst to the sync slave (the request carries
        // the full payload). By the IoV2Sync contract it serves any size inline
        // and must complete with IO_REQ_DONE (never GRANTED/DENIED). Assert it
        // honoured the contract (asserts/debug builds). Then queue one ack
        // entry per beat, each round-tripping the master's own request.
        vp::IoReqStatus st = self->out.req(req);
        self->traces.assert(st == vp::IO_REQ_DONE,
            "sync slave must reply IO_REQ_DONE inline (got %d)", (int)st);

        latency = req->get_full_latency();
        vp::IoRespStatus status = req->get_resp_status();
        uint8_t *data = req->get_data();
        uint64_t offset = 0;
        do
        {
            // A zero-size burst still gets a single zero-size ack.
            uint64_t beat = std::min<uint64_t>(total - offset,
                                               (uint64_t)self->beat_width);
            self->entries.push_back(StreamEntry{nullptr, req,
                burst_addr + offset, data + offset, beat,
                offset == 0, offset + beat >= total, burst_id, status});
            offset += beat;
        } while (offset < total);
    }
    else
    {
        // Read burst requests carry no data: serve each beat inline, at submit
        // time, into a distinct allocator-backed object whose co-allocated
        // payload receives the beat's data (the sync slave fills it directly).
        // The whole burst is read in this cycle — exactly when the previous
        // whole-burst forward read it — so data snapshot and timing are
        // unchanged; the objects then just stream upstream one per cycle. The
        // terminal master copies each payload out and frees the beat.
        uint64_t offset = 0;
        do
        {
            // A zero-size burst still gets a single zero-size completion beat.
            uint64_t beat = std::min<uint64_t>(total - offset,
                                               (uint64_t)self->beat_width);
            vp::IoReq *b = self->beat_allocator->alloc();
            b->prepare();
            b->set_addr(burst_addr + offset);
            b->set_size(beat);
            b->set_is_write(false);
            // Single-beat framing downstream (each sync call is an
            // independent request); the upstream burst framing is applied
            // after the call.
            b->is_first = true;
            b->is_last  = true;
            b->burst_id = -1;

            vp::IoReqStatus st = self->out.req(b);
            self->traces.assert(st == vp::IO_REQ_DONE,
                "sync slave must reply IO_REQ_DONE inline (got %d)", (int)st);
            latency = std::max(latency, b->get_full_latency());

            b->is_first = offset == 0;
            b->is_last  = offset + beat >= total;
            b->burst_id = burst_id;
            b->initiator = req->initiator;

            self->entries.push_back(StreamEntry{b, nullptr, 0, nullptr, 0,
                false, false, -1, vp::IO_RESP_OK});
            offset += beat;
        } while (offset < total);
    }

    // Start the FSM after the head latency. enqueue() keeps the earliest pending
    // cycle, so while already streaming (enqueued at +1) this is a no-op and the
    // queued beats just drain continuously behind the active ones.
    self->fsm_event.enqueue(std::max((int64_t)1, latency));

    return vp::IO_REQ_GRANTED;
}


vp::IoRespAck IoV2BeatToSyncAdapter::resp_handler(vp::Block *__this, vp::IoReq *req)
{
    auto *self = static_cast<IoV2BeatToSyncAdapter *>(__this);
    // A sync slave never responds asynchronously.
    self->trace.fatal("Unexpected resp() from a sync slave (req=%p)\n", req);
    return vp::IO_RESP_ACCEPTED;
}


void IoV2BeatToSyncAdapter::retry_handler(vp::Block *__this, vp::IoRetryChannel)
{
    auto *self = static_cast<IoV2BeatToSyncAdapter *>(__this);
    // A sync slave never denies, so it never retries.
    self->trace.fatal("Unexpected retry() from a sync slave\n");
}


bool IoV2BeatToSyncAdapter::emit_entry(const StreamEntry &e)
{
    vp::IoReq *r;
    if (e.beat != nullptr)
    {
        // Read beat: distinct allocator-backed object, fully prepared at
        // submit time (payload already holds the data). The terminal master
        // copies it out and frees it — the master's burst request is never
        // round-tripped as a read beat (initiator-owned convention).
        r = e.beat;
    }
    else
    {
        // Write ack: round-trip the master's own request object, mutated per
        // beat. Its data may point into the master's own buffer — writes are
        // exempt from the data-less rule since nobody else frees them.
        r = e.wreq;
        r->set_addr(e.addr);
        r->set_data(e.data);
        r->set_size(e.size);
        r->burst_id = e.burst_id;
        r->is_first = e.is_first;
        r->is_last = e.is_last;
        r->set_resp_status(e.status);
    }

    this->trace.msg(vp::Trace::LEVEL_TRACE,
        "Emit beat (req=%p, addr=0x%lx, size=%lu, first=%d, last=%d, write=%d)\n",
        r, r->get_addr(), r->get_size(), r->is_first ? 1 : 0, r->is_last ? 1 : 0,
        r->get_is_write() ? 1 : 0);

    if (this->in.resp(r) == vp::IO_RESP_DENIED)
    {
        // Upstream back-pressure: hold this exact object and re-send on
        // resp_retry. The entry was already popped by the caller, so nothing
        // else advances until the held beat is accepted.
        this->resp_held = true;
        this->held_req = r;
        return false;
    }
    return true;
}


void IoV2BeatToSyncAdapter::resp_retry_in_handler(vp::Block *__this,
                                                vp::IoRetryChannel /*channel*/)
{
    auto *self = static_cast<IoV2BeatToSyncAdapter *>(__this);
    if (!self->resp_held)
    {
        return;
    }
    // io_v2 requires the re-send to happen synchronously inside the callback.
    if (self->in.resp(self->held_req) == vp::IO_RESP_DENIED)
    {
        return;   // still busy; keep holding
    }
    self->resp_held = false;
    self->held_req = nullptr;
    if (!self->entries.empty())
    {
        self->fsm_event.enqueue(1);
    }
}


void IoV2BeatToSyncAdapter::fsm_handler(vp::Block *__this, vp::ClockEvent *)
{
    auto *self = static_cast<IoV2BeatToSyncAdapter *>(__this);

    // Blocked on upstream back-pressure: the held beat must be re-sent first
    // (from resp_retry_in_handler), so don't stream anything now.
    if (self->resp_held)
    {
        return;
    }

    if (self->entries.empty())
    {
        return;
    }

    StreamEntry e = self->entries.front();
    self->entries.pop_front();

    if (!self->emit_entry(e))
    {
        // Held on upstream back-pressure; resp_retry_in_handler resumes once
        // the master accepts.
        return;
    }

    // More queued beats? Tick again next cycle.
    if (!self->entries.empty())
    {
        self->fsm_event.enqueue(1);
    }
}


vp::DebugMemIf *IoV2BeatToSyncAdapter::resolve_debug_mem()
{
    std::vector<vp::SlavePort *> finals = this->out.get_final_ports();
    if (finals.empty() || finals[0]->get_owner() == nullptr)
    {
        return nullptr;
    }
    return finals[0]->get_owner()->debug_mem_if();
}


int IoV2BeatToSyncAdapter::debug_mem_access(uint64_t addr, uint8_t *data,
                                          uint64_t size, bool is_write)
{
    vp::DebugMemIf *target = this->resolve_debug_mem();
    return target ? target->debug_mem_access(addr, data, size, is_write) : -1;
}


void IoV2BeatToSyncAdapter::debug_mem_regions(std::vector<vp::DebugMemRegion> &regions,
    uint64_t local_base, uint64_t window_size, uint64_t entry_base, int depth)
{
    if (depth >= vp::DebugMemIf::MAX_DEPTH)
    {
        return;
    }
    vp::DebugMemIf *target = this->resolve_debug_mem();
    if (target != nullptr)
    {
        target->debug_mem_regions(regions, local_base, window_size, entry_base,
            depth + 1);
    }
}


void IoV2BeatToSyncAdapter::reset(bool active)
{
    if (active)
    {
        // Queued read beats are ours (allocator-backed) — return them to their
        // pool. Write entries reference the master's own request, not ours to
        // free.
        for (auto &e : this->entries)
        {
            if (e.beat != nullptr)
            {
                e.beat->free();
            }
        }
        this->entries.clear();
        // Same for a held (back-pressured) beat: free it if it is a read beat,
        // leave it alone if it is the master's round-tripped write ack.
        if (this->resp_held && this->held_req != nullptr
            && !this->held_req->get_is_write())
        {
            this->held_req->free();
        }
        this->resp_held = false;
        this->held_req = nullptr;
        this->fsm_event.cancel();   // safe even if not enqueued
    }
}


extern "C" vp::Component *gv_new(vp::ComponentConf &config)
{
    return new IoV2BeatToSyncAdapter(config);
}
