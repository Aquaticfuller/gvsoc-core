// SPDX-FileCopyrightText: 2026 ETH Zurich, University of Bologna and EssilorLuxottica SAS
//
// SPDX-License-Identifier: Apache-2.0
//
// Authors: Germain Haugou (germain.haugou@gmail.com)
//
// Beat -> big-packet collapse adapter on the io_v2 protocol.
//
// The inverse of IoV2BeatAdapter. It makes a beat-streaming slave (e.g. a
// KIND_BEAT router) look like a plain big-packet slave to a big-packet master.
// A master that does not speak beats (a functional/untimed router, a CPU LSU,
// ...) must never see the per-cycle beat stream of the timed layer below it;
// this adapter is the boundary that hides it.
//
// Ownership / lifetime
// --------------------
//
// This adapter is the boundary between two ownership regimes:
//
//   - Upstream (master side): the classic round-trip. The master owns its
//     request object; the adapter hands that very object back via resp(), and
//     the master frees/reuses it.
//   - Downstream (beat side): initiator-owned. For READS and ATOMICS the
//     adapter forwards its own reusable request object (dn_req) — never the
//     master's — per access. Nothing downstream frees it and it is never
//     round-tripped as a read beat, so it needs no pool. For a multi-beat
//     read the response arrives as N independent allocator-backed beat
//     objects whose co-allocated payload carries the data; this adapter
//     copies each payload into the master's buffer and frees the beat back
//     to its pool (req->free()).
//     For pure WRITES the adapter is a write-beat PRODUCER (io_v2.hpp,
//     "Write acknowledgement"): a beat target consumes and FREES a granted
//     write beat, so dn_req must never travel that path. Each write forwards
//     a size-0 allocator-backed beat whose data aliases the master's payload
//     (is_first = is_last = true — a big-packet-form write is a one-beat
//     burst). An inline DONE leaves the beat ours (relayed to the master and
//     freed here); a GRANTED transfers the beat to the target, which acks the
//     burst exactly once via resp() with a DISTINCT data-less ack object that
//     this adapter frees.
//
// Per the beat protocol, the downstream READ request is data-less (data ==
// NULL) — the payload comes back inside the response beats. An ATOMIC still
// carries the master's payload and keeps the classic round-trip (dn_req
// comes back via resp(); nobody frees it).
//
// Timing: zero added latency. A multi-beat read completes (one resp upstream)
// on the cycle its last beat arrives, which is exactly the burst's latency. An
// inline DONE from downstream is relayed inline (writes only — a data-less
// read cannot be answered inline, it has no buffer for the payload).
//
// Single outstanding request. A second master access while one is in flight is
// DENIED and retried when the first completes. Masters behind this boundary
// (in-order cores, single-refill icaches) are themselves single-outstanding,
// so this is not a throughput limit in practice.

#include <cstring>

#include <vp/vp.hpp>
#include <vp/itf/io_v2.hpp>
#include <vp/debug_mem.hpp>

class IoV2BeatCollapseAdapter : public vp::Component, public vp::DebugMemIf
{
public:
    IoV2BeatCollapseAdapter(vp::ComponentConf &config);
    void reset(bool active) override;

    // Backdoor debug path: the adapter is invisible, forward to the downstream.
    vp::DebugMemIf *debug_mem_if() override { return this; }
    int debug_mem_access(uint64_t addr, uint8_t *data, uint64_t size,
        bool is_write) override;
    void debug_mem_regions(std::vector<vp::DebugMemRegion> &regions,
        uint64_t local_base, uint64_t window_size, uint64_t entry_base,
        int depth) override;

private:
    static vp::IoReqStatus in_req(vp::Block *__this, vp::IoReq *req);
    static vp::IoRespAck   out_resp(vp::Block *__this, vp::IoReq *req);
    static void            out_retry(vp::Block *__this, vp::IoRetryChannel channel);

    void maybe_retry_input();

    vp::Trace trace;

    vp::IoSlave  in{&IoV2BeatCollapseAdapter::in_req};
    vp::IoMaster out{&IoV2BeatCollapseAdapter::out_retry,
                     &IoV2BeatCollapseAdapter::out_resp};

    // The master request currently in flight (single outstanding), handed back
    // on completion. Null when idle.
    vp::IoReq *pending = nullptr;
    // The downstream request forwarded for `pending` on READS and ATOMICS,
    // reused across transactions (initiator-owned convention: nothing
    // downstream frees it, and it is never round-tripped back as a read
    // beat — only an atomic ack, recognised by identity). Pure writes never
    // use it: their beats come from the size-0 pool below.
    vp::IoReq dn_req;
    // Size-0 pool serving the downstream write beats (data aliases the
    // master's payload) — the beat target frees a granted write beat, so it
    // must be allocator-backed.
    vp::IoReqAllocator *zero_allocator;
    // Read fill cursor: cumulative bytes copied from response-beat payloads
    // into the master's buffer.
    uint64_t pending_offset = 0;
    // We refused a master access (busy or downstream-denied) and owe it a
    // retry() once we can accept again.
    bool need_retry = false;
};


IoV2BeatCollapseAdapter::IoV2BeatCollapseAdapter(vp::ComponentConf &config)
    : vp::Component(config)
{
    this->traces.new_trace("trace", &this->trace, vp::DEBUG);
    this->new_slave_port("input", &this->in);
    this->new_master_port("output", &this->out);

    this->zero_allocator = vp::IoReqAllocator::get(0);
}

void IoV2BeatCollapseAdapter::reset(bool active)
{
    if (active)
    {
        // Nothing of ours to free: a write beat GRANTED downstream is owned
        // (and freed) by the target, and a DENIED one was already freed
        // synchronously in in_req — no beat is ever held across cycles here.
        this->pending = nullptr;
        this->pending_offset = 0;
        this->need_retry = false;
    }
}

void IoV2BeatCollapseAdapter::maybe_retry_input()
{
    if (this->need_retry && this->pending == nullptr)
    {
        this->need_retry = false;
        this->in.retry();
    }
}


vp::IoReqStatus IoV2BeatCollapseAdapter::in_req(vp::Block *__this, vp::IoReq *req)
{
    auto *self = static_cast<IoV2BeatCollapseAdapter *>(__this);

    // Single outstanding: refuse while busy; retried on completion.
    if (self->pending != nullptr)
    {
        self->need_retry = true;
        return vp::IO_REQ_DENIED;
    }

    self->trace.msg(vp::Trace::LEVEL_TRACE,
        "Submit (req=%p, addr=0x%lx, size=%lu, write=%d)\n",
        req, req->get_addr(), req->get_size(), req->get_is_write() ? 1 : 0);

    // Pure WRITE: the beat target consumes and FREES a granted write beat
    // (io_v2.hpp "Write acknowledgement"), so neither the master's object nor
    // the embedded dn_req may travel this path. Forward a size-0 pool beat
    // whose data aliases the master's payload (no copy — the buffer is valid
    // as long as the beat is unfreed). is_first = is_last = true: a
    // big-packet-form write is a one-beat burst, acknowledged once.
    if (req->get_opcode() == vp::WRITE)
    {
        vp::IoReq *beat = self->zero_allocator->alloc();
        beat->prepare();
        beat->set_addr(req->get_addr());
        beat->set_size(req->get_size());
        // Size-0 pool: data is caller-managed, set on EVERY allocation.
        beat->set_data(req->get_data());
        beat->set_opcode(vp::WRITE);
        beat->is_first = true;
        beat->is_last  = true;
        beat->burst_id = req->burst_id;
        beat->initiator = self;

        vp::IoReqStatus st = self->out.req(beat);

        if (st == vp::IO_REQ_DONE)
        {
            // Inline burst ack: ownership never transferred, the beat is
            // still ours. Relay status and full timing (head latency +
            // bandwidth duration) onto the master's request — it was
            // prepare()'d (duration==0), so both get_latency() and
            // get_full_latency() return the correct total — and recycle the
            // beat.
            req->set_resp_status(beat->get_resp_status());
            req->set_latency(beat->get_full_latency());
            beat->free();
            return vp::IO_REQ_DONE;
        }
        if (st == vp::IO_REQ_DENIED)
        {
            // Downstream busy: the master holds its own request and re-sends
            // it on our retry(), where a fresh beat is allocated — this one
            // is dead (ownership never transferred on DENIED).
            beat->free();
            self->need_retry = true;
            return vp::IO_REQ_DENIED;
        }

        // GRANTED: the target now owns the beat (it consumes and frees it);
        // the burst ack arrives in out_resp as a DISTINCT data-less
        // allocator-backed object that we free.
        self->pending = req;
        self->pending_offset = 0;
        return vp::IO_REQ_GRANTED;
    }

    // READ / ATOMIC: forward our own reusable downstream request — never the
    // master's object. Per the beat protocol a read burst request is
    // data-less (the payload comes back inside the response beats); an
    // atomic carries the master's payload and keeps the classic round-trip
    // (dn_req comes back to us via resp(), nobody else frees it).
    vp::IoReq *dn = &self->dn_req;
    dn->prepare();
    dn->set_addr(req->get_addr());
    dn->set_size(req->get_size());
    dn->set_data(req->get_is_write() ? req->get_data() : nullptr);
    dn->set_opcode(req->get_opcode());
    dn->is_first = true;
    dn->is_last  = true;
    dn->burst_id = req->burst_id;
    dn->initiator = nullptr;

    vp::IoReqStatus st = self->out.req(dn);

    if (st == vp::IO_REQ_DONE)
    {
        // Inline completion. Only a write, a zero-size read or an error can
        // complete inline: a data-less read has no buffer for the payload, so
        // a beat slave must stream its (successful) read responses.
        self->traces.assert(req->get_is_write() || req->get_size() == 0
                || dn->get_resp_status() == vp::IO_RESP_INVALID,
            "beat slave answered a data-less read inline (req=%p, size=%lu)",
            req, req->get_size());
        // Fold the downstream's full timing (head latency + bandwidth duration)
        // into the master's latency field. The master's request was prepare()'d
        // (duration==0), so both get_latency() and get_full_latency() return the
        // correct total — safe whichever the identity master reads.
        req->set_resp_status(dn->get_resp_status());
        req->set_latency(dn->get_full_latency());
        return vp::IO_REQ_DONE;
    }
    if (st == vp::IO_REQ_DENIED)
    {
        // Downstream busy: the master holds its own request and re-sends it on
        // our retry(), where we rebuild dn_req.
        self->need_retry = true;
        return vp::IO_REQ_DENIED;
    }

    // GRANTED: the beat slave will respond (N read beats, or one atomic ack).
    // The response beats are distinct producer objects; dn_req itself only
    // comes back as an atomic ack.
    self->pending = req;
    self->pending_offset = 0;
    return vp::IO_REQ_GRANTED;
}


vp::IoRespAck IoV2BeatCollapseAdapter::out_resp(vp::Block *__this, vp::IoReq *req)
{
    auto *self = static_cast<IoV2BeatCollapseAdapter *>(__this);
    vp::IoReq *master = self->pending;

    // Write burst ack (io_v2.hpp "Write acknowledgement"): a DISTINCT
    // data-less allocator-backed object — the size-0 beat we forwarded was
    // consumed and freed by the target. Latch the burst's final status onto
    // the pending master request, free the ack (the initiator frees it), and
    // hand the master its own request back.
    if (req->get_opcode() == vp::WRITE && req != &self->dn_req)
    {
        self->traces.assert(master != nullptr
                && master->get_opcode() == vp::WRITE,
            "write burst ack with no pending write (ack=%p)", req);
        self->traces.assert(req->is_last && req->get_data() == nullptr,
            "malformed write burst ack (ack=%p, last=%d)",
            req, req->is_last ? 1 : 0);

        if (req->get_resp_status() == vp::IO_RESP_INVALID)
        {
            master->set_resp_status(vp::IO_RESP_INVALID);
        }
        req->free();

        self->pending = nullptr;
        master->is_first = true;
        master->is_last  = true;
        self->in.resp(master);
        self->maybe_retry_input();
        return vp::IO_RESP_ACCEPTED;
    }

    // Two remaining response shapes, both owned by us as the consumer:
    //   - read beats: distinct allocator-backed objects the downstream producer
    //     allocated per beat (req != &dn_req) — copy each payload into the
    //     master's buffer at the running offset, then free it to its pool;
    //   - atomic acks: our own dn_req round-tripped as the ack (req == &dn_req)
    //     — nothing to copy or free.
    // Either way the master's own request was never forwarded, so we latch any
    // error onto it and, on the last beat, hand it back as one big-packet
    // response.
    if (req->get_resp_status() == vp::IO_RESP_INVALID)
    {
        master->set_resp_status(vp::IO_RESP_INVALID);
    }
    bool last = req->is_last;
    if (req != &self->dn_req)
    {
        // Distinct read beat: its payload carries the data.
        if (req->get_size() > 0)
        {
            memcpy(master->get_data() + self->pending_offset, req->get_data(),
                   req->get_size());
            self->pending_offset += req->get_size();
        }
        req->free();
    }

    if (last)
    {
        self->pending = nullptr;
        master->is_first = true;
        master->is_last  = true;
        self->in.resp(master);
        self->maybe_retry_input();
    }

    return vp::IO_RESP_ACCEPTED;
}


void IoV2BeatCollapseAdapter::out_retry(vp::Block *__this, vp::IoRetryChannel)
{
    auto *self = static_cast<IoV2BeatCollapseAdapter *>(__this);
    // The downstream that denied our forward is ready again. If we owe the
    // master a retry and can accept now, let it re-send (synchronously).
    self->maybe_retry_input();
}


// ---- backdoor debug: transparent pass-through to the downstream ------------

static vp::DebugMemIf *downstream_debug_mem(vp::IoMaster &itf)
{
    std::vector<vp::SlavePort *> finals = itf.get_final_ports();
    if (finals.empty() || finals[0]->get_owner() == nullptr)
    {
        return nullptr;
    }
    return finals[0]->get_owner()->debug_mem_if();
}

int IoV2BeatCollapseAdapter::debug_mem_access(uint64_t addr, uint8_t *data,
    uint64_t size, bool is_write)
{
    vp::DebugMemIf *child = downstream_debug_mem(this->out);
    if (child == nullptr)
    {
        return -1;
    }
    return child->debug_mem_access(addr, data, size, is_write);
}

void IoV2BeatCollapseAdapter::debug_mem_regions(
    std::vector<vp::DebugMemRegion> &regions, uint64_t local_base,
    uint64_t window_size, uint64_t entry_base, int depth)
{
    if (depth >= vp::DebugMemIf::MAX_DEPTH)
    {
        return;
    }
    vp::DebugMemIf *child = downstream_debug_mem(this->out);
    if (child != nullptr)
    {
        child->debug_mem_regions(regions, local_base, window_size, entry_base,
            depth + 1);
    }
}


extern "C" vp::Component *gv_new(vp::ComponentConf &config)
{
    return new IoV2BeatCollapseAdapter(config);
}
