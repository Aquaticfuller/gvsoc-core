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
//   - Downstream (beat side): initiator-owned. The adapter forwards its own
//     reusable request object (dn_req) — never the master's — per access.
//     Nothing downstream frees it and it is never round-tripped as a read
//     beat, so it needs no pool. For a multi-beat read the response arrives
//     as N independent allocator-backed beat objects whose co-allocated
//     payload carries the data; this adapter copies each payload into the
//     master's buffer and frees the beat back to its pool (req->free()).
//
// Per the beat protocol, the downstream READ request is data-less (data ==
// NULL) — the payload comes back inside the response beats. A WRITE still
// carries the master's payload (writes round-trip; nobody else frees them).
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
    // The downstream request forwarded for `pending`, reused across
    // transactions (initiator-owned convention: nothing downstream frees it,
    // and it is never round-tripped back as a read beat — only a write ack,
    // recognised by identity).
    vp::IoReq dn_req;
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
}

void IoV2BeatCollapseAdapter::reset(bool active)
{
    if (active)
    {
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

    // Forward our own reusable downstream request — never the master's object.
    // Per the beat protocol a read burst request is data-less (the payload
    // comes back inside the response beats); a write carries the master's
    // payload (it round-trips back to us as the ack, nobody else frees it).
    vp::IoReq *dn = &self->dn_req;
    dn->prepare();
    dn->set_addr(req->get_addr());
    dn->set_size(req->get_size());
    dn->set_data(req->get_is_write() ? req->get_data() : nullptr);
    dn->set_opcode(req->get_opcode());
    dn->set_is_write(req->get_is_write());
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

    // GRANTED: the beat slave will respond (N read beats, or one write ack).
    // The response beats are distinct producer objects; dn_req itself only
    // comes back as a write ack.
    self->pending = req;
    self->pending_offset = 0;
    return vp::IO_REQ_GRANTED;
}


vp::IoRespAck IoV2BeatCollapseAdapter::out_resp(vp::Block *__this, vp::IoReq *req)
{
    auto *self = static_cast<IoV2BeatCollapseAdapter *>(__this);
    vp::IoReq *master = self->pending;

    // Two response shapes reach us, both owned by us as the consumer:
    //   - read beats: distinct allocator-backed objects the downstream producer
    //     allocated per beat (req != &dn_req) — copy each payload into the
    //     master's buffer at the running offset, then free it to its pool;
    //   - write acks: our own dn_req round-tripped as the ack (req == &dn_req) —
    //     nothing to copy or free.
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
