// SPDX-FileCopyrightText: 2026 ETH Zurich, University of Bologna and EssilorLuxottica SAS
//
// SPDX-License-Identifier: Apache-2.0
//
// Authors: Germain Haugou (germain.haugou@gmail.com)
//
// Single-req -> beat adapter on the io_v2 protocol.
//
// Per-combination replacement of IoV2BeatCollapseAdapter for an IoV2SingleReq
// master (the ISS LSU, functional routers) bound to an IoV2Beat slave. A
// single-req master carries its per-request state itself and correlates the
// response by OBJECT IDENTITY, so the adapter's only substantive job is
// translating the two allocation conventions:
//
//   - downstream (beat side): read data arrives in DISTINCT allocator-backed
//     beats the consumer must free (initiator-owned convention);
//   - upstream (single-req side): no alloc/free at all — the async response
//     hands the master back its OWN request object.
//
// The adapter copies each beat's payload into the master's buffer, frees the
// beat, and on the last beat gives the master back its request via resp().
//
// What this drops, relative to the collapse adapter:
//
//   - The single-outstanding limitation and its DENY / need_retry machinery.
//     Each accepted read gets its own pooled context (with an embedded
//     downstream request), so ANY number of accesses can be in flight
//     concurrently — a pipelined LSU (nb_outstanding > 1) is no longer
//     serialised at the boundary, matching the HW req/gnt/r_valid fabric.
//     Flow control is stateless: a downstream DENY propagates straight
//     upstream (the master holds its own request, per the single-req
//     contract) and retry() is forwarded as-is.
//   - Any downstream request for writes and atomics: the master's own object
//     is forwarded unchanged. Write beats legitimately round-trip as the ack
//     with data pointing into the master's buffer (io_v2.hpp), so the ack
//     path is a pure pass-through — and atomics keep data / second_data
//     without any copying.
//
// Timing: zero added latency, no clock events. A read completes (one resp
// upstream) on the cycle its last beat arrives; inline DONEs are relayed
// inline (writes, zero-size or error reads — a data-less read cannot be
// answered inline with data).
//
// Ownership: the master's request is never freed here (it round-trips). The
// per-context downstream read request is a plain embedded member — it never
// crosses a consumer-frees boundary (nothing downstream frees an initiator's
// request, and it is never round-tripped as a read beat), so it needs no
// allocator. Contexts are pooled; a transaction allocates nothing in steady
// state.

#include <algorithm>
#include <cstring>
#include <vector>

#include <vp/vp.hpp>
#include <vp/itf/io_v2.hpp>
#include <vp/debug_mem.hpp>

#include <utils/io_v2_single_req_to_beat_adapter/io_v2_single_req_to_beat_adapter_config.hpp>

class IoV2SingleReqToBeatAdapter : public vp::Component, public vp::DebugMemIf
{
public:
    IoV2SingleReqToBeatAdapter(vp::ComponentConf &config);
    void reset(bool active) override;

    // Backdoor debug path: the adapter is invisible, forward to the downstream.
    vp::DebugMemIf *debug_mem_if() override { return this; }
    int debug_mem_access(uint64_t addr, uint8_t *data, uint64_t size,
        bool is_write) override;
    void debug_mem_regions(std::vector<vp::DebugMemRegion> &regions,
        uint64_t local_base, uint64_t window_size, uint64_t entry_base,
        int depth) override;

    IoV2SingleReqToBeatAdapterConfig cfg;

private:
    // One in-flight read. The downstream request is embedded: we are its
    // initiator (nothing downstream frees it, it never comes back as a read
    // beat), so no allocator is involved. Response beats carry `this` context
    // through their initiator field, so any number of reads can be in flight
    // and responses may even arrive out of order across contexts.
    struct ReadCtx
    {
        vp::IoReq dn;
        vp::IoReq *up = nullptr;
        // Fill cursor into the master's buffer. A single-req access that fits
        // in one downstream beat (the common case, asserted when
        // expect_single) completes on the first beat; a narrower beat plane
        // streams several beats and the cursor reassembles them.
        uint64_t filled = 0;
        bool expect_single = false;
    };

    static vp::IoReqStatus in_req(vp::Block *__this, vp::IoReq *req);
    static vp::IoRespAck   out_resp(vp::Block *__this, vp::IoReq *req);
    static void            out_retry(vp::Block *__this, vp::IoRetryChannel channel);

    ReadCtx *alloc_ctx();
    void retire_ctx(ReadCtx *ctx);

    vp::Trace trace;

    vp::IoSlave  in{&IoV2SingleReqToBeatAdapter::in_req};
    vp::IoMaster out{&IoV2SingleReqToBeatAdapter::out_retry,
                     &IoV2SingleReqToBeatAdapter::out_resp};

    int beat_width;
    std::vector<ReadCtx *> live;       // in-flight reads, for reset cleanup
    std::vector<ReadCtx *> ctx_pool;   // freelist (process-lifetime)
};


IoV2SingleReqToBeatAdapter::IoV2SingleReqToBeatAdapter(vp::ComponentConf &config)
    : vp::Component(config, this->cfg)
{
    this->traces.new_trace("trace", &this->trace, vp::DEBUG);

    this->beat_width = (int)this->cfg.beat_width;

    this->new_slave_port("input", &this->in);
    this->new_master_port("output", &this->out);
}


IoV2SingleReqToBeatAdapter::ReadCtx *IoV2SingleReqToBeatAdapter::alloc_ctx()
{
    if (!this->ctx_pool.empty())
    {
        ReadCtx *ctx = this->ctx_pool.back();
        this->ctx_pool.pop_back();
        ctx->up = nullptr;
        ctx->filled = 0;
        ctx->expect_single = false;
        return ctx;
    }
    return new ReadCtx();
}


void IoV2SingleReqToBeatAdapter::retire_ctx(ReadCtx *ctx)
{
    auto it = std::find(this->live.begin(), this->live.end(), ctx);
    if (it != this->live.end())
    {
        this->live.erase(it);
    }
    this->ctx_pool.push_back(ctx);
}


vp::IoReqStatus IoV2SingleReqToBeatAdapter::in_req(vp::Block *__this, vp::IoReq *req)
{
    auto *self = static_cast<IoV2SingleReqToBeatAdapter *>(__this);

    self->trace.msg(vp::Trace::LEVEL_TRACE,
        "Submit (req=%p, addr=0x%lx, size=%lu, opcode=%d)\n",
        req, req->get_addr(), req->get_size(), (int)req->get_opcode());

    // WRITE / atomic (any non-READ opcode): forward the master's own request
    // unchanged. Per the beat protocol a write round-trips back to us as the
    // ack (out_resp forwards it upstream) and its data may keep pointing into
    // the master's buffer — so the three downstream outcomes map 1:1 to the
    // three upstream ones: DONE is relayed inline (timing annotations already
    // landed on the master's object), GRANTED resolves via the ack, and a
    // DENY leaves the master holding its own request until the retry we
    // forward.
    if (req->get_is_write())
    {
        return self->out.req(req);
    }

    // READ: the single-req master routes the response by identity, so its
    // request must not travel the beat plane (read beats are distinct
    // producer-owned objects, and a beat-protocol read request must be
    // data-less). Forward the context's embedded, data-less request instead.
    self->traces.assert(req->is_first && req->is_last,
        "single-req master must send single accesses (first=%d last=%d)",
        req->is_first ? 1 : 0, req->is_last ? 1 : 0);
    self->traces.assert(req->get_data() != nullptr || req->get_size() == 0,
        "single-req read carries no destination buffer (req=%p, size=%lu)",
        req, req->get_size());

    ReadCtx *ctx = self->alloc_ctx();
    ctx->up = req;
    ctx->expect_single = req->get_size() <= (uint64_t)self->beat_width;

    vp::IoReq *dn = &ctx->dn;
    dn->prepare();
    dn->set_addr(req->get_addr());
    dn->set_size(req->get_size());
    dn->set_data(nullptr);
    dn->set_opcode(req->get_opcode());
    dn->is_first = true;
    dn->is_last = true;
    dn->burst_id = req->burst_id;
    dn->initiator = ctx;

    vp::IoReqStatus st = self->out.req(dn);

    if (st == vp::IO_REQ_DENIED)
    {
        // Stateless back-pressure: the master holds its own request (single-req
        // contract) and re-sends it on the retry we forward from out_retry.
        self->ctx_pool.push_back(ctx);
        return vp::IO_REQ_DENIED;
    }

    if (st == vp::IO_REQ_DONE)
    {
        // Inline completion: only a zero-size read or an error can complete
        // inline — a data-less read has no buffer for the payload, so a beat
        // slave must stream its (successful) read responses.
        self->traces.assert(req->get_size() == 0
                || dn->get_resp_status() == vp::IO_RESP_INVALID,
            "beat slave answered a data-less read inline (req=%p, size=%lu)",
            req, req->get_size());
        req->set_resp_status(dn->get_resp_status());
        req->set_latency(dn->get_full_latency());
        self->ctx_pool.push_back(ctx);
        return vp::IO_REQ_DONE;
    }

    self->live.push_back(ctx);
    return vp::IO_REQ_GRANTED;
}


vp::IoRespAck IoV2SingleReqToBeatAdapter::out_resp(vp::Block *__this, vp::IoReq *req)
{
    auto *self = static_cast<IoV2SingleReqToBeatAdapter *>(__this);

    // Write / atomic ack: the master's own request round-tripped by the
    // downstream — hand it straight back. A single-req master consumes its
    // response synchronously, so the upstream resp() cannot be denied.
    if (req->get_is_write())
    {
        self->traces.assert(self->in.resp(req) == vp::IO_RESP_ACCEPTED,
            "single-req master must accept its response (req=%p)", req);
        return vp::IO_RESP_ACCEPTED;
    }

    // Read beat: a distinct allocator-backed object whose initiator carries
    // our context (copied from the embedded downstream request). Copy the
    // payload out, free the beat, and on the last beat give the master back
    // its own request — the allocation-convention translation this adapter
    // exists for.
    ReadCtx *ctx = (ReadCtx *)req->initiator;
    self->traces.assert(ctx != nullptr && ctx->up != nullptr,
        "read beat with no live access (beat=%p)", req);
    self->traces.assert(req != &ctx->dn,
        "downstream round-tripped our read request as a beat (req=%p)", req);
    self->traces.assert(!ctx->expect_single || (req->is_first && req->is_last),
        "multi-beat response to a beat-sized access (beat=%p, size=%lu)",
        req, req->get_size());

    vp::IoReq *up = ctx->up;

    if (req->get_size() > 0)
    {
        memcpy(up->get_data() + ctx->filled, req->get_data(), req->get_size());
        ctx->filled += req->get_size();
    }
    if (req->get_resp_status() == vp::IO_RESP_INVALID)
    {
        up->set_resp_status(vp::IO_RESP_INVALID);
    }
    bool last = req->is_last;
    req->free();

    if (last)
    {
        self->traces.assert(ctx->filled >= up->get_size(),
            "read stream ended short (req=%p, got=%lu, size=%lu)",
            up, ctx->filled, up->get_size());
        self->retire_ctx(ctx);
        self->trace.msg(vp::Trace::LEVEL_TRACE,
            "Complete read (req=%p, addr=0x%lx, size=%lu)\n",
            up, up->get_addr(), up->get_size());
        self->traces.assert(self->in.resp(up) == vp::IO_RESP_ACCEPTED,
            "single-req master must accept its response (req=%p)", up);
    }

    return vp::IO_RESP_ACCEPTED;
}


void IoV2SingleReqToBeatAdapter::out_retry(vp::Block *__this, vp::IoRetryChannel channel)
{
    auto *self = static_cast<IoV2SingleReqToBeatAdapter *>(__this);
    // Nothing is ever held here: the master owns any denied request, so the
    // retry is a pure pass-through (the master re-sends synchronously inside
    // it, per the io_v2 contract, and in_req rebuilds a fresh context).
    self->in.retry(channel);
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

int IoV2SingleReqToBeatAdapter::debug_mem_access(uint64_t addr, uint8_t *data,
    uint64_t size, bool is_write)
{
    vp::DebugMemIf *child = downstream_debug_mem(this->out);
    if (child == nullptr)
    {
        return -1;
    }
    return child->debug_mem_access(addr, data, size, is_write);
}

void IoV2SingleReqToBeatAdapter::debug_mem_regions(
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


void IoV2SingleReqToBeatAdapter::reset(bool active)
{
    if (active)
    {
        // The upstream requests are the master's own; the downstream requests
        // are embedded in the contexts. Nothing to free — just recycle.
        for (ReadCtx *ctx : this->live)
        {
            this->ctx_pool.push_back(ctx);
        }
        this->live.clear();
    }
}


extern "C" vp::Component *gv_new(vp::ComponentConf &config)
{
    return new IoV2SingleReqToBeatAdapter(config);
}
