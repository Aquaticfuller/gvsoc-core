// SPDX-FileCopyrightText: 2026 ETH Zurich, University of Bologna and EssilorLuxottica SAS
//
// SPDX-License-Identifier: Apache-2.0
//
// Authors: Germain Haugou (germain.haugou@gmail.com)
//
// Single-req -> single-req width adapter on the io_v2 protocol.
//
// Auto-inserted between an IoV2SingleReq master and an IoV2SingleReq slave
// whose declared width granule is tighter than what the master guarantees
// (master width 0 = unknown, or wider than the slave's). The slave-side width
// models a bank-interleaved fabric (e.g. the GAP9 shared-L2 LogIco with a
// 4-byte interleave): the fabric decodes the bank from the address and
// forwards the WHOLE request there, so an access crossing an aligned granule
// boundary would land in one bank and alias a different global address. This
// adapter turns that convention into a guarantee.
//
// Behaviour:
//
//   - An access that fits within one aligned granule ((addr % width) + size
//     <= width) passes straight through: the master's own object is
//     forwarded, all three downstream outcomes map 1:1 upstream, any number
//     of such accesses can be outstanding. Zero state, zero latency.
//   - An access that would straddle a boundary is chopped into granule-
//     aligned sub-accesses issued SEQUENTIALLY on the single downstream port,
//     each pointing straight into the master's buffer (no copy — single-req
//     reads carry their destination buffer, unlike beat reads). One embedded,
//     reused sub-request serves the whole split (identity round-trip per
//     chunk). On completion the master gets its OWN request back — inline
//     IO_REQ_DONE if every chunk completed inline (with the latency folded as
//     max(chunk_index + chunk_latency), i.e. back-to-back issue), or one
//     resp() once the last chunk lands.
//   - Atomics are never split (traces.assert): splitting an AMO/LR/SC is
//     meaningless. Atomics that fit pass through untouched.
//
// Flow control: a DENY of the FIRST chunk aborts the split statelessly — the
// master holds its own request and re-sends it on the retry we forward. A
// DENY of a later chunk is held here (the transaction is committed) and
// re-sent synchronously inside retry(). One split is active at a time; a
// second split-needing access is DENIED and retried when the current one
// completes. Fitting accesses are never blocked by an active split.

#include <algorithm>
#include <vector>

#include <vp/vp.hpp>
#include <vp/itf/io_v2.hpp>
#include <vp/debug_mem.hpp>

#include <utils/io_v2_single_req_width_adapter/io_v2_single_req_width_adapter_config.hpp>

class IoV2SingleReqWidthAdapter : public vp::Component, public vp::DebugMemIf
{
public:
    IoV2SingleReqWidthAdapter(vp::ComponentConf &config);
    void reset(bool active) override;

    // Backdoor debug path: the adapter is invisible, forward to the downstream.
    vp::DebugMemIf *debug_mem_if() override { return this; }
    int debug_mem_access(uint64_t addr, uint8_t *data, uint64_t size,
        bool is_write) override;
    void debug_mem_regions(std::vector<vp::DebugMemRegion> &regions,
        uint64_t local_base, uint64_t window_size, uint64_t entry_base,
        int depth) override;

    IoV2SingleReqWidthAdapterConfig cfg;

private:
    static vp::IoReqStatus in_req(vp::Block *__this, vp::IoReq *req);
    static vp::IoRespAck   out_resp(vp::Block *__this, vp::IoReq *req);
    static void            out_retry(vp::Block *__this, vp::IoRetryChannel channel);

    bool fits(uint64_t addr, uint64_t size) const
    {
        return (addr & ((uint64_t)this->width - 1)) + size <= (uint64_t)this->width;
    }

    bool issue_chunks();
    void consume_chunk_result();
    void complete_async();

    vp::Trace trace;

    vp::IoSlave  in{&IoV2SingleReqWidthAdapter::in_req};
    vp::IoMaster out{&IoV2SingleReqWidthAdapter::out_retry,
                     &IoV2SingleReqWidthAdapter::out_resp};

    int width;

    // Split state (one split transaction active at a time; fitting accesses
    // bypass it entirely). The sub-request is embedded and reused per chunk:
    // chunks are sequential and the downstream round-trips it by identity.
    bool active = false;
    bool chunk_held = false;     // current chunk DENIED downstream, re-send on retry
    bool need_retry = false;     // we denied a split-needing access while busy
    vp::IoReq sub;
    vp::IoReq *up = nullptr;
    uint64_t sent = 0;           // bytes covered by completed chunks
    uint64_t cur_chunk = 0;      // size of the chunk currently in flight
    int idx = 0;                 // completed-chunk count, for latency folding
    int64_t inline_lat = 0;      // max(idx + chunk latency) accumulator
};


IoV2SingleReqWidthAdapter::IoV2SingleReqWidthAdapter(vp::ComponentConf &config)
    : vp::Component(config, this->cfg)
{
    this->traces.new_trace("trace", &this->trace, vp::DEBUG);

    this->width = (int)this->cfg.width;
    if (this->width <= 0 || (this->width & (this->width - 1)) != 0)
    {
        this->trace.fatal(
            "IoV2SingleReqWidthAdapter requires a power-of-two width (got %d)\n",
            this->width);
    }

    this->new_slave_port("input", &this->in);
    this->new_master_port("output", &this->out);
}


// Latch the just-completed chunk's outcome onto the upstream request and
// advance the cursor.
void IoV2SingleReqWidthAdapter::consume_chunk_result()
{
    if (this->sub.get_resp_status() == vp::IO_RESP_INVALID)
    {
        this->up->set_resp_status(vp::IO_RESP_INVALID);
    }
    this->inline_lat = std::max(this->inline_lat,
                                (int64_t)this->idx + this->sub.get_full_latency());
    this->sent += this->cur_chunk;
    this->idx++;
}


// Issue chunks from the cursor until the split completes inline (returns
// true), a chunk goes async or is held on a DENY (returns false, split still
// active), or the FIRST chunk is denied (returns false, split aborted —
// `active` cleared so the caller propagates the DENY statelessly).
bool IoV2SingleReqWidthAdapter::issue_chunks()
{
    while (this->sent < this->up->get_size())
    {
        uint64_t addr = this->up->get_addr() + this->sent;
        uint64_t chunk = std::min<uint64_t>(
            (uint64_t)this->width - (addr & ((uint64_t)this->width - 1)),
            this->up->get_size() - this->sent);
        this->cur_chunk = chunk;

        vp::IoReq *r = &this->sub;
        r->prepare();
        r->set_addr(addr);
        r->set_size(chunk);
        r->set_data(this->up->get_data() + this->sent);
        r->set_opcode(this->up->get_opcode());
        r->is_first = true;
        r->is_last = true;
        r->burst_id = -1;
        r->initiator = this;

        this->trace.msg(vp::Trace::LEVEL_TRACE,
            "Issue chunk (up=%p, addr=0x%lx, size=%lu, idx=%d)\n",
            this->up, addr, chunk, this->idx);

        vp::IoReqStatus st = this->out.req(r);

        if (st == vp::IO_REQ_DENIED)
        {
            if (this->sent == 0)
            {
                // Nothing committed yet: abort and let the master hold its own
                // request (stateless back-pressure, retry is forwarded).
                this->active = false;
                this->up = nullptr;
                return false;
            }
            // Mid-split: hold this chunk (unchanged, per the io_v2 contract)
            // and re-send it synchronously inside retry().
            this->chunk_held = true;
            return false;
        }
        if (st == vp::IO_REQ_GRANTED)
        {
            return false;   // completion continues in out_resp
        }
        // Inline DONE: consume and keep going.
        this->consume_chunk_result();
    }
    return true;
}


// Async completion: hand the master back its own request and release a
// master held on the busy-DENY.
void IoV2SingleReqWidthAdapter::complete_async()
{
    vp::IoReq *u = this->up;
    this->active = false;
    this->up = nullptr;
    this->trace.msg(vp::Trace::LEVEL_TRACE,
        "Complete split (req=%p, addr=0x%lx, size=%lu)\n",
        u, u->get_addr(), u->get_size());
    this->traces.assert(this->in.resp(u) == vp::IO_RESP_ACCEPTED,
        "single-req master must accept its response (req=%p)", u);
    if (this->need_retry)
    {
        this->need_retry = false;
        // The master re-sends its held access synchronously inside this call.
        this->in.retry();
    }
}


vp::IoReqStatus IoV2SingleReqWidthAdapter::in_req(vp::Block *__this, vp::IoReq *req)
{
    auto *self = static_cast<IoV2SingleReqWidthAdapter *>(__this);

    // Fits one aligned granule: pure pass-through of the master's own object,
    // stateless, any number outstanding.
    if (self->fits(req->get_addr(), req->get_size()))
    {
        return self->out.req(req);
    }

    self->trace.msg(vp::Trace::LEVEL_TRACE,
        "Submit split (req=%p, addr=0x%lx, size=%lu, opcode=%d)\n",
        req, req->get_addr(), req->get_size(), (int)req->get_opcode());

    // A straddling access must be a plain read or write: splitting an atomic
    // is meaningless (and the HW fabric could not do it either).
    self->traces.assert(req->get_opcode() == vp::READ || req->get_opcode() == vp::WRITE,
        "cannot split an atomic access across the width granule "
        "(req=%p, addr=0x%lx, size=%lu, opcode=%d)",
        req, req->get_addr(), req->get_size(), (int)req->get_opcode());
    self->traces.assert(req->is_first && req->is_last,
        "single-req master must send single accesses (first=%d last=%d)",
        req->is_first ? 1 : 0, req->is_last ? 1 : 0);
    self->traces.assert(req->get_data() != nullptr,
        "single-req access carries no buffer (req=%p, size=%lu)",
        req, req->get_size());

    // One split at a time: a second one is back-pressured and re-sent on the
    // retry raised when the current split completes.
    if (self->active)
    {
        self->need_retry = true;
        return vp::IO_REQ_DENIED;
    }

    self->active = true;
    self->up = req;
    self->sent = 0;
    self->idx = 0;
    self->inline_lat = 0;
    self->chunk_held = false;

    if (self->issue_chunks())
    {
        // Every chunk completed inline: relay inline, folding the chunks'
        // latencies as a back-to-back sequence (one issue per cycle).
        req->set_latency(self->inline_lat);
        self->active = false;
        self->up = nullptr;
        return vp::IO_REQ_DONE;
    }

    if (!self->active)
    {
        // First chunk denied: aborted, the master re-sends on the forwarded
        // retry.
        return vp::IO_REQ_DENIED;
    }

    return vp::IO_REQ_GRANTED;
}


vp::IoRespAck IoV2SingleReqWidthAdapter::out_resp(vp::Block *__this, vp::IoReq *req)
{
    auto *self = static_cast<IoV2SingleReqWidthAdapter *>(__this);

    // Async completion of the current split's in-flight chunk.
    if (self->active && req == &self->sub)
    {
        self->consume_chunk_result();
        if (self->sent >= self->up->get_size() || self->issue_chunks())
        {
            self->complete_async();
        }
        return vp::IO_RESP_ACCEPTED;
    }

    // Pass-through response: the master's own object round-tripped by the
    // downstream — hand it straight back.
    self->traces.assert(self->in.resp(req) == vp::IO_RESP_ACCEPTED,
        "single-req master must accept its response (req=%p)", req);
    return vp::IO_RESP_ACCEPTED;
}


void IoV2SingleReqWidthAdapter::out_retry(vp::Block *__this, vp::IoRetryChannel channel)
{
    auto *self = static_cast<IoV2SingleReqWidthAdapter *>(__this);

    // A held mid-split chunk must be re-sent synchronously inside retry().
    if (self->active && self->chunk_held)
    {
        self->chunk_held = false;
        vp::IoReqStatus st = self->out.req(&self->sub);
        if (st == vp::IO_REQ_DENIED)
        {
            self->chunk_held = true;
        }
        else if (st == vp::IO_REQ_DONE)
        {
            self->consume_chunk_result();
            if (self->sent >= self->up->get_size() || self->issue_chunks())
            {
                self->complete_async();
            }
        }
        // GRANTED: completion continues in out_resp.
    }

    // Forward upstream: the master may hold its own denied access (a fitting
    // one, a first-chunk-denied split, or one we denied while busy).
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

int IoV2SingleReqWidthAdapter::debug_mem_access(uint64_t addr, uint8_t *data,
    uint64_t size, bool is_write)
{
    vp::DebugMemIf *child = downstream_debug_mem(this->out);
    if (child == nullptr)
    {
        return -1;
    }
    return child->debug_mem_access(addr, data, size, is_write);
}

void IoV2SingleReqWidthAdapter::debug_mem_regions(
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


void IoV2SingleReqWidthAdapter::reset(bool active_)
{
    if (active_)
    {
        // The upstream request is the master's own; the sub-request is
        // embedded. Nothing to free — just drop the state.
        this->active = false;
        this->chunk_held = false;
        this->need_retry = false;
        this->up = nullptr;
    }
}


extern "C" vp::Component *gv_new(vp::ComponentConf &config)
{
    return new IoV2SingleReqWidthAdapter(config);
}
