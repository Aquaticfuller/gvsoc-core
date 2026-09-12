/*
 * Copyright (C) 2020 SAS, ETH Zurich and University of Bologna
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * io_v2 variant of the Spatz vector LSU. Derived from the v1 model in
 * spatz_vlsu.cpp so the timing model stays identical; only the IO plumbing
 * follows io_v2 semantics:
 *
 *   - master ports are vp::IoMaster from <vp/itf/io_v2.hpp>, retry/resp
 *     callbacks passed at construction (muxed variant so one pair of callbacks
 *     dispatches by port id)
 *   - statuses are IO_REQ_DONE / IO_REQ_GRANTED / IO_REQ_DENIED; errors travel
 *     on the response status (IO_RESP_OK / IO_RESP_INVALID)
 *   - io_v2 has no argument stack, so a request carries the back-link to its
 *     VlsuReq (store) or VlsuRobEntry (load) in req->initiator
 *   - io_v2 has no grant callback. A burst denied at issue is parked on its
 *     port and re-issued synchronously inside retry(), as deny/retry arbiters
 *     such as interco.log_ico_v2 require; the issue bookkeeping the v1 grant
 *     callback performed happens at submission instead (see burst_issued).
 */

#include <map>
#include <set>
#include <utility>

// Retirement-stall diagnostic state (TERANOC_VLSU_STALL_PATH). File-scope and
// keyed by instance: adding members would change the class layout, and model
// .so's embed these types by value while the build does not track headers as a
// dependency -- stale .so's then segfault during construction.
namespace { std::map<const void *, long> vlsu_stall_streak;
            std::map<std::pair<const void *,int>, long> vlsu_retry_rx, vlsu_retry_redeny;
            std::map<const void *, long> vlsu_last_retire;
            std::set<const void *> vlsu_stall_reported; }
#include <cstdio>
#include <cstdlib>
#include <algorithm>
#include <cpu/iss_v2/include/cores/vector_unit/vector_unit.hpp>

namespace {
// Current RTL uses distributed lane ROBs. Explicit zero preserves the older
// TwinROB0 law for historical calibration campaigns.
bool distributed_burst_rob()
{
    static const bool enabled = []() {
        const char *value = getenv("TERANOC_VLSU_DISTRIBUTED_ROB");
        return value == nullptr || atoi(value) != 0;
    }();
    return enabled;
}

int lane_rob_depth()
{
    static const int depth = []() {
        const char *value = getenv("TERANOC_VLSU_LANE_ROB_DEPTH");
        return value == nullptr ? 32 : atoi(value);
    }();
    return depth;
}

int burst_allocation_cycles(uint64_t bytes, int full_bytes, int ports, int legacy_cycles)
{
    if (!distributed_burst_rob() || bytes >= (uint64_t)full_bytes) return legacy_cycles;
    // Single-word remainder uses one request slot; shorter multiword bursts
    // reserve one row per cycle between the decide and send stages.
    if (bytes <= 4) return 1;
    return 2 + (bytes / 4 + ports - 1) / ports;
}
}

VuLsu::VuLsu(Vu &vu, Iss &iss)
: VuBlock(&vu, "vlsu"), vu(vu),
nb_pending_insn(*this, "nb_pending_insn", 8, true),
fsm_event(this, &VuLsu::fsm_handler),
event_label(*this, "label", 0, gv::Vcd_event_type_string)
{
    this->traces.new_trace("trace", &this->trace, vp::DEBUG);
    this->traces.new_trace_event("active", &this->event_active, 1);
    this->traces.new_trace_event("pc", &this->event_pc, 64);
    this->traces.new_trace_event("queue", &this->event_queue, 64);

    this->insns.resize(VuLsu::queue_size);

    int nb_ports = iss.get_js_config()->get_child_int("vu/nb_ports");
    this->nb_ports = nb_ports;

    this->event_addr.resize(nb_ports);
    this->event_size.resize(nb_ports);
    this->event_is_write.resize(nb_ports);

    for (int i=0; i<nb_ports; i++)
    {
        this->traces.new_trace_event("port_" + std::to_string(i) + "/addr",
            &this->event_addr[i], 64);
        this->traces.new_trace_event("port_" + std::to_string(i) + "/size",
            &this->event_size[i], 64);
        this->traces.new_trace_event("port_" + std::to_string(i) + "/is_write",
            &this->event_is_write[i], 1);
    }

    // io_v2 IoMaster has no default constructor: every port is built with its
    // retry/resp callbacks, muxed so one pair demuxes by port id.
    this->ports.reserve(nb_ports);
    for (int i=0; i<nb_ports; i++)
    {
        this->ports.emplace_back(i, &VuLsu::port_retry_muxed, &VuLsu::port_resp_muxed);
        iss.new_master_port("vlsu_" + std::to_string(i), &this->ports[i], this);
    }

    // PER-PORT ROB depth. The RTL sizes port 0 and ports 1-3 SEPARATELY
    // (spatz_vlsu.sv: "NrOutstandingLoads sizes two different things at once:
    // ROB0's burst window ... and ports 1-3's non-burst window (which need not
    // match)"). The campaign image is ROB0=128 / ROBN=16. This model sized every
    // port from one knob, so ports 1-3 -- which carry the non-burst path
    // (strided, indexed, unaligned, and the sub-burst TAIL) -- were 8 deep
    // against the reference's 16, and port 0 was 8 against 128.
    int nb_outstanding_reqs = iss.get_js_config()->get_child_int("vu/nb_outstanding_reqs");
    int nb_outstanding_reqs_n = nb_outstanding_reqs;
    if (iss.get_js_config()->get("vu/nb_outstanding_reqs_n") != NULL)
    {
        nb_outstanding_reqs_n = iss.get_js_config()->get_child_int("vu/nb_outstanding_reqs_n");
        if (nb_outstanding_reqs_n <= 0) nb_outstanding_reqs_n = nb_outstanding_reqs;
    }

    this->rob.resize(nb_ports);
    this->rob_next.resize(nb_ports);
    this->rob_first.resize(nb_ports);
    this->rob_count.resize(nb_ports);
    this->port_burst.resize(nb_ports);
    this->port_stalled.resize(nb_ports);
    this->denied_reqs.resize(nb_ports);
    for (int i=0; i<nb_ports; i++)
    {
        this->rob[i].resize(i == 0 ? nb_outstanding_reqs : nb_outstanding_reqs_n);
    }

    // Spatz port-0 burst loads. All default-off/zero when the properties are
    // absent (targets without burst support keep the legacy behavior exactly).
    js::Config *cfg = iss.get_js_config();
    this->burst_enable = cfg->get_int("vu/burst_enable");
    if (this->burst_enable && distributed_burst_rob())
    {
        if (lane_rob_depth() < 4 || (lane_rob_depth() & (lane_rob_depth() - 1)))
            this->trace.fatal("VLSU lane ROB depth must be a power of two >= 4\n");
        for (int i = 0; i < nb_ports; ++i) this->rob[i].resize(lane_rob_depth());
    }
    this->burst_sub_word = cfg->get_int("vu/burst_sub_word");
    this->burst_max_words = cfg->get_int("vu/burst_max_words");
    if (this->burst_max_words <= 0) this->burst_max_words = 16;
    this->burst_bytes = this->burst_max_words * 4;
    this->burst_rob_words = cfg->get_int("vu/burst_rob_depth");
    if (this->burst_rob_words < this->burst_max_words)
        this->burst_rob_words = 2 * this->burst_max_words;
    if (this->burst_enable && distributed_burst_rob())
        this->burst_rob_words = lane_rob_depth() * nb_ports;
    this->burst_block_alloc = cfg->get_int("vu/burst_block_alloc");
    this->burst_dual_load = cfg->get_int("vu/burst_dual_load");
    if (this->burst_dual_load <= 0) this->burst_dual_load = 1;
    this->burst_recv_ports = cfg->get_int("vu/burst_recv_ports");
    if (this->burst_recv_ports <= 0) this->burst_recv_ports = 1;
    if (this->burst_enable && distributed_burst_rob()) this->burst_recv_ports = nb_ports;
    this->burst_issue_latency = cfg->get_int("vu/burst_issue_latency");
    if (this->burst_issue_latency <= 0)
        this->burst_issue_latency = this->burst_block_alloc ? 3 : 18;

    // One entry per in-flight burst plus one spare; the word-granular
    // occupancy check is the real capacity limit.
    this->brob.resize(this->burst_rob_words / this->burst_max_words +
        (distributed_burst_rob() ? 2 : 1));

    if (this->burst_enable)
    {
        // Write beats on the IoV2Beat port are allocator-owned downstream.
        // Size-0 pool: req->data is caller-managed (re)set on every send; the
        // write-ack recycling in the beat adapters repoints it.
        this->beat_allocator = vp::IoReqAllocator::get(0);
        // Current word loads stripe across all lanes; legacy reserves port 0.
        this->load_port_base = distributed_burst_rob() ? 0 : 1;
    }
}

void VuLsu::start()
{
}

void VuLsu::reset(bool active)
{
    if (active)
    {
        uint8_t zero = 0;
        this->event_active.event(&zero);
        for (int i=0; i<nb_ports; i++)
        {
            this->event_addr[i].event(&zero);
            this->event_size[i].event(&zero);
            this->event_is_write[i].event(&zero);
        }
        this->insn_first = 0;
        this->insn_first_waiting = 0;
        this->insn_last = 0;
        this->nb_waiting_insn = 0;
        this->pending_size = 0;
        this->remaining_size = 0;
        this->insn_ongoing = 0;

        for (VuLsuPendingInsn &slot : this->insns)
        {
            slot.done = false;
            slot.nb_remaining_bursts = 0;
            slot.burst_safe = false;
            slot.is_load = false;
        }

        this->burst_mode = false;
        this->burst_full_bytes = 0;
        this->tail_phase = false;
        this->tail_base = 0;
        this->port0_next_issue = -1;
        this->brob_next = 0;
        this->brob_first = 0;
        this->brob_count = 0;
        this->brob_words_used = 0;
        for (BurstRobEntry &entry : this->brob)
        {
            entry = BurstRobEntry();
        }

        int nb_ports = this->vu.iss.get_js_config()->get_child_int("vu/nb_ports");
        int nb_outstanding_reqs = this->vu.iss.get_js_config()->get_child_int("vu/nb_outstanding_reqs");

        for (int i=0; i<nb_ports; i++)
        {
            this->rob_next[i] = 0;
            this->rob_first[i] = 0;
            this->rob_count[i] = 0;
            this->port_burst[i] = 0;
            this->port_stalled[i] = false;
            this->denied_reqs[i] = nullptr;

            // Per-port depth: rob[0] and rob[1..] differ now, so iterate the
            // ACTUAL size. Using the port-0 depth here wrote 128 entries into a
            // 16-entry vector on ports 1-3 -- an out-of-bounds write on every
            // reset, which is what segfaulted the first ROB0=128/ROBN=16 run.
            for (size_t j=0; j<this->rob[i].size(); j++)
            {
                this->rob[i][j] = VlsuRobEntry();
            }
        }

        // Put back all the requests, including the ones which were in flight
        this->reqs_free.clear();
        for (VlsuReq &req : this->reqs)
        {
            this->reqs_free.push_back(&req);
        }
        this->nb_pending_stores = 0;

        while (!this->delayed_bursts.empty())
        {
            this->delayed_bursts.pop();
        }

        this->op_timestamp = -1;
    }
}

void VuLsu::enqueue_insn(PendingInsn *pending_insn)
{
    iss_insn_t *insn = this->vu.iss.exec.get_insn(pending_insn->entry);
    this->trace.msg(vp::Trace::LEVEL_TRACE, "Enqueue instruction (pc: 0x%lx, id: %d)\n",
        insn->addr, pending_insn->id);
    uint8_t one = 1;
    this->event_active.event(&one);
    this->event_queue.event((uint8_t *)&insn->addr);

    // Push the instruction in the queue for the FSM. The +1 keeps it from being executed immediately in the same cycle.
    pending_insn->timestamp = this->vu.iss.clock.get_cycles() + 1;
    VuLsuPendingInsn &slot = this->insns[this->insn_last];
    this->insn_last = (this->insn_last + 1) % VuLsu::queue_size;
    slot.insn = pending_insn;
    slot.nb_pending_bursts = 0;
    slot.done = false;
    slot.nb_remaining_bursts = 0;
    this->nb_pending_insn.inc(1);
    this->nb_waiting_insn++;

    this->fsm_event.enable();
}

void VuLsu::isa_init()
{
    // Attach handlers to instructions so that we can quickly handle load and stores differently
    for (iss_decoder_item_t *insn: *this->vu.iss.decode.get_insns_from_tag("vload"))
    {
        insn->u.insn.block_handler = (void *)&VuLsu::handle_insn_load;
    }
    for (iss_decoder_item_t *insn: *this->vu.iss.decode.get_insns_from_tag("vstore"))
    {
        insn->u.insn.block_handler = (void *)&VuLsu::handle_insn_store;
    }
    for (iss_decoder_item_t *insn: *this->vu.iss.decode.get_insns_from_tag("vload_strided"))
    {
        insn->u.insn.block_handler = (void *)&VuLsu::handle_insn_load_strided;
    }
    for (iss_decoder_item_t *insn: *this->vu.iss.decode.get_insns_from_tag("vstore_strided"))
    {
        insn->u.insn.block_handler = (void *)&VuLsu::handle_insn_store_strided;
    }
    for (iss_decoder_item_t *insn: *this->vu.iss.decode.get_insns_from_tag("vload_indexed"))
    {
        insn->u.insn.block_handler = (void *)&VuLsu::handle_insn_load_indexed;
    }
    for (iss_decoder_item_t *insn: *this->vu.iss.decode.get_insns_from_tag("vstore_indexed"))
    {
        insn->u.insn.block_handler = (void *)&VuLsu::handle_insn_store_indexed;
    }
}

void VuLsu::handle_insn_load_strided(VuLsu *_this, iss_insn_t *insn)
{
    _this->handle_access(insn, false, insn->out_regs[0], true, _this->insns[_this->insn_first_waiting].insn->reg_2);
}

void VuLsu::handle_insn_store_strided(VuLsu *_this, iss_insn_t *insn)
{
    _this->handle_access(insn, true, insn->in_regs[1], true, _this->insns[_this->insn_first_waiting].insn->reg_3);
}

void VuLsu::handle_insn_load_indexed(VuLsu *_this, iss_insn_t *insn)
{
    _this->handle_access(insn, false, insn->out_regs[0], false, 0, insn->in_regs[1]);
}

void VuLsu::handle_insn_store_indexed(VuLsu *_this, iss_insn_t *insn)
{
    _this->handle_access(insn, true, insn->in_regs[1], false, 0, insn->in_regs[2]);
}

void VuLsu::handle_access(iss_insn_t *insn, bool is_write, int reg, bool do_stride, iss_reg_t stride, int reg_indexed)
{
    // A load or store instruction is starting, just store information about the first burst and let
    // the FSM handle all the bursts.
    unsigned int sewb = this->vu.iss.vector.sewb;
    unsigned int lmul = this->vu.iss.vector.lmul;
    this->pending_vreg = reg;
    this->pending_velem = velem_get(&this->vu.iss, reg, 0, sewb, lmul);
    this->vstart = this->vu.iss.csr.vstart.value;
    this->pending_addr = this->insns[this->insn_first_waiting].insn->reg;
    this->pending_is_write = is_write;
    int inst_elem_size = insn->uim[1] >= 5 ? 1 << (insn->uim[1] - 4) : 1 << 0;
    int elem_size = reg_indexed != -1 ? sewb : inst_elem_size;
    this->pending_size = (this->vu.iss.csr.vl.value - this->vu.iss.csr.vstart.value) * elem_size;
    this->stride = stride;
    this->strided = do_stride;
    this->elem_size = elem_size;
    this->inst_elem_size = inst_elem_size;
    this->reg_indexed = reg_indexed;
    this->burst_size = do_stride || reg_indexed != -1 ? elem_size : this->vu.lane_width;
    this->remaining_size = this->pending_size;
    for (int p = 0; p < this->nb_ports; p++)
    {
        this->port_burst[p] = 0;
    }

    // Current RTL: aligned unit-stride loads of 8..512 bytes use distributed
    // lane reservations, including short final bursts. Historical mode keeps
    // the old full-burst region followed by a drained word-path tail.
    // Byte-partial tails currently fall back to ordinary accesses.
    this->burst_mode = this->burst_enable && !is_write && !do_stride && reg_indexed == -1
        && (distributed_burst_rob() ? (elem_size <= 4) :
            this->burst_sub_word ? (elem_size == 4 || elem_size == 2)
                                 : (elem_size == 4))
        && this->pending_size >= (iss_addr_t)(distributed_burst_rob() ? 8 : this->burst_bytes)
        && this->pending_size <= (iss_addr_t)(this->burst_rob_words * 4)
        && (!distributed_burst_rob() || (this->vstart == 0 && this->pending_size % 4 == 0))
        && ((this->pending_addr & (iss_addr_t)(this->burst_bytes - 1)) == 0);
    if (!is_write)
    {
        if (this->burst_mode) this->vp_load_burst++; else this->vp_load_nonburst++;
    }
    this->burst_full_bytes = this->burst_mode ?
        (distributed_burst_rob() ? this->pending_size :
         (this->pending_size / this->burst_bytes) * this->burst_bytes) : 0;
    this->tail_phase = false;
    this->tail_base = 0;
    if (this->burst_mode)
    {
        this->burst_size = this->burst_bytes;
        // First burst send: decide -> reserve -> send cadence from the start
        // of the issue phase (the timestamp gate below adds the rest).
        this->port0_next_issue = this->vu.iss.clock.get_cycles() +
            (burst_allocation_cycles(this->pending_size, this->burst_bytes,
                this->nb_ports, this->burst_issue_latency) - 1);
    }

    VuLsuPendingInsn &slot = this->insns[this->insn_first_waiting];
    // Store bursts commit one by one on their response: they are not tracked in the
    // ROB and so not committed by group.
    if (this->burst_mode)
    {
        // Commit units: one per full burst plus one per tail word.
        slot.nb_remaining_bursts = distributed_burst_rob()
            ? (this->pending_size + this->burst_bytes - 1) / this->burst_bytes
            : this->burst_full_bytes / this->burst_bytes +
            (this->pending_size - this->burst_full_bytes) / this->vu.lane_width;
    }
    else
    {
        slot.nb_remaining_bursts = is_write ?
            0 : (this->pending_size + this->burst_size - 1) / this->burst_size;
    }
    // Runahead: current short tails remain burst-safe; legacy word tails do not.
    slot.burst_safe = this->burst_mode && (this->pending_size == this->burst_full_bytes);
    slot.is_load = !is_write;
    slot.t_last_req = -1;
    slot.t_first_beat = -1;

    // ON RTL, it takes some time to switch from one instruction to another, and more if it is from
    // load to store, probably due to latency to write to regfile.
    if (this->op_timestamp != -1)
    {
        this->op_timestamp += this->prev_is_write != is_write ? (is_write ? 7 : 3) : (is_write ? 0 : 1);
    }

    this->prev_is_write = is_write;
    this->started = false;
}

void VuLsu::handle_insn_load(VuLsu *_this, iss_insn_t *insn)
{
    _this->handle_access(insn, false, insn->out_regs[0]);
}

void VuLsu::handle_insn_store(VuLsu *_this, iss_insn_t *insn)
{
    _this->handle_access(insn, true, insn->in_regs[1]);
}

// Advance the sequencing state for a burst just submitted to its port. The RTL
// output spill retains a back-pressured request, so a burst leaves the
// instruction when it is sent, whatever the downstream answers. Issue loop
// only: a burst parked in denied_reqs already entered that spill.
void VuLsu::burst_issued(vp::IoReq *req, int port)
{
    uint64_t size = req->get_size();
    // A store is described by its request, a load by the ROB entry tracking it
    // (legacy per-port entry or port-0 burst entry).
    VlsuReq *vlsu_req;
    if (req->get_is_write())
    {
        vlsu_req = (VlsuReq *)req->initiator;
    }
    else if (this->brob_owns(req->initiator))
    {
        vlsu_req = ((BurstRobEntry *)req->initiator)->req;
    }
    else
    {
        vlsu_req = ((VlsuRobEntry *)req->initiator)->req;
    }
    VuLsuPendingInsn *slot = vlsu_req->slot;

    this->port_burst[port]++;
    this->remaining_size -= size;

    if (this->remaining_size == 0 && slot->t_last_req < 0)
    {
        slot->t_last_req = this->vu.iss.clock.get_cycles();
    }
    if (this->remaining_size == 0)
    {
        PendingInsn *pending_insn = slot->insn;

        // Keep the last bus operation time to model the gap before the next one.
        this->op_timestamp = this->vu.iss.clock.get_cycles() + 1;

        // Keep one extra cycle before retirement after the last burst is issued.
        pending_insn->timestamp = pending_insn->timestamp + 1;

        // Mark the instruction done once all bursts have been issued.
        slot->done = true;
    }
}

// A denied request's port is ready again. io_v2 requires the re-issue to happen
// synchronously here: deny/retry arbiters only forward inline during the retry.
void VuLsu::port_retry_muxed(vp::Block *__this, int id, vp::IoRetryChannel)
{
    VuLsu *_this = (VuLsu *)__this;

    vp::IoReq *req = _this->denied_reqs[id];
    if (req == nullptr)
    {
        // Spurious retry (e.g. broadcast "ready" from a slave which denied
        // someone else). Nothing is parked here, nothing to do.
        return;
    }

    vp::IoReqStatus err = _this->ports[id].req(req);

    if (err == vp::IO_REQ_DENIED)
    {
        // Lost the election again; keep holding for the next retry.
        vlsu_retry_rx[{(const void *)_this, id}]++;
        vlsu_retry_redeny[{(const void *)_this, id}]++;
        return;
    }

    // Accepted: release the port. The sequencing state advanced at submission
    // and may already belong to the next instruction.
    _this->denied_reqs[id] = nullptr;
    vlsu_retry_rx[{(const void *)_this, id}]++;
    _this->port_stalled[id] = false;

    if (err == vp::IO_REQ_DONE)
    {
        _this->handle_done(req);
    }
    // GRANTED: the response callback will complete the burst.

    _this->fsm_event.enable();
}

// io_v2 response — fires when an async (GRANTED) request completes. A burst
// request on port 0 (IoV2Beat) fires once per response beat instead.
vp::IoRespAck VuLsu::port_resp_muxed(vp::Block *__this, vp::IoReq *req, int id)
{
    VuLsu *_this = (VuLsu *)__this;

    if (_this->brob_owns(req->initiator))
    {
        // One beat of a burst response. Beats may land out of order (the L1
        // fabric reorders), so the word index comes from the address; copy the
        // payload into the VRF and mark the word present. The commit drain
        // retires words to the scoreboard in order, burst_recv_ports per cycle.
        BurstRobEntry *entry = (BurstRobEntry *)req->initiator;
        uint64_t base = entry->req->req.get_addr();
        int idx = (int)((req->get_addr() - base) / 4);
        if (idx >= 0 && idx < entry->nb_words && !(entry->word_mask & (1u << idx)))
        {
            // The word was already written to its VRF slot at the target
            // (zero-copy); the beat only conveys arrival + index.
            entry->word_mask |= 1u << idx;
            entry->words_arrived++;
            // Start of the commit stage for the owning instruction. Burst
            // responses arrive BEAT BY BEAT on this path (the whole-burst
            // path in burst_done is only taken for synchronous DONE), which
            // is why stamping it only there left the split empty.
            int64_t bnow = _this->vu.iss.clock.get_cycles();
            _this->beat_total++;
            if (bnow != _this->beat_last_cycle)
            {
                _this->beat_last_cycle = bnow;
                _this->beat_cycles++;
            }
            if (entry->slot != nullptr && entry->slot->t_first_beat < 0)
            {
                entry->slot->t_first_beat = bnow;
            }
        }
        _this->fsm_event.enable();
        return vp::IO_RESP_ACCEPTED;
    }

    _this->burst_done(req);
    _this->fsm_event.enable();

    return vp::IO_RESP_ACCEPTED;
}

// A request accepted with DONE: complete it now, or after its annotated latency.
void VuLsu::handle_done(vp::IoReq *req)
{
    if (req->get_resp_status() == vp::IO_RESP_INVALID)
    {
        this->trace.fatal("Invalid request (req: %p, addr: 0x%lx, size: 0x%lx, is_write: %d)\n",
            req, req->get_addr(), req->get_size(), req->get_is_write());
        return;
    }

    int64_t latency = req->get_full_latency();
    if (latency == 0)
    {
        this->burst_done(req);
    }
    else
    {
        // Served synchronously with a latency annotation: the priority queue models
        // that response delay.
        VlsuReq *vlsu_req;
        if (req->get_is_write())
        {
            vlsu_req = (VlsuReq *)req->initiator;
        }
        else if (this->brob_owns(req->initiator))
        {
            vlsu_req = ((BurstRobEntry *)req->initiator)->req;
        }
        else
        {
            vlsu_req = ((VlsuRobEntry *)req->initiator)->req;
        }
        uint64_t response_timestamp = this->vu.iss.clock.get_cycles() + latency;
        this->delayed_bursts.push({req, response_timestamp});
        PendingInsn *pending_insn = vlsu_req->slot->insn;
        pending_insn->timestamp = std::max<int64_t>(pending_insn->timestamp,
            (int64_t)response_timestamp);
    }
}

void VuLsu::burst_done(vp::IoReq *req)
{
    VuLsu *_this = this;

    // Whole-burst completion (synchronous DONE or delayed queue): mark every
    // word present; the commit drain retires them over the next cycles.
    if (_this->brob_owns(req->initiator))
    {
        BurstRobEntry *entry = (BurstRobEntry *)req->initiator;
        entry->words_arrived = entry->nb_words;
        entry->word_mask = entry->nb_words >= 32 ? 0xFFFFFFFFu :
            ((1u << entry->nb_words) - 1);
        if (entry->slot != nullptr && entry->slot->t_first_beat < 0)
        {
            entry->slot->t_first_beat = _this->vu.iss.clock.get_cycles();
        }
        return;
    }

    if (req->get_is_write())
    {
        // Stores allocate no ROB entry: on RTL the store ROB entry is already
        // freed once the request enters the memory-request spill register. The
        // burst is committed and accounted here, at the write response, matching
        // the RTL store_count which drains when the store response comes back.
        VlsuReq *store_req = (VlsuReq *)req->initiator;
        VuLsuPendingInsn *slot = store_req->slot;
        PendingInsn *pending_insn = slot->insn;
        iss_insn_t *insn = _this->vu.iss.exec.get_insn(pending_insn->entry);

        _this->vu.insn_commit(pending_insn, (int)req->get_size());
        _this->vu.exec_insn_chunk(insn, pending_insn, store_req->vstart,
            store_req->vstart + store_req->nb_elem, store_req->nb_elem);
        slot->nb_pending_bursts--;
        _this->nb_pending_stores--;

        _this->trace.msg("Retiring store request (req: %p, pending insn bursts: %d)\n",
            req, slot->nb_pending_bursts);

        // An allocator-backed write beat (Beat port) returns to its home pool
        // once the burst ack has been handled here.
        if (store_req->req_ext != nullptr)
        {
            store_req->req_ext->free();
            store_req->req_ext = nullptr;
        }
        _this->reqs_free.push_back(store_req);
        return;
    }

    auto *rob_entry = (VlsuRobEntry *)req->initiator;

    if (rob_entry == nullptr || rob_entry->port < 0 || rob_entry->port >= _this->nb_ports ||
        rob_entry->rob_id < 0 || rob_entry->rob_id >= (int)_this->rob[rob_entry->port].size())
    {
        _this->trace.fatal("Invalid VLSU response context (req: %p, rob_entry: %p)\n",
            req, rob_entry);
    }

    // We just received the response to one of the requests, set valid=True
    _this->rob[rob_entry->port][rob_entry->rob_id].valid = true;

    _this->trace.msg("Received data response (req: %p)\n", req);

    // This response may have completed a group the register file is waiting for, on
    // this instruction or on a younger one. An instruction reaches the ROB heads only
    // once the older ones committed everything, so walk from the oldest.
    for (int i = 0; i < _this->nb_pending_insn.get(); i++)
    {
        VuLsuPendingInsn &slot = _this->insns[(_this->insn_first + i) % VuLsu::queue_size];

        while (slot.nb_remaining_bursts > 0)
        {
            // Burst k goes on port k % nb_active_ports (offset by the load port
            // base: the burst port 0 is excluded when burst support is on) and
            // each port retires in order, so the next group sits at the head of
            // the first ports. Like RTL, it is committed only once every burst
            // it contains has its response.
            int nb_active = _this->nb_ports - _this->load_port_base;
            int group_size = std::min(slot.nb_remaining_bursts, nb_active);
            int nb_ready = 0;
            while (nb_ready < group_size)
            {
                int port = _this->load_port_base + nb_ready;
                VlsuRobEntry &entry = _this->rob[port][_this->rob_first[port]];
                if (!entry.allocated || !entry.valid || entry.req->slot != &slot)
                {
                    break;
                }
                nb_ready++;
            }

            if (nb_ready != group_size)
            {
                // WEDGE DIAGNOSTIC (TERANOC_VLSU_STALL_PATH). Retirement is
                // in-order AND cross-port synchronised: every active port's head
                // must belong to the SAME instruction slot. If the ports' heads
                // ever misalign to different slots, this break fires forever and
                // the core stops issuing -- which presents downstream as an
                // indefinite spin with no memory traffic and no FP retirement,
                // and with zero MSHR timeouts because nothing is waiting on a
                // cohort. Report the head state ONCE per core after a long stall.
                {
                    static const char *sp = nullptr; static bool ck = false;
                    if (!ck) { ck = true; sp = getenv("TERANOC_VLSU_STALL_PATH"); }
                    if (sp)
                    {
                        long &st = vlsu_stall_streak[(const void *)_this];
                        st++;
                        if (st == 200000 && !vlsu_stall_reported.count((const void *)_this))
                        {
                            vlsu_stall_reported.insert((const void *)_this);
                            static FILE *sf = nullptr;
                            if (!sf) sf = fopen(sp, "a");
                            if (sf)
                            {
                                fprintf(sf, "[VSTALL] %s cyc=%ld group_size=%d nb_ready=%d"
                                    " remaining=%d",
                                    _this->vu.iss.get_path().c_str(),
                                    (long)_this->vu.iss.clock.get_cycles(),
                                    group_size, nb_ready, slot.nb_remaining_bursts);
                                for (int q = _this->load_port_base; q < _this->nb_ports; q++)
                                {
                                    VlsuRobEntry &e2 = _this->rob[q][_this->rob_first[q]];
                                    fprintf(sf, " | p%d cnt=%d first=%d alloc=%d valid=%d"
                                        " same_slot=%d", q, _this->rob_count[q],
                                        _this->rob_first[q], (int)e2.allocated, (int)e2.valid,
                                        (e2.allocated && e2.req) ? (int)(e2.req->slot == &slot) : -1);
                                }
                                fprintf(sf, "\n"); fflush(sf);
                            }
                        }
                    }
                }
                break;
            }

            PendingInsn *pending_insn = slot.insn;
            iss_insn_t *insn = _this->vu.iss.exec.get_insn(pending_insn->entry);
            int committed_size = 0;

            for (int j = 0; j < group_size; j++)
            {
                int port = _this->load_port_base + j;
                VlsuRobEntry &entry = _this->rob[port][_this->rob_first[port]];
                VlsuReq *load_req = entry.req;

                _this->vu.exec_insn_chunk(insn, pending_insn, load_req->vstart,
                    load_req->vstart + load_req->nb_elem, load_req->nb_elem);
                committed_size += load_req->req.get_size();

                entry.allocated = false;
                slot.nb_pending_bursts--;
                _this->reqs_free.push_back(load_req);
                _this->rob_count[port]--;
                _this->rob_first[port] = (_this->rob_first[port] + 1) % _this->rob[port].size();
            }

            vlsu_stall_streak[(const void *)_this] = 0;   // real retirement clears it
            vlsu_last_retire[(const void *)_this] = (long)_this->vu.iss.clock.get_cycles();
            slot.nb_remaining_bursts -= group_size;

            // Notify the committed elements, which may start a chained instruction.
            // Only the group's own elements are reported, so a consumer never reads
            // elements still in flight on another port.
            _this->vu.insn_commit(pending_insn, committed_size);

            _this->trace.msg("Committing load bursts (id: %d, nb_bursts: %d, pending insn bursts: %d)\n",
                pending_insn->id, group_size, slot.nb_pending_bursts);
        }

        // A younger instruction owns the ROB heads only once this one is fully done.
        if (slot.nb_remaining_bursts != 0)
        {
            break;
        }
    }
}

// H1 admission condition (RTL spatz_vlsu.sv dual_safe): the waiting
// instruction must be a burst-safe unit-stride load -- VLE e32, aligned, one
// ROB batch or less, no tail, vstart==0. Evaluated on the live vector config:
// a config-changing vsetvli drains the queue first (see vsetvli_needs_drain),
// so any instruction waiting behind an in-flight elder reads the same vl.
bool VuLsu::next_insn_burst_safe(VuLsuPendingInsn &slot)
{
    iss_insn_t *insn = this->vu.iss.exec.get_insn(slot.insn->entry);
    if (insn->decoder_item->u.insn.block_handler != (void *)&VuLsu::handle_insn_load)
    {
        return false;
    }
    int elem_size = insn->uim[1] >= 5 ? 1 << (insn->uim[1] - 4) : 1 << 0;
    // Runahead uses the same element-width eligibility as burst issue. The
    // old e32-only test serialized every e16 pair despite admitting its loads
    // to the burst path. Keep the old predicate only for calibration A/Bs.
    static const bool legacy_fp32_runahead = []() {
        const char *value = getenv("TERANOC_VLSU_LEGACY_FP32_RUNAHEAD");
        return value != nullptr && atoi(value) != 0;
    }();
    if (distributed_burst_rob() && elem_size > 4) return false;
    if (!distributed_burst_rob() && elem_size != 4 &&
        (legacy_fp32_runahead || !this->burst_sub_word || elem_size != 2))
    {
        return false;
    }
    if (this->vu.iss.csr.vstart.value != 0)
    {
        return false;
    }
    iss_addr_t addr = slot.insn->reg;
    if (addr & (iss_addr_t)(this->burst_bytes - 1))
    {
        return false;
    }
    iss_addr_t bytes =
        (this->vu.iss.csr.vl.value - this->vu.iss.csr.vstart.value) * elem_size;
    if (bytes < (iss_addr_t)(distributed_burst_rob() ? 8 : this->burst_bytes) ||
        bytes > (iss_addr_t)(this->burst_rob_words * 4))
    {
        return false;
    }
    if (bytes % (distributed_burst_rob() ? 4 : this->burst_bytes))
    {
        return false;
    }
    return true;
}

// Per-cycle burst commit drain: the RTL ROB->VRF path retires one word per
// cycle, or two with TwinROB0 pair commit, strictly in order. Committing
// releases bytes to chained consumers as the words land.
void VuLsu::burst_commit_drain()
{
    int budget = this->burst_recv_ports;
    int committed_this_cycle = 0;
    while (budget > 0 && this->brob_count > 0)
    {
        BurstRobEntry &entry = this->brob[this->brob_first];
        // In-order commit: the head word must have arrived (beats may land
        // out of order, the ROB absorbs that by index).
        if (!(entry.word_mask & (1u << entry.words_committed)))
        {
            // RTL c_waitbeat: an entry is resident but its head beat has not
            // landed, so the commit port idles this cycle.
            if (committed_this_cycle == 0)
            {
                this->vp_wait_beats++;
            }
            break;
        }
        int n = 1;
        if (distributed_burst_rob())
        {
            // Ordinary lane law: commit one row of up to NrMemPorts words
            // together. Missing tail lanes are dummy entries, not VRF writes.
            n = std::min(this->nb_ports, entry.nb_words - entry.words_committed);
            uint32_t mask = ((1u << n) - 1) << entry.words_committed;
            if ((entry.word_mask & mask) != mask)
            {
                if (committed_this_cycle == 0) this->vp_wait_beats++;
                break;
            }
        }
        // Pair commit (TwinROB0): two words per cycle when both are present.
        if (!distributed_burst_rob() && budget >= 2 && entry.words_committed + 1 < entry.nb_words &&
            (entry.word_mask & (1u << (entry.words_committed + 1))))
        {
            n = 2;
        }
        entry.words_committed += n;
        budget -= n;
        committed_this_cycle += n;
        if (distributed_burst_rob()) this->brob_words_used -= this->nb_ports;
        if (n >= 2) this->vp_pair_commit++; else this->vp_single_commit++;
        this->vu.insn_commit(entry.slot->insn, n * 4);
        if (entry.words_committed == entry.nb_words)
        {
            VuLsuPendingInsn *slot = entry.slot;
            PendingInsn *pending_insn = slot->insn;
            iss_insn_t *insn = this->vu.iss.exec.get_insn(pending_insn->entry);
            VlsuReq *load_req = entry.req;
            this->vu.exec_insn_chunk(insn, pending_insn, load_req->vstart,
                load_req->vstart + load_req->nb_elem, load_req->nb_elem);
            slot->nb_pending_bursts--;
            slot->nb_remaining_bursts--;
            this->reqs_free.push_back(load_req);

            if (!distributed_burst_rob()) this->brob_words_used -= entry.nb_words;
            entry.allocated = false;
            entry.req = nullptr;
            entry.slot = nullptr;
            entry.nb_words = 0;
            entry.words_arrived = 0;
            entry.words_committed = 0;
            entry.word_mask = 0;
            this->brob_first = (this->brob_first + 1) % this->brob.size();
            this->brob_count--;
        }
        // A partial final row still consumes the lane group's one write slot.
        if (distributed_burst_rob()) break;
    }
}

// Port-0 burst issue step: one 64B request per burst_issue_latency cycles,
// gated by word-granular ROB0 room (RTL room_block: status_cnt <= NumWords -
// BlockWords, so the room test is non-strict).
void VuLsu::burst_issue_step(int64_t cycles)
{
    VuLsuPendingInsn &slot = this->insns[this->insn_ongoing];
    PendingInsn *pending_insn = slot.insn;

    // A denied burst holds the port until its retry succeeds.
    if (this->port_stalled[0])
    {
        this->vp_req_stall++;   // RTL c_reqstall: request up, memory not ready
        return;
    }
    // BlockAlloc cadence (decide -> reserve -> send).
    if (cycles < this->port0_next_issue)
    {
        this->vp_blk_stall++;   // RTL c_blkstall: eligible but not fired yet
        return;
    }
    uint64_t req_offset = (uint64_t)this->port_burst[0] * this->burst_bytes;
    if (req_offset >= this->burst_full_bytes)
    {
        // Burst region fully issued; the tail phase (if any) starts once the
        // ROB has drained, see fsm_handler.
        return;
    }

    uint64_t size = distributed_burst_rob()
        ? std::min((uint64_t)this->burst_bytes, (uint64_t)this->burst_full_bytes - req_offset)
        : this->burst_bytes;
    int reserved_words = distributed_burst_rob()
        ? ((size / 4 + this->nb_ports - 1) / this->nb_ports) * this->nb_ports
        : this->burst_max_words;
    int other_words = 0;
    if (distributed_burst_rob())
        other_words = *std::max_element(this->rob_count.begin(), this->rob_count.end()) * this->nb_ports;
    if (this->brob_words_used + other_words > this->burst_rob_words - reserved_words ||
        this->brob_count >= (int)this->brob.size())
    {
        this->vp_blk_stall++;
        return;
    }
    uint8_t *velem = this->pending_velem + req_offset;
    iss_reg_t addr = this->pending_addr + req_offset;
    int elem_idx = this->vstart + req_offset / this->elem_size;

    this->trace.msg(vp::Trace::LEVEL_TRACE,
        "Sending burst request (id: %d, addr: 0x%lx, size: 0x%lx, remaining_size: 0x%lx)\n",
        pending_insn->id, addr, size, this->remaining_size);
    this->event_addr[0].event((uint8_t *)&addr);
    this->event_size[0].event((uint8_t *)&size);
    uint8_t zero = 0;
    this->event_is_write[0].event(&zero);

    if (this->reqs_free.empty())
    {
        this->reqs.emplace_back();
        this->reqs_free.push_back(&this->reqs.back());
    }
    VlsuReq *vlsu_req = this->reqs_free.back();
    this->reqs_free.pop_back();

    vlsu_req->slot = &slot;
    vlsu_req->port = 0;
    vlsu_req->vreg = this->pending_vreg;
    vlsu_req->vstart = elem_idx;
    vlsu_req->nb_elem = size / this->elem_size;
    vlsu_req->velem = velem;

    vp::IoReq *req = &vlsu_req->req;
    req->prepare();
    req->set_resp_status(vp::IO_RESP_OK);

    BurstRobEntry &entry = this->brob[this->brob_next];
    this->brob_next = (this->brob_next + 1) % this->brob.size();
    entry.allocated = true;
    entry.req = vlsu_req;
    entry.slot = &slot;
    entry.nb_words = size / 4;
    entry.words_arrived = 0;
    entry.words_committed = 0;
    this->brob_count++;
    this->brob_words_used += reserved_words;

    req->initiator = (void *)&entry;
    req->set_addr(addr);
    req->set_is_write(false);
    req->set_size(size);
    // The VRF base rides in data: the target bank writes each word straight
    // into its final VRF slot (zero-copy, like every legacy load); response
    // beats then carry no payload, only arrival + index.
    req->set_data(velem);
    slot.nb_pending_bursts++;

    vp::IoReqStatus err = this->ports[0].req(req);

    // The burst leaves the instruction as soon as it is sent, even when the
    // downstream denies it (the output spill holds it, like RTL).
    this->burst_issued(req, 0);
    uint64_t next_bytes = this->burst_full_bytes - req_offset - size;
    this->port0_next_issue = cycles + burst_allocation_cycles(next_bytes,
        this->burst_bytes, this->nb_ports, this->burst_issue_latency);

    if (err == vp::IO_REQ_DENIED)
    {
        this->port_stalled[0] = true;
        this->denied_reqs[0] = req;
    }
    else if (err == vp::IO_REQ_DONE)
    {
        this->handle_done(req);
    }
    // GRANTED: per-beat completion arrives through port_resp_muxed.
}

void VuLsu::fsm_handler(vp::Block *__this, vp::ClockEvent *event)
{
    {
        // WEDGE DETECTOR. The FSM only disables when nb_pending_insn == 0, so while
        // the core is wedged with work outstanding this handler runs every cycle --
        // which the retirement-break probe does not, because that path is only
        // reached when a response arrives. Fires once per core.
        static const char *sp = nullptr; static bool ck = false;
        static long stall_after = 300000;
        if (!ck)
        {
            ck = true;
            sp = getenv("TERANOC_VLSU_STALL_PATH");
            // On a collapsing arm 300k cycles of no retirement is a long wall
            // wait; let the caller shorten it to get the parked-port census.
            const char *ap = getenv("TERANOC_VLSU_STALL_AFTER");
            if (ap) stall_after = strtol(ap, nullptr, 0);
        }
        auto *_t = static_cast<VuLsu *>(__this);
        if (sp && _t->nb_pending_insn.get() > 0)
        {
            long now = (long)_t->vu.iss.clock.get_cycles();
            auto it = vlsu_last_retire.find((const void *)_t);
            if (it == vlsu_last_retire.end()) vlsu_last_retire[(const void *)_t] = now;
            else if (now - it->second > stall_after &&
                     !vlsu_stall_reported.count((const void *)_t))
            {
                vlsu_stall_reported.insert((const void *)_t);
                static FILE *sf = nullptr;
                if (!sf) sf = fopen(sp, "a");
                if (sf)
                {
                    fprintf(sf, "[VWEDGE] %s cyc=%ld idle_for=%ld pending_insn=%d"
                        " waiting=%d remaining_size=%ld nb_pending_stores=%d",
                        _t->vu.iss.get_path().c_str(), now, now - it->second,
                        (int)_t->nb_pending_insn.get(), (int)_t->nb_waiting_insn,
                        (long)_t->remaining_size, (int)_t->nb_pending_stores);
                    for (int q = 0; q < _t->nb_ports; q++)
                    {
                        VlsuRobEntry &e2 = _t->rob[q][_t->rob_first[q]];
                        fprintf(sf, " | p%d cnt=%d/%d first=%d alloc=%d valid=%d stalled=%d"
                            " retries=%ld redeny=%ld parked=%d",
                            q, _t->rob_count[q], (int)_t->rob[q].size(), _t->rob_first[q],
                            (int)e2.allocated, (int)e2.valid, (int)_t->port_stalled[q],
                            vlsu_retry_rx[{(const void *)_t, q}],
                            vlsu_retry_redeny[{(const void *)_t, q}],
                            (int)(_t->denied_reqs[q] != nullptr));
                    }
                    fprintf(sf, "\n"); fflush(sf);
                }
            }
        }
    }

    VuLsu *_this = (VuLsu *)__this;

    // Requester-side latency instrumentation: sample in-flight instructions
    // on every active cycle; dump a per-VLSU window to the stats file.
    int resident = _this->nb_pending_insn.get() - _this->nb_waiting_insn;
    _this->stat_inflight_acc += (uint64_t)resident;
    _this->stat_inflight_n++;
    // RTL c_insn / c_noinsn: is a LOAD sitting at the commit head this cycle?
    if (resident > 0 && _this->insns[_this->insn_first].is_load)
    {
        _this->vp_insn_act++;
    }
    else if (resident == 0)
    {
        _this->vp_no_insn++;
    }
    // Dump cadence: the FSM only ticks while the VLSU has work, so a short
    // kernel may never reach a large interval. TERANOC_VLSU_STATS_PERIOD
    // overrides it (default 1024 active cycles).
    static int vlsu_period = 0;
    if (__builtin_expect(vlsu_period == 0, 0))
    {
        const char *pe = getenv("TERANOC_VLSU_STATS_PERIOD");
        vlsu_period = pe ? atoi(pe) : 1024;
        if (vlsu_period <= 0) vlsu_period = 1024;
    }
    if (_this->stat_inflight_n % (uint64_t)vlsu_period == 0)
    {
        static FILE *vlsu_f = nullptr;
        if (!vlsu_f)
        {
            const char *vp = getenv("TERANOC_VLSU_STATS_PATH");
            vlsu_f = fopen(vp ? vp : "/tmp/vlsu_stats.log", "a");
        }
        if (vlsu_f)
        {
            double ln = _this->lat_n ? (double)_this->lat_n : 1.0;
            fprintf(vlsu_f,
                "VLSU core=%p insns=%lu avg_lat=%.1f inflight=%.2f samples=%lu"
                " insn_act=%lu no_insn=%lu pair_commit=%lu single_commit=%lu"
                " wait_beats=%lu req_stall=%lu blk_stall=%lu insn_ret=%lu dual_adv=%lu"
                " split_n=%lu issue=%.1f flight=%.1f commit=%.1f"
                " load_burst=%lu load_nonburst=%lu beats=%lu beat_cyc=%lu\n",
                (void *)_this, (unsigned long)_this->stat_insns,
                _this->stat_insns ? (double)_this->stat_lat_issue / _this->stat_insns : 0.0,
                (double)_this->stat_inflight_acc / _this->stat_inflight_n,
                (unsigned long)_this->stat_inflight_n,
                (unsigned long)_this->vp_insn_act, (unsigned long)_this->vp_no_insn,
                (unsigned long)_this->vp_pair_commit, (unsigned long)_this->vp_single_commit,
                (unsigned long)_this->vp_wait_beats, (unsigned long)_this->vp_req_stall,
                (unsigned long)_this->vp_blk_stall, (unsigned long)_this->vp_insn_ret,
                (unsigned long)_this->vp_dual_adv, (unsigned long)_this->lat_n,
                (double)_this->lat_issue / ln, (double)_this->lat_flight / ln,
                (double)_this->lat_commit / ln,
                (unsigned long)_this->vp_load_burst,
                (unsigned long)_this->vp_load_nonburst,
                (unsigned long)_this->beat_total,
                (unsigned long)_this->beat_cycles);
            fflush(vlsu_f);
        }
    }


    // Check if any synchronous delayed burst need to be terminated
    while (!_this->delayed_bursts.empty() &&
            _this->delayed_bursts.top().timestamp <= _this->vu.iss.clock.get_cycles())
    {
        vp::IoReq *req = _this->delayed_bursts.top().req;
        _this->delayed_bursts.pop();
        _this->burst_done(req);
    }

    // Burst ROB->VRF commit drain (1 or 2 words per cycle, in order)
    if (_this->burst_enable)
    {
        _this->burst_commit_drain();
    }

    // In case nothing is on-going, disable the FSM
    if (_this->nb_pending_insn.get() == 0)
    {
        uint8_t zero = 0;
        _this->event_queue.event_highz();
        _this->event_pc.event_highz();
        _this->event_active.event(&zero);
        for (int i=0; i<_this->nb_ports; i++)
        {
            _this->event_addr[i].event_highz();
            _this->event_size[i].event_highz();
            _this->event_is_write[i].event(&zero);
        }
        _this->event_label.dump_highz();

        _this->fsm_event.disable();
    }

    // Check if the first waiting instruction can be started. Without burst
    // support this keeps the legacy rule: the on-going instruction must have
    // issued all its requests (remaining_size == 0). With burst support the
    // serialization follows the RTL (spatz_vlsu.sv):
    //  - dual_load == 1 (legacy): the elder must be fully retired (its last
    //    beat committed to the VRF) before the next memory instruction starts;
    //  - dual_load == 2 (H1 runahead): a burst-safe load is admitted as soon as
    //    the elder's requests are all issued, with at most two instructions in
    //    flight; anything else waits for the elder to retire.
    int started_unretired = _this->nb_pending_insn.get() - _this->nb_waiting_insn;
    bool start_ok;
    if (!_this->burst_enable)
    {
        start_ok = _this->remaining_size == 0;
    }
    else if (started_unretired == 0)
    {
        start_ok = true;
    }
    else if (_this->burst_dual_load == 1)
    {
        start_ok = false;
    }
    else if (_this->nb_waiting_insn > 0 && _this->remaining_size == 0 &&
        started_unretired < 2)
    {
        // H1: elder fully issued, next is a burst-safe load. The elder only has
        // to be a LOAD -- the RTL's dual_adv gates it on commit_insn_q.is_load,
        // not on the elder being burst-shaped itself (spatz_vlsu.sv:914-922).
        // Requiring elder.burst_safe here refused runahead whenever the elder
        // was a plain or tailed load, which is why the model reached N=1.06
        // against the RTL's 1.23 on the same kernel.
        VuLsuPendingInsn &elder = _this->insns[_this->insn_first];
        start_ok = elder.is_load &&
            (!distributed_burst_rob() || elder.burst_safe) &&
            _this->next_insn_burst_safe(_this->insns[_this->insn_first_waiting]);
    }
    else
    {
        start_ok = false;
    }

    if (_this->nb_waiting_insn > 0 && start_ok)
    {
        VuLsuPendingInsn &slot = _this->insns[_this->insn_first_waiting];
        PendingInsn *pending_insn = slot.insn;
        iss_insn_t *insn = _this->vu.iss.exec.get_insn(pending_insn->entry);

        if (pending_insn->timestamp <= _this->vu.iss.clock.get_cycles() && _this->vu.insn_ready(pending_insn))
        {
            // Count a younger load actually starting, not repeated eligible
            // cycles while its timestamp or register dependencies still block it.
            if (_this->burst_enable && started_unretired > 0)
            {
                _this->vp_dual_adv++;
            }
            
#ifdef CONFIG_GVSOC_STATS_ACTIVE
        // Instruction leaves the waiting queue and its memory op is set up:
        // real execution starts now, stamped for the per-label duration
        // accounted at Vu::insn_end.
            if (_this->vu.stats_enabled && pending_insn->exec_start_cycle < 0)
            {
                pending_insn->exec_start_cycle = _this->vu.iss.clock.get_cycles();
            }
#endif
            
            // If so, it becomes the on-going instruction and gets armed with the allowed start of its request issuing phase
            // according to its instruction latency. 
            // Only the active on-going instruction may consume instruction latency.
            pending_insn->timestamp = _this->vu.iss.clock.get_cycles() + insn->latency;
            ((void (*)(VuLsu *, iss_insn_t *))insn->decoder_item->u.insn.block_handler)(_this, insn);
            _this->insn_ongoing = _this->insn_first_waiting;
            _this->insns[_this->insn_ongoing].issued_at = _this->vu.iss.clock.get_cycles();
            _this->insn_first_waiting = (_this->insn_first_waiting + 1) % VuLsu::queue_size;
            _this->nb_waiting_insn--;
        }
        
        
    }

    // Tail-phase activation (burst-mode instructions with a sub-burst
    // remainder): the RTL waits for the burst region to be fully drained
    // (mem_pending == 0) before switching to the multi-port single-word phase.
    if (_this->burst_enable && _this->burst_mode && !_this->tail_phase &&
        _this->remaining_size > 0 && _this->nb_waiting_insn >= 0)
    {
        if ((uint64_t)_this->port_burst[0] * _this->burst_bytes >= _this->burst_full_bytes &&
            _this->brob_count == 0 &&
            _this->pending_size > _this->burst_full_bytes)
        {
            _this->tail_phase = true;
            _this->tail_base = _this->burst_full_bytes;
            _this->burst_size = _this->vu.lane_width;
            for (int p = 0; p < _this->nb_ports; p++)
            {
                _this->port_burst[p] = 0;
            }
        }
    }

    if (_this->remaining_size && _this->op_timestamp <= _this->vu.iss.clock.get_cycles())
    {
        VuLsuPendingInsn &slot = _this->insns[_this->insn_ongoing];
        PendingInsn *pending_insn = slot.insn;

        // Once an instruction started issuing, it owns the ports until it is done:
        // only its first burst waits for the previous instruction.
        bool phase_ready = true;
        if (!_this->started)
        {
            // On RTL the load/store phase only switches once the previous instruction
            // completed its accesses: a store completes when its responses are back, so
            // anything waits for the outstanding stores to drain; a load completes when
            // its elements reached the register file, which an empty ROB guarantees.
            // Loads do not wait for each other, their bursts being issued independently
            // of the commit queue.
            phase_ready = _this->nb_pending_stores == 0;

            if (_this->pending_is_write)
            {
                for (int i = 0; i < _this->nb_ports; i++)
                {
                    phase_ready &= _this->rob_count[i] == 0;
                }
                if (_this->burst_enable)
                {
                    // Load->store mode switch also waits for the burst ROB to drain.
                    phase_ready &= _this->brob_count == 0;
                }
            }
        }

        // If the on-going instruction is ready and its instruction latency has elapsed,
        // try to send requests to available ports. A store reads the register file once
        // per burst, so it is checked burst by burst, at the elements that burst reads.
        if (phase_ready && pending_insn->timestamp <= _this->vu.iss.clock.get_cycles() &&
            (_this->pending_is_write || _this->vu.insn_ready(pending_insn)))
        {
            iss_insn_t *insn = _this->vu.iss.exec.get_insn(pending_insn->entry);

            if (!_this->started)
            {
                _this->started = true;
                _this->event_label.dump(insn->desc->label);
                _this->event_pc.event((uint8_t *)&insn->addr);
            }

            if (_this->burst_enable && _this->burst_mode && !_this->tail_phase)
            {
                // Burst phase: 64B requests on port 0 at the BlockAlloc cadence.
                _this->burst_issue_step(_this->vu.iss.clock.get_cycles());
            }
            else
            {
            // Load ports exclude the burst port (0) when burst support is on;
            // stores keep all ports.
            int port_base = _this->pending_is_write ? 0 : _this->load_port_base;
            int nb_active_ports = _this->nb_ports - port_base;
            for (int i = port_base; i < _this->nb_ports; i++)
            {
                if (_this->remaining_size == 0) break;

                // A denied burst holds its port until it is granted
                if (!_this->port_stalled[i])
                {
                    // A load also needs a free ROB entry of the port, which is what
                    // limits the number of outstanding loads. Like the RTL reorder
                    // buffer, which reports itself full one entry before the end, the
                    // last entry is never allocated. A store needs no entry, and is
                    // thus not limited.
                    if (!_this->pending_is_write &&
                        _this->rob_count[i] + (distributed_burst_rob()
                            ? _this->brob_words_used / _this->nb_ports : 0)
                            >= (int)_this->rob[i].size() - 1)
                    {
                        continue;
                    }

                    uint32_t req_idx = _this->port_burst[i] * nb_active_ports + (i - port_base);
                    // tail_base shifts a burst-mode instruction's tail phase past
                    // its burst region; it is 0 on every other path.
                    uint64_t req_offset = _this->tail_base + req_idx * _this->burst_size;
                    if (req_offset >= _this->pending_size)
                    {
                        continue;
                    }
                    uint64_t size = std::min((uint64_t)_this->burst_size,
                        _this->pending_size - req_offset);

                    // The producer must have committed the elements this burst reads
                    if (_this->pending_is_write &&
                        !_this->vu.insn_ready(pending_insn, req_offset + size))
                    {
                        continue;
                    }

                    uint8_t *velem = _this->pending_velem + req_offset;
                    int elem_idx = _this->vstart + req_offset / _this->elem_size;
                    iss_reg_t addr;
                    if (_this->strided)
                    {
                        uint64_t offset = req_idx * _this->stride;
                        addr = _this->pending_addr + offset;
                    }
                    else if (_this->reg_indexed != -1)
                    {
                        uint64_t offset = velem_get_value(&_this->vu.iss, _this->reg_indexed, req_idx,
                            _this->inst_elem_size, _this->vu.iss.vector.lmul);
                        addr = _this->pending_addr + offset;
                    }
                    else
                    {
                        addr = _this->pending_addr + req_offset;
                    }

                    _this->trace.msg(vp::Trace::LEVEL_TRACE,
                        "Sending request (id: %d, port: %d, addr: 0x%lx, size: 0x%lx, remaining_size: 0x%lx, is_write: %d)\n",
                        pending_insn->id, i, addr, size, _this->remaining_size,
                        _this->pending_is_write);
                    _this->event_addr[i].event((uint8_t *)&addr);
                    _this->event_size[i].event((uint8_t *)&size);
                    _this->event_is_write[i].event((uint8_t *)&_this->pending_is_write);

                    // Take a request from the pool, growing it when all the requests are
                    // in flight
                    if (_this->reqs_free.empty())
                    {
                        _this->reqs.emplace_back();
                        _this->reqs_free.push_back(&_this->reqs.back());
                    }

                    VlsuReq *vlsu_req = _this->reqs_free.back();
                    _this->reqs_free.pop_back();

                    vlsu_req->slot = &slot;
                    vlsu_req->port = i;
                    vlsu_req->vreg = _this->pending_vreg;
                    vlsu_req->vstart = elem_idx;
                    vlsu_req->nb_elem = size / _this->elem_size;

                    vp::IoReq *req;
                    uint8_t *req_data = velem;
                    if (_this->pending_is_write && _this->burst_enable && i == 0)
                    {
                        // Write beats on the IoV2Beat port are allocator-owned
                        // once accepted; the payload is co-allocated and only
                        // ever copied into, never repointed.
                        req = _this->beat_allocator->alloc();
                        vlsu_req->req_ext = req;
                        req->is_first = true;
                        req->is_last = true;
                        req->burst_id = -1;
                    }
                    else
                    {
                        req = &vlsu_req->req;
                    }
                    // io_v2 reset of the per-send fields. Status starts OK; the slave
                    // downgrades it to IO_RESP_INVALID on error.
                    req->prepare();
                    req->set_resp_status(vp::IO_RESP_OK);

                    if (_this->pending_is_write)
                    {
                        // The vector register file is read now. Send a copy of the
                        // elements since the request is only applied when it reaches
                        // the target, when the register may have been overwritten.
                        vlsu_req->data.assign(velem, velem + size);
                        req_data = vlsu_req->data.data();

                        _this->nb_pending_stores++;
                        // No argument stack in io_v2: the back-link travels in
                        // the initiator field.
                        req->initiator = (void *)vlsu_req;
                    }
                    else
                    {
                        int rob_id = _this->rob_next[i];
                        _this->rob_next[i] = (rob_id + 1) % _this->rob[i].size();
                        _this->rob_count[i]++;

                        VlsuRobEntry &rob_entry = _this->rob[i][rob_id];
                        rob_entry.port = i;
                        rob_entry.rob_id = rob_id;
                        rob_entry.allocated = true;
                        rob_entry.valid = false;
                        rob_entry.req = vlsu_req;

                        req->initiator = (void *)&rob_entry;
                    }

                    req->set_addr(addr);
                    req->set_is_write(_this->pending_is_write);
                    req->set_size(size);
                    // Size-0-pool write beats carry a caller-managed data
                    // pointer (the store data copy in the VlsuReq); pooled
                    // requests point at it directly as before.
                    req->set_data(req_data);
                    slot.nb_pending_bursts++;

                    vp::IoReqStatus err = _this->ports[i].req(req);

                    // The burst has left the instruction as soon as it is sent,
                    // even when the downstream denies it.
                    _this->burst_issued(req, i);

                    if (err == vp::IO_REQ_DENIED)
                    {
                        // Park the burst on its port; it is re-issued inside the
                        // retry callback.
                        _this->port_stalled[i] = true;
                        _this->denied_reqs[i] = req;
                    }
                    else if (err == vp::IO_REQ_DONE)
                    {
                        _this->handle_done(req);
                    }
                    // GRANTED: completion arrives through port_resp_muxed.
                }
            }
            }
        }
    }

    // Check if the first enqueued instruction must be removed. A store is done once
    // its bursts got their response, which also drains the block's stores, the next
    // instruction being unable to issue before that.
    if (_this->nb_pending_insn.get() > 0)
    {
        VuLsuPendingInsn &slot = _this->insns[_this->insn_first];
        PendingInsn *pending_insn = slot.insn;
        if (slot.done && slot.nb_pending_bursts == 0 &&
            pending_insn->timestamp <= _this->vu.iss.clock.get_cycles())
        {
            _this->event_label.dump_highz_next();
            _this->insn_first = (_this->insn_first + 1) % VuLsu::queue_size;
            _this->nb_pending_insn.dec(1);
            slot.done = false;
            pending_insn->timestamp = _this->vu.iss.clock.get_cycles() + 1;
            // Requester-side latency instrumentation: issue->retire latency.
            _this->stat_insns++;
            _this->vp_insn_ret++;
            int64_t now = _this->vu.iss.clock.get_cycles();
            if (slot.issued_at > 0)
            {
                _this->stat_lat_issue += (uint64_t)(now - slot.issued_at);
                // Split L into issue / flight / commit. Only loads that took
                // the burst path carry both stamps; the rest are counted in
                // L but not in the split, so lat_n is reported separately.
                if (slot.t_last_req >= 0 && slot.t_first_beat >= 0 &&
                    slot.t_first_beat >= slot.t_last_req)
                {
                    _this->lat_n++;
                    _this->lat_issue += (uint64_t)(slot.t_last_req - slot.issued_at);
                    _this->lat_flight += (uint64_t)(slot.t_first_beat - slot.t_last_req);
                    _this->lat_commit += (uint64_t)(now - slot.t_first_beat);
                }
            }
            _this->vu.insn_end(pending_insn);
        }
    }
}
