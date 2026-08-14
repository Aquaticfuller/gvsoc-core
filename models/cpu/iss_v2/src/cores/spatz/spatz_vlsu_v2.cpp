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

#include <cpu/iss_v2/include/cores/vector_unit/vector_unit.hpp>

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

    int nb_outstanding_reqs = iss.get_js_config()->get_child_int("vu/nb_outstanding_reqs");

    this->rob.resize(nb_ports);
    this->rob_next.resize(nb_ports);
    this->rob_first.resize(nb_ports);
    this->rob_count.resize(nb_ports);
    this->port_burst.resize(nb_ports);
    this->port_stalled.resize(nb_ports);
    this->denied_reqs.resize(nb_ports);
    for (int i=0; i<nb_ports; i++)
    {
        this->rob[i].resize(nb_outstanding_reqs);
    }

    // Spatz port-0 burst loads. All default-off/zero when the properties are
    // absent (targets without burst support keep the legacy behavior exactly).
    js::Config *cfg = iss.get_js_config();
    this->burst_enable = cfg->get_int("vu/burst_enable");
    this->burst_max_words = cfg->get_int("vu/burst_max_words");
    if (this->burst_max_words <= 0) this->burst_max_words = 16;
    this->burst_bytes = this->burst_max_words * 4;
    this->burst_rob_words = cfg->get_int("vu/burst_rob_depth");
    if (this->burst_rob_words < this->burst_max_words)
        this->burst_rob_words = 2 * this->burst_max_words;
    this->burst_block_alloc = cfg->get_int("vu/burst_block_alloc");
    this->burst_dual_load = cfg->get_int("vu/burst_dual_load");
    if (this->burst_dual_load <= 0) this->burst_dual_load = 1;
    this->burst_recv_ports = cfg->get_int("vu/burst_recv_ports");
    if (this->burst_recv_ports <= 0) this->burst_recv_ports = 1;
    this->burst_issue_latency = cfg->get_int("vu/burst_issue_latency");
    if (this->burst_issue_latency <= 0)
        this->burst_issue_latency = this->burst_block_alloc ? 3 : 18;

    // One entry per in-flight burst plus one spare; the word-granular
    // occupancy check is the real capacity limit.
    this->brob.resize(this->burst_rob_words / this->burst_max_words + 1);

    if (this->burst_enable)
    {
        // Write beats on the IoV2Beat port are allocator-owned downstream.
        // Size-0 pool: req->data is caller-managed (re)set on every send; the
        // write-ack recycling in the beat adapters repoints it.
        this->beat_allocator = vp::IoReqAllocator::get(0);
        // Port 0 is reserved for bursts; other loads stripe over the rest.
        this->load_port_base = 1;
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

            for (int j=0; j<nb_outstanding_reqs; j++)
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

    // Spatz port-0 burst mode (RTL spatz_vlsu.sv use_port0_burst_req): a
    // unit-stride VLE load, e32, vl covering [one burst, one ROB batch], 64B
    // aligned base, vstart == 0. Only full bursts are formed; a sub-burst
    // remainder runs as a legacy multi-port tail phase once the burst region
    // has fully committed (RTL tail phase waits for mem_pending == 0).
    this->burst_mode = this->burst_enable && !is_write && !do_stride && reg_indexed == -1
        && elem_size == 4
        && this->vstart == 0
        && this->pending_size >= (iss_addr_t)this->burst_bytes
        && this->pending_size <= (iss_addr_t)(this->burst_rob_words * 4)
        && ((this->pending_addr & (iss_addr_t)(this->burst_bytes - 1)) == 0);
    this->burst_full_bytes = this->burst_mode ?
        (this->pending_size / this->burst_bytes) * this->burst_bytes : 0;
    this->tail_phase = false;
    this->tail_base = 0;
    if (this->burst_mode)
    {
        this->burst_size = this->burst_bytes;
        // First burst send: decide -> reserve -> send cadence from the start
        // of the issue phase (the timestamp gate below adds the rest).
        this->port0_next_issue = this->vu.iss.clock.get_cycles() +
            (this->burst_issue_latency - 1);
    }

    VuLsuPendingInsn &slot = this->insns[this->insn_first_waiting];
    // Store bursts commit one by one on their response: they are not tracked in the
    // ROB and so not committed by group.
    if (this->burst_mode)
    {
        // Commit units: one per full burst plus one per tail word.
        slot.nb_remaining_bursts = this->burst_full_bytes / this->burst_bytes +
            (this->pending_size - this->burst_full_bytes) / this->vu.lane_width;
    }
    else
    {
        slot.nb_remaining_bursts = is_write ?
            0 : (this->pending_size + this->burst_size - 1) / this->burst_size;
    }
    // H1 runahead safety (RTL dual_safe): burst-mode load without a tail.
    slot.burst_safe = this->burst_mode && (this->pending_size == this->burst_full_bytes);

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
        return;
    }

    // Accepted: release the port. The sequencing state advanced at submission
    // and may already belong to the next instruction.
    _this->denied_reqs[id] = nullptr;
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
    if (elem_size != 4)
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
    if (bytes < (iss_addr_t)this->burst_bytes ||
        bytes > (iss_addr_t)(this->burst_rob_words * 4))
    {
        return false;
    }
    if (bytes % this->burst_bytes)
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
    while (budget > 0 && this->brob_count > 0)
    {
        BurstRobEntry &entry = this->brob[this->brob_first];
        // In-order commit: the head word must have arrived (beats may land
        // out of order, the ROB absorbs that by index).
        if (!(entry.word_mask & (1u << entry.words_committed)))
        {
            break;
        }
        int n = 1;
        // Pair commit (TwinROB0): two words per cycle when both are present.
        if (budget >= 2 && entry.words_committed + 1 < entry.nb_words &&
            (entry.word_mask & (1u << (entry.words_committed + 1))))
        {
            n = 2;
        }
        entry.words_committed += n;
        budget -= n;
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

            this->brob_words_used -= entry.nb_words;
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
        return;
    }
    // BlockAlloc cadence (decide -> reserve -> send).
    if (cycles < this->port0_next_issue)
    {
        return;
    }
    if (this->brob_words_used > this->burst_rob_words - this->burst_max_words)
    {
        return;
    }

    uint64_t req_offset = (uint64_t)this->port_burst[0] * this->burst_bytes;
    if (req_offset >= this->burst_full_bytes)
    {
        // Burst region fully issued; the tail phase (if any) starts once the
        // ROB has drained, see fsm_handler.
        return;
    }

    uint64_t size = this->burst_bytes;
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
    this->brob_words_used += entry.nb_words;

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
    this->port0_next_issue = cycles + this->burst_issue_latency;

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
    VuLsu *_this = (VuLsu *)__this;


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
        // H1: elder fully issued and burst-safe, next is a burst-safe load.
        VuLsuPendingInsn &elder = _this->insns[_this->insn_first];
        start_ok = elder.burst_safe &&
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
                        _this->rob_count[i] >= (int)_this->rob[i].size() - 1)
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
            _this->vu.insn_end(pending_insn);
        }
    }
}
