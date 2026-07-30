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
 * Authors: Germain Haugou (germain.haugou@gmail.com)
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

    this->ports.resize(nb_ports);
    for (int i=0; i<nb_ports; i++)
    {
        // Callback
        this->ports[i].set_resp_meth(&VuLsu::data_response);
        this->ports[i].set_grant_meth(&VuLsu::data_grant);
        iss.new_master_port("vlsu_" + std::to_string(i), &this->ports[i], this);
    }

    int nb_outstanding_reqs = iss.get_js_config()->get_child_int("vu/nb_outstanding_reqs");

    this->rob.resize(nb_ports);
    this->rob_next.resize(nb_ports);
    this->rob_first.resize(nb_ports);
    this->rob_count.resize(nb_ports);
    this->port_burst.resize(nb_ports);
    this->port_stalled.resize(nb_ports);
    for (int i=0; i<nb_ports; i++)
    {
        this->rob[i].resize(nb_outstanding_reqs);
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

    VuLsuPendingInsn &slot = this->insns[this->insn_first_waiting];
    // Store bursts commit one by one on their response: they are not tracked in the
    // ROB and so not committed by group.
    slot.nb_remaining_bursts = is_write ?
        0 : (this->pending_size + this->burst_size - 1) / this->burst_size;

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

void VuLsu::data_grant(vp::Block *__this, vp::IoReq *req)
{
    VuLsu *_this = (VuLsu *)__this;

    uint64_t size = req->get_size();
    // A store is described by its request, a load by the ROB entry tracking it
    VlsuReq *vlsu_req = req->get_is_write() ?
        (VlsuReq *)req->get_args()[0] : ((VlsuRobEntry *)req->get_args()[0])->req;
    VuLsuPendingInsn *slot = vlsu_req->slot;
    int port = vlsu_req->port;

    _this->port_burst[port]++;
    _this->remaining_size -= size;

    if (_this->remaining_size == 0)
    {
        PendingInsn *pending_insn = slot->insn;

        // Keep the last bus operation time to model the gap before the next one.
        _this->op_timestamp = _this->vu.iss.clock.get_cycles() + 1;

        // Keep one extra cycle before retirement after the last burst is issued.
        pending_insn->timestamp = pending_insn->timestamp + 1;

        // Mark the instruction done once all bursts have been issued.
        slot->done = true;
    }

     // The last burst which stalled the port has just been granted. Release the port.
    _this->port_stalled[port] = false;
}


void VuLsu::data_response(vp::Block *__this, vp::IoReq *req)
{
    VuLsu *_this = (VuLsu *)__this;

    if (req->get_is_write())
    {
        // Stores allocate no ROB entry: on RTL the store ROB entry is already
        // freed once the request enters the memory-request spill register. The
        // burst is committed and accounted here, at the write response, matching
        // the RTL store_count which drains when the store response comes back.
        VlsuReq *store_req = (VlsuReq *)req->get_args()[0];
        VuLsuPendingInsn *slot = store_req->slot;
        PendingInsn *pending_insn = slot->insn;
        iss_insn_t *insn = _this->vu.iss.exec.get_insn(pending_insn->entry);

        _this->vu.insn_commit(pending_insn, (int)req->get_size());
        _this->vu.exec_insn_chunk(insn, pending_insn, store_req->vstart,
            store_req->vstart + store_req->nb_elem, store_req->nb_elem);
        slot->nb_pending_bursts--;

        _this->trace.msg("Retiring store request (req: %p, pending insn bursts: %d)\n",
            req, slot->nb_pending_bursts);

        req->arg_free(req->arg_current_index());
        _this->reqs_free.push_back(store_req);
        return;
    }

    auto *rob_entry = (VlsuRobEntry *)req->get_args()[0];

    if (rob_entry == nullptr || rob_entry->port < 0 || rob_entry->port >= _this->nb_ports ||
        rob_entry->rob_id < 0 || rob_entry->rob_id >= (int)_this->rob[rob_entry->port].size())
    {
        _this->trace.fatal("Invalid VLSU response context (req: %p, rob_entry: %p)\n",
            req, rob_entry);
    }

    // We just received the response to one of the requests, set valid=True
    _this->rob[rob_entry->port][rob_entry->rob_id].valid = true;

    _this->trace.msg("Received data response (req: %p)\n", req);

    // Free any arguments associated with the request, to avoid memory leaks. 
    // This is important because the request object is reused for future requests.
    int req_nb_args = req->arg_current_index();
    if (req_nb_args > 0)
    {
        req->arg_free(req_nb_args);
    }

    // This response may have completed a group the register file is waiting for, on
    // this instruction or on a younger one. An instruction reaches the ROB heads only
    // once the older ones committed everything, so walk from the oldest.
    for (int i = 0; i < _this->nb_pending_insn.get(); i++)
    {
        VuLsuPendingInsn &slot = _this->insns[(_this->insn_first + i) % VuLsu::queue_size];

        while (slot.nb_remaining_bursts > 0)
        {
            // Burst k goes on port k % nb_ports and each port retires in order, so the
            // next group sits at the head of the first ports. Like RTL, it is committed
            // only once every burst it contains has its response.
            int group_size = std::min(slot.nb_remaining_bursts, _this->nb_ports);
            int nb_ready = 0;
            while (nb_ready < group_size)
            {
                VlsuRobEntry &entry = _this->rob[nb_ready][_this->rob_first[nb_ready]];
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
                VlsuRobEntry &entry = _this->rob[j][_this->rob_first[j]];
                VlsuReq *load_req = entry.req;

                _this->vu.exec_insn_chunk(insn, pending_insn, load_req->vstart,
                    load_req->vstart + load_req->nb_elem, load_req->nb_elem);
                committed_size += load_req->req.get_size();

                entry.allocated = false;
                slot.nb_pending_bursts--;
                _this->reqs_free.push_back(load_req);
                _this->rob_count[j]--;
                _this->rob_first[j] = (_this->rob_first[j] + 1) % _this->rob[j].size();
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

void VuLsu::fsm_handler(vp::Block *__this, vp::ClockEvent *event)
{
    VuLsu *_this = (VuLsu *)__this;

    // Check if any synchronous delayed burst need to be terminated
    while (!_this->delayed_bursts.empty() &&
            _this->delayed_bursts.top().timestamp <= _this->vu.iss.clock.get_cycles())
    {
        vp::IoReq *req = _this->delayed_bursts.top().req;
        _this->data_response(_this, req);
        _this->delayed_bursts.pop();
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

    // Check if the first waiting instruction can be started:
    // - if the current on-going instruction finished issuing all requests i.e. _this->remaining_size == 0;
    // - if the first waiting instruction past its enqueue cycle,
    // - and is ready dependency-wise.
    if (_this->nb_waiting_insn > 0 && _this->remaining_size == 0)
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

    if (_this->remaining_size && _this->op_timestamp <= _this->vu.iss.clock.get_cycles())
    {
        VuLsuPendingInsn &slot = _this->insns[_this->insn_ongoing];
        PendingInsn *pending_insn = slot.insn;

        // If the on-going instruction is ready and its instruction latency has elapsed,
        // try to send requests to available ports. A store reads the register file once
        // per burst, so it is checked burst by burst, at the elements that burst reads.
        if (pending_insn->timestamp <= _this->vu.iss.clock.get_cycles() &&
            (_this->pending_is_write || _this->vu.insn_ready(pending_insn)))
        {
            iss_insn_t *insn = _this->vu.iss.exec.get_insn(pending_insn->entry);

            if (!_this->started)
            {
                _this->started = true;
                _this->event_label.dump(insn->desc->label);
                _this->event_pc.event((uint8_t *)&insn->addr);
            }

            for (int i = 0; i < _this->nb_ports; i++)
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

                    uint32_t req_idx = _this->port_burst[i] * _this->nb_ports + i;
                    uint64_t req_offset = req_idx * _this->burst_size;
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

                    vp::IoReq *req = &vlsu_req->req;
                    uint8_t *req_data = velem;
                    req->init();

                    if (_this->pending_is_write)
                    {
                        // The vector register file is read now. Send a copy of the
                        // elements since the request is only applied when it reaches
                        // the target, when the register may have been overwritten.
                        vlsu_req->data.assign(velem, velem + size);
                        req_data = vlsu_req->data.data();

                        req->arg_push((void *)vlsu_req);
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

                        req->arg_push((void *)&rob_entry);
                    }

                    req->set_addr(addr);
                    req->set_is_write(_this->pending_is_write);
                    req->set_size(size);
                    req->set_data(req_data);
                    slot.nb_pending_bursts++;

                    vp::IoReqStatus err = _this->ports[i].req(req);

                    if (err == vp::IO_REQ_INVALID)
                    {
                        _this->trace.fatal("Invalid request (req: %p, addr: 0x%lx, size: 0x%lx, is_write: %d)\n",
                            req, addr, size, _this->pending_is_write);
                    }
                    else if (err == vp::IO_REQ_OK)
                    {
                        int64_t latency = req->get_full_latency();
                        if (latency == 0)
                        {
                            _this->data_response(_this, req);
                        }
                        else
                        {
                            // The request is synchronous, use a priority queue model the synchronous response delay.
                            uint64_t response_timestamp = _this->vu.iss.clock.get_cycles() + latency;
                            _this->delayed_bursts.push({req, response_timestamp});
                            pending_insn->timestamp = std::max(pending_insn->timestamp, response_timestamp);
                        }
                    }
                    else if (err == vp::IO_REQ_DENIED)
                    {
                        _this->port_stalled[i] = true;
                    }
                    // Prepare the next burst if not DENIED
                    if (err != vp::IO_REQ_DENIED)
                    {
                        _this->port_burst[i]++;
                        _this->remaining_size -= size;
                        // Switch to next instruction once all burst have been sent
                        if (_this->remaining_size == 0)
                        {
                            // Keep the last bus operation time to model the gap before the next one.
                            _this->op_timestamp = _this->vu.iss.clock.get_cycles() + 1;

                            // Keep one extra cycle before retirement after the last burst is issued.
                            pending_insn->timestamp = pending_insn->timestamp + 1;

                            // Mark the instruction done once all bursts have been issued.
                            slot.done = true;
                        }
                    }
                }
            }
        }
    }

    // Check if the first enqueued instruction must be removed.
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
