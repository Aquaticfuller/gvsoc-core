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

#include "cpu/iss/include/iss.hpp"
#include "cpu/iss/include/cores/ara/ara.hpp"
#include <cstdio>
#include <cstring>

AraVlsu::AraVlsu(Ara &ara, IssWrapper &top)
: AraBlock(&ara, "vlsu"), ara(ara),
nb_pending_insn(*this, "nb_pending_insn", 8, true),
fsm_event(this, &AraVlsu::fsm_handler)
{
    this->traces.new_trace("trace", &this->trace, vp::DEBUG);
    this->traces.new_trace_event("active", &this->event_active, 1);
    this->traces.new_trace_event("pc", &this->event_pc, 64);
    this->traces.new_trace_event("queue", &this->event_queue, 64);
    this->traces.new_trace_event_string("label", &this->event_label);

    this->insns.resize(AraVlsu::queue_size);

    int nb_ports = top.get_js_config()->get_child_int("vu/nb_ports");
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
        top.new_master_port("vlsu_" + std::to_string(i), &this->ports[i], this);
        this->ports[i].set_resp_meth(&AraVlsu::data_response);
    }

    this->width = top.get_js_config()->get_child_int("vu/lsu_width");

    int nb_outstanding_reqs = top.get_js_config()->get_child_int("vu/nb_outstanding_reqs");
    this->req_queues.resize(nb_ports);
    for (int i=0; i<nb_ports; i++)
    {
        this->req_queues[i] = new vp::Queue(this, "port_" + std::to_string(i) + "_reqs");
    }

    this->requests.resize(nb_ports * nb_outstanding_reqs);
}

void AraVlsu::start()
{
}

void AraVlsu::data_response(vp::Block *__this, vp::IoReq *req)
{
    AraVlsu *_this = (AraVlsu *)__this;
    // Args were pushed in order: slot, port_id, vreg, size — pop in reverse.
    // vreg/size are only meaningful for the async (IO_REQ_PENDING/DENIED) path: the
    // synchronous path commits immediately at issue time and pops (and discards) these
    // same four slots right there instead of leaving them for this callback.
    int size = (int)(uintptr_t)req->arg_pop();
    int vreg = (int)(uintptr_t)req->arg_pop();
    int port_id = (int)(uintptr_t)req->arg_pop();
    AraVlsuPendingInsn *slot = (AraVlsuPendingInsn *)req->arg_pop();
    _this->req_queues[port_id]->push_back(req);
    slot->nb_pending_bursts--;
    // Only now, once the data has actually landed, tell the scoreboard the elements are
    // committed. Committing at issue time (as used to happen) is only safe when the
    // interconnect completes synchronously; CachePool's NUMA/cache paths routinely return
    // IO_REQ_PENDING/DENIED for remote-bank accesses, so a chained consumer instruction
    // could previously start reading a vector register before this burst's data was
    // actually written, silently consuming stale/zero elements.
    _this->ara.insn_commit(vreg, size);
    // Re-enable the FSM so it can issue the next burst or commit the instruction.
    _this->fsm_event.enable();
}

void AraVlsu::reset(bool active)
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

        // Since the request queues are cleared with the reset, we need to put back requests
        // in each queue
        int nb_ports = this->ara.iss.top.get_js_config()->get_child_int("vu/nb_ports");
        int nb_outstanding_reqs = this->ara.iss.top.get_js_config()->get_child_int("vu/nb_outstanding_reqs");
        int req_id = 0;
        for (int i=0; i<nb_ports; i++)
        {
            for (int j=0; j<nb_outstanding_reqs; j++)
            {
                this->req_queues[i]->push_back(&this->requests[req_id++]);
            }
        }
    }
}

void AraVlsu::enqueue_insn(PendingInsn *pending_insn)
{
    this->trace.msg(vp::Trace::LEVEL_TRACE, "Enqueue instruction (pc: 0x%lx)\n", pending_insn->pc);
    uint8_t one = 1;
    this->event_active.event(&one);
    this->event_queue.event((uint8_t *)&pending_insn->pc);

    // Just push the instruction and let the FSM handle it if needed.
    // A delay is added to take into account the time needed on RTL to start the instruction
    pending_insn->timestamp = this->ara.iss.top.clock.get_cycles() + 5;
    AraVlsuPendingInsn &slot = this->insns[this->insn_last];
    this->insn_last = (this->insn_last + 1) % AraVlsu::queue_size;
    slot.insn = pending_insn;
    slot.nb_pending_bursts = 0;
    this->nb_pending_insn.inc(1);
    this->nb_waiting_insn++;

    this->fsm_event.enable();
}

void AraVlsu::isa_init()
{
    // Attach handlers to instructions so that we can quickly handle load and stores differently
    for (iss_decoder_item_t *insn: *this->ara.iss.decode.get_insns_from_tag("vload"))
    {
        insn->u.insn.block_handler = (void *)&AraVlsu::handle_insn_load;
    }
    for (iss_decoder_item_t *insn: *this->ara.iss.decode.get_insns_from_tag("vstore"))
    {
        insn->u.insn.block_handler = (void *)&AraVlsu::handle_insn_store;
    }
    for (iss_decoder_item_t *insn: *this->ara.iss.decode.get_insns_from_tag("vload_strided"))
    {
        insn->u.insn.block_handler = (void *)&AraVlsu::handle_insn_load_strided;
    }
    for (iss_decoder_item_t *insn: *this->ara.iss.decode.get_insns_from_tag("vstore_strided"))
    {
        insn->u.insn.block_handler = (void *)&AraVlsu::handle_insn_store_strided;
    }
    for (iss_decoder_item_t *insn: *this->ara.iss.decode.get_insns_from_tag("vload_indexed"))
    {
        insn->u.insn.block_handler = (void *)&AraVlsu::handle_insn_load_indexed;
    }
    for (iss_decoder_item_t *insn: *this->ara.iss.decode.get_insns_from_tag("vstore_indexed"))
    {
        insn->u.insn.block_handler = (void *)&AraVlsu::handle_insn_store_indexed;
    }
}

void AraVlsu::handle_insn_load_strided(AraVlsu *_this, iss_insn_t *insn)
{
    _this->handle_access(insn, false, insn->out_regs[0], true, _this->insns[_this->insn_first_waiting].insn->reg_2);
}

void AraVlsu::handle_insn_store_strided(AraVlsu *_this, iss_insn_t *insn)
{
    _this->handle_access(insn, true, insn->in_regs[1], true, _this->insns[_this->insn_first_waiting].insn->reg_3);
}

void AraVlsu::handle_insn_load_indexed(AraVlsu *_this, iss_insn_t *insn)
{
    _this->handle_access(insn, false, insn->out_regs[0], false, 0, insn->in_regs[1]);
}

void AraVlsu::handle_insn_store_indexed(AraVlsu *_this, iss_insn_t *insn)
{
    _this->handle_access(insn, true, insn->in_regs[1], false, 0, insn->in_regs[2]);
}

void AraVlsu::handle_access(iss_insn_t *insn, bool is_write, int reg, bool do_stride, iss_reg_t stride, int reg_indexed)
{
    // A load or store instruction is starting, just store information about the first burst and let
    // the FSM handle all the bursts.
    unsigned int sewb = this->ara.iss.vector.sewb;
    unsigned int lmul = this->ara.iss.vector.lmul;
    this->pending_vreg = reg;
    this->pending_velem = velem_get(&this->ara.iss, reg, 0, sewb, lmul);
    this->pending_addr = this->insns[this->insn_first_waiting].insn->reg;
    this->pending_is_write = is_write;
    int inst_elem_size = insn->uim[1] >= 5 ? 1 << (insn->uim[1] - 4) : 1 << 0;
    int elem_size = reg_indexed != -1 ? sewb : inst_elem_size;
    this->pending_size = (this->ara.iss.csr.vl.value - this->ara.iss.csr.vstart.value) * elem_size;
    this->stride = stride;
    this->strided = do_stride;
    this->elem_size = elem_size;
    this->inst_elem_size = inst_elem_size;
    this->reg_indexed = reg_indexed;
    this->pending_elem = 0;
}

void AraVlsu::handle_insn_load(AraVlsu *_this, iss_insn_t *insn)
{
    _this->handle_access(insn, false, insn->out_regs[0]);
}

void AraVlsu::handle_insn_store(AraVlsu *_this, iss_insn_t *insn)
{
    _this->handle_access(insn, true, insn->in_regs[1]);
}

void AraVlsu::fsm_handler(vp::Block *__this, vp::ClockEvent *event)
{
    AraVlsu *_this = (AraVlsu *)__this;

    // Check if any synchronous delayed burst has reached its commit timestamp — drain ALL of them
    // through the same completion path the async response uses (pop args, return the req to the port
    // queue, decrement in-flight, commit the elements). Draining all eligible per firing (not one) is
    // required: the VLSU issues up to nb_ports bursts/cycle, so a 1/cycle drain would cap commit
    // throughput and artificially serialize memory-bound streams.
    while (!_this->delayed_bursts.empty() &&
           _this->delayed_bursts_timestamps.front() <= _this->ara.iss.top.clock.get_cycles())
    {
        data_response(_this, _this->delayed_bursts.front());
        _this->delayed_bursts.pop();
        _this->delayed_bursts_timestamps.pop();
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
        _this->event_label.event_string((char *)1, false);

        _this->fsm_event.disable();
    }

    // Check if the first waiting instruction can be started
    if (_this->nb_waiting_insn > 0)
    {
        AraVlsuPendingInsn &slot = _this->insns[_this->insn_first_waiting];
        PendingInsn *pending_insn = slot.insn;

        if (pending_insn->timestamp <=_this->ara.iss.top.clock.get_cycles() &&
            _this->pending_size == 0)
        {
            _this->event_label.event_string(pending_insn->insn->desc->label, false);
            _this->event_pc.event((uint8_t *)&pending_insn->pc);

            iss_insn_t *insn = pending_insn->insn;
            ((void (*)(AraVlsu *, iss_insn_t *))insn->decoder_item->u.insn.block_handler)(_this, insn);
        }
    }

    if (_this->pending_size)
    {
        // If a pending request is ready, try to send requests to available ports
        for (int i=0; i<_this->ports.size(); i++)
        {
            if (_this->pending_size == 0) break;

            // Only use this port if it has requests available otherwise it means too many
            // are pending
            if (!_this->req_queues[i]->empty())
            {
                uint64_t size;

                if (_this->strided ||  _this->reg_indexed != -1)
                {
                    size = _this->elem_size;
                }
                else
                {
                    size = std::min((iss_addr_t)_this->width, _this->pending_size);
                }

                _this->trace.msg(vp::Trace::LEVEL_TRACE,
                    "Sending request (port: %d, addr: 0x%lx, size: 0x%lx, pending_size: 0x%lx, is_write: %d)\n",
                    i, _this->pending_addr, size, _this->pending_size, _this->pending_is_write);

                _this->event_addr[i].event((uint8_t *)&_this->pending_addr);
                _this->event_size[i].event((uint8_t *)&size);
                _this->event_is_write[i].event((uint8_t *)&_this->pending_is_write);

                /// Pop a request from this port queue to limit number of outstanding requests
                vp::IoReq *req = (vp::IoReq *)_this->req_queues[i]->pop();

                req->prepare();

                iss_reg_t addr = _this->pending_addr;

                if (_this->reg_indexed != -1)
                {
                    uint64_t offset = velem_get_value(&_this->ara.iss, _this->reg_indexed, _this->pending_elem,
                        _this->inst_elem_size, _this->ara.iss.vector.lmul);
                    addr += offset;
                    _this->pending_elem++;
                }

                req->set_addr(addr);
                req->set_is_write(_this->pending_is_write);
                req->set_size(size);

                req->set_data(_this->pending_velem);

                // Push slot pointer, port index, target vreg and burst size onto the
                // request arg stack so that data_response can find them for async
                // (IO_REQ_PENDING/DENIED) completions and commit the scoreboard only once
                // the data has actually arrived (see data_response for why).
                AraVlsuPendingInsn &slot = _this->insns[_this->insn_first_waiting];
                req->arg_push((void *)&slot);
                req->arg_push((void *)(uintptr_t)i);
                req->arg_push((void *)(uintptr_t)_this->pending_vreg);
                req->arg_push((void *)(uintptr_t)size);

                vp::IoReqStatus err = _this->ports[i].req(req);

                if (err == vp::IO_REQ_OK)
                {
                    const int64_t latency = req->get_full_latency();
                    if (latency > 0)
                    {
                        // Sync completion carrying a real latency (the InSitu-cache path stamps
                        // hit/miss latency on OK). Commit at (now + latency) instead of at issue:
                        // hold the burst in delayed_bursts and commit it in fsm_handler when its
                        // timestamp elapses — exactly like the async PENDING path but completing on a
                        // timer. This makes vector traffic actually CONSUME the calibrated cache
                        // latency (scoreboard/vreg writeback + ROB backpressure at the right time),
                        // instead of every vector access being ~0-cycle. Args stay on the req for
                        // data_response to pop at drain.
                        slot.nb_pending_bursts++;
                        _this->delayed_bursts.push(req);
                        _this->delayed_bursts_timestamps.push(
                            _this->ara.iss.top.clock.get_cycles() + latency);
                    }
                    else
                    {
                        // Sync: data is already valid, commit now. Pop our args (in reverse
                        // push order) and return the request to the port queue.
                        (void)req->arg_pop();  // size
                        (void)req->arg_pop();  // vreg
                        (void)req->arg_pop();  // port_id
                        (void)req->arg_pop();  // slot
                        _this->req_queues[i]->push_back(req);
                        _this->ara.insn_commit(_this->pending_vreg, size);
                    }
                }
                else if (err == vp::IO_REQ_PENDING)
                {
                    // Async: data_response callback will push req back and decrement
                    // nb_pending_bursts once the response arrives from the interconnect.
                    slot.nb_pending_bursts++;
                }
                else if (err == vp::IO_REQ_DENIED)
                {
                    // The NI queued the req in its denied list; it will process it when
                    // ready and call response via the Router's response callback.
                    // Treat exactly like PENDING: count it as in-flight and advance
                    // address so remaining ports issue at their correct addresses.
                    // The synchronous Router does not propagate grants so we don't
                    // rely on a grant callback; the response will close the burst.
                    slot.nb_pending_bursts++;
                }
                else
                {
                    _this->trace.fatal("Unsupported IO response status: %d", (int)err);
                }

                // Prepare the next burst (for IO_REQ_OK, IO_REQ_PENDING, IO_REQ_DENIED).
                if (_this->reg_indexed == -1)
                {
                    _this->pending_addr += _this->strided ? _this->stride : size;
                }
                _this->pending_size -= size;
                _this->pending_velem += size;

                // Switch to next instruction once all burst have been sent
                if (_this->pending_size == 0)
                {
                    _this->nb_waiting_insn--;
                    _this->insn_first_waiting = (_this->insn_first_waiting + 1) % AraVlsu::queue_size;
                }
            }
        }
    }

    // Check if the first enqueued instruction must be removed. This must run every cycle,
    // NOT only while _this->pending_size != 0 (i.e. while some other, newer instruction
    // happens to be mid-issue): the head instruction's own bursts complete asynchronously
    // via data_response(), which can happen well after this instruction (and every
    // instruction issued after it) has finished ISSUING all its bursts — at which point
    // pending_size is already back to 0 and this check would otherwise never run again,
    // permanently stalling Ara's global instruction queue even though AraVlsu itself has
    // nothing left outstanding. (Confirmed via [VLSU_FSM_DBG]/[ARA_DBG]: AraVlsu reaches
    // nb_waiting_insn=0, pending_size=0 while nb_pending_insn stays stuck at 1 forever, with
    // Ara's global queue full and its head instruction never marked done.)
    if (_this->nb_pending_insn.get() > 0)
    {
        AraVlsuPendingInsn &slot = _this->insns[_this->insn_first];
        PendingInsn *pending_insn = slot.insn;
        if (_this->pending_size == 0 && slot.nb_pending_bursts == 0 &&
            pending_insn->timestamp <= _this->ara.iss.top.clock.get_cycles())
        {
            _this->insn_first = (_this->insn_first + 1) % AraVlsu::queue_size;
            _this->nb_pending_insn.dec(1);
            _this->ara.insn_end(pending_insn);
        }
    }
}
