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

#include "cpu/iss_v2/include/iss.hpp"
#include "cpu/iss_v2/include/cores/vector_unit/vector_unit.hpp"

VuCompute::VuCompute(Vu &vu, std::string name)
: VuBlock(&vu, name), vu(vu),
fsm_event(this, &VuCompute::fsm_handler)
{
    // Absent from the config both read back 0, i.e. the lane-wide reduction.
    this->reduction_is_serial =
        vu.iss.get_js_config()->get_int("vu/reduction_is_serial") != 0;
    this->reduction_step_latency = std::max<int>(1,
        vu.iss.get_js_config()->get_int("vu/reduction_step_latency"));

    this->traces.new_trace("trace", &this->trace, vp::DEBUG);
    this->traces.new_trace_event("active", &this->event_active, 1);
    this->traces.new_trace_event("pc", &this->event_pc, 64);
    this->traces.new_trace_event_string("label", &this->event_label);
}

void VuCompute::reset(bool active)
{
    if (active)
    {
        uint8_t zero = 0;
        this->draining.clear();
        this->event_active.event(&zero);
    }
}

void VuCompute::enqueue_insn(PendingInsn *pending_insn)
{
    iss_insn_t *insn = this->vu.iss.exec.get_insn(pending_insn->entry);
    this->trace.msg(vp::Trace::LEVEL_TRACE, "Enqueue instruction (pc: 0x%lx, id: %d)\n",
        insn->addr, pending_insn->id);
    uint8_t one = 1;
    this->event_active.event(&one);

    // Just push the instruction and let the FSM handle it if needed.
    // It is marked for execution in the next cycle so that the FSM does not handle it in this
    // cycle in case the FSM is already active
    pending_insn->timestamp++;
    this->insns.push(pending_insn);
    this->fsm_event.enable();
}

void VuCompute::fsm_handler(vp::Block *__this, vp::ClockEvent *event)
{
    VuCompute *_this = (VuCompute *)__this;

    // Release the instructions which finished draining through the FPU
    // pipeline (plus their completion latency). Their scoreboard entry is
    // released here; they did not block the issue of the instructions
    // behind them.
    for (auto it = _this->draining.begin(); it != _this->draining.end();)
    {
        if ((*it)->timestamp <= _this->vu.iss.clock.get_cycles())
        {
            // Notify vu so that it removes it from the pending instructions and updates the
            // scoreboard
            _this->vu.insn_end(*it);
            it = _this->draining.erase(it);
            if (_this->draining.empty() && _this->insns.empty())
            {
                _this->event_pc.event_highz();
                _this->event_label.event_string((char *)1, false);
            }
        }
        else
        {
            ++it;
        }
    }

    // Check if a new instruction can starts.
    // Take the first one from the queue and see if its timestamp has passed. This was set during
    // enqueue to make sure the instruction starts at best the cycle after.
    if (_this->insns.size() != 0 &&
        _this->insns.front()->timestamp <= _this->vu.iss.clock.get_cycles())
    {
        PendingInsn *pending_insn = _this->insns.front();
        iss_insn_t *insn = _this->vu.iss.exec.get_insn(pending_insn->entry);
        bool ready= _this->vu.insn_ready(pending_insn);

        _this->trace.msg(vp::Trace::LEVEL_TRACE, "Check ready (pc: 0x%lx, id: %d, ready: %d)\n",
            insn->addr, pending_insn->id, ready);

        if (ready)
        {
            // Widening/narrowing instructions process elements at half the
            // nominal rate (elem_rate_shift = 1): the RTL VFU consumes the
            // operand word over two cycles for widening and halves
            // nr_elem_word for narrowing.
            int nb_elem_per_cycle = (_this->vu.nb_lanes * _this->vu.lane_width /
                _this->vu.iss.vector.sewb) >> insn->desc->elem_rate_shift;

            // A serially-reducing unit feeds the accumulator back from the FPU
            // result, so it retires one element per round trip instead of a
            // lane-wide chunk per cycle.
            bool serial_reduction = pending_insn->is_reduction &&
                _this->reduction_is_serial;
            if (serial_reduction)
            {
                nb_elem_per_cycle = 1;
            }

            if (pending_insn->nb_bytes_done == 0)
            {
                _this->event_pc.event((uint8_t *)&insn->addr);
                _this->event_label.event_string(insn->desc->label, false);

#ifdef CONFIG_GVSOC_STATS_ACTIVE
                // Real execution starts now (first chunk); stamp it for the
                // per-label duration accounted at Vu::insn_end.
                if (_this->vu.stats_enabled && pending_insn->exec_start_cycle < 0)
                {
                    pending_insn->exec_start_cycle = _this->vu.iss.clock.get_cycles();
                }
#endif

                unsigned int nb_elems = _this->vu.iss.csr.vl.value - _this->vu.iss.csr.vstart.value;
                _this->total_size = nb_elems * _this->vu.iss.vector.sewb;
                _this->vstart = _this->vu.iss.csr.vstart.value;
                _this->vend = std::min((int)_this->vu.iss.csr.vl.value,
                    _this->vstart + nb_elem_per_cycle);
            }

            _this->trace.msg(vp::Trace::LEVEL_TRACE, "Exec chunk (pc: 0x%lx, id: %d, start: %d, end: %d)\n",
                insn->addr, pending_insn->id, _this->vstart, _this->vend);

            // Now that the instruction is over, execute the handler to functionally model it. This will
            // write the output register.
            // Store the instruction register as it will be used by the handler.
            _this->vu.current_insn_reg = pending_insn->reg;
            _this->vu.current_insn_reg_2 = pending_insn->reg_2;

            _this->vu.insn_latency = 0;
            _this->vu.exec_insn_chunk(insn, pending_insn, _this->vstart, _this->vend, nb_elem_per_cycle);
            // Each reduction step is an FPU round trip, so the accumulator
            // element only becomes available after the pipeline latency. This
            // overrides the flat cost Vu::insn_handle_reduction leaves in
            // insn_latency, which is not a per-element rate.
            if (serial_reduction)
            {
                // One FPU round trip per element. The generic path below spaces
                // chunks by latency-1, so set the interval directly here.
                pending_insn->timestamp = _this->vu.iss.clock.get_cycles() +
                    _this->reduction_step_latency;
            }
            else if (_this->vu.insn_latency > 0)
            {
                pending_insn->timestamp = _this->vu.iss.clock.get_cycles() +
                    _this->vu.insn_latency - 1;
            }

            _this->vstart += nb_elem_per_cycle;
            _this->vend = std::min((int)_this->vu.iss.csr.vl.value,
                _this->vstart + nb_elem_per_cycle);

            _this->vu.insn_commit(pending_insn, nb_elem_per_cycle * _this->vu.iss.vector.sewb);

            if (pending_insn->nb_bytes_done >= _this->total_size)
            {
                // All chunks executed: the instruction completes without
                // blocking the issue of the next instruction in the block.
                // The FPU pipeline drain is not added to the scoreboard
                // release: the release also gates WAW/WAR consumers and the
                // queue slot retirement, which the RTL frees at commit time
                // — the drain is only visible to RAW consumers, carried by
                // the chaining gate through pipeline_latency,
                // commit_done_cycle and chain_release_cycle.
                _this->insns.pop();
                pending_insn->commit_done_cycle = _this->vu.iss.clock.get_cycles();
                pending_insn->timestamp = _this->vu.iss.clock.get_cycles() +
                    insn->latency + 1;
                _this->draining.push_back(pending_insn);
            }

        }
    }

    // In case nothing is on-going, disable the FSM
    if (_this->insns.size() == 0 && _this->draining.empty())
    {
        uint8_t zero = 0;
        _this->event_active.event(&zero);
        _this->fsm_event.disable();
    }
}
