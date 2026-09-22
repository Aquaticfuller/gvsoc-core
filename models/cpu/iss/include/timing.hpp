/*
 * Copyright (C) 2020 GreenWaves Technologies, SAS, ETH Zurich and
 *                    University of Bologna
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
 * Authors: Germain Haugou, GreenWaves Technologies (germain.haugou@greenwaves-technologies.com)
 */

#pragma once

#include <vp/vp.hpp>
#include "probe/perf_probe.hpp"
#include <cpu/iss/include/types.hpp>

class Timing
{
public:
    Timing(Iss &iss);
    void build();
    inline void stall_fetch_account(int count);
    inline void stall_taken_branch_account();
    inline void stall_insn_account(int cycles);
    inline void stall_insn_dependency_account(int latency);
    inline void stall_load_dependency_account(int latency);
    inline void stall_jump_account();
    inline void stall_misaligned_account();
    inline void stall_load_account(int cycles);
    inline void insn_account();
    inline void insn_stall_account();
    inline void cycle_account();
    inline void insn_stall_start();
    inline void insn_stall_stop();

    inline void event_load_account(int incr);
    inline void event_rvc_account(int incr);
    inline void event_store_account(int incr);
    inline void event_branch_account();
    inline void event_taken_branch_account(int incr);
    inline void event_jump_account(int incr);
    inline void event_misaligned_account(int incr);
    inline void event_apu_contention_account(int incr);
    inline void event_load_load_account(int incr);
    // No-op shim — iss_v2 cores override this on their Events class to
    // model operand-dependent divider latency. iss v1 keeps the cycle
    // count via stall_insn_dependency_account elsewhere in the div
    // handlers, so the hook is just a placeholder here for the shared
    // rv32m.hpp.
    inline void event_div_account(iss_reg_t dividend, iss_reg_t divisor,
                                  bool is_signed, bool is_rem) {}

    inline void event_trace_account(unsigned int event, int cycles);
    inline void event_trace_set(unsigned int event);
    inline void event_trace_reset(unsigned int event);
    inline int event_trace_is_active(unsigned int event);

    inline void stall_cycles_account(int incr);

    inline void event_account(unsigned int event, int incr);
    inline void handle_pending_events();

    void reset(bool active);

    vp::ClockEvent *ipc_clock_event;
    vp::Trace state_event;
    vp::Trace pc_trace_event;
    vp::Trace active_pc_trace_event;
    vp::Trace func_trace_event;
    vp::Trace inline_trace_event;
    vp::Trace line_trace_event;
    vp::Trace file_trace_event;
    vp::Trace user_func_trace_event;
    vp::Trace user_inline_trace_event;
    vp::Trace user_line_trace_event;
    vp::Trace user_file_trace_event;
    vp::Trace binaries_trace_event;
    vp::Trace pcer_trace_event[32];
    int64_t pcer_trace_pending_cycles[32];
    vp::Trace insn_trace_event;
    vp::WireMaster<uint32_t> ext_counter[32];
    std::vector<vp::PowerSource> insn_groups_power;
    vp::PowerSource power_stall_first;
    vp::PowerSource power_stall_next;
    vp::PowerSource background_power;
    uint32_t pcer_trace_active_events;

    // ---- perf-probe counters (prompt/perf_probe_design.md §4.1) ----
    // Always on and cumulative; the perf_probe collector reads them through IssWrapper. Unlike the
    // pcer counters above they do not depend on software enabling PCMR, and they never touch
    // timing: every hook is a bare increment.
    enum probe_stall_e
    {
        PROBE_STALL_MEM = 0,    // waiting for a scalar load/store response (or a denied request)
        PROBE_STALL_FETCH,      // instruction fetch
        PROBE_STALL_DEP,        // load-use / scoreboard dependency (modelled cycles + retries)
        PROBE_STALL_FPU,        // FPU sequencer back-pressure
        PROBE_STALL_VGRANT,     // retry cycles waiting for the shared-Spatz grant (multi-scalar)
        PROBE_STALL_VQFULL,     // retry cycles because the Spatz queue is full
        PROBE_STALL_BARRIER,    // parked in the hardware barrier
        PROBE_STALL_WFI,        // wfi
        PROBE_STALL_OTHER,      // fetch-enable low, misaligned, jumps, ...
        PROBE_STALL_NB,
    };
    uint64_t probe_invocations = 0;   // instruction handler invocations (one per insn_account())
    uint64_t probe_retries = 0;       // invocations that returned the same pc to be re-executed
    uint64_t probe_ld = 0, probe_st = 0, probe_branch = 0, probe_taken = 0, probe_jump = 0;
    uint64_t probe_amo = 0, probe_fpu_off = 0, probe_vec_issue = 0;
    uint64_t probe_stall[PROBE_STALL_NB] = {0};
    // Reason charged when the next stalled_inc() opens an interval; a stall site sets it right
    // before calling insn_stall()/stalled_inc(), and it resets to MEM when the interval closes.
    int probe_stall_reason = PROBE_STALL_MEM;
    int64_t probe_stall_start = -1;
    probe::Occupancy probe_lsu_occ;   // Σ(outstanding scalar requests · cycles)
    inline void probe_stall_open(int64_t now)
    {
        if (this->probe_stall_start < 0) this->probe_stall_start = now;
    }
    inline void probe_stall_close(int64_t now)
    {
        if (this->probe_stall_start >= 0)
        {
            if (now > this->probe_stall_start)
                this->probe_stall[this->probe_stall_reason] += (uint64_t)(now - this->probe_stall_start);
            this->probe_stall_start = -1;
        }
        this->probe_stall_reason = PROBE_STALL_MEM;
    }

private:

    Iss &iss;
    bool declare_binaries = true;
};
