// SPDX-FileCopyrightText: 2026 ETH Zurich and University of Bologna
// SPDX-License-Identifier: Apache-2.0
//
// InSitu cache — RTL-faithful flush/sync FSM (structural model, Step 6).
//
// Header-only transcription of the `gen_sync_ctrl_fsm` state machine in
// `insitu_cache_tcdm_wrapper.sv:281-1097`: the 7-state controller that services a `cache_sync`
// instruction by walking every cache set, evicting dirty lines via write-through, and
// clearing/invalidating tags. The structural facts it captures (latency EMERGES from the walk +
// drain, not a knob):
//
//   STATES (wrapper.sv:281-289): IDLE, READ_BANK, INIT, CHECK_PEND, FLUSH, INVALID, FINISH.
//   OPCODES cache_sync_insn (wrapper.sv:119): 0=flush+invalidate, 1=flush only, 2=invalidate only,
//           3=bank initialization.
//
//   FLOW:
//     IDLE       — on cache_sync_valid: latch insn, ptr = cache_part_base (the SPM base set);
//                  insn==3 (init) forces ptr=0. → READ_BANK                       (wrapper.sv:795-804)
//     READ_BANK  — issue a meta read of set ptr; when accepted → CHECK_PEND. ALWAYS routes through
//                  CHECK_PEND so the install pipeline is drained before sync writes. (:806-824)
//     CHECK_PEND — wait for the drain AND (no outstanding refill, no core miss/evic/WT valid, no
//                  preread task, retr-fifo empty, no proc write) to hold for CheckPendDrainCycles=20
//                  CONSECUTIVE cycles; any glitch resets the counter. Then insn==3 → INIT, else
//                  → FLUSH (ptr ← cache_part_base, kick a background read). (:845-927, :563)
//     INIT       — write zeros to set ptr; ptr++; at CacheBankDepth-1 → FINISH.   (:826-843)
//     FLUSH      — read dirty_rf[ptr] (a per-set flop, always fresh): if any way dirty, write-through
//                  evict the lowest dirty way, then on wt_ready mark it INVALID + clear dirty and STAY
//                  on ptr (re-check next cycle handles multi-way dirty); if the whole set is clean,
//                  clear it (status/dirty/tag=0) and ptr++. At CacheBankDepth-1 → FINISH. A background
//                  read of the next set runs in parallel.                          (:929-1075)
//     INVALID    — empty/unreachable in this revision (invalidation is folded into FLUSH's cleanup
//                  meta-write).                                                    (:1077-1078)
//     FINISH     — assert cache_sync_ready (handshake complete); → IDLE.           (:1080-1083)
//
//   sync_block_upstream (wrapper.sv:722-724): ALL upstream requests are gated off whenever the FSM is
//   not in IDLE or FINISH — the cache is busy for the whole walk. (l1d_busy at the cluster level.)
//
// SCOPE (the FSM control flow + the drain interlock + the set-walk + per-dirty-way eviction — the
// structural essential). The PartSplit>1 multi-cycle `flush_full_*` dance (wrapper.sv:982-1033) is
// stubbed (canonical config is PartSplit=1, the `else` branch at :1034-1049). The dirty/clean query
// and the eviction/meta-write handshakes are supplied by the owner (the wrapper/core that holds
// dirty_rf and the write-through master). Validated standalone; folding into the wrapper's tag/dirty
// arrays + write-through master lands with the integrated tile + calibration.

#pragma once
#include <cstdint>

namespace insitu {

enum SyncState {
    SYNC_IDLE = 0,
    SYNC_READ_BANK,
    SYNC_INIT,
    SYNC_CHECK_PEND,
    SYNC_FLUSH,
    SYNC_INVALID,    // unreachable in this RTL revision (kept for enum fidelity)
    SYNC_FINISH
};

// CHECK_PEND additional drain delay (wrapper.sv:563): advance to FLUSH/INIT only after the drain
// conditions hold for this many CONSECUTIVE cycles.
static constexpr uint32_t kCheckPendDrainCycles = 20;

struct SyncInputs {
    bool     sync_valid       = false;  // a cache_sync instruction is presented
    uint8_t  insn             = 0;      // 0=flush+inv, 1=flush, 2=inv, 3=init
    uint32_t cache_part_base  = 0;      // SPM base set (flush starts here; init starts at 0)
    uint32_t cache_bank_depth = 128;    // total sets to walk
    bool     drain_now        = false;  // AND of all drain conditions (CHECK_PEND gate)
    bool     read_ready       = true;   // flush_read_cache_ready (meta read accepted)
    bool     write_ready      = true;   // flush_write_cache_req_ready (meta write accepted)
    bool     wt_ready         = true;   // write_through_ready (eviction accepted)
};

struct SyncOutputs {
    bool     block_upstream = false;    // sync_block_upstream — gate ALL upstream traffic
    bool     ready          = false;    // cache_sync_ready (asserted in FINISH)
    bool     read_valid     = false;    // a meta read is issued this cycle
    uint32_t read_set       = 0;
    bool     evict_valid    = false;    // a dirty line is written back (write-through) this cycle
    uint32_t evict_set      = 0;
    uint32_t evict_way      = 0;
    bool     meta_write     = false;    // a meta write (clear/invalidate) is issued this cycle
    uint32_t meta_set       = 0;
};

class SyncFsm {
public:
    void reset() { state_ = SYNC_IDLE; insn_ = 0; ptr_ = 0; drain_cnt_ = 0; }
    SyncState state() const { return (SyncState)state_; }
    bool busy() const { return state_ != SYNC_IDLE && state_ != SYNC_FINISH; }
    // The current set pointer (sync_ctrl_ptr_q). The owner indexes dirty_rf[ptr()] when supplying
    // dirty_mask_at_ptr to step() — exactly the RTL `dirty_rf[sync_ctrl_ptr_q]` combinational read.
    uint32_t ptr() const { return ptr_; }

    // One clock. `dirty_mask_at_ptr` = the per-way dirty bitmap of dirty_rf[ptr] (the owner reads its
    // flop array). The owner is responsible for actually clearing a dirty bit when evict_valid &&
    // wt_ready, and for performing the meta writes the outputs request.
    void step(const SyncInputs &in, uint32_t dirty_mask_at_ptr, uint32_t num_ways, SyncOutputs &out) {
        out = SyncOutputs{};
        out.block_upstream = busy();

        switch (state_) {
        case SYNC_IDLE:
            if (in.sync_valid) {
                insn_ = in.insn;
                ptr_  = in.cache_part_base;
                if (insn_ == 3) ptr_ = 0;          // bank init walks from set 0
                state_ = SYNC_READ_BANK;
            }
            break;

        case SYNC_READ_BANK:
            out.read_valid = true; out.read_set = ptr_;
            if (in.read_ready) state_ = SYNC_CHECK_PEND;   // always drain via CHECK_PEND
            break;

        case SYNC_CHECK_PEND:
            if (in.drain_now) {
                if (drain_cnt_ < kCheckPendDrainCycles) {
                    drain_cnt_++;
                } else {
                    drain_cnt_ = 0;
                    if (insn_ == 3) {                       // init-all: ptr already 0
                        state_ = SYNC_INIT;
                    } else {
                        state_ = SYNC_FLUSH;
                        ptr_   = in.cache_part_base;
                        out.read_valid = true; out.read_set = ptr_;   // kick background read
                    }
                }
            } else {
                drain_cnt_ = 0;                             // any glitch resets the consecutive count
            }
            break;

        case SYNC_INIT:
            out.meta_write = true; out.meta_set = ptr_;     // write zeros
            if (in.write_ready) {
                if (ptr_ == in.cache_bank_depth - 1) state_ = SYNC_FINISH;
                else ptr_++;
            }
            break;

        case SYNC_FLUSH: {
            // dirty_rf[ptr] is a per-set flop — always fresh for the current ptr (wrapper.sv:957-969).
            bool     has_dirty = false;
            uint32_t dway      = 0;
            for (uint32_t w = 0; w < num_ways; w++)
                if (dirty_mask_at_ptr & (1u << w)) { has_dirty = true; dway = w; break; }

            if (has_dirty) {
                // PartSplit==1 branch (wrapper.sv:1034-1049): evict the dirty way, then on wt_ready
                // mark it INVALID + clear dirty. Stay on ptr — next cycle re-checks dirty_rf (which now
                // shows the cleared way), so multi-way dirty sets drain one way per cycle.
                out.evict_valid = true; out.evict_set = ptr_; out.evict_way = dway;
                if (in.wt_ready) { out.meta_write = true; out.meta_set = ptr_; }
            } else {
                // whole set clean → clear it and advance (wrapper.sv:1050-1067).
                out.meta_write = true; out.meta_set = ptr_;
                if (ptr_ == in.cache_bank_depth - 1) state_ = SYNC_FINISH;
                else { ptr_++; out.read_valid = true; out.read_set = ptr_; }   // background read next set
            }
            break;
        }

        case SYNC_FINISH:
            out.ready = true;
            state_ = SYNC_IDLE;
            break;

        default:
            state_ = SYNC_IDLE;
            break;
        }
    }

private:
    uint32_t state_     = SYNC_IDLE;
    uint8_t  insn_      = 0;
    uint32_t ptr_       = 0;
    uint32_t drain_cnt_ = 0;
};

} // namespace insitu
