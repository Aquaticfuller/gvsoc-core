// SPDX-FileCopyrightText: 2026 ETH Zurich and University of Bologna
// SPDX-License-Identifier: Apache-2.0
//
// InSitu cache — RTL-faithful SRAM bank model (structural model, Step 2).
//
// A faithful, header-only transcription of the bank-conflict timing of:
//   - insitu_cache_tcdm_wrapper.sv :: pseudo_dual_port_tcdm_wrapper (the 6-state R-vs-W FSM)
//   - utilities/pseudo_dual_port_bank.sv (the WR_CONFLICT condition)
//   - utilities/folded_data_bank.sv (one single-port SRAM per way ⇒ ways are independent)
//
// This is the STRUCTURAL replacement for the cycle-approximate `set_busy_until_` cyclestamp:
// it models the real pseudo-dual-port (BankFactor = NumPseudoDualBanks, typically 2) banking,
// where a read and a write in the SAME cycle to the SAME way + SAME bank-select but DIFFERENT
// row collide (WR_CONFLICT) and the READ retries the next cycle (read_ready_o=0,
// tcdm_wrapper.sv:2259-2264) — the 1-cycle bank-conflict penalty. Different bank-selects
// (WR_DIFF_BANK) run 1R+1W concurrently; same row (WR_SAME_ADDR) forwards from the write buffer.
//
// Functional line DATA lives in the cache core's data array; this model owns only the
// per-cycle conflict scoreboard + the access classification. It is sub-state of the core
// (no own vp::Component), so it shares the core's clock.
//
// Banking facts (tcdm_wrapper.sv:102-112, 2170-2178): bank_select = the LOW
// log2(NumPseudoDualBanks) bits of the ROW (= set/depth) address; ways use independent
// single-port SRAMs (folded_data_bank.sv) so cross-way accesses never conflict; SRAM read
// Latency = 1.

#pragma once
#include <cstdint>
#include <vector>
#include <algorithm>

namespace insitu {

// SRAM read latency (folded_data_bank.sv Latency=1): a granted read returns data 1 cycle later.
static constexpr int kSramReadLatency = 1;

// pseudo_dual_port_bank.sv 6-state status (proc status decode, :147-162).
enum class PseudoStatus { IDLE, R_ONLY, W_ONLY, WR_DIFF_BANK, WR_SAME_ADDR, WR_CONFLICT };

// Per-way, per-cycle bank scoreboard. A "row" is the set/depth index within a way.
class BankArray {
public:
    void init(uint32_t num_ways, uint32_t num_pseudo_banks) {
        num_ways_ = num_ways ? num_ways : 1;
        num_pseudo_banks_ = num_pseudo_banks ? num_pseudo_banks : 1;
        // RTL hardcodes a power-of-two BankFactor; the bank-select is a low-bit slice.
        sel_mask_ = num_pseudo_banks_ - 1;
        wrote_row_.assign((size_t)num_ways_ * num_pseudo_banks_, -1);
        cycle_stamp_ = -1;
    }

    // tcdm_wrapper.sv:2170-2178 — bank_select = low log2(banks) bits of the row address.
    uint32_t bank_select(uint64_t row) const {
        return (num_pseudo_banks_ <= 1) ? 0u : (uint32_t)(row & sel_mask_);
    }

    // tcdm_wrapper.sv:2189-2208 — same-cycle read-vs-write classification for one way.
    PseudoStatus classify(bool rd, bool wr_has_data, uint64_t r_row, uint64_t w_row) const {
        if (rd && wr_has_data) {
            if (bank_select(r_row) != bank_select(w_row)) return PseudoStatus::WR_DIFF_BANK;
            if (r_row == w_row)                            return PseudoStatus::WR_SAME_ADDR;
            return PseudoStatus::WR_CONFLICT;
        }
        if (rd)          return PseudoStatus::R_ONLY;
        if (wr_has_data) return PseudoStatus::W_ONLY;
        return PseudoStatus::IDLE;
    }

    // Reset the per-cycle scoreboard at the top of each cycle (auto-fires when `now` advances).
    // This is the key structural difference vs set_busy_until_: a PER-CYCLE scoreboard, not a
    // monotonic cyclestamp.
    void begin_cycle(int64_t now) {
        if (now != cycle_stamp_) {
            cycle_stamp_ = now;
            std::fill(wrote_row_.begin(), wrote_row_.end(), (int64_t)-1);
        }
    }

    // A write (refill install / store-hit commit / writeback) issues UNCONDITIONALLY this cycle
    // (tcdm_wrapper.sv:2224-2229 write-priority) — record which (way, bank_select) + row it took.
    void commit_write(int64_t now, uint32_t way, uint64_t row) {
        begin_cycle(now);
        wrote_row_[(size_t)way * num_pseudo_banks_ + bank_select(row)] = (int64_t)row;
    }

    // WR_CONFLICT for a read presented this cycle: a write already took this (way, bank_select)
    // to a DIFFERENT row ⇒ read_ready_o=0 ⇒ the read must retry next cycle (+1 penalty).
    // Same row (WR_SAME_ADDR) forwards instead — not a conflict. Different bank or no write — fine.
    bool read_conflict(int64_t now, uint32_t way, uint64_t r_row) {
        begin_cycle(now);
        int64_t w = wrote_row_[(size_t)way * num_pseudo_banks_ + bank_select(r_row)];
        return (w >= 0) && ((uint64_t)w != r_row);
    }

    uint32_t num_ways() const { return num_ways_; }
    uint32_t num_pseudo_banks() const { return num_pseudo_banks_; }

private:
    uint32_t num_ways_ = 1;
    uint32_t num_pseudo_banks_ = 1;
    uint64_t sel_mask_ = 0;
    int64_t  cycle_stamp_ = -1;
    std::vector<int64_t> wrote_row_;   // [way * num_pseudo_banks + bank_select] = row written this cycle (-1 none)
};

} // namespace insitu
