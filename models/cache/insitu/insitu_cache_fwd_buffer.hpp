// SPDX-FileCopyrightText: 2026 ETH Zurich and University of Bologna
// SPDX-License-Identifier: Apache-2.0
//
// InSitu cache — RTL-faithful single-entry forwarding buffer (structural model, Step 3).
//
// A header-only transcription of the 1-entry write-back register-cache `sram_forwarding_buffer.sv`:
// a single cached SRAM-row line sitting between the access controller and the data/meta banks.
// Its job (and the latency it saves) :
//   - READ-SUPPRESS: a read whose line + requested parts are resident is served from the buffer,
//     skipping the SRAM bank read (sram_forwarding_buffer.sv rd_buf_hit, :234).
//   - WRITE-ABSORB: a write to the buffered line merges into buf_data (byte mask) and marks it
//     dirty — lazily, no immediate bank write (:267-345).
//   - LAZY WRITEBACK: when a different line must be buffered, a dirty victim is written back to the
//     bank first (wb_needed, :348).
//   - PARTIAL VALIDITY: a per-part bitmap tracks which parts of the line are resident; a read of an
//     unresident part misses (:214-216).
//   - RAW: a read after a write to the same word forwards the just-written value (the buffer holds it).
//
// SCOPE (single-entry, in-order — the structural-model essential): models the buffer as processed
// one access at a time (the caller's per-cycle order). DEFERRED to the timing-calibration phase
// (and the core integration that goes with it): the double-buffered _q/_d nonblocking same-cycle
// comb-vs-commit split, the in-flight-SRAM-populate merge (rd_inflight_hit / EnableInflightWriteMerge),
// and the N-entry variant (sram_forwarding_buffer_multi.sv). Those refine *timing* under same-cycle
// hazards; this models the functional read-suppress/write-absorb/WB/RAW behaviour. Validated
// standalone; NOT yet wired into the core data path (that lands with calibration, where its
// latency effect is tuned against the RTL reference).

#pragma once
#include <cstdint>
#include <vector>
#include <cstring>

namespace insitu {

struct FwdResult {
    bool rd_hit        = false;   // read served from buffer → suppress the bank read
    bool wr_absorbed   = false;   // write merged into buffer → suppress the bank write
    bool wb_fire       = false;   // a dirty victim must be written back to the bank
    uint64_t wb_addr   = 0;
    bool need_populate = false;   // the bank must be read to bring this line into the buffer (miss)
    uint64_t pop_addr  = 0;       // line addr to populate
};

class FwdBuffer {
public:
    // part_split = number of independently-tracked parts of a line (DataPartSplit; 1 = whole line).
    void init(uint32_t line_bytes, uint32_t part_split) {
        line_bytes_ = line_bytes;
        part_split_ = part_split ? part_split : 1;
        part_bytes_ = line_bytes_ / part_split_;
        data_.assign(line_bytes_, 0);
        parts_valid_.assign(part_split_, false);
        valid_ = dirty_ = false; addr_ = 0;
    }

    uint64_t line_base(uint64_t a) const { return a & ~((uint64_t)line_bytes_ - 1); }
    uint32_t part_of(uint64_t a) const {
        return part_split_ <= 1 ? 0u : (uint32_t)(((a & (line_bytes_ - 1)) / part_bytes_) % part_split_);
    }
    const uint8_t *wb_data() const { return data_.data(); }

    // One access. `wmask`/`wdata` are line-sized byte buffers for writes; `rd_out` (line-sized)
    // receives the served bytes on a read hit. `all_parts` forces whole-line parts (refill/full).
    FwdResult access(bool is_write, uint64_t addr, uint32_t size,
                     const uint8_t *wmask, const uint8_t *wdata,
                     bool all_parts, uint8_t *rd_out) {
        FwdResult r;
        const uint64_t line = line_base(addr);
        const uint32_t off = (uint32_t)(addr & (line_bytes_ - 1));
        const uint32_t p = part_of(addr);
        const bool same_line = valid_ && (addr_ == line);
        const bool part_resident = same_line && (all_parts ? all_parts_valid() : parts_valid_[p]);

        if (is_write) {
            if (!same_line) {                       // victim → lazy writeback, then re-home buffer
                if (valid_ && dirty_) { r.wb_fire = true; r.wb_addr = addr_; }
                valid_ = true; dirty_ = false; addr_ = line;
                for (uint32_t i = 0; i < part_split_; i++) parts_valid_[i] = false;
            }
            // absorb the write bytes (byte mask) into the buffered line; mark dirty + parts resident
            uint32_t n = size; if (off + n > line_bytes_) n = line_bytes_ - off;
            for (uint32_t b = 0; b < n; b++)
                if (!wmask || (wmask[(off + b) >> 3] & (1u << ((off + b) & 7)))) data_[off + b] = wdata[off + b];
            dirty_ = true;
            mark_parts(off, n, all_parts);
            r.wr_absorbed = true;
            return r;
        }

        // read
        if (part_resident) {                        // read-suppress: serve from buffer
            if (rd_out) { uint32_t n = size; if (off + n > line_bytes_) n = line_bytes_ - off;
                          memcpy(rd_out + off, &data_[off], n); }
            r.rd_hit = true;
            return r;
        }
        // read miss: evict a dirty victim (different line), then populate this line from the bank
        if (!same_line && valid_ && dirty_) { r.wb_fire = true; r.wb_addr = addr_; }
        r.need_populate = true; r.pop_addr = line;
        return r;
    }

    // Install a line read from the bank into the buffer (the populate the access controller drives
    // after need_populate). `all_parts` true ⇒ the whole line was fetched.
    void populate(uint64_t addr, const uint8_t *line_from_bank, bool all_parts, uint32_t part_idx) {
        const uint64_t line = line_base(addr);
        if (!valid_ || addr_ != line) {             // re-home (the dirty victim was already WB'd)
            valid_ = true; dirty_ = false; addr_ = line;
            for (uint32_t i = 0; i < part_split_; i++) parts_valid_[i] = false;
        }
        if (line_from_bank) memcpy(data_.data(), line_from_bank, line_bytes_);
        if (all_parts || part_split_ <= 1) { for (uint32_t i = 0; i < part_split_; i++) parts_valid_[i] = true; }
        else parts_valid_[part_idx % part_split_] = true;
    }

    void invalidate() { valid_ = dirty_ = false; for (uint32_t i = 0; i < part_split_; i++) parts_valid_[i] = false; }
    bool valid() const { return valid_; }
    bool dirty() const { return dirty_; }
    uint64_t addr() const { return addr_; }

private:
    bool all_parts_valid() const { for (uint32_t i = 0; i < part_split_; i++) if (!parts_valid_[i]) return false; return true; }
    void mark_parts(uint32_t off, uint32_t n, bool all_parts) {
        if (all_parts || part_split_ <= 1) { for (uint32_t i = 0; i < part_split_; i++) parts_valid_[i] = true; return; }
        for (uint32_t b = 0; b < n; b++) parts_valid_[((off + b) / part_bytes_) % part_split_] = true;
    }

    uint32_t line_bytes_ = 64, part_split_ = 1, part_bytes_ = 64;
    bool valid_ = false, dirty_ = false;
    uint64_t addr_ = 0;
    std::vector<uint8_t> data_;
    std::vector<bool> parts_valid_;   // proxy type — index-assign only (never range-for &)
};

} // namespace insitu
