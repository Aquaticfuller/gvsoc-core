// SPDX-FileCopyrightText: 2026 ETH Zurich and University of Bologna
// SPDX-License-Identifier: Apache-2.0
//
// InSitu cache — RTL-faithful par_coalescer datapath (structural model, Step 5).
//
// Header-only transcription of the N→1 coalescing datapath of par_coalescer_equal_window.sv /
// req_coalescer_v2.sv / rsp_spliter_v2.sv: the REAL wide-merge + response-split that replaces the
// `enable_input_coalesce` latency-trick (and the P1 par_coalescer skeleton's "followers inherit
// latency"). The structural facts it captures:
//   - Same-cycle narrow ports to the SAME line that are the SAME type (read or write) coalesce into
//     ONE wide (512b) cache beat. The write-bit is folded into the coalescing key MSB
//     (write_mixed_addr, equal_window.sv:162) so a read and a write to the same line NEVER merge.
//   - The beat carries a HITMAP (which ports merged) + per-port word OFFSET (which 32b word of the
//     line each port maps to: ofst = (addr>>2) & (NumWord-1), NumWord = 512/32 = 16).
//   - WRITE merge: each merged port's bytes go to its word offset in the 512b line, last-writer-wins
//     (req_coalescer_v2 wide merge). READ split: the one wide response line is split back so each
//     merged port gets the word at its offset (rsp_spliter_v2).
//
// SCOPE (functional group-coalescing + wide-merge + split — the structural essential). DEFERRED to
// the timing-calibration phase (with the per-cycle component integration): the CSHR FSM
// (IDLE/VALID + watchdog), the per-port depth-4 request FIFOs, the equal- vs extend-window policy,
// and the round-robin next-line arbiter — those set the per-cycle *timing* of when a window closes;
// this models WHICH ports coalesce and the resulting wide beat / split. Validated standalone; the
// per-cycle CSHR timing + wiring into the structural core land with calibration.

#pragma once
#include <cstdint>
#include <vector>
#include <cstring>

namespace insitu {

// One narrow upstream access presented to the coalescer this cycle.
struct NarrowAccess {
    uint32_t port = 0;
    bool     is_write = false;
    uint64_t addr = 0;
    uint32_t size = 0;
    // for writes: the bytes + byte-mask for THIS access's word (word_bytes wide)
    const uint8_t *wdata = nullptr;   // word_bytes
    const uint8_t *wstrb = nullptr;   // 1 bit per byte of the word (LSB = byte0)
};

// One coalesced wide beat: several same-line same-type ports served by one cache access.
struct CoalGroup {
    bool is_write = false;
    uint64_t line = 0;                 // line base address
    std::vector<uint32_t> ports;       // hitmap: the merged upstream ports
    std::vector<uint32_t> word_ofst;   // per-merged-port word index within the line
    std::vector<uint8_t> wdata;        // wide line bytes (writes; last-writer-wins merge)
    std::vector<uint8_t> wmask;        // wide byte-bitmask (writes)
};

class Coalescer {
public:
    void init(uint32_t line_bytes, uint32_t word_bytes) {
        line_bytes_ = line_bytes;
        word_bytes_ = word_bytes ? word_bytes : 4;
        num_words_  = line_bytes_ / word_bytes_;
    }
    uint64_t line_base(uint64_t a) const { return a & ~((uint64_t)line_bytes_ - 1); }
    uint32_t word_ofst(uint64_t a) const { return (uint32_t)(((a & (line_bytes_ - 1)) / word_bytes_) % num_words_); }

    // Coalesce a set of same-cycle narrow accesses into wide groups. Ports merge iff same line AND
    // same type (write-bit in the key). Within a write group, later-listed ports overwrite earlier
    // bytes (last-writer-wins), matching the RTL wide merge.
    std::vector<CoalGroup> coalesce(const std::vector<NarrowAccess> &acc) const {
        std::vector<CoalGroup> groups;
        for (const auto &a : acc) {
            const uint64_t line = line_base(a.addr);
            // find an open group with the same {is_write, line}
            CoalGroup *g = nullptr;
            for (auto &cand : groups)
                if (cand.is_write == a.is_write && cand.line == line) { g = &cand; break; }
            if (!g) {
                groups.push_back(CoalGroup{});
                g = &groups.back();
                g->is_write = a.is_write; g->line = line;
                if (a.is_write) { g->wdata.assign(line_bytes_, 0); g->wmask.assign((line_bytes_ + 7) / 8, 0); }
            }
            const uint32_t wo = word_ofst(a.addr);
            g->ports.push_back(a.port);
            g->word_ofst.push_back(wo);
            if (a.is_write && a.wdata) {                  // merge this port's bytes into the wide line
                const uint32_t base = wo * word_bytes_;
                for (uint32_t b = 0; b < word_bytes_; b++) {
                    if (!a.wstrb || (a.wstrb[b >> 3] & (1u << (b & 7)))) {
                        g->wdata[base + b] = a.wdata[b];
                        g->wmask[(base + b) >> 3] |= (uint8_t)(1u << ((base + b) & 7));
                    }
                }
            }
        }
        return groups;
    }

    // Split a wide read response (the 512b line bytes) back to one merged port's word.
    // `idx` is the index within g.ports; writes `word_bytes_` bytes into `out`.
    void split_read(const CoalGroup &g, uint32_t idx, const uint8_t *wide_line, uint8_t *out) const {
        const uint32_t base = g.word_ofst[idx] * word_bytes_;
        memcpy(out, wide_line + base, word_bytes_);
    }

    uint32_t num_words() const { return num_words_; }

private:
    uint32_t line_bytes_ = 64, word_bytes_ = 4, num_words_ = 16;
};

} // namespace insitu
