// SPDX-FileCopyrightText: 2026 ETH Zurich and University of Bologna
// SPDX-License-Identifier: Apache-2.0
//
// InSitu cache — RTL-faithful L2 refill address scramble + NAPOT channel decode (structural model, Step 7).
//
// Header-only transcription of `cachepool_pkg.sv` scrambleAddr/revertAddr (:495-540) and the cluster
// NAPOT channel decode (`cachepool_cluster.sv:618-661`): how a cache miss's refill/eviction byte address
// is transformed on its way to the DRAM channels. The structural facts it captures:
//
//   The address arriving at the cluster (already inverse-rotated back to the true DRAM address by the
//   tile, then bank_id/tile_id-stamped) is split into four fields by bit position:
//       [Reminder | Scramble(channel-id) | InterChange | Constant]
//   scrambleAddr SWAPS the Scramble and InterChange fields →
//       [Reminder | InterChange | Scramble(channel-id) | Constant]
//   so the channel-id lands at [SizeOffsetBits +: ScrambleBits] where the NAPOT mask selects it. This
//   spreads consecutive cache lines across channels (interleaving). revertAddr is the exact inverse.
//   Field widths (cachepool_pkg.sv:502-506):
//       ConstantBits    = log2(L2BankBeWidth * Interleave)     // never involved in channel select
//       SizeOffsetBits  = log2(DramPerChSize) = log2(DramSize/NumL2Channel)
//       ScrambleBits    = log2(NumL2Channel)
//       InterChangeBits = SizeOffsetBits - ConstantBits
//       ReminderBits    = AddrWidth - ScrambleBits - SizeOffsetBits
//   The swap is GATED on (L2BankBeWidth * Interleave) < DramPerChSize; otherwise the address passes
//   through unchanged (scrambleAddr/revertAddr/channel are identity / plain NAPOT).
//
//   NAPOT channel decode (cluster:626-653): channel i has base = DramAddr + DramPerChSize*i, mask =
//   ~(DramPerChSize-1); match iff (addr & mask)==(base & mask). On the SCRAMBLED address this reduces to
//   reading the channel-id field directly: channel = (scrambled >> SizeOffsetBits) & (NumL2Channel-1).
//
//   System constants (cachepool_pkg.sv): DRAM type DDR4 (:241), DramAddr=0x8000_0000 (:248),
//   DramSize=0x4000_0000=1GiB (:249), NumL2Channel = L2_CHANNEL (enum L2Channel0..3 → 4, :377-381),
//   DramPerChSize = DramSize/NumL2Channel (:250). Responses route back by tile_id/bank_id sideband
//   (cluster:719-722), NOT by un-scrambling — revertAddr is defined but unused in RTL (kept here for
//   completeness / DRAMSys-side un-scramble if the DRAM model needs the linear address).
//
// SCOPE: the pure address transform + channel select (what each refill beat needs). The per-channel
// AXI arbitration / DRAMSys timing is the downstream memory model (calibration). Validated standalone.

#pragma once
#include <cstdint>

namespace insitu {

struct L2AddrMap {
    uint64_t dram_base       = 0x80000000ull;  // DramAddr
    uint64_t dram_size       = 0x40000000ull;  // DramSize (1 GiB)
    uint32_t num_l2_channel  = 4;              // NumL2Channel
    uint32_t addr_width      = 48;             // SpatzAxiAddrWidth
    uint32_t l2_bank_be_width = 64;            // L2BankWidth/8
    uint32_t interleave      = 1;              // L2_INTERLEAVE
    // derived (cachepool_pkg.sv:502-506)
    uint64_t dram_per_ch_size = 0x10000000ull; // DramSize / NumL2Channel
    uint32_t constant_bits   = 6;
    uint32_t size_offset_bits = 28;
    uint32_t scramble_bits   = 2;
    uint32_t interchange_bits = 22;
    uint32_t reminder_bits   = 18;
    bool     scramble_active = true;

    static uint32_t clog2(uint64_t v) { uint32_t b = 0; while ((1ull << b) < v) b++; return b; }

    void init(uint64_t base, uint64_t size, uint32_t channels, uint32_t addr_w,
              uint32_t bank_be_width, uint32_t interleave_factor) {
        dram_base = base; dram_size = size; num_l2_channel = channels; addr_width = addr_w;
        l2_bank_be_width = bank_be_width; interleave = interleave_factor;
        dram_per_ch_size = dram_size / num_l2_channel;
        constant_bits    = clog2((uint64_t)l2_bank_be_width * interleave);
        size_offset_bits = clog2(dram_per_ch_size);
        scramble_bits    = clog2(num_l2_channel);
        interchange_bits = size_offset_bits - constant_bits;
        reminder_bits    = addr_width - scramble_bits - size_offset_bits;
        scramble_active  = ((uint64_t)l2_bank_be_width * interleave) < dram_per_ch_size;
    }

    static uint64_t mask(uint32_t bits) { return bits >= 64 ? ~0ull : ((1ull << bits) - 1); }

    // cachepool_pkg.sv:495-516 — [Reminder|Scramble|InterChange|Constant] → [Reminder|InterChange|Scramble|Constant].
    uint64_t scramble(uint64_t addr) const {
        if (!scramble_active) return addr;
        const uint64_t reminder    = (addr >> (size_offset_bits + scramble_bits)) & mask(reminder_bits);
        const uint64_t interchange = (addr >> (constant_bits + scramble_bits))    & mask(interchange_bits);
        const uint64_t channel     = (addr >> constant_bits)                      & mask(scramble_bits);
        const uint64_t constant    =  addr                                        & mask(constant_bits);
        return (reminder    << (size_offset_bits + scramble_bits))
             | (channel     << size_offset_bits)
             | (interchange << constant_bits)
             | constant;
    }

    // cachepool_pkg.sv:518-540 — exact inverse of scramble().
    uint64_t revert(uint64_t addr) const {
        if (!scramble_active) return addr;
        const uint64_t reminder    = (addr >> (size_offset_bits + scramble_bits)) & mask(reminder_bits);
        const uint64_t channel     = (addr >> size_offset_bits)                   & mask(scramble_bits);
        const uint64_t interchange = (addr >> constant_bits)                      & mask(interchange_bits);
        const uint64_t constant    =  addr                                        & mask(constant_bits);
        return (reminder    << (size_offset_bits + scramble_bits))
             | (interchange << (constant_bits + scramble_bits))
             | (channel     << constant_bits)
             | constant;
    }

    // cachepool_cluster.sv:618-653 — NAPOT channel select on the SCRAMBLED address.
    uint32_t channel_of(uint64_t scrambled_addr) const {
        if (!scramble_active) return (uint32_t)(((scrambled_addr - dram_base) / dram_per_ch_size) % num_l2_channel);
        return (uint32_t)((scrambled_addr >> size_offset_bits) & mask(scramble_bits));
    }
};

} // namespace insitu
