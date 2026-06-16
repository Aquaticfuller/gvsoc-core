// SPDX-FileCopyrightText: 2026 ETH Zurich and University of Bologna
// SPDX-License-Identifier: Apache-2.0
//
// InSitu cache — RTL-faithful SPM partition address remap (structural model, Step 6).
//
// Header-only transcription of the address-translation block of
// `insitu_cache_tcdm_wrapper_partitionable_flushable.sv:184-218`: the integer DIV/MOD remap that
// carves a contiguous block of cache sets out as scratchpad (SPM) and shrinks the cache's effective
// set count. The structural facts it captures (NOT a power-of-2 mask — exact integer arithmetic):
//
//   cache_partition_set_for_SPM   = NumPseudoDualBanks * bank_depth_for_SPM      (sets reserved for SPM)
//   cache_partition_set_for_cache = CacheBankDepth - cache_partition_set_for_SPM (sets left for cache)
//
//   UPSTREAM (req → cache addr):
//     line_index = addr >> log2(CacheLineWidth/8)
//     tag = line_index / cache_partition_set_for_cache
//     set = line_index % cache_partition_set_for_cache + cache_partition_set_for_SPM   ← shifted up past SPM
//     cache_addr = {tag, set, byt=0}
//
//   DOWNSTREAM (cache addr → restored req addr, for refill/eviction to the NoC):
//     {tag, set, byt} = cache_addr
//     restored = tag * cache_partition_set_for_cache + (set - cache_partition_set_for_SPM)
//     req_addr = restored << log2(CacheLineWidth/8)
//
// When bank_depth_for_SPM == 0 (the common no-partition case) SPM sets = 0, cache sets = CacheBankDepth,
// and the remap collapses to the plain tag=line/sets, set=line%sets decode (and restore is the exact
// inverse → identity address round-trip). The DIV/MOD by a non-power-of-2 cache_partition_set_for_cache
// is why this is NOT a bit-mask: reserving e.g. 3 SPM bank-rows leaves a non-power-of-2 set count and the
// modulo spreads lines unevenly — faithfully reproduced here. Validated standalone; wiring into the
// wrapper's upstream-decode / downstream-restore lands with the integrated tile + calibration.

#pragma once
#include <cstdint>

namespace insitu {

struct SpmCacheAddr {
    uint64_t tag = 0;
    uint64_t set = 0;
};

struct SpmRemap {
    uint32_t line_bytes      = 64;    // CacheLineWidth/8
    uint32_t cache_bank_depth = 128;  // NumCacheEntry / SetAssociativity (total sets per way)
    uint32_t num_pseudo_banks = 1;    // NumPseudoDualBanks
    uint32_t off_bits        = 6;     // log2(line_bytes)

    void init(uint32_t cacheline_bytes, uint32_t bank_depth, uint32_t pseudo_banks) {
        line_bytes = cacheline_bytes;
        cache_bank_depth = bank_depth;
        num_pseudo_banks = pseudo_banks ? pseudo_banks : 1;
        off_bits = log2_exact(line_bytes);
    }
    static uint32_t log2_exact(uint32_t v) { uint32_t b = 0; while ((1u << b) < v) b++; return b; }

    uint32_t spm_sets(uint32_t bank_depth_for_spm) const { return num_pseudo_banks * bank_depth_for_spm; }
    uint32_t cache_sets(uint32_t bank_depth_for_spm) const { return cache_bank_depth - spm_sets(bank_depth_for_spm); }

    // partitionable_flushable.sv:200-205 — upstream request address → {tag, set}.
    SpmCacheAddr remap_upstream(uint64_t addr, uint32_t bank_depth_for_spm) const {
        const uint32_t cs = cache_sets(bank_depth_for_spm);
        const uint32_t ss = spm_sets(bank_depth_for_spm);
        const uint64_t li = addr >> off_bits;
        SpmCacheAddr a;
        a.tag = li / cs;
        a.set = (li % cs) + ss;
        return a;
    }

    // partitionable_flushable.sv:214-217 — cache {tag, set} → restored byte address for the NoC.
    uint64_t restore_downstream(uint64_t tag, uint64_t set, uint32_t bank_depth_for_spm) const {
        const uint32_t cs = cache_sets(bank_depth_for_spm);
        const uint32_t ss = spm_sets(bank_depth_for_spm);
        const uint64_t restored = tag * cs + (set - ss);
        return restored << off_bits;
    }

    // Convenience: the SPM region occupies cache sets [0 .. cache_base_for_SPM); a cache request never
    // indexes into it (its set is always >= cache_base_for_SPM).
    uint32_t cache_base_for_spm(uint32_t bank_depth_for_spm) const { return spm_sets(bank_depth_for_spm); }
};

} // namespace insitu
