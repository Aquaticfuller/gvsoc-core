// SPDX-FileCopyrightText: 2026 ETH Zurich and University of Bologna
// SPDX-License-Identifier: Apache-2.0
//
// InSitu cache — RTL-faithful decode/encode datapath (structural model, Step 1).
//
// A faithful, header-only transcription of the RTL combinational datapath:
//   - insitu_cache_decoder.sv  (hit/miss classify, hash-way / LRU victim select)
//   - insitu_cache_encoder.sv  (LRU-credit update, masked data merge, meta write)
//
// This is pure logic (no ports, no events) used by the structural cache core
// (Step 4). It carries NONE of the cycle-approximate latency knobs — timing is the
// core's job; this file decides *what* happens (which way, hit/miss/pend/conflict,
// how the LRU credits move), exactly as the RTL does.
//
// Status bit-encoding (insitu_cache_pkg.sv, asserted in the decoder):
//   INVALID=0 (00), VALID=1 (01), READ_PEND=2 (10), WRITE_PEND=3 (11);
//   s1 = status>>1 = "pending", s0 = status&1.

#pragma once
#include <cstdint>

namespace insitu {

enum CacheStatus : uint8_t { INVALID = 0, VALID = 1, READ_PEND = 2, WRITE_PEND = 3 };

// Per-line miss metadata (decoder/encoder miss_meta_t) — used by the multi-read-pend
// linked-list arm; carried here for fidelity even when MRP is off.
struct MissMeta { bool is_full = false; bool is_prime = false; bool link_enable = false; uint32_t link_ptr = 0; };

// Per-way line metadata for ONE set. (Line DATA lives in the bank array, Step 2.)
struct WayMeta {
    CacheStatus status = INVALID;
    bool        dirty  = false;
    uint64_t    tag    = 0;
    uint32_t    lru    = 0;   // LRU credit: 0 == victim (encoder.sv credit scheme)
    int64_t     ready_cycle = 0;  // D1: cycle a PEND line's refill logically lands (sync-slave clamp)
    MissMeta    meta;
};

// Cache geometry — address field widths derived once from the config.
struct CacheGeom {
    uint32_t line_bytes  = 64;
    uint32_t num_ways    = 4;
    uint32_t bank_depth  = 256;   // sets per way = NumCacheEntry / SetAssociativity
    bool     use_hash_way = true;
    bool     original_lru = false; // USE_ORIGINAL_LRU (min-LRU vs first-credit-0)
    uint32_t off_bits = 6, depth_bits = 8, assoc_bits = 2;  // derived

    static uint32_t ilog2(uint32_t n) { uint32_t r = 0; while ((1u << r) < n) r++; return r; }

    void init(uint32_t lb, uint32_t nw, uint32_t bd, bool hash, bool orig_lru) {
        line_bytes = lb; num_ways = nw; bank_depth = bd;
        use_hash_way = hash; original_lru = orig_lru;
        off_bits = ilog2(lb); depth_bits = ilog2(bd); assoc_bits = ilog2(nw);
    }
    uint32_t set_index(uint64_t addr) const {
        return (uint32_t)((addr >> off_bits) & ((1u << depth_bits) - 1));
    }
    uint64_t tag_of(uint64_t addr) const { return addr >> (off_bits + depth_bits); }
    // RTL: hash_way = addr[off+depth_bits +: assoc] ^ addr[off +: assoc]
    //              = (low tag bits) XOR (low set bits).
    uint32_t hash_way(uint64_t addr) const {
        if (num_ways <= 1) return 0;
        const uint32_t mask = (1u << assoc_bits) - 1;
        uint32_t a = (uint32_t)((addr >> (off_bits + depth_bits)) & mask); // low tag bits
        uint32_t b = (uint32_t)((addr >> off_bits) & mask);                // low set bits
        return (a ^ b) & mask;
    }
};

struct Decode {
    uint32_t way = 0;
    bool is_write       = false;
    bool is_hit         = false;
    bool is_hit_pend    = false;  // same-type pending hit (read→READ_PEND / write→WRITE_PEND)
    bool is_hit_conflit = false;  // opposite-type pending hit
    bool is_all_pend    = false;  // hash mode: the hash-way is pending; full: every way pending
};

// Faithful transcription of insitu_cache_decoder.sv proc_bank_decode (request path).
// `ways` points at this set's num_ways WayMeta entries.
inline Decode decode_request(const CacheGeom& g, const WayMeta* ways, uint64_t addr, bool is_write) {
    Decode d; d.is_write = is_write;
    const uint64_t tag = g.tag_of(addr);

    if (g.use_hash_way && g.num_ways > 1) {                 // proc_hash_way_req
        const uint32_t hw = g.hash_way(addr);
        d.way = hw;
        const WayMeta& w = ways[hw];
        const bool tag_hit = (w.tag == tag);
        const int s1 = (w.status >> 1) & 1;
        const int s0 = w.status & 1;
        const int wr = is_write ? 1 : 0;
        d.is_hit         = tag_hit && !s1 && (s0 != 0);
        d.is_hit_pend    = tag_hit &&  s1 && ((s0 ^ wr) == 0);
        d.is_hit_conflit = tag_hit &&  s1 && ((s0 ^ wr) != 0);
        d.is_all_pend    = (s1 != 0);
        return d;
    }

    // proc_full_assoc_req
    d.is_all_pend = true;
    for (uint32_t way = 0; way < g.num_ways; way++) {
        const WayMeta& w = ways[way];
        if (w.status == VALID && w.tag == tag) { d.is_hit = true; d.way = way; }
        if (w.status == READ_PEND && w.tag == tag) {
            if (is_write) { d.is_hit_conflit = true; d.way = way; }
            else          { d.is_hit_pend = true;    d.way = way; }
        }
        if (w.status == WRITE_PEND && w.tag == tag) {
            if (is_write) { d.is_hit_pend = true;    d.way = way; }
            else          { d.is_hit_conflit = true; d.way = way; }
        }
        if (w.status == VALID || w.status == INVALID) d.is_all_pend = false;
    }
    // proc3_find_miss_way (LRU victim) — only on a genuine miss.
    if (!d.is_hit && !d.is_hit_pend && !d.is_hit_conflit && !d.is_all_pend) {
        if (g.original_lru) {                               // USE_ORIGINAL_LRU: min-LRU among VALID/INVALID
            for (uint32_t way = 0; way < g.num_ways; way++) {
                const bool wvi = (ways[way].status == VALID || ways[way].status == INVALID);
                const bool dvi = (ways[d.way].status == VALID || ways[d.way].status == INVALID);
                if (wvi) d.way = (!dvi) ? way : (ways[way].lru < ways[d.way].lru ? way : d.way);
            }
        } else {                                            // default: first way with credit 0
            for (uint32_t way = 0; way < g.num_ways; way++) { if (ways[way].lru == 0) { d.way = way; break; } }
        }
    }
    return d;
}

// encoder.sv proc_max_lru: number of VALID|INVALID ways.
inline uint32_t max_lru_credit(const CacheGeom& g, const WayMeta* ways) {
    uint32_t c = 0;
    for (uint32_t w = 0; w < g.num_ways; w++)
        if (ways[w].status == VALID || ways[w].status == INVALID) c++;
    return c;
}

// encoder.sv LRU_array_update — the credit-rebalance for one way's status transition.
// CALL BEFORE writing the way's new status into ways[] (it reads pre-write statuses/LRU).
// `before`/`after` = the modified way's status pre/post this op.
inline void lru_update(const CacheGeom& g, WayMeta* ways, uint32_t way,
                       CacheStatus before, CacheStatus after) {
    const uint32_t mlc  = max_lru_credit(g, ways);  // on pre-write statuses
    const uint32_t orig = ways[way].lru;            // pre-write LRU of the modified way
    // snapshot pre-write LRU so the in-place rebalance reads originals (RTL is _in→_out, pure)
    uint32_t snap[64];
    for (uint32_t w = 0; w < g.num_ways; w++) snap[w] = ways[w].lru;
    auto isVI = [](CacheStatus s) { return s == VALID || s == INVALID; };
    const bool bp = (before == READ_PEND || before == WRITE_PEND);
    const bool ap = (after  == READ_PEND || after  == WRITE_PEND);

    if (bp && ap) {
        // pend -> pend : nothing
    } else if (isVI(before) && ap) {                 // allocate a pending line
        for (uint32_t w = 0; w < g.num_ways; w++) {
            if (w == way) ways[w].lru = g.num_ways - 1;
            else if (snap[w] > orig && isVI(ways[w].status)) ways[w].lru = snap[w] - 1;
        }
    } else if (bp && isVI(after)) {                  // refill completes a pending line
        ways[way].lru = (after == INVALID) ? 0 : mlc;
    } else {                                         // VALID->VALID (hit MRU bump), etc.
        for (uint32_t w = 0; w < g.num_ways; w++) {
            if (w == way) ways[w].lru = (mlc == 0) ? (g.num_ways - 1) : (mlc - 1);
            else if (snap[w] > orig && isVI(ways[w].status)) ways[w].lru = snap[w] - 1;
        }
    }
}

// encoder.sv masked byte merge: overlay `wdata` into `line` where `wmask` byte-bit is set.
// (n = line_bytes; wmask is a byte-granular bitmask, bit i guards byte i.)
inline void masked_merge(uint8_t* line, const uint8_t* wdata, const uint8_t* wmask, uint32_t n) {
    for (uint32_t b = 0; b < n; b++)
        if (wmask[b >> 3] & (1u << (b & 7))) line[b] = wdata[b];
}

} // namespace insitu
