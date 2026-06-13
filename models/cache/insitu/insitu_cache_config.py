#
# Copyright (C) 2026 ETH Zurich and University of Bologna
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#

"""Configuration classes for the InSitu Cache performance model.

These classes are the single source of truth for every parameter the model exposes.
Defaults mirror the **latest** ``cachepool_cache_ctrl`` RTL configuration documented in
``prompt/insitu_cache_architecture_v2.md``. The pre-2026-05-22 defaults (from
``prompt/insitu_cache_architecture.md``) corresponded to a different RTL revision; see
``prompt/insitu_cache_architecture_v2.md`` §0 for the diff.

The three ``Config`` subclasses below map 1:1 onto the three GVSoC ``Component`` subclasses
that make up the model: ``InsituCacheController``, ``InsituCacheCoalescer``, and
``InsituCacheInterco``. Each Component's constructor takes its matching Config and the
Config's fields become the component's published properties (read from C++ via
``get_js_config()``).

``InsituCacheTileConfig`` is a plain Python class (not a ``Config``) that bundles the three
together for the tile wrapper.

**Phase-A status (2026-05-22):** new config fields added for `WriteThroughMode`,
`EnableMultiReadPend`, `EnableSPM` / `BankDepthForSPM`, `EnableFlush`. The actual C++
behaviour for those fields is partly stubbed — see the controller `.cpp` for what's wired up
today vs. what's documented for a future Phase-B refactor.

**2026-06-01 update:** the shipping `cachepool_512` config is now the **production**
cache (**folded + hash-way + forwarding-buffer ON**), so :func:`make_cachepool_512_config`
defaults to that (`use_hash_way_select=True`, folded penalties, new
`use_forwarding_buffer=True`). The unfolded + LRU + no-fwd-buffer config — this model's
prior 2026-05-22 default — is now :func:`make_cachepool_512_conventional_config`. See
`insitu_cache_architecture_v2.md` §0.1.
"""

from __future__ import annotations

from dataclasses import dataclass, field

from config_tree import Config, cfg_field


class InsituCacheControllerConfig(Config):
    """Per-cache-controller configuration."""

    # -------- Geometry --------
    cache_line_bytes: int = cfg_field(default=64, dump=True, desc=(
        "Cache line size in bytes (512b = 64B in the canonical RTL)"
    ))
    num_ways: int = cfg_field(default=4, dump=True, desc=(
        "Set associativity per controller"
    ))
    num_sets: int = cfg_field(default=128, dump=True, desc=(
        "Sets per controller (CacheBankDepth in the RTL)"
    ))
    tcdm_word_bytes: int = cfg_field(default=4, dump=True, desc=(
        "Upstream request granularity in bytes (NarrowDataWidth/8 = 4)"
    ))
    refill_beat_bytes: int = cfg_field(default=16, dump=True, desc=(
        "Width of each refill beat in bytes (128b = 16B). The cache issues one refill "
        "request per miss with size=cache_line_bytes and uses set_duration() to model "
        "the multi-beat occupancy on the refill port."
    ))

    # -------- Replacement policy --------
    use_hash_way_select: bool = cfg_field(default=True, dump=True, desc=(
        "When true, the victim way on a miss is derived from a deterministic hash of "
        "(tag, set). When false, full LRU victim search is used. Hits always use "
        "tag-match. The shipping `cachepool_512` config defaults to hash-way (=1); the "
        "RTL forces hash-way whenever the cache is folded OR the forwarding buffer is on "
        "(see `use_forwarding_buffer`). LRU is legal only for the unfolded conventional "
        "cache with the buffer off."
    ))

    # -------- New write & MSHR mode knobs (latest RTL) --------
    write_through_mode: bool = cfg_field(default=False, dump=True, desc=(
        "When true, writes go straight to L2 via the coalescer (write-through). When "
        "false (default, matches the latest RTL `WriteThroughMode=0`), the cache is in "
        "pure write-back: write hits mark the line dirty and don't push to L2 until "
        "eviction. Phase-A note: the controller still issues a coalescer event on "
        "write hits regardless of this flag — Phase B will gate it."
    ))
    enable_multi_read_pend: bool = cfg_field(default=False, dump=True, desc=(
        "When true, multiple read requests to the same in-flight (READ_PEND) cache line "
        "queue on a per-line linked list rather than stalling with MSHR_FULL. Matches "
        "the RTL `ENABLE_MULTI_READ_PEND` compile-time option. Reduces MSHR-full stalls "
        "on read-heavy multi-port workloads."
    ))
    use_forwarding_buffer: bool = cfg_field(default=True, dump=True, desc=(
        "Model the RTL `UseForwardingBuffer` (1-entry write-back SRAM row cache between "
        "the access controller and the data/meta banks; `sram_forwarding_buffer.sv`). "
        "Production `cachepool_512` default = on. RTL constraint: the buffer requires "
        "`use_hash_way_select=True`. Perf effect (suppressing the SRAM-read latency for "
        "repeated same-row accesses, same-cycle RAW resolution) is currently folded into "
        "`hit_latency_cycles` — this flag is informational/fidelity until an explicit "
        "same-row forward model lands (Phase B)."
    ))

    # -------- Hyper-SPM partitioning --------
    enable_spm: bool = cfg_field(default=False, dump=True, desc=(
        "Enable Hyper-SPM partitioning. When true, `bank_depth_for_spm` rows of bank "
        "depth are reserved for software-managed SPM. The cache's effective capacity "
        "shrinks accordingly. See architecture v2 doc §5."
    ))
    bank_depth_for_spm: int = cfg_field(default=0, dump=True, desc=(
        "Number of bank depth rows reserved for SPM (only meaningful if `enable_spm`). "
        "Each row × NumPseudoDualBanks × SetAssociativity × line_bytes is taken out of "
        "the cache's effective capacity."
    ))

    # -------- Flush / Invalidate --------
    enable_flush: bool = cfg_field(default=False, dump=True, desc=(
        "Reserved. When true, the controller exposes a `cache_sync` interface mirroring "
        "the RTL's 2-bit `cache_sync_insn_i` (flush+inval / flush-only / inval-only / "
        "init). Phase-A: not yet implemented in the C++ model — set true to reserve the "
        "Python-side API surface; flush has no runtime effect yet."
    ))

    # -------- Timing knobs --------
    hit_latency_cycles: int = cfg_field(default=4, dump=True, desc=(
        "Cycles from request acceptance at the controller to response emit on a read hit "
        "(the ISOLATED / pipeline-fill latency)"
    ))
    streaming_hit_latency_cycles: int = cfg_field(default=-1, dump=True, desc=(
        "Steady-state per-access read-hit latency once the hit pipeline is full. The RTL hit "
        "path has three decoupling registers (coalescer req-spill, resp-spill, "
        "rsp_spliter/output-FIFO) that an isolated access fills in series (→ hit_latency_cycles) "
        "but a back-to-back stream keeps continuously occupied (→ this value). Modelled as a "
        "per-controller warmth gradient: base = streaming + min(hit_latency-streaming, "
        "cycles_since_last_read_hit), so an isolated hit pays the full fill latency, a streaming "
        "hit pays this, and a gapped hit interpolates (RTL gap-sweep 7/8/9/10). READ hits only "
        "(writes/forwarded reads keep their own latency). -1 = OFF (every hit pays "
        "hit_latency_cycles, unchanged — the Spatz/default path)."
    ))
    bank_accept_cycles: int = cfg_field(default=1, dump=True, desc=(
        "Per-set bank ACCEPT interval (cycles between accepting two successive accesses to the "
        "same set). The data bank is PIPELINED: it accepts a new access every this-many cycles, "
        "each responding `latency` cycles later — so back-to-back accesses to a hot/heavily-reused "
        "set PIPELINE rather than serialize at the full hit latency. Default 1 = one access per "
        "set per cycle (BankFactor=1). Real kernels reuse few lines intensely across many ports; "
        "advancing the per-set busy stamp by the full latency (the pre-2026-06 behaviour) "
        "over-serialized those and inflated per-access latency ~6x on real traces (see "
        "prompt/insitu_cache_realkernel_alignment_2026-06-12.md)."
    ))
    write_hit_latency_cycles: int = cfg_field(default=-1, dump=True, desc=(
        "Cycles from acceptance to response on a WRITE hit. The RTL acks a write earlier "
        "than a read returns (write-info FIFO push: `wresp_valid = ~winfo_fifo_empty`), so "
        "the warm-write latency (RTL 8) is below the read-hit latency (RTL 10). Default -1 "
        "means 'same as hit_latency_cycles' (no change for existing configs)."
    ))
    write_commit_cycles: int = cfg_field(default=1, dump=True, desc=(
        "Min cycles between accepting two write hits (a controller-wide write-commit "
        "backpressure). The RTL write path serializes through the write-info FIFO + per-way "
        "commit, so sustained write throughput is ~½ the read throughput (RTL 0.49 vs 0.86 "
        "acc/cyc). Default 1 = no extra serialization (writes pipeline like reads)."
    ))
    fwd_hit_latency_cycles: int = cfg_field(default=-1, dump=True, desc=(
        "Read-hit latency when the line is resident in the 1-entry forwarding buffer (the "
        "immediately-previous access touched the same line). The RTL forwards from the "
        "buffer combinationally, skipping the SRAM read → faster than a normal hit (RTL "
        "read-after-write same word = 7 vs normal hit 10). Only used when "
        "`use_forwarding_buffer=True`. Default -1 = 'same as hit_latency_cycles'."
    ))
    refill_bank_write_cycles: int = cfg_field(default=1, dump=True, desc=(
        "Extra cycles after a refill response arrives before pending MSHR requests serve. "
        "Default 1 reflects the unfolded `DataPartSplit=1` RTL default; set to 2 if "
        "modelling a folded `PartSplit>1` configuration."
    ))
    miss_penalty_cycles: int = cfg_field(default=0, dump=True, desc=(
        "Fixed cache-pipeline overhead added to a refill's completion, on top of the "
        "memory latency and refill_bank_write_cycles (miss detect + downstream issue + "
        "refill ingest/install + response). Calibrated against the RTL standalone "
        "testbench, where a cold read-miss = MemLatency + 17 cyc (see "
        "prompt/insitu_cache_calib_report.md). Default 0 (no change for existing configs)."
    ))
    folded_evict_penalty_cycles: int = cfg_field(default=0, dump=True, desc=(
        "Extra cycles on a dirty eviction to model the folded-SRAM full-line read. "
        "Default 0 (unfolded). Set to 3 to mimic the old `PartSplit=4` mode."
    ))
    mshr_drain_cycles_per_subarray: int = cfg_field(default=1, dump=True, desc=(
        "Cycles added per pending MSHR subarray when draining after a refill"
    ))

    # -------- Occupancy model (deferred-completion path; default OFF = inline) --------
    # The default model resolves a miss INLINE (synchronous): issue_refill -> the memory's
    # OK response -> refill_resp_handler in the same cycle, so FIFO/resource counters never
    # accumulate depth and per-access latency is flat. That suffices for latency + memory-
    # traffic calibration and is what the Spatz integration uses. Setting defer_refills=True
    # switches to an occupancy model where refills complete via scheduled events, so cache
    # resources (refill pool, evic FIFO, MSHR) bind, outstanding accumulates, and per-access
    # latency inflates under load — needed to match the RTL wide-refill (BurstLength=1) miss
    # *throughput* (see prompt/insitu_cache_calib_report.md §9.1 + REPORT_BL1.md).
    defer_refills: bool = cfg_field(default=False, dump=True, desc=(
        "Enable the deferred-completion occupancy path. Default False = inline resolution "
        "(unchanged behaviour; Spatz/BL4 take this path). True = refill completions are "
        "serialized through the install pipeline (refill_drain_cycles) so per-access latency "
        "inflates under load and miss throughput is install-rate-bound (calib wide config)."
    ))
    refill_drain_cycles: int = cfg_field(default=0, dump=True, desc=(
        "Min cycles between successive refill/writeback *completions* draining into the "
        "cache (only when defer_refills). Models the near-serial single-line install "
        "pipeline (~1 completion per N cycles) that inflates per-access latency under load "
        "and sets wide-mode miss throughput. 0 = no completion serialization. The driver's "
        "outstanding budget + this rate together reproduce the RTL wide-refill numbers."
    ))

    # -------- FIFO depths (§3.3) --------
    resp_fifo_depth: int = cfg_field(default=4, dump=True, desc=(
        "Hit response FIFO depth"
    ))
    retr_fifo_depth: int = cfg_field(default=16, dump=True, desc=(
        "MSHR retrieval FIFO depth (16 in newer RTL builds)"
    ))
    miss_fifo_depth: int = cfg_field(default=4, dump=True, desc=(
        "Miss request FIFO depth"
    ))
    evic_fifo_depth: int = cfg_field(default=4, dump=True, desc=(
        "Eviction (writeback) FIFO depth"
    ))
    wt_fifo_depth: int = cfg_field(default=4, dump=True, desc=(
        "Write-through FIFO depth"
    ))


class InsituCacheCoalescerConfig(Config):
    """Configuration for the write-through coalescer (RTL §10)."""

    cache_line_bytes: int = cfg_field(default=64, dump=True, desc=(
        "Cache line size in bytes — must match the controller"
    ))
    watchdog_cycles: int = cfg_field(default=4, dump=True, desc=(
        "Cycles with no new coalescable write before forcing a flush"
    ))


class InsituCacheIntercoConfig(Config):
    """Configuration for the upstream TCDM → cache-controller interconnect (RTL §5)."""

    num_inputs: int = cfg_field(default=20, dump=True, desc=(
        "Number of upstream TCDM request ports (num_cores * tcdm_ports_per_core)"
    ))
    num_outputs: int = cfg_field(default=4, dump=True, desc=(
        "Number of downstream cache controllers"
    ))
    dynamic_offset: int = cfg_field(default=2, dump=True, desc=(
        "Bit offset at which log2(num_outputs) bits select the controller. "
        "Default 2 => 4-byte granularity interleaving (bits [3:2] for num_outputs=4)."
    ))
    interco_latency_cycles: int = cfg_field(default=1, dump=True, desc=(
        "Fixed forward latency through the interco (1 cycle per §5)"
    ))
    enable_input_coalesce: bool = cfg_field(default=False, dump=True, desc=(
        "Model the input par-coalescer: same-cycle, same-line READ-HIT requests across "
        "ports to the same output collapse into ONE cache lookup. The first such read "
        "forwards normally; followers in the same cycle to the same line inherit its "
        "latency and do NOT re-consume the per-output accept slot — so N same-line reads "
        "cost ~one bank access (RTL coal_warm = ~4x the single-port hit rate). Misses are "
        "forwarded through (the controller's MSHR merge handles them → coal_cold mem_rd "
        "unchanged). Default False = no coalescing (Spatz/default path unchanged)."
    ))
    cache_line_bytes: int = cfg_field(default=64, dump=True, desc=(
        "Cache line size in bytes — used only by enable_input_coalesce to group same-line "
        "requests. Must match the controller's cache_line_bytes."
    ))
    coalesce_max_latency: int = cfg_field(default=-1, dump=True, desc=(
        "Only a forwarded read whose resulting latency is <= this many cycles seeds the "
        "coalescing window (i.e. only TRUE warm hits coalesce). A cold line that was just "
        "refilled inline reports a large (refill-sized) latency and so does NOT coalesce — "
        "its same-cycle followers fall through to the controller's MSHR merge, keeping the "
        "cold-miss-stream throughput unchanged. -1 = no limit (merge any OK read)."
    ))


@dataclass
class InsituCacheTileConfig:
    """Top-level tile configuration. Plain Python container — not a ``Config``.

    The tile Python component (``InsituCacheTile``) uses this to decide how many
    controllers/coalescers/intercos to instantiate, and passes the ``Config`` sub-objects
    down to each instantiated component.
    """

    num_controllers: int = 4
    num_cores: int = 4
    tcdm_ports_per_core: int = 5
    controller: InsituCacheControllerConfig = field(
        default_factory=InsituCacheControllerConfig)
    coalescer: InsituCacheCoalescerConfig = field(
        default_factory=InsituCacheCoalescerConfig)
    interco: InsituCacheIntercoConfig = field(
        default_factory=InsituCacheIntercoConfig)

    @property
    def num_tcdm_ports(self) -> int:
        """Total TCDM ports entering the tile (interco inputs)."""
        return self.num_cores * self.tcdm_ports_per_core


def make_cachepool_512_config() -> InsituCacheTileConfig:
    """Canonical ``cachepool_512`` configuration — the **production** CachePool cache.

    As of 2026-06-01 the shipping `cachepool_512` config is the production cache:
    **folded + hash-way + forwarding-buffer ON** (`l1d_use_folded=1`,
    `l1d_use_hash_way=1`, `l1d_use_fwd_buf=1`; see `insitu_cache_architecture_v2.md`
    §0.1). This factory tracks that default. (The earlier 2026-05-22 default of
    unfolded + LRU is now the explicit opt-in *conventional* cache — see
    :func:`make_cachepool_512_conventional_config`.)

    Geometry: 4-way, 64 B line, write-back, no SPM, no flush.

    Note on topology: this factory still produces a `num_controllers=4` tile,
    matching the prior OLD architecture. The latest RTL has a single wide cache +
    an N→1 coalescer (see v2 doc §11 Phase B). For Phase-A we keep the
    multi-controller GVSoC topology and tune the per-controller knobs to the
    production defaults. Full topology migration is tracked under Phase B.
    """
    controller = InsituCacheControllerConfig(
        cache_line_bytes=64,
        num_ways=4,
        num_sets=128,
        tcdm_word_bytes=4,
        refill_beat_bytes=16,
        # Production cachepool_512: hash-way select (folded + fwd-buffer require it).
        use_hash_way_select=True,
        use_forwarding_buffer=True,
        # Pure write-back (WriteThroughMode=0).
        write_through_mode=False,
        # Multi-read MSHR is opt-in; the legacy GVSoC behaviour matches `False`.
        enable_multi_read_pend=False,
        # SPM and flush off by default — see v2 doc §5/§6.
        enable_spm=False,
        bank_depth_for_spm=0,
        enable_flush=False,
        # Calibrated against the RTL standalone testbench (prompt/insitu_cache_calib_report.md),
        # which elaborates the production (folded+hash+fwd) DUT:
        #   warm read-hit (isolated) = 10 cyc → interco(1) + hit_latency(9).
        #   cold read-miss (isolated) = MemLatency + 17 cyc → folded refill_bank_write(2)
        #   + miss_penalty(7) + interco + drain. (Unfolded would be bank_write=1 +
        #   miss_penalty=8 — same total; see make_cachepool_512_conventional_config.)
        hit_latency_cycles=9,
        miss_penalty_cycles=7,
        # Write path (calibrated vs RTL warm_write): write hit acks 2 cyc faster than a
        # read returns (8 vs 10 incl. interco), and writes serialize at ~½ read throughput.
        write_hit_latency_cycles=7,   # interco(1) + 7 = 8 (RTL warm write)
        write_commit_cycles=2,        # ~0.49 acc/cyc sustained (RTL ~0.49)
        # Forwarding buffer: a read on the just-touched line forwards combinationally.
        fwd_hit_latency_cycles=6,     # interco(1) + 6 = 7 (RTL read-after-write same word)
        # DataPartSplit=4 (folded) ⇒ +1 refill bank-write cycle, folded-evict penalty.
        refill_bank_write_cycles=2,
        folded_evict_penalty_cycles=3,
        mshr_drain_cycles_per_subarray=1,
        resp_fifo_depth=4,
        retr_fifo_depth=16,
        miss_fifo_depth=4,
        evic_fifo_depth=4,
        wt_fifo_depth=4,
    )
    coalescer = InsituCacheCoalescerConfig(cache_line_bytes=64, watchdog_cycles=4)
    interco = InsituCacheIntercoConfig(
        num_inputs=4 * 5, num_outputs=4, dynamic_offset=2, interco_latency_cycles=1,
    )
    return InsituCacheTileConfig(
        num_controllers=4, num_cores=4, tcdm_ports_per_core=5,
        controller=controller, coalescer=coalescer, interco=interco,
    )


def make_cachepool_512_conventional_config() -> InsituCacheTileConfig:
    """The opt-in **conventional** (unfolded + LRU + no forwarding-buffer) cache.

    Matches the RTL `l1d_use_folded=0 / l1d_use_hash_way=0 / l1d_use_fwd_buf=0`
    selection (the plain set-associative cache). This was the default this model
    tracked on 2026-05-22; it is now an alternative. Same calibrated latency
    target (cold miss = MemLatency + 17, warm hit = 10) reached with the unfolded
    knob values (`refill_bank_write_cycles=1`, `miss_penalty_cycles=8`,
    `folded_evict_penalty_cycles=0`).
    """
    cfg = make_cachepool_512_config()
    cfg.controller.use_hash_way_select = False
    cfg.controller.use_forwarding_buffer = False
    cfg.controller.refill_bank_write_cycles = 1
    cfg.controller.folded_evict_penalty_cycles = 0
    cfg.controller.miss_penalty_cycles = 8
    return cfg


def make_cachepool_512_calib_config() -> InsituCacheTileConfig:
    """Single-controller geometry matching the RTL calibration DUT (one
    ``cachepool_cache_ctrl`` with 5 core ports), for use with the
    ``insitu_cache_calib`` testbench.

    The RTL standalone calibration testbench (``ManyRVData_rebase/reports/
    cache_calib/``) instruments exactly **one** controller serving 5 core ports
    (4 Spatz-VLSU + 1 Snitch scalar) and a single line-refill port. To diff
    GVSoC against it 1:1 we collapse the tile to a single controller and route
    all 5 TCDM ports to it:

    - 1 controller, 5 input ports (interco ``num_outputs=1``).
    - 4-way × 256-set × 64 B line = **64 KiB / controller**, i.e. 1024 lines —
      the RTL ``NumCacheEntry=1024`` per controller.
    - Inherits the **production** config (folded + hash-way + fwd-buffer,
      write-back), matching the RTL calib DUT elaboration banner
      (``PartSplit=4, Folded=1, Hash=1``). For the conventional cache instead,
      start from :func:`make_cachepool_512_conventional_config`.

    Note: the GVSoC tile has no input-side `par_coalescer` and no scalar
    bypass-xbar (Phase-B items), so port 4 is treated like the VLSU ports and the
    coalescer adds no input-path latency. Hit-pipeline depth is therefore tuned
    via the controller / interco latency knobs to match the RTL 10/7-cyc numbers.
    """
    cfg = make_cachepool_512_config()
    cfg.num_controllers = 1
    cfg.num_cores = 1
    cfg.tcdm_ports_per_core = 5            # 4 VLSU + 1 scalar → 5 input ports
    cfg.controller.num_sets = 256          # 1024 entries / 4 ways = 64 KiB / ctrl
    cfg.interco.num_inputs = 5
    cfg.interco.num_outputs = 1
    cfg.interco.dynamic_offset = 2
    # Input par-coalescer: the 4 VLSU ports reading the same line in the same cycle merge
    # into one cache lookup (RTL coal_warm). Hit-path only; misses still go through the
    # MSHR merge. Calib-only — the spatz tile's interco keeps this off.
    cfg.interco.enable_input_coalesce = True
    cfg.interco.cache_line_bytes = cfg.controller.cache_line_bytes
    # Only true warm hits coalesce: interco(1) + hit_latency(9) ≈ 10. A cold line refilled
    # inline reports a refill-sized latency (>> this) so its same-cycle followers do NOT
    # merge — they fall through to the controller's MSHR merge (coal_cold throughput intact).
    cfg.interco.coalesce_max_latency = cfg.controller.hit_latency_cycles + 7
    # Streaming read-hit pipelining: an isolated hit pays interco(1)+9 = 10 (RTL), a
    # back-to-back stream settles to interco(1)+6 = 7 (RTL). The coalescer inherits the
    # first reader's latency, so coal_warm follows to 7/7/7 with no further change.
    cfg.controller.streaming_hit_latency_cycles = cfg.controller.hit_latency_cycles - 3
    return cfg


def make_cachepool_512_legacy_config() -> InsituCacheTileConfig:
    """Pre-2026-05-22 (folded, hash-way, write-back+WT-coalesced) configuration.

    Retained for regression: this matches the OLD ``cachepool_512`` RTL on the
    ``cachepool_dev_refactoring`` branch as documented in
    `insitu_cache_architecture.md` (pre-v2). Use this for tests that pin the old
    timing baseline.
    """
    cfg = make_cachepool_512_config()
    cfg.controller.use_hash_way_select = True
    cfg.controller.use_forwarding_buffer = False  # predates the fwd-buffer param
    cfg.controller.write_through_mode = False  # was "hybrid" — see v1 doc §15
    cfg.controller.refill_bank_write_cycles = 2
    cfg.controller.folded_evict_penalty_cycles = 3
    return cfg
