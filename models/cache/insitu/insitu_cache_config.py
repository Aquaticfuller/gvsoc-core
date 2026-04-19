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
Defaults mirror the canonical ``cachepool_512`` RTL configuration documented in
``prompt/insitu_cache_architecture.md`` (§1.3 and §16).

The three ``Config`` subclasses below map 1:1 onto the three GVSoC ``Component`` subclasses
that make up the model: ``InsituCacheController``, ``InsituCacheCoalescer``, and
``InsituCacheInterco``. Each Component's constructor takes its matching Config and the
Config's fields become the component's published properties (read from C++ via
``get_js_config()``).

``InsituCacheTileConfig`` is a plain Python class (not a ``Config``) that bundles the three
together for the tile wrapper.
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
        "(tag, set). When false, LRU is used. Hits always use tag-match."
    ))

    # -------- Timing knobs (match insitu_cache_architecture.md §12) --------
    hit_latency_cycles: int = cfg_field(default=4, dump=True, desc=(
        "Cycles from request acceptance at the controller to response emit on a hit"
    ))
    refill_bank_write_cycles: int = cfg_field(default=2, dump=True, desc=(
        "Extra cycles after a refill response arrives before pending MSHR requests serve"
    ))
    folded_evict_penalty_cycles: int = cfg_field(default=3, dump=True, desc=(
        "Extra cycles on a dirty eviction to model the folded-SRAM full-line read"
    ))
    mshr_drain_cycles_per_subarray: int = cfg_field(default=1, dump=True, desc=(
        "Cycles added per pending MSHR subarray when draining after a refill"
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
    """Canonical ``cachepool_512`` configuration from RTL §1.3.

    1 tile, 4 cores, 5 TCDM ports per core, 4 controllers, 4-way, 128 sets per way,
    512b line (64B), write-through coalesced, hash way select.
    """
    controller = InsituCacheControllerConfig(
        cache_line_bytes=64,
        num_ways=4,
        num_sets=128,
        tcdm_word_bytes=4,
        refill_beat_bytes=16,
        use_hash_way_select=True,
        hit_latency_cycles=4,
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
