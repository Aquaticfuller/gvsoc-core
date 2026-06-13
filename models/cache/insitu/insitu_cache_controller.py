#
# Copyright (C) 2026 ETH Zurich and University of Bologna
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#

"""Single InSitu cache controller (GVSoC performance model).

Corresponds to ``insitu_cache_tcdm_wrapper`` + ``insitu_cache_core`` in the RTL
(``prompt/insitu_cache_architecture.md`` §§6–7). One Python component instance maps to
one C++ module compiled from ``insitu_cache_controller.cpp``.

Exposed ports:
    i_INPUT       (slave, io)         — TCDM request from upstream interco (32-bit word)
    o_REFILL      (master, io)        — line-fill request to L2 (size = cache_line_bytes)
    o_EVICT       (master, io)        — dirty writeback (size = cache_line_bytes)
    o_WRITE_THROUGH (master, io)      — write-through output (typically wired to coalescer)

Counters emitted at end of simulation (via trace):
    rd_hit, rd_miss, wr_hit, wr_miss, mshr_merge, evict, wb_dirty,
    stall_miss_fifo, stall_evic_fifo, stall_mshr_full
"""

from __future__ import annotations

from gvsoc.systree import Component, SlaveItf

from cache.insitu.insitu_cache_config import InsituCacheControllerConfig


class InsituCacheController(Component):
    """One in-situ cache controller.

    The model is cycle-approximate, not cycle-exact: pipeline stages are folded into fixed
    per-transaction latencies and aggregate busy-until cyclestamps. See §1 of
    ``prompt/insitu_cache_gvsoc_plan.md`` for the modeling philosophy.

    Parameters
    ----------
    parent, name : standard GVSoC systree args
    config : InsituCacheControllerConfig
        Knobs. Defaults match RTL ``cachepool_512``.
    """

    def __init__(self, parent: Component, name: str,
                 config: InsituCacheControllerConfig | None = None):
        if config is None:
            config = InsituCacheControllerConfig()

        super().__init__(parent, name, config=config)

        self.add_sources(['cache/insitu/insitu_cache_controller.cpp'])

        # Mirror Config fields to properties so the C++ side can read them via
        # get_js_config()->get_child_int()/get_child_bool().
        self.add_properties({
            'cache_line_bytes': config.cache_line_bytes,
            'num_ways': config.num_ways,
            'num_sets': config.num_sets,
            'tcdm_word_bytes': config.tcdm_word_bytes,
            'refill_beat_bytes': config.refill_beat_bytes,
            'use_hash_way_select': config.use_hash_way_select,
            'use_forwarding_buffer': config.use_forwarding_buffer,
            'write_through_mode': config.write_through_mode,
            'enable_multi_read_pend': config.enable_multi_read_pend,
            'enable_spm': config.enable_spm,
            'bank_depth_for_spm': config.bank_depth_for_spm,
            'enable_flush': config.enable_flush,
            'hit_latency_cycles': config.hit_latency_cycles,
            'streaming_hit_latency_cycles': config.streaming_hit_latency_cycles,
            'bank_accept_cycles': config.bank_accept_cycles,
            'scalar_bypass_port': config.scalar_bypass_port,
            'scalar_hit_latency_cycles': config.scalar_hit_latency_cycles,
            'write_hit_latency_cycles': config.write_hit_latency_cycles,
            'write_commit_cycles': config.write_commit_cycles,
            'fwd_hit_latency_cycles': config.fwd_hit_latency_cycles,
            'miss_penalty_cycles': config.miss_penalty_cycles,
            'refill_bank_write_cycles': config.refill_bank_write_cycles,
            'folded_evict_penalty_cycles': config.folded_evict_penalty_cycles,
            'mshr_drain_cycles_per_subarray': config.mshr_drain_cycles_per_subarray,
            'defer_refills': config.defer_refills,
            'refill_drain_cycles': config.refill_drain_cycles,
            'resp_fifo_depth': config.resp_fifo_depth,
            'retr_fifo_depth': config.retr_fifo_depth,
            'miss_fifo_depth': config.miss_fifo_depth,
            'evic_fifo_depth': config.evic_fifo_depth,
            'wt_fifo_depth': config.wt_fifo_depth,
        })

    # -------- Port factories --------

    def i_INPUT(self) -> SlaveItf:
        """TCDM request input (from upstream interco)."""
        return SlaveItf(self, 'input', signature='io')

    def o_REFILL(self, itf: SlaveItf):
        """Bind the line-fill request port to the next memory level."""
        self.itf_bind('refill', itf, signature='io')

    def o_EVICT(self, itf: SlaveItf):
        """Bind the eviction (dirty writeback) port."""
        self.itf_bind('evict', itf, signature='io')

    def o_WRITE_THROUGH(self, itf: SlaveItf):
        """Bind the write-through output (go to coalescer, then L2)."""
        self.itf_bind('write_through', itf, signature='io')
