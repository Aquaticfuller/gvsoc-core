#
# Copyright (C) 2026 ETH Zurich and University of Bologna
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#

"""InSitu cache parallel coalescer (``par_coalescer``) — per-controller front-end.

Phase-1 increment 1: the structural extraction of the interco's
``enable_input_coalesce`` window. One par_coalescer sits between an interco output and
its cache controller and performs the same-cycle / same-line read merge plus the
output-accept arbitration for that controller's request stream. See
``prompt/insitu_cache_architecture_v2.md`` §8 and ``insitu_cache_par_coalescer.cpp``.

Wired by :class:`InsituCacheTile` only when ``use_structural_coalescer=True``; otherwise
the merge stays inline in the interco (today's calibrated path) and this component is not
instantiated.
"""

from __future__ import annotations

from gvsoc.systree import Component, SlaveItf

from cache.insitu.insitu_cache_config import InsituCacheParCoalescerConfig


class InsituCacheParCoalescer(Component):
    """1-input / 1-output read-merge + output-arbitration front-end for one controller."""

    def __init__(self, parent: Component, name: str,
                 config: InsituCacheParCoalescerConfig | None = None):
        if config is None:
            config = InsituCacheParCoalescerConfig()

        super().__init__(parent, name, config=config)

        self.add_sources(['cache/insitu/insitu_cache_par_coalescer.cpp'])

        self.add_properties({
            'interco_latency_cycles': config.interco_latency_cycles,
            'merge_same_line_reads': config.merge_same_line_reads,
            'cache_line_bytes': config.cache_line_bytes,
            'coalesce_max_latency': config.coalesce_max_latency,
            'output_accept_width': config.output_accept_width,
            'per_cycle_output_arb': config.per_cycle_output_arb,
        })

    def i_INPUT(self) -> SlaveItf:
        """Upstream request port (from the interco output)."""
        return SlaveItf(self, 'in', signature='io')

    def o_OUTPUT(self, itf: SlaveItf):
        """Bind the downstream cache-controller input."""
        self.itf_bind('out', itf, signature='io')
