#
# Copyright (C) 2026 ETH Zurich and University of Bologna
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#

"""InSitu cache write-through coalescer.

Implements the watchdog-based coalescing state machine from
``prompt/insitu_cache_architecture.md`` §10. Absorbs a burst of word-granularity writes
to a single cache line and emits one wide (full-line) burst downstream.

Ports:
    i_INPUT        (slave, io)    — incoming write-through traffic from the cache controller
    o_OUT          (master, io)   — out to L2 (wide, full-line)
    i_READ_SNOOP   (slave, io)    — read snoop: a matching-tag read forces immediate flush

Note: for the initial phase-1 wiring we bypass the coalescer and forward writes directly.
This component is plugged in once the rest is validated (see plan §3).
"""

from __future__ import annotations

from gvsoc.systree import Component, SlaveItf

from cache.insitu.insitu_cache_config import InsituCacheCoalescerConfig


class InsituCacheCoalescer(Component):
    def __init__(self, parent: Component, name: str,
                 config: InsituCacheCoalescerConfig | None = None):
        if config is None:
            config = InsituCacheCoalescerConfig()

        super().__init__(parent, name, config=config)

        self.add_sources(['cache/insitu/insitu_cache_coalescer.cpp'])

        self.add_properties({
            'cache_line_bytes': config.cache_line_bytes,
            'watchdog_cycles': config.watchdog_cycles,
        })

    def i_INPUT(self) -> SlaveItf:
        return SlaveItf(self, 'input', signature='io')

    def i_READ_SNOOP(self) -> SlaveItf:
        return SlaveItf(self, 'read_snoop', signature='io')

    def o_OUT(self, itf: SlaveItf):
        self.itf_bind('out', itf, signature='io')
