#
# Copyright (C) 2026 ETH Zurich and University of Bologna
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#

"""InSitu cache — structural per-cell par_coalescer (4 VLSU lanes → 1 wide cache beat).

Wraps the validated ``insitu_cache_coalesce.hpp`` datapath in a per-cycle shell: same-cycle same-line
reads across the N input lanes coalesce into one wide line-read to the core, and the wide response is
split back per merged port. Writes pass through. The RTL ``par_coalescer`` inside ``cachepool_cache_ctrl``
(``cachepool_cache_ctrl.sv:344``). The scalar lane bypasses this and feeds the core's second input.
"""

from __future__ import annotations

from gvsoc.systree import Component, SlaveItf


class InsituCacheCellCoalescer(Component):
    """N VLSU-lane inputs → 1 wide output to the cache core, with read coalescing + response split."""

    def __init__(self, parent: Component, name: str, *,
                 num_inputs: int, cache_line_bytes: int = 64, word_bytes: int = 4):
        super().__init__(parent, name)
        self.add_sources(['cache/insitu/insitu_cache_cell_coalescer.cpp'])
        self.add_properties({
            'num_inputs': num_inputs,
            'cache_line_bytes': cache_line_bytes,
            'word_bytes': word_bytes,
        })

    def i_INPUT(self, port: int) -> SlaveItf:
        """VLSU lane input ``port`` (0 ≤ port < num_inputs)."""
        return SlaveItf(self, f'in_{port}', signature='io')

    def o_OUTPUT(self, itf: SlaveItf):
        """Bind the single wide output to the cache core's (VLSU-aggregate) input."""
        self.itf_bind('out', itf, signature='io')
