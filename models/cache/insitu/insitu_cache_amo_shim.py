#
# Copyright (C) 2026 ETH Zurich and University of Bologna
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#

"""InSitu cache — AMO / LR-SC shim (spatz_cache_amo), on the scalar lane (j=4) of each cell.

Wraps ``insitu_cache_amo.hpp`` (ALU + LR/SC reservation). READ/WRITE pass through; LR sets the
reservation + presents a load; SC returns 0/1; true AMOs do a read-modify-write against the core.
One per controller; the Spatz VLSU lanes bypass it. See ``cachepool_tile.sv:658``.
"""

from __future__ import annotations

from gvsoc.systree import Component, SlaveItf


class InsituCacheAmo(Component):
    """AMO/LR-SC shim: 1 input (scalar lane) → 1 output (to the cache core's scalar input)."""

    def __init__(self, parent: Component, name: str, *, word_bytes: int = 4):
        super().__init__(parent, name)
        self.add_sources(['cache/insitu/insitu_cache_amo_shim.cpp'])
        self.add_properties({'word_bytes': word_bytes})

    def i_INPUT(self) -> SlaveItf:
        """Scalar-lane request input (from the tile's lane-4 crossbar)."""
        return SlaveItf(self, 'input', signature='io')

    def o_OUTPUT(self, itf: SlaveItf):
        """Bind the output to the cache core's scalar input."""
        self.itf_bind('out', itf, signature='io')
