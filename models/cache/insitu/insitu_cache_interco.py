#
# Copyright (C) 2026 ETH Zurich and University of Bologna
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#

"""InSitu cache interconnect — hashed TCDM ports → cache controllers.

Mirrors the ``tcdm_cache_interco`` RTL module described in
``prompt/insitu_cache_architecture.md`` §5.

Routing: ``controller_id = (addr >> dynamic_offset) & (num_outputs - 1)``.
Arbitration: simple round-robin per cycle per output port (at most 1 accepted per cycle
per output). Requests past the first on a contended output pay a serialization cost.
"""

from __future__ import annotations

from gvsoc.systree import Component, SlaveItf

from cache.insitu.insitu_cache_config import InsituCacheIntercoConfig


class InsituCacheInterco(Component):
    """N-input M-output hashed crossbar (address-interleaved at the cacheline word level).

    Parameters
    ----------
    parent, name : standard GVSoC systree args
    config : InsituCacheIntercoConfig
    """

    def __init__(self, parent: Component, name: str,
                 config: InsituCacheIntercoConfig | None = None):
        if config is None:
            config = InsituCacheIntercoConfig()

        super().__init__(parent, name, config=config)

        self.add_sources(['cache/insitu/insitu_cache_interco.cpp'])

        self.add_properties({
            'num_inputs': config.num_inputs,
            'num_outputs': config.num_outputs,
            'dynamic_offset': config.dynamic_offset,
            'interco_latency_cycles': config.interco_latency_cycles,
            'enable_input_coalesce': config.enable_input_coalesce,
            'cache_line_bytes': config.cache_line_bytes,
            'coalesce_max_latency': config.coalesce_max_latency,
            'forward_initiator': config.forward_initiator,
            'per_cycle_output_arb': config.per_cycle_output_arb,
            'output_accept_width': config.output_accept_width,
        })

    def i_INPUT(self, port: int) -> SlaveItf:
        """Upstream TCDM request port ``port`` (0 ≤ port < num_inputs)."""
        return SlaveItf(self, f'in_{port}', signature='io')

    def o_OUTPUT(self, port: int, itf: SlaveItf):
        """Bind the downstream cache-controller ``port`` (0 ≤ port < num_outputs)."""
        self.itf_bind(f'out_{port}', itf, signature='io')
