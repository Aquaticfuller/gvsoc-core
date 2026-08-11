#
# Copyright (C) 2026 ETH Zurich and University of Bologna
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#

"""InSitu cache — inter-tile remote crossbar (cachepool_remote_xbar), one per port-class.

Routes a cross-tile request from a source tile's remote-out to the TARGET tile's remote-in by the
address's TileID field (extends the shared L1 across tiles). num_tiles inputs × num_tiles outputs.
See ``cachepool_group.sv:399-432``.
"""

from __future__ import annotations

from gvsoc.systree import Component, SlaveItf

# L1-NoC tunnel: off-group requests are re-addressed to `NOC_TUNNEL_BASE + group * NOC_TUNNEL_STRIDE
# + addr` so the mesh routes on the destination the crossbar computed with the CURRENT runtime
# geometry, instead of re-decoding the address with a map fixed at elaboration time (the interleaving
# granularity is runtime-programmable via XBAR_OFFSET). The NoC map strips the tunnel again with
# remove_offset, so the destination sees the untouched address. A full 32-bit space per group, above
# 4 GiB where nothing else lives. The cluster imports these so both sides cannot drift.
NOC_TUNNEL_BASE = 1 << 32
NOC_TUNNEL_STRIDE = 1 << 32


class InsituCacheRemoteXbar(Component):
    """One per-port-class inter-tile router: route by target tile-id."""

    def __init__(self, parent: Component, name: str, *,
                 num_tiles: int, num_cores: int, num_cache: int, num_remote_port_core: int = 1,
                 dynamic_offset: int = 6, addr_width: int = 32, hop_latency_cycles: int = 0,
                 num_groups: int = 1, tiles_per_group: int = 0, group_id: int = 0,
                 noc_tunnel_base: int = NOC_TUNNEL_BASE,
                 noc_tunnel_stride: int = NOC_TUNNEL_STRIDE):
        super().__init__(parent, name)
        self.add_sources(['cache/insitu/insitu_cache_remote_xbar.cpp'])
        self.add_properties({
            'num_tiles': num_tiles,
            'num_cores': num_cores,
            'num_cache': num_cache,
            'num_remote_port_core': num_remote_port_core,
            'dynamic_offset': dynamic_offset,
            'addr_width': addr_width,
            'hop_latency_cycles': hop_latency_cycles,
            # P1: num_groups>1 makes num_tiles the CLUSTER-GLOBAL tile count and adds the NoC
            # egress/ingress slots; tiles_per_group + group_id say which targets are local.
            'num_groups': num_groups,
            'tiles_per_group': tiles_per_group if tiles_per_group else num_tiles,
            'group_id': group_id,
            'noc_tunnel_base': noc_tunnel_base,
            'noc_tunnel_stride': noc_tunnel_stride,
        })

    def i_NOC_IN(self, slot: int) -> SlaveItf:
        """Off-group request arriving from the L1 NoC (routed on to a local tile)."""
        return SlaveItf(self, f'noc_in_{slot}', signature='io')

    def o_NOC_OUT(self, slot: int, itf: SlaveItf):
        """Off-group request leaving toward the L1 NoC."""
        self.itf_bind(f'noc_out_{slot}', itf, signature='io')

    def i_INPUT(self, slot: int) -> SlaveItf:
        """Input slot = src_tile*num_remote_port_core + r (a source tile's remote-out)."""
        return SlaveItf(self, f'in_{slot}', signature='io')

    def o_OUTPUT(self, slot: int, itf: SlaveItf):
        """Output slot = tgt_tile*num_remote_port_core + r (bind to a target tile's remote-in)."""
        self.itf_bind(f'out_{slot}', itf, signature='io')
