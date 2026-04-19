#
# Copyright (C) 2026 ETH Zurich and University of Bologna
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#

"""InSitu Cache tile — composite component.

Assembles one tile's worth of cache: a hashed interco on the TCDM side, N cache
controllers (one per core in the canonical config), N write-through coalescers (one per
controller), and a simple fan-in router on the L2 side.

Topology (default ``cachepool_512``):

    i_INPUT(0..19) ─► interco ─┬─► ctrl[0] ─┬─► REFILL ───────► l2_router ─► o_L2
                               ├─► ctrl[1] ─┼─► EVICT  ───────►
                               ├─► ctrl[2] ─┴─► WRITE_THROUGH ─► coal[0..3] ─►
                               └─► ctrl[3]

The tile's ``o_L2`` is a direct pass-through of the inner ``l2_router``'s default output,
so the external caller can bind to any downstream memory (SPM, DRAM, …) without the tile
imposing extra plumbing.
"""

from __future__ import annotations

from gvsoc.systree import Component, SlaveItf
import interco.router as router

from cache.insitu.insitu_cache_config import (
    InsituCacheTileConfig,
    make_cachepool_512_config,
)
from cache.insitu.insitu_cache_controller import InsituCacheController
from cache.insitu.insitu_cache_coalescer import InsituCacheCoalescer
from cache.insitu.insitu_cache_interco import InsituCacheInterco


class InsituCacheTile(Component):
    """One tile of the InSitu cache.

    Exposes ``i_INPUT(port)`` for each TCDM request port (total = num_cores *
    tcdm_ports_per_core) and a single ``o_L2`` master that fans in miss / evict /
    write-through traffic from all inner controllers.

    Parameters
    ----------
    parent, name : standard GVSoC systree args
    config : InsituCacheTileConfig | None
        Full tile config. If ``None``, :func:`make_cachepool_512_config` is used.
    """

    def __init__(self, parent: Component, name: str,
                 config: InsituCacheTileConfig | None = None):
        super().__init__(parent, name)

        if config is None:
            config = make_cachepool_512_config()
        self._config = config

        n_ctrl = config.num_controllers
        n_ports = config.num_tcdm_ports

        # Keep interco fields consistent with tile topology.
        config.interco.num_inputs = n_ports
        config.interco.num_outputs = n_ctrl

        # -------- sub-components --------

        self._interco = InsituCacheInterco(self, 'interco', config=config.interco)

        self._ctrls = []
        self._coals = []
        for i in range(n_ctrl):
            ctrl = InsituCacheController(self, f'ctrl_{i}', config=config.controller)
            coal = InsituCacheCoalescer(self, f'coal_{i}', config=config.coalescer)
            self._ctrls.append(ctrl)
            self._coals.append(coal)

        # Simple fan-in router. Synchronous + no bandwidth cap — the downstream memory
        # is expected to carry the BW model. Each distinct inner-master (refill/evict/wt)
        # gets its own input port so they don't artificially serialize at the router.
        self._l2_router = router.Router(self, 'l2_router', bandwidth=0, synchronous=True)

        # -------- wiring --------

        # TCDM inputs → interco inputs (pass-through this composite's slave ports)
        for p in range(n_ports):
            self.bind(self, f'in_{p}', self._interco, f'in_{p}')

        # Interco outputs → cache controllers
        # Each controller: refill + evict + WT → dedicated l2_router input
        next_router_port = 0
        for i in range(n_ctrl):
            self._interco.o_OUTPUT(i, self._ctrls[i].i_INPUT())

            # WRITE_THROUGH → coalescer → router
            self._ctrls[i].o_WRITE_THROUGH(self._coals[i].i_INPUT())
            self._coals[i].o_OUT(self._l2_router.i_INPUT(next_router_port))
            next_router_port += 1

            # REFILL → router (distinct port)
            self._ctrls[i].o_REFILL(self._l2_router.i_INPUT(next_router_port))
            next_router_port += 1

            # EVICT → router (distinct port)
            self._ctrls[i].o_EVICT(self._l2_router.i_INPUT(next_router_port))
            next_router_port += 1

    # ---------- port factories ----------

    def i_INPUT(self, port: int) -> SlaveItf:
        """TCDM request port ``port`` (0 ≤ port < num_cores * tcdm_ports_per_core)."""
        if port < 0 or port >= self._config.num_tcdm_ports:
            raise RuntimeError(
                f'InsituCacheTile port {port} out of range [0, '
                f'{self._config.num_tcdm_ports})')
        return SlaveItf(self, f'in_{port}', signature='io')

    def o_L2(self, itf: SlaveItf):
        """Bind the tile's L2 output directly to ``itf``.

        The binding is installed on the inner l2_router's default output map — requests
        at any router input end up on this external slave.
        """
        # Catch-all mapping (no base/size → accept everything, rm_base=False to preserve
        # absolute addresses).
        self._l2_router.o_MAP(itf, rm_base=False)
