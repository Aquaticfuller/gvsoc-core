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

        # -------- wiring --------
        # TCDM inputs → interco inputs (pass-through this composite's slave ports)
        for p in range(n_ports):
            self.bind(self, f'in_{p}', self._interco, f'in_{p}')

        # Interco outputs → controllers; controllers' write-through → coalescers.
        # All cross-tile L2 traffic (refill + evict + write-through-flush) fans into a
        # single composite master port `l2` which the external caller binds via o_L2().
        # This follows the hierarchical_cache.py pattern (multiple inner masters → one
        # composite master → one external slave).
        for i in range(n_ctrl):
            self._interco.o_OUTPUT(i, self._ctrls[i].i_INPUT())
            self._ctrls[i].o_WRITE_THROUGH(self._coals[i].i_INPUT())

            # Fan-in to the tile's composite `l2` master. The binding framework
            # multiplexes the inner masters onto the single external destination.
            self.bind(self._coals[i], 'out',   self, 'l2')
            self.bind(self._ctrls[i], 'refill', self, 'l2')
            self.bind(self._ctrls[i], 'evict',  self, 'l2')

            # Flush: expose one composite slave port per controller so the cluster L1D-flush
            # peripheral can invalidate every controller (cache_sync flush-all). Only in the
            # multi-controller (cluster) config — the single-controller open-loop calib DUT has
            # no flush driver, so binding an externally-undriven composite port there is skipped.
            if n_ctrl > 1:
                self.bind(self, f'flush_{i}', self._ctrls[i], 'flush')

    # ---------- port factories ----------

    def i_INPUT(self, port: int) -> SlaveItf:
        """TCDM request port ``port`` (0 ≤ port < num_cores * tcdm_ports_per_core)."""
        if port < 0 or port >= self._config.num_tcdm_ports:
            raise RuntimeError(
                f'InsituCacheTile port {port} out of range [0, '
                f'{self._config.num_tcdm_ports})')
        return SlaveItf(self, f'in_{port}', signature='io')

    def num_flush_ports(self) -> int:
        """Number of flush ports = number of controllers (one per controller)."""
        return self._config.num_controllers

    def i_FLUSH(self, ctrl: int) -> SlaveItf:
        """Flush/invalidate trigger for controller ``ctrl`` (a write invalidates its lines)."""
        return SlaveItf(self, f'flush_{ctrl}', signature='io')

    def o_L2(self, itf: SlaveItf):
        """Bind the tile's L2 output to ``itf``.

        Fan-in of every controller's refill + evict + each coalescer's write-through
        flush to a single external slave. The composite framework routes any of the
        inner masters (bound to `self.l2` in ``__init__``) to ``itf``.
        """
        self.itf_bind('l2', itf, signature='io')
