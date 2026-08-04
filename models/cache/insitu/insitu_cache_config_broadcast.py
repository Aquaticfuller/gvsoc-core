#
# Copyright (C) 2026 ETH Zurich and University of Bologna
#
# Licensed under the Apache License, Version 2.0 (the "License").
#

"""InSitu cache — E3 partition-config broadcast shim (1 slave / M masters)."""

from __future__ import annotations

from gvsoc.systree import Component, SlaveItf


class InsituCacheConfigBroadcast(Component):

    def __init__(self, parent: Component, name: str, *, nb_masters: int):
        super().__init__(parent, name)
        self.add_sources(['cache/insitu/insitu_cache_config_broadcast.cpp'])
        self.add_properties({'nb_masters': nb_masters})

    def i_INPUT(self) -> SlaveItf:
        return SlaveItf(self, 'input', signature='io')

    def o_OUTPUT(self, port: int, itf: SlaveItf):
        self.itf_bind(f'out_{port}', itf, signature='io')
