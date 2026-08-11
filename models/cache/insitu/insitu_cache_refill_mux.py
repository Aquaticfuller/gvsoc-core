#
# Copyright (C) 2026 ETH Zurich and University of Bologna
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#

"""InSitu cache — group-level refill mux (v3-P3).

N wide refill masters share one downstream port. In the intended design each group aggregates
16 bank refill ports (4 tiles x 4 banks) plus 1 L2 instruction-cache port = 17 inputs, and the
single output feeds that group's L2 NoC router (v3-P4). The same component does the 4->1
aggregation of the tiles' L1 instruction-cache refills into the group L2 I$.

Instruction inputs are the LAST ``nb_priority_inputs`` inputs and win by strict priority; data
inputs are served round-robin. One request leaves per cycle, so the contention a shared port
creates is real simulated time rather than a latency stamp — stamped latency is discarded by an
asynchronous requester.
"""

from __future__ import annotations

from gvsoc.systree import Component, SlaveItf


class InsituCacheRefillMux(Component):
    """N-to-1 wide refill arbiter: instruction strict-priority, data round-robin, 1 req/cycle."""

    def __init__(self, parent: Component, name: str, *,
                 num_inputs: int,
                 nb_priority_inputs: int = 0,
                 forward_initiator: bool = True):
        super().__init__(parent, name)
        self.add_sources(['cache/insitu/insitu_cache_refill_mux.cpp'])
        self.add_properties({
            'num_inputs': num_inputs,
            # The last nb_priority_inputs inputs are the instruction ports (strict priority). For the
            # 17->1 group mux that is 1, making input 16 the L2 I$ port; the 4->1 icache mux uses 0,
            # so its four tile ports are plain round-robin.
            'nb_priority_inputs': nb_priority_inputs,
            # Stamp the winning input id onto the request (the RTL's user field). The response routes
            # itself back along the preserved resp-port chain; this is for observability and for
            # downstream components that arbitrate on the requester.
            'forward_initiator': forward_initiator,
        })

    def i_INPUT(self, slot: int) -> SlaveItf:
        """Refill master `slot`. Data ports first, then the instruction port(s)."""
        return SlaveItf(self, f'in_{slot}', signature='io')

    def o_OUTPUT(self, itf: SlaveItf):
        """The single shared downstream port (v3-P4: the group's L2 NoC router)."""
        self.itf_bind('out', itf, signature='io')
