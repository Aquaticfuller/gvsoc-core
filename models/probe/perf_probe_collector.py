#
# Copyright (C) 2026 ETH Zurich and University of Bologna
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

import gvsoc.systree

# Software probe port window, see core/models/probe/perf_probe_events.h.
PERF_PROBE_BASE = 0xC002_0000
PERF_PROBE_WINDOW_SIZE = 0x4_0000


class PerfProbeCollector(gvsoc.systree.Component):
    """Time-sliced performance-probe collector (prompt/perf_probe_design.md).

    Instantiate once per system. Every model that implements ``probe::Source`` finds this
    component through the engine's service registry and is sampled every ``slice_cycles``
    cycles; the per-slice deltas go to ``<out_dir>/<kind>.csv``. The ``input`` io slave is the
    software probe port: map ``PERF_PROBE_BASE`` (size ``PERF_PROBE_WINDOW_SIZE``) to it on the
    interconnect the cores reach the peripherals through.

    Parameters
    ----------
    slice_cycles : int
        Sampling period in cycles of this component's clock domain.
    out_dir : str
        Output directory, created if missing. Relative paths are relative to the run directory.
    kinds : str
        Comma-separated list of probe kinds to record (``core,cache,noc_router`` ...). Empty
        records every kind.
    """

    def __init__(self, parent, name, slice_cycles: int = 1000, out_dir: str = 'perf_probe',
                 kinds: str = ''):
        super().__init__(parent, name)
        self.add_sources(['probe/perf_probe_collector.cpp'])
        self.add_properties({
            'slice_cycles': int(slice_cycles),
            'out_dir': out_dir,
            'kinds': kinds,
        })

    def i_INPUT(self) -> gvsoc.systree.SlaveItf:
        """The software probe port (write-only, decoded by address)."""
        return gvsoc.systree.SlaveItf(self, 'input', signature='io')
