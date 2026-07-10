#
# Copyright (C) 2026 GreenWaves Technologies, SAS, ETH Zurich and
#                    University of Bologna
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

"""
Python generator for the IoV2SingleReqWidthAdapter component.

Auto-inserted by the gvrun2 systree binding pass when an ``IoV2SingleReq``
master binds an ``IoV2SingleReq`` slave whose declared width granule is
tighter than what the master guarantees (master width 0 = unknown, or wider
than the slave's — see ``IoV2SingleReq`` in ``gvsoc/signature.py``).

The slave-side width models a bank-interleaved fabric (e.g. the GAP9 shared-L2
``LogIco`` with a 4-byte interleave): an access crossing an aligned granule
boundary would be routed whole to one bank and alias a different address. The
adapter makes the granule a guarantee instead of a convention: any access that
fits within one aligned granule passes straight through (stateless, zero
latency, any number outstanding); an access that would straddle a boundary is
chopped into granule-aligned sub-accesses issued sequentially on the single
downstream port, and the master still gets its own request object back on
completion (single-req identity contract). Atomics are never split (asserted).
"""

from config_tree import Config, cfg_field
from gvsoc.systree import Component, SlaveItf
from gvsoc.signature import IoV2SingleReq


class IoV2SingleReqWidthAdapterConfig(Config):
    """Configuration for the io_v2 single-req width adapter."""

    width: int = cfg_field(default=0, dump=True, desc=(
        "Downstream width granule in bytes (power of two): accesses are split "
        "so that none crosses an aligned granule boundary."
    ))


class IoV2SingleReqWidthAdapter(Component):

    def __init__(self, parent: Component, name: str, width: int):
        if width <= 0 or (width & (width - 1)) != 0:
            raise RuntimeError(
                f'IoV2SingleReqWidthAdapter requires a power-of-two width '
                f'(got {width})')
        super().__init__(parent, name, config=IoV2SingleReqWidthAdapterConfig(
            width=width))
        self.set_component('utils.io_v2_single_req_width_adapter')
        self._width = width

    def i_INPUT(self) -> SlaveItf:
        return SlaveItf(self, 'input', signature=IoV2SingleReq())

    def o_OUTPUT(self, slave: SlaveItf):
        self.itf_bind('output', slave, signature=IoV2SingleReq(self._width))
