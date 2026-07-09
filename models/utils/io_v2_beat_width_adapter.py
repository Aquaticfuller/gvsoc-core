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
Python generator for the IoV2BeatWidthAdapter component.

Auto-inserted by the gvrun2 systree binding pass when a master port declares
signature ``IoV2Beat(input_width)`` and the bound slave port declares
signature ``IoV2Beat(output_width)`` with a *different* width (see
``IoV2Beat.bridge_to`` in ``gvsoc/signature.py``). Both sides speak the same
beat sub-protocol; only the beat granularity changes. The adapter repacks the
per-beat streams in both directions — N narrow beats become one wide beat and
vice versa — so each binding runs at its own natural width: the narrow side
carries one beat per cycle at full occupancy while the wide side carries the
same bytes in fewer, wider beats (one every ratio cycles). Aggregate byte
bandwidth is bounded by the narrow side, as in a HW bus-width converter.

The wider width must be an integer multiple of the narrower one.
"""

from config_tree import Config, cfg_field
from gvsoc.systree import Component, SlaveItf
from gvsoc.signature import IoV2Beat


class IoV2BeatWidthAdapterConfig(Config):
    """Configuration for the io_v2 beat-width conversion adapter."""

    input_width: int = cfg_field(default=0, dump=True, desc=(
        "Beat width in bytes on the input (upstream master) side."
    ))
    output_width: int = cfg_field(default=0, dump=True, desc=(
        "Beat width in bytes on the output (downstream slave) side."
    ))
    read_fifo_depth: int = cfg_field(default=0, dump=True, desc=(
        "Read response FIFO depth, in input_width beats: how many repacked "
        "upstream read beats the adapter buffers before back-pressuring the "
        "downstream producer (IO_RESP_DENIED). 0 = auto (2x the width ratio, "
        "the minimum for full streaming). Larger values absorb bursts — e.g. "
        "an upstream master that stalls its response channel — without "
        "stalling the downstream."
    ))
    write_fifo_depth: int = cfg_field(default=0, dump=True, desc=(
        "Write FIFO depth, in output_width chunks: how many complete "
        "downstream write beats may be queued (built but not yet issued) "
        "before upstream write requests are back-pressured (IO_REQ_DENIED). "
        "0 = auto (2). Larger values let the adapter absorb a whole write "
        "burst at the upstream rate before throttling the writer."
    ))


class IoV2BeatWidthAdapter(Component):

    def __init__(self, parent: Component, name: str, input_width: int,
                 output_width: int, read_fifo_depth: int = 0,
                 write_fifo_depth: int = 0):
        if input_width <= 0 or output_width <= 0:
            raise RuntimeError(
                f'IoV2BeatWidthAdapter requires positive beat widths '
                f'(got input={input_width}, output={output_width})')
        if max(input_width, output_width) % min(input_width, output_width) != 0:
            raise RuntimeError(
                f'IoV2BeatWidthAdapter requires the wider beat width to be an '
                f'integer multiple of the narrower one '
                f'(got input={input_width}, output={output_width})')

        super().__init__(parent, name, config=IoV2BeatWidthAdapterConfig(
            input_width=input_width, output_width=output_width,
            read_fifo_depth=read_fifo_depth, write_fifo_depth=write_fifo_depth))
        self.set_component('utils.io_v2_beat_width_adapter')
        self._input_width = input_width
        self._output_width = output_width

    def i_INPUT(self) -> SlaveItf:
        return SlaveItf(self, 'input', signature=IoV2Beat(self._input_width))

    def o_OUTPUT(self, slave: SlaveItf):
        self.itf_bind('output', slave, signature=IoV2Beat(self._output_width))
