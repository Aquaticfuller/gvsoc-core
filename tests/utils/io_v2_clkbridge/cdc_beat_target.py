# SPDX-FileCopyrightText: 2026 ETH Zurich, University of Bologna and EssilorLuxottica SAS
#
# SPDX-License-Identifier: Apache-2.0
#
# Authors: Germain Haugou (germain.haugou@gmail.com)

import gvsoc.systree
from gvsoc.signature import IoV2Beat


class CDCBeatTarget(gvsoc.systree.Component):
    """Beat-plane io_v2 memory target stub for the clock-bridge testbench.

    Implements the per-burst write-acknowledgement contract (io_v2.hpp,
    "Write acknowledgement"): every granted write beat is consumed and
    freed (non-last beats get no response), and each burst is acknowledged
    exactly once with a distinct data-less pool-backed ack. Reads arrive as
    data-less burst requests and are answered with distinct allocator-backed
    response beats, one per ``beat_width`` per cycle.
    """

    def __init__(self, parent: gvsoc.systree.Component, name: str, *,
                 size: int, beat_width: int):
        super().__init__(parent, name)
        self.add_sources(['cdc_beat_target.cpp'])
        self.add_property('size', size)
        self.add_property('beat_width', beat_width)
        self.beat_width = beat_width

    def i_INPUT(self) -> gvsoc.systree.SlaveItf:
        return gvsoc.systree.SlaveItf(
            self, 'input', signature=IoV2Beat(beat_width=self.beat_width))
