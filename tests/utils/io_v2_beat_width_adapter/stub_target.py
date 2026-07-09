# SPDX-FileCopyrightText: 2026 ETH Zurich, University of Bologna and EssilorLuxottica SAS
#
# SPDX-License-Identifier: Apache-2.0
#
# Authors: Germain Haugou (germain.haugou@gmail.com)

import gvsoc.systree
from gvsoc.signature import IoV2Beat


class StubTarget(gvsoc.systree.Component):
    """io_v2 beat testbench target of width ``beat_width``.

    Streams distinct allocator-backed read response beats one per cycle and
    checks/acks per-beat writes. Declaring ``IoV2Beat(beat_width)`` on its
    input makes the framework insert IoV2BeatWidthAdapter when the bound
    master uses a different width.
    """
    def __init__(self, parent, name, beat_width=4, latency=0, error=False,
                 async_ack=False, deny_count=0, retry_delay=2,
                 base=0, size=0, logname=None):
        super().__init__(parent, name)
        self.add_sources(['stub_target.cpp'])
        self.add_property('logname', logname or name)
        self.add_property('beat_width', beat_width)
        self.add_property('latency', latency)
        self.add_property('error', error)
        self.add_property('async_ack', async_ack)
        self.add_property('deny_count', deny_count)
        self.add_property('retry_delay', retry_delay)
        self.add_property('base', base)
        self.add_property('size', size)
        self._beat_width = beat_width

    def i_INPUT(self):
        return gvsoc.systree.SlaveItf(self, 'input',
                                      signature=IoV2Beat(self._beat_width))
