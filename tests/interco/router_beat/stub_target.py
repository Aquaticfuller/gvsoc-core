# SPDX-FileCopyrightText: 2026 ETH Zurich, University of Bologna and EssilorLuxottica SAS
#
# SPDX-License-Identifier: Apache-2.0
#
# Authors: Germain Haugou (germain.haugou@gmail.com)

import gvsoc.systree
from gvsoc.signature import IoV2Beat


class StubTarget(gvsoc.systree.Component):
    """io_v2 beat-mode testbench target.

    By default the target is a legacy-string big-packet slave (the framework
    inserts the general beat adapter between a beat router's output and it).
    With ``native_beat_width`` set it declares an ``IoV2Beat`` input of that
    width and binds the router output raw, implementing the native
    beat-target write contract: non-last write beats are consumed and freed
    (GRANTED, no resp), the last beat answers inline DONE (``done``
    behavior) or GRANTED plus one deferred data-less burst ack (``granted``
    behavior).
    """
    def __init__(self, parent, name, rules=None, logname=None,
                 native_beat_width=0):
        super().__init__(parent, name)
        self.add_sources(['stub_target.cpp'])
        self.add_property('logname', logname or name)
        self.add_property('rules', rules or [])
        self.add_property('native_beat', 1 if native_beat_width else 0)
        self.native_beat_width = native_beat_width

    def i_INPUT(self):
        if self.native_beat_width:
            return gvsoc.systree.SlaveItf(self, 'input',
                signature=IoV2Beat(self.native_beat_width))
        return gvsoc.systree.SlaveItf(self, 'input', signature='io_v2')
