# SPDX-FileCopyrightText: 2026 ETH Zurich, University of Bologna and EssilorLuxottica SAS
#
# SPDX-License-Identifier: Apache-2.0
#
# Authors: Germain Haugou (germain.haugou@gmail.com)

import gvsoc.systree
from gvsoc.signature import IoV2SingleReq


class StubTarget(gvsoc.systree.Component):
    """Width-declaring io_v2 single-req testbench target.

    Declares ``IoV2SingleReq(width)`` on its input — the bank-interleaved
    fabric contract — and POLICES that no access crosses an aligned width
    granule. Binding an unknown- or wider-width single-req master makes the
    framework insert IoV2SingleReqWidthAdapter.
    """
    def __init__(self, parent, name, width=4, latency=0, error=False,
                 async_resp=False, deny_at=None, retry_delay=2,
                 base=0, size=0, logname=None):
        super().__init__(parent, name)
        self.add_sources(['stub_target.cpp'])
        self.add_property('logname', logname or name)
        self.add_property('width', width)
        self.add_property('latency', latency)
        self.add_property('error', error)
        self.add_property('async_resp', async_resp)
        self.add_property('deny_at', deny_at or [])
        self.add_property('retry_delay', retry_delay)
        self.add_property('base', base)
        self.add_property('size', size)
        self._width = width

    def i_INPUT(self):
        return gvsoc.systree.SlaveItf(self, 'input',
                                      signature=IoV2SingleReq(self._width))
