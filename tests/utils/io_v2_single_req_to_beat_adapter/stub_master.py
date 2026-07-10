# SPDX-FileCopyrightText: 2026 ETH Zurich, University of Bologna and EssilorLuxottica SAS
#
# SPDX-License-Identifier: Apache-2.0
#
# Authors: Germain Haugou (germain.haugou@gmail.com)

import gvsoc.systree
from gvsoc.signature import IoV2SingleReq


class StubMaster(gvsoc.systree.Component):
    """io_v2 single-req testbench initiator.

    Declares signature ``IoV2SingleReq`` on its output so binding it to an
    ``IoV2Beat`` target makes the framework auto-insert
    IoV2SingleReqToBeatAdapter.
    """
    def __init__(self, parent, name, schedule=None, logname=None,
                 quit_after_cycles=200):
        super().__init__(parent, name)
        self.add_sources(['stub_master.cpp'])
        self.add_property('logname', logname or name)
        self.add_property('schedule', schedule or [])
        self.add_property('quit_after_cycles', quit_after_cycles)

    def o_OUTPUT(self, itf):
        self.itf_bind('output', itf, signature=IoV2SingleReq())
