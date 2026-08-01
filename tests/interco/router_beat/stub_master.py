# SPDX-FileCopyrightText: 2026 ETH Zurich, University of Bologna and EssilorLuxottica SAS
#
# SPDX-License-Identifier: Apache-2.0
#
# Authors: Germain Haugou (germain.haugou@gmail.com)

import gvsoc.systree


class StubMaster(gvsoc.systree.Component):
    """io_v2 burst-mode testbench initiator."""
    def __init__(self, parent, name, schedule=None, logname=None, quit_after_cycles=100,
                 resp_deny_count=0, resp_retry_delay=1):
        super().__init__(parent, name)
        self.add_sources(['stub_master.cpp'])
        self.add_property('logname', logname or name)
        self.add_property('schedule', schedule or [])
        self.add_property('quit_after_cycles', quit_after_cycles)
        # Back-pressure the response channel: deny the first `resp_deny_count`
        # responses, calling resp_retry() `resp_retry_delay` cycles later. The
        # router must not re-offer a denied response before that retry.
        self.add_property('resp_deny_count', resp_deny_count)
        self.add_property('resp_retry_delay', resp_retry_delay)

    def o_OUTPUT(self, itf):
        self.itf_bind('output', itf, signature='io_v2')
