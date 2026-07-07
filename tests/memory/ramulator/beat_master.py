# SPDX-FileCopyrightText: 2026 ETH Zurich, University of Bologna and EssilorLuxottica SAS
#
# SPDX-License-Identifier: Apache-2.0
#
# Authors: Germain Haugou (germain.haugou@gmail.com)

import gvsoc.systree
import gvsoc.signature


class BeatMaster(gvsoc.systree.Component):
    """Beat-streaming io_v2 master.

    Declares ``IoV2Beat(beat_width)`` on its output so it binds directly to a
    beat slave. Each schedule entry (a burst) is streamed as a sequence of
    beat-sized requests (one per cycle), sharing a burst_id, with is_first /
    is_last framing. Schedule entry keys: cycle, addr, size, is_write, name,
    [data_hex].
    """

    def __init__(self, parent: gvsoc.systree.Component, name: str,
                 schedule: list | None = None, beat_width: int = 64,
                 logname: str | None = None, quit_after_cycles: int = 200,
                 quiet: bool = False, chain: bool = False, repeat: int = 1):
        super().__init__(parent, name)
        self.beat_width = beat_width
        self.add_sources(['beat_master.cpp'])
        self.add_property('logname', logname or name)
        self.add_property('beat_width', beat_width)
        self.add_property('quit_after_cycles', quit_after_cycles)
        # Speed-benchmark knobs: quiet suppresses logging/asserts; chain+repeat
        # re-issue the schedule 'repeat' times (one burst in flight) for a long
        # run whose wall time measures simulation speed.
        self.add_property('quiet', quiet)
        self.add_property('chain', chain)
        self.add_property('repeat', repeat)
        self.add_property('schedule', schedule or [])

    def o_OUTPUT(self, itf: gvsoc.systree.SlaveItf):
        self.itf_bind('output', itf,
                      signature=gvsoc.signature.IoV2Beat(self.beat_width))
