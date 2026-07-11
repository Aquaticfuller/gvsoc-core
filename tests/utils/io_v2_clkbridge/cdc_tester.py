# SPDX-FileCopyrightText: 2026 ETH Zurich, University of Bologna and EssilorLuxottica SAS
#
# SPDX-License-Identifier: Apache-2.0
#
# Authors: Germain Haugou (germain.haugou@gmail.com)

import gvsoc.systree
from gvsoc.signature import IoV2Beat, IoV2BigPacket


class CDCTester(gvsoc.systree.Component):
    """Bidirectional io_v2 master driving the clock-bridge testbench.

    Each instance issues ``nb_accesses`` writes followed by ``nb_accesses``
    reads against ``base`` on its single ``output`` master port. The Chip
    wires the two testers' output ports to memories in the *other* clock
    domain so the IoV2ClockBridge is exercised on both crossings.

    Quit coordination is handled by two single-bit wires: each tester
    pulses ``done_out`` when its local PASS line is printed; receiving the
    partner's pulse on ``done_in`` plus being locally done causes
    ``engine->quit()`` to fire.

    ``burst_beats`` selects the beat mode (0 = off): each access becomes a
    beat-form burst of ``burst_beats`` beats of ``access_size`` bytes,
    following the io_v2 per-burst write-acknowledgement contract
    (allocator-backed write beats the target consumes and frees, one ack
    per burst; data-less read burst requests answered by distinct response
    beats). The output signature switches to :class:`IoV2Beat` accordingly.
    """

    def __init__(self, parent: gvsoc.systree.Component, name: str, *,
                 base: int,
                 access_size: int = 4,
                 nb_accesses: int = 16,
                 pattern_seed: int = 0xa5,
                 quit_after_cycles: int = 100_000,
                 pipeline_burst: int = 1,
                 burst_beats: int = 0,
                 logname: str | None = None):
        super().__init__(parent, name)
        self.add_sources(['cdc_tester.cpp'])
        self.add_property('base',              base)
        self.add_property('access_size',       access_size)
        self.add_property('nb_accesses',       nb_accesses)
        self.add_property('pattern_seed',      pattern_seed)
        self.add_property('quit_after_cycles', quit_after_cycles)
        self.add_property('pipeline_burst',    pipeline_burst)
        self.add_property('burst_beats',       burst_beats)
        self.add_property('logname',           logname or name)
        self.access_size = access_size
        self.burst_beats = burst_beats

    def o_OUTPUT(self, itf: gvsoc.systree.SlaveItf):
        # Class-based signature so the systree auto-bridge pass sees the
        # binding and inserts an utils.io_v2_clock_bridge across the two
        # clock domains. In beat mode the tester is a beat-plane master
        # (one beat per access_size); otherwise IoV2BigPacket is the most
        # permissive v2 master signature β€” it accepts any of the three
        # slave response forms.
        if self.burst_beats > 0:
            sig = IoV2Beat(beat_width=self.access_size)
        else:
            sig = IoV2BigPacket(allow=True)
        self.itf_bind('output', itf, signature=sig)

    def o_DONE(self, itf: gvsoc.systree.SlaveItf):
        self.itf_bind('done_out', itf, signature='wire<bool>')

    def i_DONE(self) -> gvsoc.systree.SlaveItf:
        return gvsoc.systree.SlaveItf(self, 'done_in', signature='wire<bool>')
