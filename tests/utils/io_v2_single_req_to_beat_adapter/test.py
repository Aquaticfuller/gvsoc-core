# SPDX-FileCopyrightText: 2026 ETH Zurich, University of Bologna and EssilorLuxottica SAS
#
# SPDX-License-Identifier: Apache-2.0
#
# Authors: Germain Haugou (germain.haugou@gmail.com)

import gvsoc.systree
import gvsoc.runner
import vp.clock_domain
from gvrun.parameter import TargetParameter

from stub_master import StubMaster
from stub_target import StubTarget

BASE = 0x1000_0000
SIZE = 0x1_0000
BEAT_WIDTH = 8


def build_case(case_name: str) -> dict:
    # Single beat-sized read: one distinct beat downstream, the master's own
    # request handed back with the data copied into its buffer.
    if case_name == 'r1':
        return {
            'schedule': [dict(cycle=10, addr=BASE, size=8, name='r0')],
        }

    # Read smaller than the beat width: still exactly one beat.
    if case_name == 'r_small':
        return {
            'schedule': [dict(cycle=10, addr=BASE, size=4, name='r0')],
        }

    # Access wider than the beat plane (8 B access, 4 B beats): the target
    # streams two beats and the adapter reassembles them into the master's
    # buffer before handing the request back.
    if case_name == 'r_wide':
        return {
            'target': dict(beat_width=4),
            'schedule': [dict(cycle=10, addr=BASE, size=8, name='r0')],
        }

    if case_name == 'w1':
        return {
            'schedule': [dict(cycle=10, addr=BASE, size=8, is_write=True,
                              name='w0')],
        }

    # Async write ack: the master's own object is forwarded downstream,
    # GRANTED, and round-trips later as the ack.
    if case_name == 'w_async':
        return {
            'target': dict(async_ack=True, latency=2),
            'schedule': [dict(cycle=10, addr=BASE, size=8, is_write=True,
                              name='w0')],
        }

    # THE case this adapter exists for: four reads issued back-to-back into a
    # slow target (latency 4), so all four are outstanding concurrently before
    # the first completes. The collapse adapter would DENY every access after
    # the first (single-outstanding); this adapter forwards them all and the
    # responses then stream one per cycle.
    if case_name == 'pipelined':
        return {
            'target': dict(latency=4),
            'schedule': [dict(cycle=10 + i, addr=BASE + i * 8, size=8,
                              name=f'r{i}') for i in range(4)],
        }

    # Pipelined mix of reads and writes in flight together.
    if case_name == 'pipelined_mix':
        return {
            'target': dict(async_ack=True, latency=4),
            'schedule': [
                dict(cycle=10, addr=BASE, size=8, name='r0'),
                dict(cycle=11, addr=BASE + 0x100, size=8, is_write=True, name='w0'),
                dict(cycle=12, addr=BASE + 8, size=8, name='r1'),
                dict(cycle=13, addr=BASE + 0x108, size=8, is_write=True, name='w1'),
            ],
        }

    # The beat slave reports IO_RESP_INVALID inline on the data-less read
    # (the one legal inline read completion besides zero-size).
    if case_name == 'err':
        return {
            'target': dict(error=True),
            'schedule': [dict(cycle=10, addr=BASE, size=8, expect_status=1,
                              name='r0')],
        }

    # Degenerate zero-size read.
    if case_name == 'zero':
        return {
            'schedule': [dict(cycle=10, addr=BASE, size=0, name='r0')],
        }

    # Downstream request-path back-pressure: the target denies the forwarded
    # request once; the adapter propagates the DENY upstream statelessly and
    # forwards the retry, on which the master re-sends.
    if case_name == 'deny':
        return {
            'target': dict(deny_count=1, retry_delay=3),
            'schedule': [dict(cycle=10, addr=BASE, size=8, name='r0')],
        }

    raise ValueError(f'Unknown case: {case_name}')


class Chip(gvsoc.systree.Component):
    def __init__(self, parent, name=None):
        super().__init__(parent, name)
        case = TargetParameter(
            self, name='case', value='r1',
            description='Which test case to run', cast=str,
        ).get_value()

        spec = build_case(case)
        clock = vp.clock_domain.Clock_domain(self, 'clock', frequency=100_000_000)

        master = StubMaster(self, 'master', schedule=spec['schedule'],
                            logname='master')
        clock.o_CLOCK(master.i_CLOCK())

        target_args = dict(beat_width=BEAT_WIDTH)
        target_args.update(spec.get('target', {}))
        target = StubTarget(self, 'target', base=BASE, size=SIZE,
                            logname='target', **target_args)
        clock.o_CLOCK(target.i_CLOCK())

        # IoV2SingleReq master -> IoV2Beat slave: the framework auto-inserts
        # IoV2SingleReqToBeatAdapter between them.
        master.o_OUTPUT(target.i_INPUT())


class Target(gvsoc.runner.Target):
    gapy_description = 'io_v2_single_req_to_beat_adapter testbench'
    model = Chip
    name = 'test'
