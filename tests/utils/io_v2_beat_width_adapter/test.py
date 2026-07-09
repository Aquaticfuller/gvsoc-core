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


def build_case(case_name: str) -> dict:
    # Downsize path: wide master (8 B beats) onto a narrow slave (4 B beats).
    # The narrow side streams every cycle; the wide side carries the same
    # bytes in half as many beats, one every 2 cycles.
    if case_name == 'read_downsize':
        return {
            'in_w': 8, 'out_w': 4,
            'schedule': [dict(cycle=10, addr=BASE, size=32, burst_id=7,
                              name='r0')],
        }

    # Upsize path: narrow master (4 B) onto a wide slave (8 B). The upstream
    # drains one narrow beat per cycle; the downstream producer is throttled
    # (resp DENIED + resp_retry) to one wide beat every 2 cycles.
    if case_name == 'read_upsize':
        return {
            'in_w': 4, 'out_w': 8,
            'schedule': [dict(cycle=10, addr=BASE, size=64, burst_id=3,
                              name='r0')],
        }

    # Burst size not a width multiple: 20 B at 8->4 gives upstream beats of
    # 8, 8, 4 bytes and five 4-byte downstream beats.
    if case_name == 'read_short':
        return {
            'in_w': 8, 'out_w': 4,
            'schedule': [dict(cycle=10, addr=BASE, size=20, name='r0')],
        }

    # Big-packet 32 B write chopped into eight 4-byte downstream beats
    # (one per cycle); four 8-byte acks upstream.
    if case_name == 'write_big_downsize':
        return {
            'in_w': 8, 'out_w': 4,
            'schedule': [dict(cycle=10, addr=BASE, size=32, is_write=True,
                              burst_id=5, name='w0')],
        }

    # Beat-form 32 B write (four 8-byte beats): each upstream beat needs two
    # downstream cycles, so the master is back-pressured (REQHOLD) to one
    # beat every 2 cycles while the downstream streams every cycle.
    if case_name == 'write_beats_downsize':
        return {
            'in_w': 8, 'out_w': 4,
            'schedule': [dict(cycle=10, addr=BASE, size=32, is_write=True,
                              write_beats=True, name='w0')],
        }

    # Beat-form 32 B write (eight 4-byte beats) onto an 8-byte slave: pairs
    # of upstream beats are packed into one wide downstream beat every 2
    # cycles; eight acks upstream.
    if case_name == 'write_beats_upsize':
        return {
            'in_w': 4, 'out_w': 8,
            'schedule': [dict(cycle=10, addr=BASE, size=32, is_write=True,
                              write_beats=True, name='w0')],
        }

    # Big-packet 32 B write onto a wide slave: four 8-byte downstream beats,
    # eight 4-byte acks upstream.
    if case_name == 'write_big_upsize':
        return {
            'in_w': 4, 'out_w': 8,
            'schedule': [dict(cycle=10, addr=BASE, size=32, is_write=True,
                              name='w0')],
        }

    # Async downstream write acks (GRANTED + resp()) through the chop path.
    if case_name == 'write_async':
        return {
            'in_w': 8, 'out_w': 4,
            'target': dict(async_ack=True, latency=2),
            'schedule': [dict(cycle=10, addr=BASE, size=32, is_write=True,
                              name='w0')],
        }

    # The beat slave reports IO_RESP_INVALID inline on the read descriptor;
    # every upstream beat must carry the error.
    if case_name == 'err_read':
        return {
            'in_w': 8, 'out_w': 4,
            'target': dict(error=True),
            'schedule': [dict(cycle=10, addr=BASE, size=16, expect_status=1,
                              name='r0')],
        }

    # Degenerate zero-size read -> exactly one is_first=is_last zero-size beat.
    if case_name == 'zero_read':
        return {
            'in_w': 8, 'out_w': 4,
            'schedule': [dict(cycle=10, addr=BASE, size=0, name='r0')],
        }

    # Upstream response back-pressure: the master denies one wide beat once;
    # the adapter holds the exact beat and re-sends it on resp_retry().
    if case_name == 'bp_read':
        return {
            'in_w': 8, 'out_w': 4,
            'schedule': [dict(cycle=10, addr=BASE, size=32, burst_id=2,
                              deny_beats=[1], retry_delay=3, name='r0')],
        }

    # Downstream request-path back-pressure: the target denies the read
    # descriptor once; the adapter propagates the DENY upstream (REQHOLD) and
    # forwards the retry.
    if case_name == 'deny_read':
        return {
            'in_w': 8, 'out_w': 4,
            'target': dict(deny_count=1, retry_delay=3),
            'schedule': [dict(cycle=10, addr=BASE, size=32, name='r0')],
        }

    # Downstream request-path back-pressure on the write chop path: the first
    # chunk is denied; the adapter holds it and re-sends inside retry().
    if case_name == 'deny_write':
        return {
            'in_w': 8, 'out_w': 4,
            'target': dict(deny_count=1, retry_delay=3),
            'schedule': [dict(cycle=10, addr=BASE, size=32, is_write=True,
                              name='w0')],
        }

    # Degenerate zero-size write -> one zero-size chunk, one zero-size ack.
    if case_name == 'zero_write':
        return {
            'in_w': 8, 'out_w': 4,
            'schedule': [dict(cycle=10, addr=BASE, size=0, is_write=True,
                              name='w0')],
        }

    # Same traffic as read_upsize, but with a read FIFO deep enough to hold
    # the whole repacked burst: the downstream producer is never denied
    # (no BHOLD) and streams its 8 wide beats every cycle. The adapter is
    # instantiated manually to set the non-default depth (the auto-inserted
    # bridge uses the config defaults).
    if case_name == 'read_upsize_fifo':
        return {
            'in_w': 4, 'out_w': 8,
            'adapter': dict(read_fifo_depth=16),
            'schedule': [dict(cycle=10, addr=BASE, size=64, burst_id=3,
                              name='r0')],
        }

    # Same traffic as write_beats_downsize, but with a write FIFO deep enough
    # to absorb the whole burst's chunks: the upstream writer streams its four
    # 8 B beats every cycle without ever being denied (no REQHOLD), while the
    # downstream still drains one 4 B chunk per cycle.
    if case_name == 'write_beats_downsize_fifo':
        return {
            'in_w': 8, 'out_w': 4,
            'adapter': dict(write_fifo_depth=8),
            'schedule': [dict(cycle=10, addr=BASE, size=32, is_write=True,
                              write_beats=True, name='w0')],
        }

    raise ValueError(f'Unknown case: {case_name}')


class Chip(gvsoc.systree.Component):
    def __init__(self, parent, name=None):
        super().__init__(parent, name)
        case = TargetParameter(
            self, name='case', value='read_downsize',
            description='Which test case to run', cast=str,
        ).get_value()

        spec = build_case(case)
        clock = vp.clock_domain.Clock_domain(self, 'clock', frequency=100_000_000)

        master = StubMaster(self, 'master', schedule=spec['schedule'],
                            beat_width=spec['in_w'], logname='master')
        clock.o_CLOCK(master.i_CLOCK())

        target = StubTarget(self, 'target', beat_width=spec['out_w'],
                            base=BASE, size=SIZE, logname='target',
                            **spec.get('target', {}))
        clock.o_CLOCK(target.i_CLOCK())

        if 'adapter' in spec:
            # Non-default FIFO depths: instantiate the adapter manually (the
            # auto-inserted bridge uses the config defaults). The signature
            # checks keep the user-instantiated adapter on the path.
            from utils.io_v2_beat_width_adapter import IoV2BeatWidthAdapter
            adapter = IoV2BeatWidthAdapter(self, 'adapter',
                                           input_width=spec['in_w'],
                                           output_width=spec['out_w'],
                                           **spec['adapter'])
            clock.o_CLOCK(adapter.i_CLOCK())
            master.o_OUTPUT(adapter.i_INPUT())
            adapter.o_OUTPUT(target.i_INPUT())
        else:
            # IoV2Beat(in_w) master -> IoV2Beat(out_w) slave: the framework
            # auto-inserts IoV2BeatWidthAdapter between them.
            master.o_OUTPUT(target.i_INPUT())


class Target(gvsoc.runner.Target):
    gapy_description = 'io_v2_beat_width_adapter testbench'
    model = Chip
    name = 'test'
