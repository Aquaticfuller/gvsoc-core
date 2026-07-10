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
WIDTH = 4


def build_case(case_name: str) -> dict:
    # Aligned granule-sized read: fits, pure pass-through.
    if case_name == 'pt':
        return {
            'schedule': [dict(cycle=10, addr=BASE, size=4, name='r0')],
        }

    # Unaligned but still within one granule (offset 1, 2 bytes): pass-through.
    if case_name == 'pt_fit':
        return {
            'schedule': [dict(cycle=10, addr=BASE + 1, size=2, name='r0')],
        }

    # Aligned 8-byte read on a 4-byte granule: split into two chunks.
    if case_name == 'split2':
        return {
            'schedule': [dict(cycle=10, addr=BASE, size=8, name='r0')],
        }

    # THE aliasing case from the interleaved fabric: a 4-byte access at
    # offset 2 crosses the granule boundary and must become 2+2.
    if case_name == 'split_straddle':
        return {
            'schedule': [dict(cycle=10, addr=BASE + 2, size=4, name='r0')],
        }

    # Unaligned 10-byte read at offset 3: chunks 1+4+4+1.
    if case_name == 'split4':
        return {
            'schedule': [dict(cycle=10, addr=BASE + 3, size=10, name='r0')],
        }

    # 8-byte write split into two chunks; the target checks each chunk's
    # payload against the addr-derived pattern (right slice, right offset).
    if case_name == 'wsplit':
        return {
            'schedule': [dict(cycle=10, addr=BASE, size=8, is_write=True,
                              name='w0')],
        }

    # Chunks answered async (GRANTED + resp): the split advances one chunk
    # per completion and the master still gets one response on its own object.
    if case_name == 'async_split':
        return {
            'target': dict(async_resp=True, latency=2),
            'schedule': [dict(cycle=10, addr=BASE, size=8, name='r0')],
        }

    # The fabric reports IO_RESP_INVALID on one chunk: latched onto the
    # master's request.
    if case_name == 'err_split':
        return {
            'target': dict(error=True),
            'schedule': [dict(cycle=10, addr=BASE, size=8, expect_status=1,
                              name='r0')],
        }

    # First chunk denied: the split aborts statelessly, the DENY propagates
    # upstream (REQHOLD) and the master re-sends on the forwarded retry.
    if case_name == 'deny_first':
        return {
            'target': dict(deny_at=[0], retry_delay=3),
            'schedule': [dict(cycle=10, addr=BASE, size=8, name='r0')],
        }

    # Second chunk denied mid-split: the transaction is committed, so the
    # adapter holds the chunk itself and re-sends it inside retry() — no
    # upstream REQHOLD.
    if case_name == 'deny_mid':
        return {
            'target': dict(deny_at=[1], retry_delay=3),
            'schedule': [dict(cycle=10, addr=BASE, size=8, name='r0')],
        }

    # A second split-needing access while one is active: DENIED (REQHOLD) and
    # re-sent on the retry raised when the first split completes.
    if case_name == 'busy':
        return {
            'target': dict(async_resp=True, latency=3),
            'schedule': [
                dict(cycle=10, addr=BASE, size=8, name='r0'),
                dict(cycle=11, addr=BASE + 0x100, size=8, name='r1'),
            ],
        }

    # Fitting accesses are never blocked by an active split: a 4-byte read
    # passes straight through while the 8-byte split is still in flight.
    if case_name == 'mixed':
        return {
            'target': dict(async_resp=True, latency=4),
            'schedule': [
                dict(cycle=10, addr=BASE, size=8, name='r0'),
                dict(cycle=11, addr=BASE + 0x100, size=4, name='r1'),
            ],
        }

    raise ValueError(f'Unknown case: {case_name}')


class Chip(gvsoc.systree.Component):
    def __init__(self, parent, name=None):
        super().__init__(parent, name)
        case = TargetParameter(
            self, name='case', value='pt',
            description='Which test case to run', cast=str,
        ).get_value()

        spec = build_case(case)
        clock = vp.clock_domain.Clock_domain(self, 'clock', frequency=100_000_000)

        master = StubMaster(self, 'master', schedule=spec['schedule'],
                            logname='master')
        clock.o_CLOCK(master.i_CLOCK())

        target_args = dict(width=WIDTH)
        target_args.update(spec.get('target', {}))
        target = StubTarget(self, 'target', base=BASE, size=SIZE,
                            logname='target', **target_args)
        clock.o_CLOCK(target.i_CLOCK())

        # Width-0 IoV2SingleReq master -> IoV2SingleReq(width) slave: the
        # framework auto-inserts IoV2SingleReqWidthAdapter between them.
        master.o_OUTPUT(target.i_INPUT())


class Target(gvsoc.runner.Target):
    gapy_description = 'io_v2_single_req_width_adapter testbench'
    model = Chip
    name = 'test'
