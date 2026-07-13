# SPDX-FileCopyrightText: 2026 ETH Zurich, University of Bologna and EssilorLuxottica SAS
#
# SPDX-License-Identifier: Apache-2.0
#
# Authors: Germain Haugou (germain.haugou@gmail.com)

"""Thermal model testbench.

Two memory_v3 instances with temperature-dependent power tables, watched
by a ThermalModel through two sync points. A stub io_v2 master drives
write traffic into mem0 (mem1 stays idle) so that:

  - mem0 heats up through the RC thermal model while mem1 stays at
    ambient,
  - the rising temperature visibly increases mem0's reported power
    (leakage feedback),
  - a mid-run --power capture (magic writes, ``capture`` case) does not
    perturb the thermal power sampling and gets a correct report itself.

The clock runs at 100 MHz (10 ns cycles) and the thermal period is 1 us
(100 cycles), so a full closed-loop run only takes ~1000 cycles.
"""

from __future__ import annotations

import gvsoc.systree
import gvsoc.runner
import vp.clock_domain
from memory.memory_v3 import Memory, MemoryV3Config
from thermal.thermal_model import (ThermalModel, ThermalModelConfig,
                                   load_thermal_yaml)
from vp.power_config import load_power_yaml
from gvrun.parameter import TargetParameter

from stub_master import StubMaster

# Thermal update period: 1 us = 100 cycles at 100 MHz
PERIOD_PS = 1_000_000


def _writes(first_cycle: int, last_cycle: int, prefix: str) -> list:
    """One 4-byte write every 2 cycles in [first_cycle, last_cycle)."""
    return [dict(cycle=cycle, addr=0x20, size=4, is_write=True,
                 name=f'{prefix}{index}')
            for index, cycle in enumerate(range(first_cycle, last_cycle, 2))]


def build_case(case_name: str) -> dict:
    if case_name == 'heating':
        # Constant write traffic into mem0 for 10 thermal periods: 50
        # writes of 5000 pJ per 1 us period = 0.25 W dynamic power.
        # mem0 must ramp towards 25 + P*Rth while mem1 stays at ambient,
        # and the rising temperature must increase mem0's reported power
        # through the leakage table (0.002 W/K).
        return {
            'schedule': _writes(10, 1010, 'w'),
            'power_trigger': False,
        }

    if case_name == 'capture':
        # Interleave a --power capture with the thermal loop:
        #   period 1 (cycles 0-100):    45 writes (heavy traffic)
        #   period 2 (cycles 100-200):  16 writes at 110-140, then the
        #       capture starts at cycle 160, in the quiet second half
        #   period 3 (cycles 200-300):  16 writes at 210-240, inside the
        #       capture window, before the thermal tick at cycle 300
        #   period 4:                   capture stops at cycle 380
        # The thermal tick at cycle 200 must still see the period-2
        # traffic (the capture start must not reset the thermal power
        # sampling), and the capture report must cover its full window
        # (the thermal ticks at cycles 200/300 must not truncate it).
        schedule = []
        schedule += _writes(10, 100, 'a')
        schedule += _writes(110, 142, 'b')
        schedule.append(dict(cycle=160, addr=0x0, size=4, is_write=True,
                             name='cap_start', data_hex='baabbaab'))
        schedule += _writes(210, 242, 'c')
        schedule.append(dict(cycle=380, addr=0x0, size=4, is_write=True,
                             name='cap_stop', data_hex='cacaadde'))
        return {
            'schedule': schedule,
            'power_trigger': True,
        }

    raise ValueError(f'Unknown case: {case_name}')


class Chip(gvsoc.systree.Component):
    def __init__(self, parent, name=None):
        super().__init__(parent, name)
        case = TargetParameter(
            self, name='case', value='heating',
            description='Which thermal test case to run', cast=str,
        ).get_value()

        spec = build_case(case)

        clock = vp.clock_domain.Clock_domain(self, 'clock', frequency=100_000_000)

        mems = []
        for mem_name in ['mem0', 'mem1']:
            config = MemoryV3Config(size=0x1000, latency=1,
                                    power_trigger=spec['power_trigger'])
            config.power = load_power_yaml('mem_power.yaml')
            mem = Memory(self, mem_name, config=config)
            clock.o_CLOCK(mem.i_CLOCK())
            mems.append(mem)

        master = StubMaster(self, 'master', schedule=spec['schedule'],
                            logname='master')
        clock.o_CLOCK(master.i_CLOCK())
        master.o_OUTPUT(mems[0].i_INPUT())

        thermal_config = ThermalModelConfig(period=PERIOD_PS, temp_ambient=25.0,
                                            temp_init=25.0, verbose=True)
        thermal_config.sync_points = load_thermal_yaml('thermal.yaml')
        thermal = ThermalModel(self, 'thermal', config=thermal_config)
        clock.o_CLOCK(thermal.i_CLOCK())


class Target(gvsoc.runner.Target):
    gapy_description = 'thermal model testbench'
    model = Chip
    name = 'test'
