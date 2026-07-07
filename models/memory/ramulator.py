# SPDX-FileCopyrightText: 2026 ETH Zurich, University of Bologna and EssilorLuxottica SAS
#
# SPDX-License-Identifier: Apache-2.0
#
# Authors: Germain Haugou (germain.haugou@gmail.com)

"""Generator for the ``memory.ramulator`` model.

Wraps Ramulator 2.x (a cycle-level DRAM simulator) as a GVSoC memory model
used for *timing only*: the actual bytes live in a local backing store, while
Ramulator drives the request latency.

The C++ model links ``libramulator`` directly (see the model CMakeLists.txt),
so it is only available when Ramulator has been vendored/built. It is selected
by name with ``set_component('memory.ramulator')``, like the neighbouring
``memory.dramsys`` model.

The Ramulator behaviour is described by a YAML config that must use the
``External`` frontend. Generate it once from a Python config (see
``tests/memory/ramulator/external_config.py``) with::

    python3 -m ramulator export external_config.py -o config.yaml

and pass its path via ``config_yaml``.
"""

from __future__ import annotations

import gvsoc.systree as st
import gvsoc.signature
from config_tree import Config, cfg_field, HasSize


class RamulatorConfig(Config, HasSize):
    """Configuration for the ``memory.ramulator`` model.

    Single config object holding every tunable, snake-cased to
    ``memory/ramulator/ramulator_config.hpp`` at build time and read by
    ``ramulator.cpp`` from the compiled struct (memory_v3 style — no
    ``get_js_config``).

    Attributes
    ----------
    size: int
        Functional backing-store size in bytes. Requests beyond this range are
        refused with ``IO_RESP_INVALID``.
    config_yaml: str
        Path to the exported Ramulator YAML (must use the ``External``
        frontend). Generate once with
        ``python3 -m ramulator export external_config.py -o config.yaml``.
    beat_width: int
        Response beat width in bytes == the DRAM transaction granularity
        (Ramulator ``get_tx_bytes()``, 64 for DDR4). Asserted against Ramulator
        at runtime.
    num_banks: int
        Number of bank lanes for the GUI command timeline (= product of the
        bank-identifying level sizes; 16 for the default DDR4 config). Asserted
        against Ramulator's geometry at runtime.
    truncate: bool
        Mask the incoming address with ``size - 1`` (repeating window); ``size``
        must then be a power of two.
    init: bool
        Poison the backing store with ``0x57`` bytes at construction (skipped
        for memories >= 32 MiB), matching memory_v3.
    """

    size: int = cfg_field(default=0, fmt="hex", dump=True, desc=(
        "Functional backing store size in bytes"))
    config_yaml: str = cfg_field(default="", dump=True, desc=(
        "Path to the exported Ramulator YAML (External frontend)"))
    beat_width: int = cfg_field(default=64, dump=True, desc=(
        "Response beat width in bytes (== DRAM tx_bytes)"))
    num_banks: int = cfg_field(default=16, dump=True, desc=(
        "Number of bank lanes for the GUI command timeline"))
    truncate: bool = cfg_field(default=True, dump=True, desc=(
        "Mask the incoming address with size-1 (repeating window)"))
    init: bool = cfg_field(default=True, dump=True, desc=(
        "Poison the backing store with 0x57 bytes at construction"))


class Ramulator(st.Component):
    """Ramulator-backed DRAM timing model on the io_v2 (beat) protocol.

    Parameters
    ----------
    parent, name :
        Standard systree parent/name.
    config : RamulatorConfig
        Full configuration. Every tunable lives on this object.
    """

    def __init__(self, parent: st.Component, name: str, config: RamulatorConfig):
        super().__init__(parent, name, config=config)

        # Built via CMake (external libramulator link), so selected by name.
        self.set_component('memory.ramulator')

        # Kept for the port signature / GUI generation below.
        self.beat_width = config.beat_width
        self.num_banks = config.num_banks

    def i_INPUT(self) -> st.SlaveItf:
        """io_v2 beat input port.

        Declared as ``IoV2Beat(beat_width)``: a beat-aware master (or a legacy
        ``'io_v2'`` string master) binds directly. For a read the master sends
        one request with the full burst size; the wrapper returns
        ``IO_REQ_GRANTED`` and streams the response back as per-cycle beats (one
        ``beat_width`` slice per cycle, with ``is_first`` / ``is_last`` /
        ``burst_id`` set) as Ramulator completes the underlying DRAM
        transactions. Writes are stored and acknowledged synchronously.
        """
        return st.SlaveItf(self, 'input',
                           signature=gvsoc.signature.IoV2Beat(self.beat_width))

    def gen_gui(self, parent_signal):
        """Expose the per-bank DRAM command timeline in gvsoc-gui3.

        One group per bank (plus a 'scope' group for rank/all-bank commands like
        refresh), each with the command label, the open row, and the numeric
        command id. Populated by the GvsocBridge plugin (DRAM commands as
        Ramulator issues them). Enable with e.g. ``--event=.*mem.*``.
        """
        import gvsoc.gui

        dram = gvsoc.gui.Signal(self, parent_signal, name=self.name,
            is_group=True, groups=['dram'], opened=True)

        def lane(prefix, label):
            # Command label (ACT/RD/WR/PRE/REF) as a string box, plus the open
            # row as a decimal box. The numeric command id is emitted too (for
            # VCD / automated checks) but kept out of the default view.
            gvsoc.gui.Signal(self, dram, name=f'{label} cmd', path=f'{prefix}_cmd',
                display=gvsoc.gui.DisplayStringBox(), groups=['dram'])
            gvsoc.gui.Signal(self, dram, name=f'{label} row', path=f'{prefix}_row',
                display=gvsoc.gui.DisplayBox(format='dec'), groups=['dram'])

        for i in range(self.num_banks):
            lane(f'bank{i}', f'bank{i}')
        lane('scope', 'scope')
