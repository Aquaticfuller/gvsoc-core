# SPDX-FileCopyrightText: 2026 ETH Zurich, University of Bologna and EssilorLuxottica SAS
#
# SPDX-License-Identifier: Apache-2.0
#
# Authors: Germain Haugou (germain.haugou@gmail.com)

"""Memcheck region declaration component.

The memory checker registry (``vp::MemCheck``) identifies each allocator-backed
memory range by a region ID: the runtime allocator passes it in the memcheck
semihosting calls, and the reports name the region a buffer belongs to (and the
one a faulty access lands in).

This component carries the list of regions of a target in its typed config and,
at construction, declares them to the registry (``declare_region``). The buffer
checks themselves happen in the core models at issue time; nothing in the
request path needs any memcheck configuration.

The component has no ports — it is a pure declaration carrier, following the
same pattern as :class:`utils.watchpoint_alias.WatchpointAlias`. Instantiate it
once per target and pass the regions via :class:`MemcheckRegionsConfig`. The
region IDs must match the ones used by the runtime allocator instrumentation.
"""

from __future__ import annotations

from typing import ClassVar
from config_tree import Config, cfg_field
from gvsoc.systree import Component


class MemcheckRegion(Config):
    """One allocator-backed memory region."""

    _defer_parent_init: ClassVar[bool] = True

    mem_id: int = cfg_field(default=-1, dump=True, desc=(
        "Region ID, as passed by the runtime allocator in the memcheck "
        "semihosting calls"
    ))
    name: str = cfg_field(default="", dump=True, desc=(
        "User-friendly region name used in error reports"
    ))
    base: int = cfg_field(default=0, dump=True, fmt="hex", desc=(
        "Global base address of the region"
    ))
    size: int = cfg_field(default=0, dump=True, fmt="hex", desc=(
        "Size of the region, in bytes"
    ))


class MemcheckRegionsConfig(Config):
    """Configuration for :class:`MemcheckRegions`: the list of regions."""

    regions: list[MemcheckRegion] = cfg_field(default_factory=list, init=False, desc=(
        "Memcheck regions to declare to the registry"
    ))

    def add_regions(self, *regions: MemcheckRegion):
        """Append regions (adopting each into this config)."""
        for region in regions:
            region.adopt(self)
            self.regions.append(region)
        return self


class MemcheckRegions(Component):
    """Declares the memcheck regions of a target to the registry."""

    def __init__(self, parent: Component, name: str, config: MemcheckRegionsConfig):
        super().__init__(parent, name, config=config)
        self.add_sources(['utils/memcheck_regions.cpp'])
