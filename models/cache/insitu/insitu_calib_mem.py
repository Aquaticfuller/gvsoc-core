#
# Copyright (C) 2026 ETH Zurich and University of Bologna
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#

"""Fixed-latency, serializing refill-memory model for cache calibration.

GVSoC twin of the RTL ``refill_mem_model.sv`` (see
``ManyRVData_rebase/reports/cache_calib/CALIB_IMPLEMENTATION.md`` §2). Bind it to
the InSitu cache's L2 / refill port. It answers line refills (reads) and
writebacks (writes) with the same deterministic timing contract as the RTL so the
GVSoC perf model can be diffed 1:1 against RTL on a shared trace.

The key behaviour reproduced is **serialization**: at most one job is serviced at
a time (``mem_busy_until`` cyclestamp), so back-to-back distinct-line misses are
memory-latency-bound and do NOT pipeline — exactly the RTL controller's
single-outstanding-refill behaviour, which dominates miss throughput.
"""

from __future__ import annotations

from gvsoc.systree import Component, SlaveItf


class InsituCalibMem(Component):
    """Fixed-latency serializing refill responder.

    Parameters
    ----------
    parent, name : standard GVSoC systree args
    mem_latency : int
        Cycles from a request being accepted to its first response beat.
    beat_gap : int
        Idle cycles between consecutive read-burst beats (default 0).
    accept_every : int
        Min cycles between accepting two successive requests (default 1).
    refill_beat_bytes : int
        Width of one refill beat in bytes (128b = 16B → BurstLength=4 for a 64B line).
    word_bytes : int
        Word granularity of the deterministic data pattern (only used if fill_pattern).
    fill_pattern : bool
        When True, read responses are filled with the self-describing pattern
        (word @ A == A). Off by default — calibration is timing-only.
    writeback_overlap : bool
        When True, write jobs (dirty-victim evictions) do NOT consume the read
        serialization budget (`mem_busy_until`) — they overlap with refill reads on a
        separate writeback path, matching the RTL where a writeback hides within the
        per-miss budget rather than adding a serial stall. They are still counted as
        `mem_wr` and still take MemLatency to complete. Default False (writes fully
        serialize with reads).
    """

    def __init__(self, parent: Component, name: str,
                 mem_latency: int = 50,
                 beat_gap: int = 0,
                 accept_every: int = 1,
                 refill_beat_bytes: int = 16,
                 word_bytes: int = 4,
                 fill_pattern: bool = False,
                 writeback_overlap: bool = False):
        super().__init__(parent, name)

        self.add_sources(['cache/insitu/insitu_calib_mem.cpp'])

        self.add_properties({
            'mem_latency': mem_latency,
            'beat_gap': beat_gap,
            'accept_every': accept_every,
            'refill_beat_bytes': refill_beat_bytes,
            'word_bytes': word_bytes,
            'fill_pattern': fill_pattern,
            'writeback_overlap': writeback_overlap,
        })

    def i_INPUT(self) -> SlaveItf:
        """Refill / writeback request input (bind the cache's o_L2 here)."""
        return SlaveItf(self, 'input', signature='io')
