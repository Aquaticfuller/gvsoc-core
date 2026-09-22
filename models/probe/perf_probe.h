/*
 * Copyright (C) 2026 ETH Zurich and University of Bologna
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * Kernel-side software performance probes. Copy this file (and perf_probe_events.h) into the
 * kernel tree, or add core/models/probe to the include path.
 *
 * One posted 32-bit store per event, to an address that encodes the hart and the event id
 * (perf_probe_events.h). The GVSoC collector decodes it; an RTL testbench snoops the same window,
 * so the instrumentation in the kernel is identical for both engines.
 *
 * EVERYTHING COMPILES AWAY unless the build defines RLC_PROBE=1, so the default binaries stay
 * byte-identical. Enable with -DRLC_PROBE=1.
 *
 * Usage:
 *
 *     #include "perf_probe.h"
 *     perf_probe_role(PERF_PROBE_ROLE_CONSUMER);
 *     perf_probe_entity(PROBE_EVT_SDU_RX, uid, node->data_size);
 *     perf_probe(PROBE_EVT_TTI_BEGIN, tti_index);
 *
 * The cost is one store to an uncached peripheral address. That store is real traffic on the
 * narrow interconnect -- it is not free and it is not invisible to the timing model. Put probes
 * at task boundaries (a packet, a PDU, a grant, a phase), never inside an inner loop.
 */

#ifndef PERF_PROBE_H
#define PERF_PROBE_H

#include <stdint.h>
#include "perf_probe_events.h"

#ifndef RLC_PROBE
#define RLC_PROBE 0
#endif

#if RLC_PROBE

#include <snrt.h>

/* Cached once per core: snrt_cluster_core_idx() is a load through the team pointer, and the probe
   address is otherwise recomputed at every event. */
static inline volatile uint32_t *perf_probe_base(void)
{
    return (volatile uint32_t *)(PERF_PROBE_BASE +
                                 (snrt_cluster_core_idx() << PERF_PROBE_HART_SHIFT));
}

/* One event with a plain 32-bit value. */
static inline void perf_probe(uint32_t evt, uint32_t val)
{
    perf_probe_base()[evt] = val;
}

/* One event carrying an RLC entity id: value = (entity << 16) | (v & 0xffff). */
static inline void perf_probe_entity(uint32_t evt, uint32_t entity, uint32_t v)
{
    perf_probe_base()[evt] =
        (entity << PERF_PROBE_ENTITY_SHIFT) | (v & PERF_PROBE_VALUE_MASK);
}

static inline void perf_probe_role(uint32_t role)  { perf_probe(PROBE_EVT_ROLE, role); }
static inline void perf_probe_phase(uint32_t ph)   { perf_probe(PROBE_EVT_PHASE, ph); }

#else  /* !RLC_PROBE — every probe vanishes */

#define perf_probe(evt, val)                  ((void)0)
#define perf_probe_entity(evt, entity, v)     ((void)0)
#define perf_probe_role(role)                 ((void)0)
#define perf_probe_phase(ph)                  ((void)0)

#endif /* RLC_PROBE */

#endif /* PERF_PROBE_H */
