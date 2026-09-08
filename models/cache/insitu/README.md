# CachePool — GVSoC Performance Model

A cycle-approximate GVSoC model of the CachePool cluster: many Snitch+Spatz cores over a
**shared** InSitu L1 data cache, in a multi-group mesh. Targets <5% cycle error against RTL.

The deployed target is **`cachepool_v3`**. It runs the unmodified CachePool CI binaries.

- Architecture spec: `prompt/insitu_cache_architecture_v2.md`
- Structure map (dated, badged by model status): `prompt/insitu_cache_structure_map_*.md` — read the newest
- Development log: `prompt/WORKLOG.md`

---

## 1. CachePool, and what the model reproduces

### The hierarchy

```
CLUSTER
└── GROUP  × (nb_x × nb_y)                     mesh; two NoC levels cross it
    └── TILE × tiles_per_group
        ├── CORE COMPLEX × cc_per_tile         ← the unit that owns a Spatz + one cache bank
        │   ├── scalar hart × NumScalarPerCC   ← 1 today, 2 on the dual-scalar config
        │   └── Spatz vector unit              ← ONE per complex, shared by its harts
        ├── L1 instruction cache               one per tile
        ├── per-port-class crossbars           any core → any bank, by address
        └── L1 data cache slice                one bank per core complex
```

**The core complex, not the core, is the unit of ownership.** A complex owns one Spatz and
one L1 cache controller; its harts own neither. So `NumCores = NumCC × NumScalarPerCC` while
`NumL1CacheCtrl = NumCC`. Anything counted by the peripheral or the barrier is per hart;
anything provided by the cache or the vector unit is per complex.

### The L1 is shared, not private

This is the defining property and the easiest thing to get wrong. A tile's crossbars route
**any** core to **any** bank by address; remote crossbars extend that across tiles, and the L1
NoC across groups. A core's "own" bank is not privileged — the address decides. Each port
class (the Spatz VLSU lanes, and one scalar port per hart) gets its own crossbar plane, so
vector and scalar traffic never contend for the same arbiter.

### Two NoC levels

| Level | Carries | Topology |
|---|---|---|
| **L1 NoC** | core → L1, one mesh per port class | groups on a mesh, XY routing, cross-group by tunnel |
| **L2 refill mesh** | L1 → memory (refill, eviction, write-through) | group per node; memory channels hang off the unused edge directions |

### Inside a cache bank

Per-port-class crossbar → optional part-coalescer → AMO/LR-SC shim on each scalar lane →
the cache core (tag array, MSHR, hash-or-LRU victim selection, refill, eviction). Refill and
eviction leave on a wide egress that the group arbitrates.

### The multi-scalar core complex

When `NumScalarPerCC > 1`, several scalar harts share one Spatz. Hardware arbitrates:

- **Locked** — one hart owns the vector unit; its requests pass through fully pipelined. The
  other hart stalls on its next vector instruction. This is the fast path.
- **Free** — nobody holds the lock; both harts get round-robin access, but only **one request
  outstanding at a time**, and a vector load/store blocks the next grant from *either* hart
  until it has fully drained. Vector arithmetic still pipelines.

Software takes the lock through two peripheral addresses (acquire / release) that always
complete immediately and return an outcome code. Note the consequence: **Free mode is not the
fast path.** A kernel that uses the vector unit alone still pays Free-mode serialisation if it
never acquires the lock.

### The cluster peripheral

Hardware barrier, boot control, end-of-computation, and the L1D configuration block. The
barrier is **two-level and masked**, not a global counter: a barrier *read* means "all cores of
my tile", a barrier *write* carries a per-core participant mask as its data, and a
cluster-level register selects which tiles participate. Partial barriers are real, and a
partial barrier is only safe when nothing outside the participating set is waiting at a
barrier anywhere in the cluster.

---

## 2. Running the model

### Build

All commands run from the inner `gvsoc/` folder.

```bash
eval "$(scripts/setup_elfutils_headers.sh --env)"
CXX=g++-14.2.0 CC=gcc-14.2.0 CMAKE=cmake-3.18.1 make build TARGETS="cachepool_v3"
```

`cachepool_v3` is **not** in the default target list — name it explicitly.

### Run

```bash
source sourceme.sh
gvsoc --target=cachepool_v3 --binary <elf> image flash run
```

Test binaries live in the ManyRVData tree under `software/build/CachePoolTests/`.

### Topology

Set by environment variable at elaboration time. Defaults give 16 cores.

| Variable | Default | Meaning |
|---|---:|---|
| `CACHEPOOL_V3_NB_X_GROUPS` / `_NB_Y_GROUPS` | 1 / 1 | mesh dimensions — `2×2` = 64 cores, `4×4` = 256 |
| `CACHEPOOL_V3_TILES_PER_GROUP` | 4 | tiles per group |
| `CACHEPOOL_V3_CORES_PER_TILE` | 4 | **core complexes** per tile, not harts |
| `CACHEPOOL_V3_SCALAR_PER_CC` | 1 | scalar harts per complex; `2` = the dual-scalar config |
| `CACHEPOOL_V3_BANKS_PER_TILE` | = cores | cache banks per tile |
| `CACHEPOOL_V3_MEM_LATENCY` | 50 | backing-store latency in cycles |
| `CACHEPOOL_V3_PERIPH_MAP` | `auto` | peripheral register generation — see below |

```bash
# 64 cores, single scalar
CACHEPOOL_V3_NB_X_GROUPS=2 CACHEPOOL_V3_NB_Y_GROUPS=2 \
  gvsoc --target=cachepool_v3 --binary <elf> image flash run

# 32 core complexes × 2 harts = 64 harts, dual scalar
CACHEPOOL_V3_NB_X_GROUPS=2 CACHEPOOL_V3_SCALAR_PER_CC=2 \
  gvsoc --target=cachepool_v3 --binary <elf> image flash run
```

### The peripheral map — get this right first

Three register-map generations are in circulation and **choosing the wrong one fails
silently**: the barrier read lands on scratch and stops blocking, the end-of-computation write
goes nowhere, and the run simply never terminates with no error.

| Map | HW_BARRIER | BOOT_CONTROL | EOC | Used by |
|---|---|---|---|---|
| `legacy` | 0x10 | 0x20 | 0x24 | the `CachePoolTests` binaries |
| `rlc_next` | 0x00 | 0x10 | 0x14 | current RTL working tree |
| `multi_scalar` | 0x00 | 0x18 | 0x1c | the dual-scalar RTL branch |

`auto` picks `legacy` for single-scalar and `multi_scalar` for dual. Override when the
binaries disagree. **The reliable check is whether the program terminates** — every wrong-map
failure is silent by construction, so look for the `[EOC]` line, not for plausible output.

### Model-fidelity switches

| Variable | Default | Effect |
|---|---|---|
| `CACHEPOOL_V3_SYNC_CACHE` | 0 | 1 = the calibrated synchronous cache path instead of the async per-cycle FSM |
| `CACHEPOOL_V3_L2_NOC` | 1 | 0 = flat router tree instead of the refill mesh |
| `CACHEPOOL_V3_CELL_COALESCER` | 0 | per-cell part-coalescer (see limitations) |
| `CACHEPOOL_V3_DRAMSYS` | 0 | 1 = one DRAMSys DRAM per memory channel of the refill mesh, instead of a flat backing store |
| `CACHEPOOL_V3_DRAM_TYPE` | `hbm2-example.json` | DRAM config, from `core/models/memory/dramsys_configs/` |
| `SPATZ_VLSU_LINE_SPLIT` | 0 | 1 = stop a unit-stride vector access being coalesced across a cache line (see limitations) |
| `SPATZ_LOCK_NO_LSU_GATE` | 0 | 1 = disable the Free-mode load/store gate, for A/B |
| `CACHEPOOL_BARRIER_COUNTING` | 0 | 1 = restore the old global counting barrier, for A/B |

### Running with real DRAM timing

By default the memory behind the refill mesh is a flat store with a fixed latency. With
`CACHEPOOL_V3_DRAMSYS=1` each **memory channel of the mesh gets its own DRAMSys DRAM**,
which is what the RTL testbench has. The two DRAM windows are contiguous, so they become one
address space striped across the channels; a refill routed to mesh channel *n* lands in DRAM
*n*, because the mesh and the interleaver select on the same address bits.

It needs SystemC preloaded and the SystemC-enabled launcher — `dramsys.so` does not link
SystemC itself, so without the preload the `sc_api_version` symbol is unresolved:

```bash
# once
make dramsys_preparation

# then: generate the config, and run it under the SystemC launcher
CACHEPOOL_V3_DRAMSYS=1 gvsoc --target=cachepool_v3 --binary <elf> image flash run   # writes gvsoc_config.json
LD_PRELOAD="$PWD/third_party/systemc_install/lib64/libsystemc.so.3.0.1 \
  $PWD/add_dramsyslib_patches/build_dynlib_from_github_dramsys5/DRAMSys/build/lib/libDRAMSys_Simulator.so" \
  install/bin/gvsoc_launcher_sc --config=gvsoc_config.json
```

Expect **10-100× the wall-clock** of the flat store. Use small kernels.

### Diagnostics

All are environment-gated and off by default.

| Variable | Reports |
|---|---|
| `CACHEPOOL_BARRIER_STATS` | per barrier completion: who was released, who stayed parked, the tile mask |
| `CACHEPOOL_PERIPH_SELFTEST` | what a pre-write read of the map-critical registers returns |
| `SPATZ_LOCK_STATS` | per core complex: grant handovers, denials, load/store-gate blocks, concurrent in-flight |
| `INSITU_MUX_STATS` | refill-mux queue depth and forward counts |
| `INSITU_SHADOW` | per-bank last-written vs served comparison |
| `INSITU_AMO_DEBUG=N` | AMO events — **the value is the line budget**, so `=1` yields one line and looks like silence |

Cache counters print unconditionally at end of simulation, and cross-line truncation events
print at power-of-two milestones during the run, because most kernels never reach `stop()`.

---

## 3. Component map

Under `core/models/cache/insitu/`:

| File | Role |
|---|---|
| `insitu_cache_core.{cpp,py}` | per-cycle cache FSM — tag array, MSHR, refill, eviction (the deployed path) |
| `insitu_cache_controller.{cpp,py}` | cycle-approximate controller (the calibrated alternative path) |
| `insitu_cache_xbar.{cpp,py}` | per-port-class crossbar: any core → any bank by address |
| `insitu_cache_remote_xbar.{cpp,py}` | cross-tile and cross-group extension of the same |
| `insitu_cache_amo_shim.{cpp,py}` | AMO / LR-SC on each scalar lane |
| `insitu_cache_par_coalescer.{cpp,py}`, `insitu_cache_cell_coalescer.{cpp,py}`, `insitu_cache_coalescer.{cpp,py}` | write coalescing, three granularities |
| `insitu_cache_refill_mux.{cpp,py}` | group-level refill/eviction arbitration |
| `insitu_cache_config_broadcast.{cpp,py}` | fans a partition-config write to every xbar and bank |
| `insitu_cache_interco.{cpp,py}` | flat hashed N→M interco (pre-structural path) |
| `insitu_cache_tile.py`, `insitu_cache_group.py` | composition |
| `insitu_cache_config.py` | all configuration dataclasses and the canonical factories |
| `insitu_calib_mem.{cpp,py}` | fixed-latency serializing refill responder for the calibration harness |

Topology for the target itself lives in `pulp/pulp/cachepool_v3/`, and the Spatz ownership
lock in `cachepool_v3_spatz_lock.{cpp,py}`.

---

## 4. Timing Characteristics

This section describes the latency each transaction class accrues in the model, and
how that corresponds to the RTL microarchitecture. Numbers in parentheses use the
canonical `cachepool_512` defaults.
**This section describes the cycle-approximate controller path.** The deployed `cachepool_v3`
target runs the per-cycle cache core instead, where latency emerges from the pipeline rather
than from these constants; the hop structure and the accounting model below still apply. All cycle counts are added to
the request's latency via `vp::IoReq::inc_latency()`; the simulator propagates them
back to the originating core.

### 4.1 Latency accounting model

The model distinguishes three latency sources that compose at each hop:

- **Pipeline latency** — a fixed, config-driven cycle count added by each hop for
  its own pipeline depth (`hit_latency_cycles`, `interco_latency_cycles`, etc.).
- **Resource-busy latency** — added only when a shared resource is still busy
  serving a prior transaction. Tracked via per-resource `_busy_until` cyclestamps
  (per-set bank port on the controller, per-output port on the interco). Computed as
  `max(0, busy_until - now)`.
- **Transport latency** — for outgoing refill/eviction requests, the downstream
  memory's return latency, measured via `vp::IoReq::get_full_latency()` on response.

Request-object mechanics: on a hit the controller calls `req->inc_latency(...)` and
returns `IO_REQ_OK` (synchronous response). On a miss it calls `req->save()`, parks
the request on an MSHR side-deque, and returns `IO_REQ_PENDING`; when the refill
response arrives, the controller drains the MSHR queue, adds the refill-path
latency, calls `req->restore()`, and issues `resp()` to the originator.

### 4.2 Hop-by-hop latency stackup (from core to cache and back)

For a request from a core to one of the tile's TCDM input ports:

| Hop | Component | Latency (cycles) | Notes |
|---|---|---:|---|
| 1 | Core TCDM port → `cores_ico` (per-core router) | 0–1 | Router's own latency, unrelated to the cache. |
| 2 | `cores_ico` → tile's `i_INPUT(port)` slave | 0 | Pure binding pass-through (composite port). |
| 3 | Interco forward + arbitration | `interco_latency_cycles` (1) + contention | `+(output_busy_until − now)` if another input targets the same controller this cycle. |
| 4 | Controller hit/miss classification + bank read | `hit_latency_cycles` (4) + set-busy | `+(set_busy_until[set] − now)` on back-to-back same-set hits. |
| 5 | Response path (mirror of the request path) | implicit in hop 3 & 4 | Modeled as part of hop 4's `hit_latency_cycles` — the controller returns `IO_REQ_OK` with the full round-trip latency baked in. |

Minimum cache-tile overhead on a clean read hit with no contention: **5 cycles**
(1 interco + 4 controller). Adding the per-core router and the core's own load-use
delay brings the whole-core round-trip to ≈ 7 cycles — matches the RTL §12.1
hand-calc of `cachepool_512`.

### 4.3 Per-transaction timing reference

#### Read hit on VALID line

**RTL behavior.** Request enters the controller, preread arbiter selects it, bank
read issues, hit detected in the next cycle, response FIFO push, return.

**Model.** `handle_request()` (`insitu_cache_controller.cpp:247`) on a `VALID` tag
match computes:

```
latency = hit_latency_cycles                             // 4
         + max(0, line.ready_cycle - now)                 // 0 when line settled
         + max(0, set_busy_until[set] - now)              // 0 when no back-to-back conflict
set_busy_until[set] = now + latency                       // bank port busy window
req->inc_latency(latency)
return IO_REQ_OK
```

**Typical cost.** 4 cycles in steady state. A second read to the same set in the
same cycle as the first one pays `4 + 4 = 8` (the second one waits out the first's
bank-port busy window). A read immediately after a just-completed refill pays
`4 + refill_bank_write_cycles = 6` (line.ready_cycle is still a few cycles in the
future).

#### Read hit on a pending line (MSHR merge)

**RTL behavior.** Request targets a line already in `READ_PEND` or `WRITE_PEND`.
Instead of issuing another refill, the controller stores the request's info into the
next MSHR subarray slot and returns nothing until refill lands.

**Model.**

```
if retr_fifo_level >= retr_fifo_depth:
    return IO_REQ_DENIED                                  // back-pressure, core retries
else:
    req->save()
    mshr[set].push_back({req, arrival_cycle = now})
    retr_fifo_level++
    return IO_REQ_PENDING                                 // core waits
```

On refill response (see §4.4 below) `fsm_drain_mshr()` replays each queued request
with:

```
base_latency = max(line.ready_cycle - arrival_cycle, hit_latency_cycles)
             + subarray_idx * mshr_drain_cycles_per_subarray    // 0, 1, 2, … for queued order
req->inc_latency(base_latency); req->restore(); req->get_resp_port()->resp(req)
```

**Typical cost.** `refill_latency_seen - time_waited_before_miss_landed + hit_latency`
for the first mergee, plus 1 cycle per subsequent mergee to model sequential MSHR
drain. A read that merges onto a pending line *just* issued (same cycle) sees the
full miss penalty; a read that merges at the tail of a long wait sees only
`hit_latency_cycles + subarray_idx`.

#### Read miss (new line)

**RTL behavior.** Pick victim, if dirty push to evict FIFO, mark line `READ_PEND`,
push to miss FIFO, issue refill request to L2, wait for refill beats, update cache,
drain queued requests.

**Model.** `handle_request()` on a `tag-miss` path:

```
if miss_fifo_level >= miss_fifo_depth:  return IO_REQ_DENIED
victim_way = hash(tag, set)   or  LRU-last
if lines[set,victim_way].dirty:
    if evic_fifo_level >= evic_fifo_depth:  return IO_REQ_DENIED
    issue_eviction(old_line_addr)                         // see §4.5
lines[set,victim_way].state = READ_PEND
req->save(); mshr[set].push_back({req, now})
retr_fifo_level++; miss_fifo_level++
issue_refill(line_base_addr, set, victim_way):
    refill_req.set_size(cache_line_bytes)                 // 64 B
    refill_req.set_duration(cache_line_bytes / refill_beat_bytes)   // 4 beats
    refill_itf.req(refill_req)                            // downstream memory
return IO_REQ_PENDING
```

When the refill response arrives, `refill_resp_handler()`:

```
line.state = VALID
line.ready_cycle = now + req->get_full_latency() + refill_bank_write_cycles
fsm_drain_mshr(set)       // see read-hit-on-pending
```

**Typical cost.** `hit_latency + L2_round_trip + refill_bank_write_cycles`. For the
canonical L2 (the cluster's SPM banks) with ~1-cycle latency and 4-beat refill:
roughly `4 + (1 + 4) + 2 = 11` cycles *if* the memory serves all 4 beats in 1 cycle
(bandwidth-unlimited SPM). For a DRAM model with 20-cycle latency it grows to
`4 + (20 + 4) + 2 ≈ 30` cycles. The per-beat time is carried via `set_duration(4)`
so the memory can compute occupancy-based back-pressure on adjacent refills.

#### Write hit on VALID line

**RTL behavior.** Write merges into the cache line (marks dirty for eventual
write-back on eviction) **and** simultaneously emits the new bytes to the write-
through coalescer, which batches and sends to L2.

**Model.** Same pipeline as a read hit, plus two side effects:

```
line.dirty = true
issue_write_through(user_req)                             // fire-and-forget to coalescer
```

The write-through emission is not in the user request's critical path — the user
gets `IO_REQ_OK` with the same `hit_latency_cycles` as a read hit. The coalescer
independently absorbs the write (adding 1 cycle of its own, §4.6) and flushes to
L2 asynchronously.

**Typical cost.** 4 cycles from the core's perspective. Matches RTL §12.3
"Write hit: ~4 cycles (fire-and-forget)".

#### Write miss

**RTL behavior.** Same refill pipeline as a read miss, but the line is marked
`WRITE_PEND`; when the refill arrives, the incoming write's bytes are merged into
the refilled line before it becomes `VALID`.

**Model.** Identical to the read miss branch with `state = WRITE_PEND`. On MSHR
drain, the request is marked dirty and `issue_write_through()` fires:

```
if req->get_is_write():
    line.dirty = true
    issue_write_through(req)      // emit to coalescer
```

**Typical cost.** Same as read miss (the write is not acknowledged to the core
until the refill lands — ≈ 11 cycles for the SPM-as-L2 config, ≈ 30 for DRAM).
After the core gets its OK, the write-through to L2 proceeds asynchronously through
the coalescer.

#### Eviction of a dirty victim

**RTL behavior.** When a miss picks a dirty victim, the old line is pushed to the
eviction FIFO and a writeback is issued to L2 *before* the refill data arrives.
Folded SRAMs may need a multi-part read to assemble the full line, costing extra
cycles.

**Model.** In the miss path:

```
if line.dirty:
    evic_fifo_level++                   // full → DENIED, core retries
    evict_req.set_size(64).set_duration(4)
    evict_itf.req(evict_req)            // fire-and-forget currently
    evic_fifo_level--                   // v1 model releases immediately
```

Folded-SRAM assembly cost is not on the request's critical path in the model (the
dirty line is logically snapshotted when the miss is accepted); instead the RTL
§7.7 penalty shows up on the *next* refill via `refill_bank_write_cycles`. If
finer fidelity is needed, enable `folded_evict_penalty_cycles > 0` — it adds to the
next refill's `line.ready_cycle` to represent the bank still busy with the multi-
beat eviction.

**Typical cost added over a plain miss.** 0 in the v1 release (eviction is
asynchronous). With a bandwidth-constrained L2 the eviction will serialize with the
refill on the same port — captured automatically by `set_duration(4)` on both
requests and the memory's `next_packet_start` book-keeping.

### 4.4 Refill response path

`refill_resp_handler()` is called by the downstream memory via the master port's
`set_resp_meth`. It decodes the set/tag from the request's address, finds the
pending way, marks the line `VALID`, and captures:

```
line.ready_cycle = now + req->get_full_latency() + refill_bank_write_cycles
```

`get_full_latency()` combines the memory's `set_latency()` and `set_duration()`
values, so both fixed latency and bandwidth-driven duration are reflected. New hits
to the same line before `ready_cycle` pay the difference as extra latency (§4.3,
read-hit formula).

If the downstream memory responded synchronously (`IO_REQ_OK` on the original
request), `issue_refill()` calls `refill_resp_handler()` inline in the same cycle —
the line becomes `VALID` immediately; hits pay only `refill_bank_write_cycles` as
the "still settling" penalty.

### 4.5 Interco arbitration timing

Per-output round-robin serialization is modeled via `output_busy_until[out_id]` in
`insitu_cache_interco.cpp:90-108`:

```
latency = interco_latency_cycles                          // 1
if output_busy_until[out] > now:
    latency += (output_busy_until[out] - now)
output_busy_until[out] = now + latency
req->inc_latency(latency)
return outputs[out]->req_forward(req)
```

**Typical cost.** 1 cycle when the target controller's input was idle this cycle.
If two TCDM ports hash to the same controller in the same cycle, the second pays
`1 + 1 = 2` cycles, the third `1 + 2 = 3`, etc. This captures the RTL's
single-accept-per-cycle arbitration at the controller's request-buffer input.

### 4.6 Coalescer timing

`insitu_cache_coalescer.cpp:139`: the coalescer acks each incoming write with
`req->inc_latency(1)` and returns `IO_REQ_OK` immediately. A ClockEvent re-arms the
watchdog every cycle while the FSM is in `WRITE_COAL`. On flush (new tag, watchdog
= 0, or read-snoop match), one line-wide `IoReq` is issued downstream.

**Typical cost** on the user's critical path. **1 cycle** per write, regardless of
whether it's merged or flushes a previous burst. The actual DRAM-side reduction
(4–16× fewer wide bursts vs. word-level writes) shows up as reduced downstream
occupancy, not as saved cycles on the core's request.

### 4.7 Back-pressure: stall costs

When a resource is full, the model returns `IO_REQ_DENIED`. `DENIED` causes the
upstream to retry **next cycle** (standard GVSoC convention). Each such retry is a
wasted 1-cycle attempt captured in the corresponding counter:

| Counter | Trigger | Cost per event |
|---|---|---|
| `cnt_stall_miss_fifo` | `miss_fifo_level ≥ miss_fifo_depth` on a new miss | 1 cycle wasted attempt + whatever the FIFO takes to drain |
| `cnt_stall_evic_fifo` | `evic_fifo_level ≥ evic_fifo_depth` on a dirty-victim miss | same |
| `cnt_stall_mshr_full` | `retr_fifo_level ≥ retr_fifo_depth` on an MSHR-merge | same |

These correspond to the `*_STALL` FSM states in RTL §7.1. They only matter under
high miss pressure (e.g. random-access workloads with small caches); typical
streaming/GEMM keeps FIFO levels well below capacity so these counters stay low.

### 4.8 End-to-end worked examples

Assume canonical config (`hit_latency=4`, `interco_latency=1`,
`refill_bank_write=2`, `refill_beat_bytes=16`, L2 = SPM with 1-cycle latency),
tile-entry to tile-exit latency from the core's view:

| Scenario | Cycle count | Derivation |
|---|---:|---|
| Read hit, no contention | **5** | interco(1) + ctrl(4) |
| Read hit, same-set repeated | **9** | interco(1) + ctrl(4) + prior-set-busy(4) |
| Read hit on just-refilled line | **7** | interco(1) + ctrl(4) + refill_bank_write(2) |
| Read miss, clean victim, L2 SPM | **12** | interco(1) + ctrl(4) + L2(1) + beats(4) + refill_bank_write(2) |
| Read miss, clean victim, L2 DRAM (20-cycle) | **31** | interco(1) + ctrl(4) + L2(20) + beats(4) + refill_bank_write(2) |
| Write hit on VALID line | **5** | interco(1) + ctrl(4) — coalescer is off-critical-path |
| Write miss, dirty victim, L2 SPM | **12** | same as read miss (dirty eviction is asynchronous) |
| Read hit that merges on in-flight line | **≈ refill_remaining_cycles + 4** | waits for `line.ready_cycle` plus `hit_latency` |
| 2nd mergee in the same drain batch | **prev + 1** | `subarray_idx` increment, governed by `mshr_drain_cycles_per_subarray` |

These match RTL §12 within ±1 cycle when the downstream memory's own latency/
bandwidth matches the RTL AXI profile.

---

## 5. Configuration

### Quick-start: canonical config

```python
from cache.insitu.insitu_cache_config import make_cachepool_512_config
cfg = make_cachepool_512_config()
# 4 controllers, 4-way, 128 sets, 512b line, 4-cycle coalescer watchdog, hash way-select
```

### Building a custom config in Python

```python
from cache.insitu.insitu_cache_config import (
    InsituCacheControllerConfig, InsituCacheCoalescerConfig,
    InsituCacheIntercoConfig, InsituCacheTileConfig,
)

ctrl = InsituCacheControllerConfig(
    cache_line_bytes=128,          # 1024b line
    num_ways=8,
    num_sets=64,                   # 64 sets × 8 ways × 128 B = 64 KB per ctrl
    use_hash_way_select=False,     # use LRU instead of hash
    hit_latency_cycles=5,
    retr_fifo_depth=32,
)
coal = InsituCacheCoalescerConfig(cache_line_bytes=128, watchdog_cycles=8)
intc = InsituCacheIntercoConfig(dynamic_offset=4)   # coarser interleave

tile_cfg = InsituCacheTileConfig(
    num_controllers=2, num_cores=2, tcdm_ports_per_core=3,
    controller=ctrl, coalescer=coal, interco=intc,
)
```

Then pass `tile_cfg` into an `InsituCacheTile(...)`. Note that `cachepool_v3` builds its
own configuration in `pulp/pulp/cachepool_v3/cachepool_v3_system.py` and stamps each tile
with its id and topology — for that target, edit there or use the environment knobs in §2
rather than constructing a config by hand.

### Parameter reference

#### Controller (`InsituCacheControllerConfig`)

| Field | Default | Meaning |
|---|---|---|
| `cache_line_bytes` | 64 | Line size (512b = 64B in the canonical RTL). |
| `num_ways` | 4 | Set associativity. |
| `num_sets` | 128 | Sets per controller (`CacheBankDepth`). |
| `tcdm_word_bytes` | 4 | Upstream request granularity. |
| `refill_beat_bytes` | 16 | Refill beat width — used to compute `set_duration()` for multi-beat occupancy modeling. |
| `use_hash_way_select` | `True` | If True, victim way = hash(tag, set); else LRU. |
| `hit_latency_cycles` | 4 | Hit-path latency (request-buf + preread + bank-read + hit-detect). |
| `refill_bank_write_cycles` | 2 | Extra cycles after refill response before pending MSHR requests can be served. |
| `folded_evict_penalty_cycles` | 3 | Extra cycles on dirty eviction to model folded-SRAM full-line read. Set to 0 if `PartSplit=1`. |
| `mshr_drain_cycles_per_subarray` | 1 | Per-pending-request drain latency after a refill. |
| `resp_fifo_depth` | 4 | |
| `retr_fifo_depth` | 16 | MSHR retrieval FIFO. Governs max outstanding mergers per set. |
| `miss_fifo_depth` | 4 | Full → new miss is `DENIED` (upstream retries). |
| `evic_fifo_depth` | 4 | Full → a miss with dirty victim is `DENIED`. |
| `wt_fifo_depth` | 4 | Reserved for WT-side modeling refinement. |

#### Coalescer (`InsituCacheCoalescerConfig`)

| Field | Default | Meaning |
|---|---|---|
| `cache_line_bytes` | 64 | Must match the controller. |
| `watchdog_cycles` | 4 | Cycles with no new coalescable write before forcing a flush. |

#### Interco (`InsituCacheIntercoConfig`)

| Field | Default | Meaning |
|---|---|---|
| `num_inputs` | 20 | Number of upstream TCDM ports (auto-synced by the tile from `num_cores × tcdm_ports_per_core`). |
| `num_outputs` | 4 | Number of cache controllers (auto-synced by the tile). |
| `dynamic_offset` | 2 | Bit offset at which `log2(num_outputs)` bits select a controller. Bits `[3:2]` for `num_outputs=4`. |
| `interco_latency_cycles` | 1 | Fixed 1-cycle forward latency. Contended outputs add extra serialization cycles. |

#### Tile (`InsituCacheTileConfig`)

| Field | Default | Meaning |
|---|---|---|
| `num_controllers` | 4 | Tile-wide controller count. |
| `num_cores` | 4 | Cores served by this tile. |
| `tcdm_ports_per_core` | 5 | Scalar + lanes (Spatz: 1 + 4 = 5). |
| `controller` | RTL defaults | Passed to every controller in the tile (they are identical). |
| `coalescer` | RTL defaults | Passed to every coalescer. |
| `interco` | RTL defaults | Auto-has `num_inputs`/`num_outputs` synced by the tile. |

### Overriding at run time

Topology, fidelity and backing store are environment variables — see §2. They are read
at elaboration, so a change needs `rm -f gvsoc_config.json` before the next run.

Cache *geometry* (line size, ways, sets, FIFO depths) is not exposed that way: it comes
from `make_cachepool_fpu_512_config()` in `insitu_cache_config.py`, which
`cachepool_v3_system.py` then stamps with the topology. Change it there.

---

## 6. Telemetry

Each component keeps per-instance counters. The cache core reports unconditionally at end of
simulation; anything that can be silently wrong reports during the run instead, at power-of-two
milestones, because most CachePool kernels never reach `stop()`.

### Cache core

```
rd_hit, rd_miss, wr_hit, wr_miss            # classification
refill, evict, flush                        # line movement
served_lat_avg, HIT avg/min, MISS avg/min   # measured service latency
XLINE                                       # cross-line truncation events + bytes dropped
```

`served_lat_avg` split by hit and miss is the number to compare against RTL; the aggregate
average moves with hit rate and is not a latency measurement.

### Coalescer

```
cnt_writes_absorbed, cnt_merged_bursts      # ratio = effective coalescing
cnt_flushes_new_tag / _watchdog / _snoop    # why each flush happened
```

### Arbiters

`SPATZ_LOCK_STATS` and `INSITU_MUX_STATS` (§2) cover the Spatz ownership arbiter and the
group refill mux respectively.

---

## 7. Standalone Testbench

Minimal SoC for driving scripted traffic:

- File: `pulp/insitu_cache_tb.py`
- Target name: `insitu_cache_tb`
- Hierarchy: `RV32 host → ico → {scratch memory, cache tile, stdout}`; cache tile's
  `o_L2` → `memory.Memory(latency=20)`.
- Cache configured with 1 TCDM input port + 4 controllers (scaled down from the
  canonical `cachepool_512` to match the single-host TB).

Run:

```bash
gvsoc --target=insitu_cache_tb --binary path/to/rv32im_test.elf run
gvsoc --target=insitu_cache_tb --binary <elf> --trace=insitu_cache run
```

This is the recommended harness for focused microbenchmarks (random reads, streaming
writes, blocked GEMM) when comparing cycle counts against RTL.

---

## 8. Current status and limitations

**Calibrated.** 64-core RLC anchor against RTL, held stable across the model's development.
The residual error is **confined to the refill path**, and its sign flips with workload class:
throughput-bound kernels run slow, latency-bound kernels run slightly fast. That sign flip is
the evidence the error is localised rather than global.

**Known limitations, in rough order of how much they cost you:**

1. **Refill path** — one outstanding miss per controller. This is the dominant modelling gap
   and the reason throughput-bound kernels are slow. The other half of it — every mesh channel
   sharing one flat backing store — is addressed by `CACHEPOOL_V3_DRAMSYS=1` (§2), which gives
   each channel its own DRAM; whether real DRAM timing closes the gap has not yet been measured.
2. **Cross-line truncation** — a vector access that straddles a cache line has its tail bytes
   dropped, silently, with `IO_REQ_OK` still returned. The straddle is generated inside the
   model, not requested by software: unit-stride vector accesses are sized at the lane width
   regardless of element size. `SPATZ_VLSU_LINE_SPLIT=1` prevents it and costs nothing on the
   calibration anchor, but is **default off** pending review, so the default build still
   truncates. Any run that reports `XLINE` events has lost data.
3. **Cross-core shared-data visibility** — cross-core write/barrier/read patterns show
   mismatches that scale with core count. Root cause open. Per-core-private data is unaffected.
4. **Cell coalescer** — `CACHEPOOL_V3_CELL_COALESCER=1` crashes in the multi-tile response
   path. Default off.
5. **Address scrambling** — the RTL's exact hash-way polynomial is approximated by a
   Knuth-style hash. Only observable if set-index aliasing becomes workload-visible.
6. **Cache partitioning** — `l1d_part` / `l1d_xbar_config` / `l1d_flush` are accepted as
   scratch and are no-ops.
7. **Shared Spatz fidelity** — each hart carries its own vector register file, mutually
   excluded in time rather than physically shared, so a program that illegally relies on
   retaining vector state across a lock handover passes here and fails on RTL. Scalar FP is
   not gated by the lock. `acc_mux`'s writeback FIFO is not modelled, so a writeback-heavy
   Free-mode stream is optimistic.

**A note on how to read a passing run.** Several of the defects above are silent in the result
and loud only in the log — truncation is the clearest case. A run that prints `XLINE` lines and
still reports PASS has produced a self-consistent wrong answer. Grep the log before trusting
the verdict.

---

## 9. Troubleshooting

| Symptom | Likely cause / fix |
|---|---|
| `ModuleNotFoundError: typing_extensions` / `override` | Host Python < 3.10. Use the shim (see §2 Prerequisites). |
| `ModuleNotFoundError: pkg_resources` | `setuptools ≥ 81`. Downgrade: `python3.12 -m pip install --user 'setuptools<81'`. |
| `gen_cache_insitu_... not found at runtime` | The component was never instantiated, so it was never compiled. Rebuild with the configuration that uses it — the build compiles only what the elaborated target asks for. |
| Infinite loop / hang on first miss | Refill response path: the downstream memory isn't calling back via `resp_meth`. Check `o_L2` is bound to something that actually completes the request (a `memory.Memory` or a proper router chain). |
| Address decomposition wrong — low hit rate on an expected streaming pattern | `rm_base` mismatch: the cache is wired to see absolute addresses. If a router in the path removes the offset, the set index shifts and the access pattern scatters. |
| `Signature mismatch` at Python bind time | The Python port signatures are declared in each component's `o_XX`/`i_XX` factories. Make sure the bound interface uses `signature='io'` for IO ports and a matching tag elsewhere. |

---
| Run produces no output at all and never ends | Wrong peripheral map. Check `CACHEPOOL_V3_PERIPH_MAP` against the binary; look for the `[EOC]` line as the signal, not for plausible output. |
| `gvsoc_config.json` ignores a topology change | It is not regenerated if it already exists. `rm -f gvsoc_config.json` after any Python-side change. |
| Simulator crashes in an unrelated constructor after editing an ISS header | Generated model targets do not track header dependencies, so objects compiled against the old layout get linked against newly compiled ones. `rm -rf build/engine/CMakeFiles/gen_isa_*` — **all** of them, not just the target you are building: each ISA variant has its own object directory, and a stale one only bites when you next build the target that uses it. A fresh clone is unaffected. |
| A run looks suspiciously silent | The `gvsoc` wrapper can swallow stdout/stderr. Generate the config, then invoke `install/bin/gvsoc_launcher --config=gvsoc_config.json` directly. |
| Kernel passes but the log shows `XLINE` events | Cross-line truncation has dropped data. The pass is self-consistent and wrong. See §8. |

---

## 10. Where to Dig Next

- **Architecture spec**: `prompt/insitu_cache_architecture_v2.md` — current RTL microarchitecture.
- **Structure map**: `prompt/insitu_cache_structure_map_*.md` — the hierarchy with every node
  badged by model status (implemented / approximated / not modelled). Read the newest.
- **Development log**: `prompt/WORKLOG.md` — why each change was made, and what was measured.
- **Calibration harness**: `pulp/insitu_cache_calib/` plus `insitu_calib_mem` — replays the same
  trace through the same memory model as the RTL testbench so the two can be diffed per access.
- **Target topology**: `pulp/pulp/cachepool_v3/` — cluster, group, tile, and the Spatz lock.
