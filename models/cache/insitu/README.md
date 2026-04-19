# InSitu Cache — GVSoC Performance Model

A cycle-approximate GVSoC model of the CachePool InSitu L1 data cache. Targets
<5% cycle error vs. RTL on typical streaming + random-access workloads.

- **Architecture spec** (RTL-side reference): `prompt/insitu_cache_architecture.md`
- **Implementation plan** (design decisions, phases, fidelity knobs): `prompt/insitu_cache_gvsoc_plan.md`

---

## 1. What's in the Box

Four components, three C++ / four Python files under `core/models/cache/insitu/`:

| Component | Files | Role |
|---|---|---|
| **Controller** | `insitu_cache_controller.{cpp,py}` | One cache controller. Tag array, hit/miss, MSHR merge, hash-or-LRU victim select, eviction, refill, write-through hook. Serves a subset of lines (interleaved). |
| **Interco** | `insitu_cache_interco.{cpp,py}` | Hashed N-to-M crossbar. Routes upstream TCDM ports to cache controllers by address bits `[dynamic_offset +: log2(num_outputs)]`, with per-output round-robin arbitration. |
| **Coalescer** | `insitu_cache_coalescer.{cpp,py}` | Write-through merger. 3-state FSM (`IDLE` / `WRITE_COAL` / `FLUSH`) with a watchdog timeout, tag-change flush, and read-snoop flush. |
| **Tile** (composite) | `insitu_cache_tile.py` | Composes the three above into one tile: interco → N controllers → N coalescers → L2 fan-in router. Exposes `i_INPUT(port)` per TCDM port and a single `o_L2` master. |
| **Config** | `insitu_cache_config.py` | `Config` subclasses for each atomic component plus a plain `InsituCacheTileConfig` Python dataclass bundling them. Includes `make_cachepool_512_config()` for the canonical RTL defaults. |
| **Standalone TB** | `pulp/insitu_cache_tb.py` | Minimal single-host testbench for microbenchmarking the tile in isolation (no Spatz cluster). Target name: `insitu_cache_tb`. |

### Topology (canonical `cachepool_512`)

```
TCDM request ports (20 = 4 cores × 5 ports per core)
     │
     ▼
┌───────────────────────────────┐
│ InsituCacheInterco (20 → 4)   │   hash bits [3:2] → controller id
└───┬───────┬───────┬───────┬───┘
    │       │       │       │
  ┌─▼─┐   ┌─▼─┐   ┌─▼─┐   ┌─▼─┐
  │C0 │   │C1 │   │C2 │   │C3 │     4-way, 128 sets, 512b line, 256 KB total
  └─┬─┘   └─┬─┘   └─┬─┘   └─┬─┘     hash victim-way by default
    │       │       │       │
    │ WRITE_THROUGH paths (per controller)
    │  ▼       ▼       ▼       ▼
    │ Coal0  Coal1  Coal2  Coal3   4-cycle watchdog coalescers
    │  │       │       │       │
    │  └───────┴───┬───┴───────┘
    │              │
    │ REFILL, EVICT (direct, bypass coalescer)
    ▼              ▼
┌────────────────────────┐
│ l2_router (fan-in)     │
└────────┬───────────────┘
         ▼
      o_L2 (bound by caller to SPM, DRAM, wide_axi, …)
```

### What's modeled

Latency, throughput, hit/miss classification, MSHR coalescing, write coalescing,
FIFO backpressure, per-set bank serialization, eviction + refill cost, and the
pipeline-depth hit latency.

### What's **not** modeled (on purpose — see plan §1.2)

- Folded SRAM column skewing (`PartSplit`) — folded-eviction cost captured as a single
  `folded_evict_penalty_cycles` knob
- Pseudo-dual-port per-word conflicts — folded into aggregate per-set busy timestamps
- Forwarding-buffer internal FSM — optional fidelity refinement for Phase 7
- Exact RTL hash polynomial — any deterministic hash on `(tag, set)` is used

---

## 2. Build

### Prerequisites

Python ≥ 3.10 is required by the GVSoC Python code. On hosts where `python3` points at
Python 3.9, shim a 3.12:

```bash
ln -sf /usr/bin/python3.12 /tmp/py312_shims/python3
export PATH=/tmp/py312_shims:$PATH
```

Make sure the 3.12 site-packages contain the GVSoC deps. On a fresh host:

```bash
python3.12 -m pip install --user \
    typing_extensions prettytable rich pexpect pycryptodome ppk2_api pyelftools \
    psutil lz4 numpy pandas matplotlib mako hjson jsonref 'setuptools<81'
```

(The `setuptools<81` pin is needed because GVSoC's `reggen` imports the deprecated
`pkg_resources` API.)

### Build the code

The model is compiled on demand — it's triggered by any target that instantiates the
`InsituCacheTile`. Two canonical invocations:

```bash
# Standalone testbench (recommended for first build — smallest surface):
CXX=g++-14.2.0 CC=gcc-14.2.0 CMAKE=cmake-3.18.1 make all TARGETS=insitu_cache_tb

# Spatz cluster with the cache enabled:
CXX=g++-14.2.0 CC=gcc-14.2.0 CMAKE=cmake-3.18.1 \
    make all TARGETS="spatz:use_insitu_cache=True"
```

Three shared libraries are installed to `install/models/`:

```
gen_cache_insitu_insitu_cache_controller_cpp_*.so
gen_cache_insitu_insitu_cache_interco_cpp_*.so
gen_cache_insitu_insitu_cache_coalescer_cpp_*.so
```

---

## 3. Running

After building, source the install directory:

```bash
source sourceme.sh
```

### Spatz cluster with the cache

```bash
gvsoc --target=spatz --target-property use_insitu_cache=True \
      --binary path/to/your/spatz_rv32.elf run
```

When `use_insitu_cache=False` (or omitted) the cluster falls back to the legacy direct
core↔TCDM path — **no behavior change vs. the pre-cache build**.

### Standalone testbench

```bash
gvsoc --target=insitu_cache_tb --binary path/to/rv32im_test.elf run
```

This target brings up one RV32 scalar host, a single `InsituCacheTile`, a latency-20
backing memory, and a stdout at 0x8000_0004. Address map:

- `0x0000_0000 – 0x0003_FFFF` : scratch (uncached, for stack)
- `0x1000_0000 – 0x1003_FFFF` : cached region (goes through the tile)
- `0x8000_0004` : stdout

Use it to drive focused microbenchmarks (random reads, streaming writes, blocked
GEMM) against the cache without bringing up a full Spatz cluster.

### Tracing

The model registers standard `vp::Trace` channels. Useful filters:

```bash
# Per-controller transaction log (addr, hit/miss, way, latency)
gvsoc --target=spatz --target-property use_insitu_cache=True \
      --trace=insitu_cache/ctrl_.*/trace \
      --binary <elf> run

# Full interco routing log (which input → which controller)
gvsoc ... --trace=insitu_cache/interco/trace ...

# Coalescer FSM events (writes absorbed, flushes by watchdog/new-tag/snoop)
gvsoc ... --trace=insitu_cache/coal_.*/trace ...

# Everything
gvsoc ... --trace=insitu_cache ...
```

Trace levels: `TRACE` (per-request), `DEBUG` (FSM transitions), `INFO` (instantiation).

---

## 4. Timing Characteristics

This section describes the latency each transaction class accrues in the model, and
how that corresponds to the RTL microarchitecture. Numbers in parentheses use the
canonical `cachepool_512` defaults (§1 of this doc). All cycle counts are added to
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

Then pass `tile_cfg` into an `InsituCacheTile(...)` or into the spatz cluster's
`ClusterArch(... insitu_cache_cfg=tile_cfg)`.

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

### Overriding via CLI

The spatz target surfaces one user property today:

```bash
gvsoc --target=spatz --target-property use_insitu_cache=True ...
```

Any other knob (line size, ways, FIFO depths, …) currently requires constructing a
custom `InsituCacheTileConfig` in Python and routing it through a modified cluster
target. Exposing more knobs as CLI properties is a Phase-7 refinement.

---

## 6. Integration in the Spatz Cluster

**File touched:** `pulp/pulp/snitch/snitch_cluster/snitch_cluster.py`

`ClusterArch.__init__` gained two new arguments:

```python
ClusterArch(..., use_insitu_cache=False, insitu_cache_cfg=None)
```

When `use_insitu_cache=True`:

- An `InsituCacheTile` is created inside the cluster.
- Each core's scalar `o_DATA` → `cores_ico[core_id]` → `cache_tile.i_INPUT(port)` for
  TCDM-range addresses (`rm_base=False` so the cache sees absolute addresses).
- Each Spatz vector lane's `o_VLSU(lane)` → `cache_tile.i_INPUT(port)`.
- Cache's `o_L2` → `wide_axi.i_INPUT()`. `wide_axi`'s existing map routes TCDM-range
  requests to `tcdm.i_DMA_INPUT()`, so misses land in the SPM.
- DMA (`idma.o_TCDM`) continues to bind directly to `tcdm.i_DMA_INPUT()`, **bypassing
  the cache** (matches the RTL: DMA does not go through the L1).

When `use_insitu_cache=False` (default), the cluster is wired exactly as before — no
performance change on existing runs.

The user property is plumbed through `SnitchArchProperties.declare_target_properties()`
in `pulp/pulp/chips/snitch/snitch.py`.

### Pointing L2 at DDR instead of SPM

The cache's `o_L2` goes into `wide_axi`. Whatever memory-map entry on `wide_axi` covers
the cached region determines where the refills land. To back the cache with DRAM
instead of the cluster SPM, extend the SoC-level `wide_axi` mapping so TCDM-range
addresses route to a `dramsys` / `memory.Memory` component outside the cluster, or
change the cluster's cached region to an address range that the SoC already maps to
DRAM. The cache model itself doesn't care about the backing memory type.

---

## 7. Telemetry

Each component keeps per-instance counters that are accessible via
`vp::Trace::msg(LEVEL_INFO, ...)` prints and will be (Phase 7) exposed as stat signals
mirroring the RTL's end-of-sim `$display` format.

### Controller

```
cnt_rd_hit, cnt_rd_miss, cnt_wr_hit, cnt_wr_miss,
cnt_mshr_merge,                            # read hit on a PEND line
cnt_evict, cnt_wb_dirty,                   # eviction events, dirty writebacks
cnt_stall_miss_fifo,                       # miss FIFO full → DENIED
cnt_stall_evic_fifo,                       # evict FIFO full → DENIED
cnt_stall_mshr_full,                       # MSHR full → DENIED
cnt_refills_issued, cnt_writes_through     # outbound requests
```

### Coalescer

```
cnt_writes_absorbed                        # incoming writes
cnt_flushes_new_tag                        # flush due to line change
cnt_flushes_watchdog                       # flush due to 4-cycle timeout
cnt_flushes_snoop                          # flush due to read-snoop match
cnt_merged_bursts                          # emitted wide bursts
```

Effective coalescing ratio = `cnt_writes_absorbed / cnt_merged_bursts`.

### Interco

Per-output round-robin and busy-until are internal state; no counters exposed yet.

---

## 8. Standalone Testbench

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

This is the recommended harness for Phase-6 microbenchmarks (random reads, streaming
writes, blocked GEMM) when comparing cycle counts against RTL.

---

## 9. Known Limitations & Phase-7 Refinements

From `prompt/insitu_cache_gvsoc_plan.md` §7:

1. **Per-way bank-busy tracking** — currently aggregate per-set. Promote to per-way if
   Phase-6 validation shows bank-conflict workloads drifting >5%.
2. **Forwarding buffer modeling** — meta-bank write absorption is disabled by default.
   Turn on via `enable_meta_fwd_buffer` (future controller field) if LRU-churn
   workloads show the model stalling where RTL doesn't.
3. **Async preread arbitration** — current model is synchronous; RTL's priority
   ordering (refill > request) may differ in corner cases.
4. **Hash polynomial match** — Phase-1 uses a Knuth-style hash; RTL uses a specific
   polynomial. Only matters if set-index aliasing is workload-visible.
5. **PartSplit folded-SRAM modeling** — captured as a flat penalty today. Replace with
   per-way folding if eviction-heavy traces show 3–4% residual error.

### Phase 6 (validation) — not yet done

Curated workloads: `cache-line-rw-smoke`, random reads, streaming writes, blocked
GEMM, vector AXPY. Method: same binary on RTL and GVSoC, diff cycle counts and stat
counters. Target: <5% cycle error, <1% counter error. See plan §6.

---

## 10. Troubleshooting

| Symptom | Likely cause / fix |
|---|---|
| `ModuleNotFoundError: typing_extensions` / `override` | Host Python < 3.10. Use the shim (see §2 Prerequisites). |
| `ModuleNotFoundError: pkg_resources` | `setuptools ≥ 81`. Downgrade: `python3.12 -m pip install --user 'setuptools<81'`. |
| `gen_cache_insitu_... not found at runtime` | The target didn't actually instantiate the tile. Check `use_insitu_cache=True` is set on the target property. Look for `insitu_cache` in `gvsoc --trace=insitu_cache` output during instantiation. |
| Infinite loop / hang on first miss | Refill response path: the downstream memory isn't calling back via `resp_meth`. Check `o_L2` is bound to something that actually completes the request (a `memory.Memory` or a proper router chain). |
| Address decomposition wrong — low hit rate on expected streaming pattern | `rm_base` mismatch. The tile is wired with `rm_base=False` in the spatz cluster so the cache sees absolute addresses; if you override with `rm_base=True`, adjust expectations. |
| `Signature mismatch` at Python bind time | The Python port signatures are declared in each component's `o_XX`/`i_XX` factories. Make sure the bound interface uses `signature='io'` for IO ports and a matching tag elsewhere. |

---

## 11. Where to Dig Next

- **Architecture**: `prompt/insitu_cache_architecture.md` — RTL microarchitecture (SOT).
- **Plan**: `prompt/insitu_cache_gvsoc_plan.md` — modeling philosophy, phases, validation
  plan.
- **Existing cache reference**: `core/models/cache/cache_v3.{cpp,py}` — LFSR-based
  set-associative cache, similar patterns for save/restore and refill queuing.
- **Interleaver reference**: `pulp/pulp/cluster/l1_interleaver_impl.cpp` — the pattern
  the hashed interco follows.
- **Memory model reference**: `core/models/memory/memory_v2.cpp` — bandwidth/latency
  modeling idioms.
- **Spatz target**: `pulp/spatz.py` → `SpatzBoard` → `SnitchCluster` in
  `pulp/pulp/snitch/snitch_cluster/snitch_cluster.py`.
