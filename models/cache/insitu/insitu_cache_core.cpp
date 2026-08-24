// SPDX-FileCopyrightText: 2026 ETH Zurich and University of Bologna
// SPDX-License-Identifier: Apache-2.0
//
// InSitu cache — RTL-faithful STRUCTURAL cache core (structural model, Step 4, first runnable).
//
// A per-cycle ClockEvent FSM transcription of insitu_cache_core.sv's request pipeline, consuming
// the Step-1 decode/encode datapath (insitu_cache_decode.hpp) and the Step-2 pseudo-dual-port bank
// model (insitu_cache_bank_array.hpp). Latency EMERGES from cycles spent in the pipeline / stalls
// (calibration of the per-tick step counts is a later phase), not from latency knobs.
//
// SCOPE (first runnable version — open-loop / async only):
//   - 2-stage pipeline (stage-0 arbitrate {request, refill} → preread_q_; stage-1 decode + FSM +
//     one bank write + output-FIFO drain), single bank port, single-outstanding refill gate.
//   - REQ_PROC: read-hit, write-hit, read-hit-pend (in-situ MSHR append), miss-allocate,
//     victim dirty-writeback; refill install + drain of all queued readers. Bank WR_CONFLICT
//     (Step-2) makes a same-cycle read retry next tick.
//   - Functional data path (serve reads / apply writes / install refills), like the controller.
//   - ASYNC resp (park IoReq → resp() on a later tick). Validated via the open-loop calib replay;
//     the closed-loop cluster keeps InsituCacheController until the synchronous-slave run-to-
//     completion mode is added (see prompt/insitu_cache_structural_plan_2026-06-16.md must-fix #1).
// DEFERRED (TODO, faithful follow-ups): the explicit 7-state stall enum (here FIFO-full →
//   IO_REQ_DENIED backpressure + functional stall), the forwarding-buffer FSM (Step 3), the
//   multi-read-pend linked list, the synchronous-slave inline mode for the cluster.

#include <cstdint>
#include <vector>
#include <deque>
#include <unordered_map>
#include <cstdlib>
#include <cstring>

#include <vp/vp.hpp>
#include <vp/itf/io.hpp>

#include "insitu_cache_decode.hpp"
#include "insitu_cache_bank_array.hpp"
#include "insitu_cache_route.hpp"

using namespace insitu;

class InsituCacheCore : public vp::Component
{
public:
    explicit InsituCacheCore(vp::ComponentConf &conf);
    void reset(bool active) override;
    void stop() override {
        // End-of-sim counter dump (diagnostics; one line per cell).
        fprintf(stderr, "[INSITU-CORE %s] rd_hit=%lu rd_miss=%lu wr_hit=%lu wr_miss=%lu refill=%lu evict=%lu flush=%lu/%lu | lat_sum=%lu b1=%lu clamp=%lu/%lu winfo=%lu wcommit=%lu\n",
                this->get_path().c_str(), (unsigned long)cnt_rd_hit_, (unsigned long)cnt_rd_miss_,
                (unsigned long)cnt_wr_hit_, (unsigned long)cnt_wr_miss_,
                (unsigned long)cnt_refill_, (unsigned long)cnt_evict_,
                (unsigned long)cnt_flush_, (unsigned long)cnt_flush_dirty_,
                (unsigned long)lat_sum_, (unsigned long)lat_b1_, (unsigned long)lat_clamp_,
                (unsigned long)n_clamp_, (unsigned long)lat_winfo_, (unsigned long)lat_wcommit_);
        // Measured per-access served latency on the ASYNC path: accept cycle -> resp cycle in real
        // simulated time. This is the number to calibrate against the RTL reference (warm read-hit
        // 10 cycles isolated / 7 streaming); stamped latency does not reach the requester here.
        if (served_lat_n_)
            fprintf(stderr, "[INSITU-CORE %s] served_lat_avg=%.2f over %lu accesses "
                            "| HIT avg=%.2f min=%ld n=%lu | MISS avg=%.2f min=%ld n=%lu "
                            "(resp_delay=%d miss_extra=%d hit_floor=%d)\n",
                    this->get_path().c_str(), (double)served_lat_sum_ / (double)served_lat_n_,
                    (unsigned long)served_lat_n_,
                    hit_lat_n_ ? (double)hit_lat_sum_ / (double)hit_lat_n_ : 0.0,
                    (long)hit_lat_min_, (unsigned long)hit_lat_n_,
                    miss_lat_n_ ? (double)miss_lat_sum_ / (double)miss_lat_n_ : 0.0,
                    (long)miss_lat_min_, (unsigned long)miss_lat_n_,
                    (int)resp_latency_cycles_, (int)miss_extra_cycles_, (int)hit_latency_floor_);
        vp::Component::stop();
    }

private:
    static vp::IoReqStatus req_handler(vp::Block *__this, vp::IoReq *req);
    static vp::IoReqStatus flush_req_handler(vp::Block *__this, vp::IoReq *req);
    static vp::IoReqStatus config_handler(vp::Block *__this, vp::IoReq *req);   // E3
    static void refill_resp_handler(vp::Block *__this, vp::IoReq *req);
    static void evict_resp_handler(vp::Block *__this, vp::IoReq *req);
    static void tick(vp::Block *__this, vp::ClockEvent *event);

    void schedule_tick(int64_t cycles = 1);
    vp::IoReqStatus run_flush(vp::IoReq *req);         // F1: flush-all (write back dirty, invalidate, gate)
    void stage1_process();                 // decode + FSM + bank write
    vp::IoReqStatus run_request_sync(vp::IoReq *req);  // synchronous-slave: resolve in-call, inc_latency, OK
    bool process_request(vp::IoReq *req);  // REQ_PROC body; returns true=done, false=stalled (retry)
    bool maybe_install_refill();           // install a ready refill (priority bank op); true if installed
    void install_refill();                 // refill block
    void drain_outputs();                  // one beat per output FIFO
    void stage0_arbitrate();               // pick next preread task
    int64_t hit_ready_cyc(vp::IoReq *req);  // eligibility cycle for a hit (floor or legacy addend)
    bool any_work() const;

    // --- functional data (RETAINED from controller.cpp:118-130) ---
    inline void exchange_line_data(vp::IoReq *req, uint32_t set, int way, bool line_to_req) {
        if (req->get_data() == nullptr) return;
        uint32_t off = (uint32_t)(req->get_addr() & (cache_line_bytes_ - 1));
        uint32_t n = (uint32_t)req->get_size();
        if (off + n > cache_line_bytes_) {
            static int _xl_warns = 0;
            if (_xl_warns < 12) {
                fprintf(stderr, "[XLINE] cross-line access addr=0x%lx size=%u off=%u (line=%u) %s -> TRUNCATED\n",
                        (unsigned long)req->get_addr(), n, off, cache_line_bytes_, line_to_req ? "rd" : "wr");
                _xl_warns++;
            }
            n = cache_line_bytes_ - off;
        }
        uint8_t *line = &data_[((size_t)set * num_ways_ + (uint32_t)way) * cache_line_bytes_ + off];
        if (line_to_req) memcpy(req->get_data(), line, n);
        else             memcpy(line, req->get_data(), n);
        dbg_ev(line_to_req ? "serve-rd" : "serve-wr", l2_addr(req->get_addr()), n,
               (const uint8_t *)req->get_data());
        shadow_check(req, line_to_req, n);
    }
    void functional_write_mem(vp::IoReq *user_req) {
        if (!functional_writethrough_ || !evict_itf_.is_bound() || user_req->get_data() == nullptr) return;
        funcwr_req_.init();
        funcwr_req_.set_addr(l2_addr(user_req->get_addr()));
        funcwr_req_.set_size(user_req->get_size());
        funcwr_req_.set_is_write(true);
        funcwr_req_.set_data(user_req->get_data());
        (void)evict_itf_.req(&funcwr_req_);
    }

    uint64_t addr_line(uint64_t a) const { return a & ~((uint64_t)cache_line_bytes_ - 1); }
    // Inverse of the tile xbar's MSB rotation (route.hpp::rotate_addr — E1). The bank's tags/sets live
    // in ROTATED address space, so every L2-side egress (refill, dirty writeback, functional WT, and
    // the bypass fallback) must unrotate back to the global address before hitting the NoC.
    // INSITU_SHADOW=1: a per-bank shadow of the last value written to each 4-byte word, checked on
    // every read serve. Any word this bank served differently from what was last written THROUGH it is
    // a lost or stale store — which is exactly the failure shape of the async vector-stress mismatch
    // (the test rewrites the same pattern every pass, so reordering is idempotent and only a dropped
    // store can be observed). Words never written through this bank (loader-initialised data) are not
    // in the map and are skipped, so the check cannot fire spuriously.
    static bool shadow_on() {
        static const bool v = [](){ const char *e = getenv("INSITU_SHADOW"); return e && e[0] != '0'; }();
        return v;
    }
    std::unordered_map<uint64_t, uint32_t> shadow_;
    void shadow_check(vp::IoReq *req, bool line_to_req, uint32_t nbytes)
    {
        if (!shadow_on() || req->get_data() == nullptr) return;
        const uint64_t base = l2_addr(req->get_addr());
        const uint8_t *d = (const uint8_t *)req->get_data();
        for (uint32_t off = 0; off + 4 <= nbytes; off += 4) {
            uint32_t v; memcpy(&v, d + off, 4);
            const uint64_t a = base + off;
            if (line_to_req) {
                auto it = shadow_.find(a);
                if (it != shadow_.end() && it->second != v) {
                    static int budget = 30;
                    if (budget > 0) {
                        budget--;
                        fprintf(stderr, "[SHADOW-MISMATCH %s] cyc=%ld addr=0x%lx served=0x%x last_written=0x%x\n",
                                this->get_path().c_str(), (long)clock.get_cycles(),
                                (unsigned long)a, v, it->second);
                    }
                }
            } else {
                shadow_[a] = v;
            }
        }
    }

    // TEMP async bring-up instrumentation: INSITU_WATCH=0x<addr> traces one cache line end-to-end.
    static uint64_t dbg_watch() {
        static uint64_t v = [](){ const char *e = getenv("INSITU_WATCH");
                                  return e ? strtoull(e, nullptr, 0) : 0ULL; }();
        return v;
    }
    void dbg_ev(const char *tag, uint64_t l2a, uint32_t size, const uint8_t *d) {
        const uint64_t w = dbg_watch();
        if (!w || (l2a >> 6) != (w >> 6)) return;
        uint32_t v = 0; if (d) memcpy(&v, d, size < 4 ? size : 4);
        fprintf(stderr, "[WATCH %-8s] %s cyc=%ld l2a=0x%lx rot=%u sz=%u val=0x%x bank=%s\n", tag,
                inline_sync_ ? "sync " : "async", (long)clock.get_cycles(),
                (unsigned long)l2a, rotate_bits_, size, v, this->get_path().c_str());
    }

    // rotate_bits_=0 (rotation disabled / N=0 bank) → identity.
    uint64_t l2_addr(uint64_t a) const {
        return rotate_bits_ ? rotate_geom_.unrotate_addr(a, rotate_bits_) : a;
    }
    WayMeta *set_ways(uint32_t set) { return &meta_[(size_t)set * num_ways_]; }

    // --- config / geometry ---
    uint32_t cache_line_bytes_, num_ways_, num_sets_;
    CacheGeom geom_;
    insitu::RouteGeom rotate_geom_{};   // only dyn_offset/addr_width are used (l2_addr unrotation)
    uint32_t rotate_bits_ = 0;
    // E3 runtime partition (set via the config slave port): the identity of THIS bank, needed to
    // recompute rotate_bits_ from the shared bits_to_rotate table and to make flush class-selective.
    int32_t  bank_index_ = 0, num_cache_ = 1, num_tiles_ = 1, num_private_cache_ = 0;
    bool functional_writethrough_;
    int32_t miss_penalty_cycles_, refill_bank_write_cycles_;
    uint32_t retr_fifo_depth_, miss_fifo_depth_, evic_fifo_depth_;
    // synchronous-slave (closed-loop) mode
    bool    inline_sync_ = false;
    int32_t hit_latency_cycles_ = 0;
    int32_t write_hit_latency_cycles_ = -1;   // D2: store-ack latency (winfo FIFO); -1 → hit value
    int32_t write_commit_cycles_ = 0;
    int64_t write_commit_busy_until_ = 0;
    // D2 winfo-FIFO acceptance window (insitu_cache_tcdm_wrapper.sv WRespFifoDepth=4): a store's ack
    // drains ~2 cy after acceptance; a 5th store inside the window stalls until the oldest ack drains.
    std::deque<int64_t> wresp_win_;           // ack-drain cycles of accepted-but-undrained stores
    // B1 per-cell request serialization (insitu_cache_core.sv: one 1-deep req_buf + the
    // i_pre_reader_arbiter → ≤1 cache access/cycle/cell). Shared by all input ports (VLSU-aggregate
    // and scalar-bypass alike — the RTL 2:1 bypass xbar serializes them into the one pipeline).
    int64_t cell_busy_until_ = 0;
    // F1 flush: while set, ALL upstream traffic to this cell is gated (the RTL l1d_busy_i).
    int64_t flush_busy_until_ = 0;
    int32_t flush_base_cycles_ = 277;   // the RTL walk (CHECK_PEND drain 21 + 256 sets)
    int32_t flush_evict_cycles_ = 20;   // per dirty line: the serialized downstream eviction
    // Refill occupancy (calibration step 2). The RTL controller keeps ONE outstanding line-refill and
    // holds it until the miss is installed+served — the next refill-read only issues at the previous
    // line's ready cycle, so cold-miss throughput is ~1/(ML + pipeline), not ~1/ML. The sync path resolves
    // each miss inline, so without an explicit gate the +pipeline (bank_write + miss_penalty) overlaps the
    // next refill's memory latency and throughput is over-predicted (~53/miss vs RTL 67/miss at ML=50).
    //
    // Mechanism: gate the refill ISSUE on sync_refill_busy_until_ (the previous line's ready cycle), and
    // stamp the request with (issue + ML_nominal + bank_write + miss_penalty) - now. ML_nominal is the
    // backing store's UNGATED refill latency (min full_latency seen; on the first refill of a run the
    // store is ungated). We deliberately do NOT use the store's per-call gated full_latency for timing:
    // the store serializes refills at its own ML (which would dominate and re-create the overlap), while
    // the RTL's serialization is the CONTROLLER's occupancy, which is what sync_refill_busy_until_ models.
    // The cachepool backing store (plain memory.cpp) does not serialize refills at all, so this gate is
    // REQUIRED there for RTL-faithful miss throughput; on the calib TB it composes with the store's gate.
    int64_t sync_refill_busy_until_ = 0;
    int64_t ml_nominal_ = -1;
    int32_t install_tail_cycles_ = 0;   // refill-pipeline tail after the response (occupancy, not latency)

    // --- state ---
    std::vector<WayMeta> meta_;            // [num_sets*num_ways]
    std::vector<uint8_t> data_;            // [num_sets*num_ways*line_bytes]
    BankArray bank_;

    // Input acceptance queue (stage-0 admission). The RTL upstream streams requests in (the
    // requester holds valid until accepted, up to NumSpatzOutstandingLoads=32 in flight); model
    // that as a bounded accept queue rather than a 1-deep buffer, so a request is accepted at its
    // arrival cycle and the emergent latency reflects the cache pipeline + MSHR queueing, not an
    // artificial 1-outstanding DENY-retry backpressure. Genuine backpressure (queue full) still
    // returns IO_REQ_DENIED.
    std::deque<vp::IoReq*> in_q_;
    uint32_t in_q_cap_ = 32;   // ~NumSpatzOutstandingLoads

    // CALIBRATION (async path). Latency stamped with inc_latency() is DISCARDED by the requester on
    // this path (iss lsu.cpp data_response zeroes pending_latency), so per-access latency here is
    // whatever real simulated time the pipeline spends — about 2-3 cycles for a hit, against the
    // RTL-derived reference of 10 cycles isolated / 7 streaming. resp_latency_cycles_ closes that gap
    // STRUCTURALLY: a completed access becomes eligible to respond only this many cycles later, so the
    // delay is real time the requester actually observes. 0 keeps the raw pipeline behaviour.
    int32_t  resp_latency_cycles_ = 0;
    // HIT-side FLOOR (preferred over the addend above; > 0 enables it).
    //
    // resp_latency_cycles_ is an ADDEND at the pipeline tail: a hit becomes eligible
    // `resp_latency_cycles_` cycles after it completes. Isolated that is right — the pipeline costs
    // ~2 cycles, +8 lands on the RTL's 10. Under load it is wrong, because the request has ALREADY
    // spent real time queueing in in_q_, in stage-0/stage-1, on bank conflicts and on MSHR waits, and
    // the 8 is added on top of all of it. The error is per-access, so it compounds: measured against
    // the RTL's own 64-core RLC baseline (reports/rlc_64core_baseline_2026-08-24) the model ran
    // +60.8 %, and sweeping this constant moved the kernel by ~8,100 cycles PER CYCLE of the constant
    // — at 0 the fastest cores matched RTL to 0.6 %. See prompt/rtl_multigroup_comparison_2026-08-25.md.
    //
    // The floor expresses what the RTL number actually means: a hit takes `hit_latency_floor_` cycles
    // END TO END from arrival, not "pipeline plus a constant". Eligibility becomes
    // max(now, accept_cyc + floor), so an isolated hit still lands exactly on the floor while a loaded
    // hit that already spent more than that adds nothing.
    //
    // Applies to HITS only. The miss path keeps the addend: its dominant term is the refill's real
    // memory latency, which is genuinely serial rather than double-counted, and the addend is
    // calibrated exactly on the RTL's cold-miss reference (MemLatency + 17 = 67 at ML=50).
    int32_t  hit_latency_floor_ = 0;
    // Miss-side term. A hit and a miss cannot share one constant: the RTL reference is a warm read-hit
    // of 10 cycles isolated and a cold read-miss of MemLatency + 17, while this path reaches only about
    // MemLatency + 11 (memory latency + ~3 pipeline + resp_latency_cycles_). This adds the remainder
    // for accesses that actually waited on a refill, spent structurally like the rest.
    int32_t  miss_extra_cycles_ = 0;
    // served-latency accounting, reported at stop(): accept cycle -> resp cycle, per access.
    std::unordered_map<vp::IoReq *, int64_t> accept_cyc_;
    uint64_t served_lat_sum_ = 0, served_lat_n_ = 0;
    // split hit/miss, with the MINIMUM as well as the mean: the RTL figures are ISOLATED costs, which
    // the mean overstates once several accesses queue behind one refill.
    uint64_t hit_lat_sum_ = 0, hit_lat_n_ = 0, miss_lat_sum_ = 0, miss_lat_n_ = 0;
    int64_t  hit_lat_min_ = -1, miss_lat_min_ = -1;
    std::unordered_map<vp::IoReq *, bool> was_miss_;

    // Admission stall queue: requests that arrived while in_q_ was full. They are parked (PENDING, never
    // dropped) and re-admitted as space frees — required for async-capable masters (the updated Spatz VLSU
    // treats PENDING/DENIED as async and waits for a resp(); a DENIED + dropped request would hang it).
    std::deque<vp::IoReq*> admission_stall_q_;

    // stage-0→stage-1 pipeline register
    struct PrereadTask { bool valid=false; bool is_refill=false; vp::IoReq *req=nullptr;
                         uint64_t addr=0; bool is_write=false; } preread_q_;

    // in-situ MSHR: per pending line (set,way) the queued readers (+ a deferred write merge req)
    std::vector<std::deque<vp::IoReq*>> mshr_;   // [num_sets*num_ways]

    // output FIFOs (structural, capacity-bounded)
    std::deque<vp::IoReq*> resp_fifo_;
    std::deque<int64_t>    resp_done_cyc_;   // READY cycle of each resp_fifo_ entry (structural delay)            // completed reads/writes to resp() (retr+resp merged)
    std::deque<uint64_t>   miss_fifo_;            // line addrs needing a refill
    // Async writeback queue. The dirty line's BYTES are snapshotted at eviction time: queuing only the
    // address and handing evict_data_buf_ to the drain sent whatever that shared buffer last held — and
    // on this path nothing ever fills it (only the sync-miss and flush paths do), so every async
    // writeback wrote 64 bytes of ZEROS over L2. A later refill of the same line then read those zeros
    // back, which is how fdotp's dotp_l.M turned from 0x2000 into 0 mid-run: the kernel then computed
    // elem_per_core = 0, did no work, and still "passed" by verifying zeros against zeros.
    struct EvictEnt { uint64_t addr; std::vector<uint8_t> data; };
    std::deque<EvictEnt>   evic_fifo_;            // dirty lines to write back (addr + bytes)
    std::vector<uint8_t>   evict_wb_buf_;         // bytes of the writeback currently in flight
    bool                   evict_wb_pending_ = false;   // async L2: one writeback in flight at a time
    uint32_t retr_level_ = 0;

    // single-outstanding refill
    bool refill_pending_ = false;          // a refill req is outstanding on the async path (awaiting resp())
    bool refill_spill_valid_ = false;      // refilled line is ready to install (stage-1 next tick)
    int64_t refill_ready_cycle_ = -1;      // sync responder: cycle the refill's data is actually ready
                                           //   (= issue cycle + the latency the responder stamped via
                                           //   inc_latency). Until then the line is in flight; the next
                                           //   refill is gated on it → serialized miss throughput. -1 = none.
    uint64_t pending_refill_addr_ = 0;

    // ports
    vp::IoSlave  input_itf_;                    // port "input" (lane 0 / the single-port default)
    std::vector<vp::IoSlave *> extra_inputs_;   // "input_1".."input_{N-1}" — the multi-lane core port
                                                 // (RTL controller has a 5-wide core port); all feed in_q_
    vp::IoMaster refill_itf_;
    vp::IoMaster evict_itf_;
    vp::IoSlave  flush_itf_;
    vp::IoSlave  config_itf_;                   // E3: runtime partition config (csr-id in addr)
    vp::IoReq    refill_req_, evict_req_, funcwr_req_;
    std::vector<uint8_t> refill_data_buf_, evict_data_buf_;

    vp::ClockEvent *tick_event_ = nullptr;
    vp::Trace trace_;
    // telemetry
    uint64_t cnt_rd_hit_=0, cnt_wr_hit_=0, cnt_rd_miss_=0, cnt_wr_miss_=0, cnt_mshr_merge_=0,
             cnt_refill_=0, cnt_evict_=0, cnt_bank_conflict_=0;
    // latency-budget diagnostics: requests, total stamped latency, and the per-mechanism waits
    uint64_t lat_sum_=0, lat_b1_=0, lat_clamp_=0, lat_winfo_=0, lat_wcommit_=0, n_clamp_=0;
    uint64_t cnt_flush_=0, cnt_flush_dirty_=0;
};

InsituCacheCore::InsituCacheCore(vp::ComponentConf &conf) : vp::Component(conf)
{
    auto *cfg = this->get_js_config();
    cache_line_bytes_ = cfg->get_child_int("cache_line_bytes");
    num_ways_         = cfg->get_child_int("num_ways");
    num_sets_         = cfg->get_child_int("num_sets");
    functional_writethrough_ = cfg->get_child_bool("functional_writethrough");
    miss_penalty_cycles_     = cfg->get_child_int("miss_penalty_cycles");
    refill_bank_write_cycles_= cfg->get_child_int("refill_bank_write_cycles");
    retr_fifo_depth_  = cfg->get_child_int("retr_fifo_depth");
    miss_fifo_depth_  = cfg->get_child_int("miss_fifo_depth");
    evic_fifo_depth_  = cfg->get_child_int("evic_fifo_depth");
    uint32_t bank_factor = cfg->get_child_int("bank_factor");
    if (bank_factor < 1) bank_factor = 1;

    // Synchronous-slave mode (closed-loop, e.g. the Spatz cluster whose VLSU rejects async resp): resolve
    // each request in-call and return IO_REQ_OK with the latency folded in, mirroring the calibrated
    // controller's inline_sync_miss. Default off → the async per-cycle path (open-loop calib) is unchanged.
    inline_sync_       = cfg->get_child_bool("inline_sync_miss");
    hit_latency_cycles_= cfg->get_child_int("hit_latency_cycles");
    write_commit_cycles_= cfg->get_child_int("write_commit_cycles");
    // Structural sync-slave overrides (the analytic path's decomposition differs from the async
    // controller's — see insitu_cache_config.py). -1 = fall back to the shared knobs.
    int32_t shl = cfg->get_child_int("structural_hit_latency_cycles");
    if (shl >= 0) hit_latency_cycles_ = shl;
    int32_t smp = cfg->get_child_int("structural_miss_penalty_cycles");
    if (smp >= 0) miss_penalty_cycles_ = smp;
    install_tail_cycles_ = cfg->get_child_int("structural_install_tail_cycles");
    // F1 flush knobs (insitu_cache_tcdm_wrapper.sv 7-state FSM): the base walk = CHECK_PEND drain
    // (21) + the 256-set sweep; each dirty line adds a serialized downstream eviction.
    int32_t fbc = cfg->get_child_int("flush_base_cycles");
    if (fbc > 0) flush_base_cycles_ = fbc;
    int32_t fec = cfg->get_child_int("flush_evict_cycles");
    if (fec > 0) flush_evict_cycles_ = fec;
    // D2 store-ack latency (the winfo-FIFO ack; RTL ~8 cy regardless of hit/miss). -1 → hit value.
    write_hit_latency_cycles_ = cfg->get_child_int("structural_write_hit_latency_cycles");

    // E1 MSB-rotation inverse: the tile xbar rotates the N routing bits above dyn_offset to the MSB
    // before a request reaches this bank (route.hpp::rotate_addr), so this bank's tags/sets live in
    // ROTATED space. Every L2-side egress unrotates via l2_addr(). rotate_bits=0 → identity (the
    // pre-E1 behaviour; also the single-bank / single-tile-N=0 case).
    rotate_bits_               = cfg->get_child_int("rotate_bits");
    resp_latency_cycles_       = cfg->get("resp_latency_cycles")
        ? cfg->get_child_int("resp_latency_cycles") : 0;
    // INSITU_RESP_LAT overrides it, so the value can be swept without rebuilding during calibration.
    if (const char *e = getenv("INSITU_RESP_LAT")) resp_latency_cycles_ = atoi(e);
    hit_latency_floor_ = cfg->get("hit_latency_floor")
        ? cfg->get_child_int("hit_latency_floor") : 0;
    // INSITU_HIT_FLOOR overrides it, so the floor can be swept without rebuilding during calibration.
    if (const char *e = getenv("INSITU_HIT_FLOOR")) hit_latency_floor_ = atoi(e);
    miss_extra_cycles_ = cfg->get("miss_extra_cycles")
        ? cfg->get_child_int("miss_extra_cycles") : 0;
    if (const char *e = getenv("INSITU_MISS_EXTRA")) miss_extra_cycles_ = atoi(e);
    rotate_geom_.dyn_offset    = cfg->get_child_int("rotate_dyn_offset");
    rotate_geom_.addr_width    = cfg->get_child_int("rotate_addr_width");
    // E3: this bank's identity (default partition from elaboration; the config slave can repartition
    // at runtime). num_private_cache default matches the xbar's (all-private single-tile, all-shared group).
    bank_index_       = cfg->get_child_int("bank_index");
    num_cache_        = cfg->get_child_int("num_cache");
    num_tiles_        = cfg->get_child_int("num_tiles");
    num_private_cache_= cfg->get_child_int("num_private_cache");

    geom_.init(cache_line_bytes_, num_ways_, num_sets_, cfg->get_child_bool("use_hash_way_select"), false);
    bank_.init(num_ways_, bank_factor);

    meta_.assign((size_t)num_sets_ * num_ways_, WayMeta{});
    data_.assign((size_t)num_sets_ * num_ways_ * cache_line_bytes_, 0);
    mshr_.assign((size_t)num_sets_ * num_ways_, {});
    // seed LRU credits 0..ways-1 per set so victim selection has a defined order
    for (uint32_t s = 0; s < num_sets_; s++)
        for (uint32_t w = 0; w < num_ways_; w++) meta_[(size_t)s*num_ways_+w].lru = w;
    refill_data_buf_.assign(cache_line_bytes_, 0);
    evict_data_buf_.assign(cache_line_bytes_, 0);

    input_itf_.set_req_meth(&InsituCacheCore::req_handler);
    new_slave_port("input", &input_itf_);
    // Multi-lane core port (RTL cachepool_cache_ctrl has a 5-wide core port). Extra lanes all share the
    // same admission handler → the single in_q_; the per-cycle pipeline + bank then serialize them onto
    // the one bank port (the coalescer/bypass that merges them faithfully is wired in front by the tile).
    {
        uint32_t num_input_ports = (uint32_t)cfg->get_child_int("num_input_ports");
        if (num_input_ports < 1) num_input_ports = 1;
        for (uint32_t i = 1; i < num_input_ports; i++) {
            auto *p = new vp::IoSlave();
            p->set_req_meth(&InsituCacheCore::req_handler);
            new_slave_port("input_" + std::to_string(i), p);
            extra_inputs_.push_back(p);
        }
    }
    flush_itf_.set_req_meth(&InsituCacheCore::flush_req_handler);   // F1: real flush (was accept-OK stub)
    new_slave_port("flush", &flush_itf_);
    // E3: runtime partition config (the peripheral broadcasts on the partition-commit writes).
    config_itf_.set_req_meth(&InsituCacheCore::config_handler);
    new_slave_port("config", &config_itf_);
    refill_itf_.set_resp_meth(&InsituCacheCore::refill_resp_handler);
    new_master_port("refill", &refill_itf_);
    evict_itf_.set_resp_meth(&InsituCacheCore::evict_resp_handler);
    new_master_port("evict", &evict_itf_);

    tick_event_ = event_new(&InsituCacheCore::tick);
    traces.new_trace("trace", &trace_, vp::DEBUG);
    trace_.msg(vp::Trace::LEVEL_INFO, "InsituCacheCore sets=%u ways=%u line=%u bankfac=%u (STRUCTURAL)\n",
               num_sets_, num_ways_, cache_line_bytes_, bank_factor);
}

void InsituCacheCore::reset(bool active)
{
    if (!active) return;
    for (auto &m : meta_) { m.status = INVALID; m.dirty = false; m.tag = 0; }
    for (uint32_t s = 0; s < num_sets_; s++)
        for (uint32_t w = 0; w < num_ways_; w++) meta_[(size_t)s*num_ways_+w].lru = w;
    for (auto &q : mshr_) q.clear();
    resp_fifo_.clear(); resp_done_cyc_.clear(); was_miss_.clear(); miss_fifo_.clear(); evic_fifo_.clear(); evict_wb_pending_ = false;
    in_q_.clear(); preread_q_ = PrereadTask{};
    refill_pending_ = refill_spill_valid_ = false; refill_ready_cycle_ = -1; retr_level_ = 0;
}

void InsituCacheCore::schedule_tick(int64_t cycles)
{
    if (cycles < 1) cycles = 1;
    if (!tick_event_->is_enqueued()) event_enqueue(tick_event_, cycles);
}

bool InsituCacheCore::any_work() const
{
    return preread_q_.valid || !in_q_.empty() || refill_spill_valid_ || refill_ready_cycle_ >= 0 ||
           !resp_fifo_.empty() || !miss_fifo_.empty() || !evic_fifo_.empty();
}

// ---------- stage-0 admission ----------
vp::IoReqStatus InsituCacheCore::req_handler(vp::Block *__this, vp::IoReq *req)
{
    InsituCacheCore *_this = static_cast<InsituCacheCore *>(__this);
    // Synchronous-slave (closed-loop): resolve in-call, return IO_REQ_OK — never PENDING/DENIED, never
    // resp() (the calibrated, deployed path). The async path below returns PENDING and resp()s later.
    if (_this->inline_sync_) return _this->run_request_sync(req);
    //
    // NO req->save() here, deliberately. save() arg_push'es 4 slots at `current_arg`, and for a request
    // coming from the scalar LSU `current_arg` is 0 — but the LSU keeps its request id in ABSOLUTE slot 0
    // (`req_id = *((int *)req->arg_get(0))`, iss/src/lsu.cpp), which arg_get() reads without any
    // current_arg offset. save() therefore overwrites the id with the address, and restore() only pops
    // the stack depth back — it never repairs the slot's contents. The LSU then dispatches
    // stall_callback[req_id] on the wrong outstanding access: register writebacks get swapped between
    // two in-flight loads (seen as `exp 0x3 got 0x2` + the mirror image in load-store_M16), and on a
    // VLSU port the aliased index segfaulted in AraVlsu::data_response.
    //
    // We do not need it anyway: the core never mutates addr/size/data/is_write on a parked request (it
    // keeps its own copies in preread_q_/mshr_), so there is nothing to save and restore.
    _this->accept_cyc_[req] = _this->clock.get_cycles();
    if (_this->in_q_.size() >= _this->in_q_cap_) {
        // Accept queue full: park the request (PENDING) and re-admit it as space frees (stage0_arbitrate).
        // Never DENY + drop — an async-capable master would wait forever for a resp() that never comes.
        _this->admission_stall_q_.push_back(req);
        _this->schedule_tick();
        return vp::IO_REQ_PENDING;
    }
    _this->in_q_.push_back(req);
    _this->schedule_tick();
    return vp::IO_REQ_PENDING;
}

// ---------- synchronous-slave (closed-loop) ----------
// F1 flush-all (the RTL 7-state FSM, analytic form): write back every dirty line (fire-and-forget
// like the eviction path — REQUIRED for data persistence across the flush), invalidate everything
// (PEND lines too — their ready_cycle is cleared so the lazy sweep can't resurrect them), then gate
// all upstream traffic for the walk's duration (l1d_busy_i). The response carries the duration so
// the peripheral's FLUSH_STATUS spins for the right length.
vp::IoReqStatus InsituCacheCore::config_handler(vp::Block *__this, vp::IoReq *req)
{
    InsituCacheCore *_this = static_cast<InsituCacheCore *>(__this);
    uint32_t value = 0;
    if (req->get_data() != nullptr) memcpy(&value, req->get_data(), req->get_size() < 4 ? req->get_size() : 4);
    const uint32_t csr = (uint32_t)req->get_addr();
    switch (csr) {
        case 0:  // L1D_PRIVATE: repartition — recompute this bank's rotation width from the shared table.
            _this->num_private_cache_ = (int32_t)value;
            _this->rotate_bits_ = insitu::bits_to_rotate(
                (uint32_t)_this->bank_index_, (uint32_t)_this->num_private_cache_, (uint32_t)_this->num_cache_,
                insitu::RouteGeom::log2_up((uint32_t)_this->num_cache_),
                insitu::RouteGeom::log2_up((uint32_t)_this->num_tiles_));
            break;
        case 2:  // XBAR_OFFSET: the BankSel LSB also shifts the unrotation geometry.
            _this->rotate_geom_.dyn_offset = value;
            break;
        default: // csr 1 (L1D_ADDR): routing-only, the core doesn't route — ignore.
            break;
    }
    return vp::IO_REQ_OK;
}

vp::IoReqStatus InsituCacheCore::flush_req_handler(vp::Block *__this, vp::IoReq *req)
{
    InsituCacheCore *_this = static_cast<InsituCacheCore *>(__this);
    return _this->run_flush(req);
}

vp::IoReqStatus InsituCacheCore::run_flush(vp::IoReq *req)
{
    const int64_t now = clock.get_cycles();
    // E3: the flush insn code rides in the req's address (the peripheral smuggles cp_l1d[CFG_L1D_INSN]).
    // Participation is partition-class-selective (insitu_cache_tcdm_wrapper.sv: insn 0 = PRIVATE banks
    // (bank < num_private), 1 = SHARED (bank >= num_private), 2 = ALL, 3 = invalidate-all NO writeback).
    const uint32_t insn = (uint32_t)req->get_addr();
    const bool no_wb   = (insn == 3);
    const bool private_ = (insn == 0), shared_ = (insn == 1);
    const bool participate = no_wb || (insn == 2) ||
        (private_ && bank_index_ <  num_private_cache_) ||
        (shared_  && bank_index_ >= num_private_cache_);
    if (!participate) { req->inc_latency(0); return vp::IO_REQ_OK; }   // 0 latency; status reflects slowest participant

    uint64_t n_dirty = 0;
    for (uint32_t s = 0; s < num_sets_; s++) {
        WayMeta *ways = set_ways(s);
        for (uint32_t w = 0; w < num_ways_; w++) {
            if (ways[w].status == INVALID) continue;
            if (ways[w].dirty) {
                n_dirty++;
                if (evict_itf_.is_bound() && !no_wb) {   // insn 3 = invalidate-all: NO writeback
                    const uint64_t old_line = ((uint64_t)ways[w].tag << (geom_.off_bits + geom_.depth_bits)) |
                                              ((uint64_t)s << geom_.off_bits);
                    // Queue it with its OWN data snapshot rather than issuing inline. Issuing inline
                    // reused one request object and one buffer for every dirty line, which is only safe
                    // while the downstream answers IO_REQ_OK inside the call. As soon as anything that
                    // can answer PENDING sits in the path — the P3 refill mux — all of a flush's
                    // writebacks shared one in-flight object and carried the last line's bytes, and
                    // load-store (the flush-heavy kernel) reported data mismatches. No depth check:
                    // a flush walk must be able to queue every dirty line it finds.
                    EvictEnt ent;
                    ent.addr = old_line;
                    const uint8_t *src = &data_[((size_t)s * num_ways_ + w) * cache_line_bytes_];
                    ent.data.assign(src, src + cache_line_bytes_);
                    evic_fifo_.push_back(std::move(ent));
                }
            }
            ways[w].status = INVALID;
            ways[w].dirty = false;
            ways[w].ready_cycle = 0;
        }
    }
    const int64_t dur = (int64_t)flush_base_cycles_ + (int64_t)n_dirty * flush_evict_cycles_;
    flush_busy_until_ = now + dur;
    if (!evic_fifo_.empty()) schedule_tick();   // drain the queued writebacks
    cnt_flush_++;
    cnt_flush_dirty_ += n_dirty;
    req->inc_latency(dur);
    return vp::IO_REQ_OK;
}

// Analytic one-shot, mirroring InsituCacheController::inline_sync_miss (controller.cpp:438-598): decode,
// serve/install, fold the latency into req->inc_latency(), and return IO_REQ_OK in the same call — NO
// save(), NO resp(), NO tick/event, NO in_q_/preread_q_/mshr_/FIFOs/refill_*_ async state. At most one
// request is in flight (the cluster LSU is single-outstanding), so the refill needs no FIFO.
vp::IoReqStatus InsituCacheCore::run_request_sync(vp::IoReq *req)
{
    const int64_t now = clock.get_cycles();
    const uint64_t addr = req->get_addr();
    const bool is_write = req->get_is_write();
    const uint32_t set = geom_.set_index(addr);
    WayMeta *ways = set_ways(set);

    // F1: all upstream traffic is gated while a flush walk is in progress (l1d_busy_i).
    if (now < flush_busy_until_) req->inc_latency(flush_busy_until_ - now);

    // B1 per-cell request serialization: every lookup — read or write, hit or miss — occupies the
    // cell's single request pipeline for one cycle (the 1-deep req_buf + pre-reader arbiter).
    // Contention folds into the response latency. D1 clamp waits below do NOT hold the cell (a
    // PEND-stalled request re-arbitrates after the wait — RTL: it waits in the xbar input spill).
    const int64_t accept = (cell_busy_until_ > now) ? cell_busy_until_ : now;
    if (accept > now) { req->inc_latency(accept - now); lat_b1_ += (uint64_t)(accept - now); }
    cell_busy_until_ = accept + 1;

    // D1 lazy install: a PEND line whose refill has logically landed (ready_cycle <= now) becomes
    // VALID. Its data was memcpy'd into the line at miss-allocate time; a WRITE_PEND keeps its dirty
    // flag. Runs before decode so the victim scan never sees an install-overdue line as free.
    for (uint32_t w = 0; w < num_ways_; w++) {
        if ((ways[w].status == READ_PEND || ways[w].status == WRITE_PEND) && ways[w].ready_cycle <= now) {
            CacheStatus b = ways[w].status;
            lru_update(geom_, ways, w, b, VALID);
            ways[w].status = VALID;
        }
    }

    Decode d = decode_request(geom_, ways, addr, is_write);

    // Write-commit serialization → ADDED latency on an OK (never DENY; the LSU cannot retry).
    if (is_write && write_commit_cycles_ > 1) {
        const int64_t accept = (write_commit_busy_until_ > now) ? write_commit_busy_until_ : now;
        req->inc_latency(accept - now);
        lat_wcommit_ += (uint64_t)(accept - now);
        write_commit_busy_until_ = accept + write_commit_cycles_;
    }

    // D1 PEND-line ready-cycle clamp (insitu_cache_core.sv: same-type followers merge as MSHR
    // subarrays and WAIT; opposite-type stall in WR_CONFLICT_STALL; all-pend sets stall). The wait
    // happens in-call. A clamped follower then serves from its PEND line WITHOUT installing it —
    // installing at the clamp's virtual cycle would leak the future VALID state to requests still at
    // an earlier real cycle (observed: follower #1's clamp installed the line at real t, so followers
    // #2/#3 took 12-cycle early hits instead of clamping). The line installs only in real time (the
    // sweep above) — or below when a genuine miss needs a way (the misser logically executes at the
    // clamped cycle, so the install is on its own timeline).
    int serve_pend_way = -1;
    for (int guard = 0; guard <= (int)num_ways_; guard++) {
        if (d.is_hit) break;
        int64_t ready = -1;
        if (d.is_hit_pend || d.is_hit_conflit) {
            ready = ways[d.way].ready_cycle;
        } else if (d.is_all_pend) {
            for (uint32_t w = 0; w < num_ways_; w++)
                if ((ways[w].status == READ_PEND || ways[w].status == WRITE_PEND) &&
                    (ready < 0 || ways[w].ready_cycle < ready)) ready = ways[w].ready_cycle;
        } else if (ways[d.way].status == READ_PEND || ways[d.way].status == WRITE_PEND) {
            ready = ways[d.way].ready_cycle;   // victim scan picked a not-yet-ready PEND way — wait it out
        } else {
            break;   // genuine miss with a free/valid victim — proceed to allocate
        }
        if (ready < 0) break;                  // paranoia: no PEND line found — treat as miss
        const int64_t virt0 = now + req->get_full_latency();
        if (ready > virt0) { req->inc_latency(ready - virt0); lat_clamp_ += (uint64_t)(ready - virt0); n_clamp_++; }
        if (d.is_hit_pend || d.is_hit_conflit) {
            serve_pend_way = (int)d.way;       // serve from the PEND line; NO install (see above)
            break;
        }
        // all_pend / PEND-victim: the misser needs a way — install the lines its wait reached, re-decode.
        const int64_t vnow = now + req->get_full_latency();
        for (uint32_t w = 0; w < num_ways_; w++) {
            if ((ways[w].status == READ_PEND || ways[w].status == WRITE_PEND) && ways[w].ready_cycle <= vnow) {
                CacheStatus b = ways[w].status;
                lru_update(geom_, ways, w, b, VALID);
                ways[w].status = VALID;
            }
        }
        d = decode_request(geom_, ways, addr, is_write);
    }

    // D2 winfo-FIFO acceptance window (insitu_cache_tcdm_wrapper.sv:731, WRespFifoDepth=4): a store's
    // ack comes from the winfo FIFO ~2 cy after acceptance; a 5th store inside the window stalls
    // until the oldest ack drains. Evaluated after any PEND clamp (a conflicted store is accepted late).
    if (is_write) {
        int64_t virt = now + req->get_full_latency();
        while (!wresp_win_.empty() && wresp_win_.front() <= virt) wresp_win_.pop_front();
        if (wresp_win_.size() >= 4) {
            req->inc_latency(wresp_win_.front() - virt);
            lat_winfo_ += (uint64_t)(wresp_win_.front() - virt);
            virt = now + req->get_full_latency();
            wresp_win_.pop_front();
        }
        wresp_win_.push_back(virt + 2);
    }
    const int32_t wr_hit_lat = (write_hit_latency_cycles_ >= 0) ? write_hit_latency_cycles_
                                                                : hit_latency_cycles_;

    // --- HIT (also a D1-clamped follower served from its PEND line at serve_pend_way: the clamp
    // already waited out the refill; the +hit latency models the post-install drain through the
    // shared response arbiter, 1/cycle) ---
    if (d.is_hit || serve_pend_way >= 0) {
        const uint32_t hw = (serve_pend_way >= 0) ? (uint32_t)serve_pend_way : d.way;
        if (is_write) {
            exchange_line_data(req, set, hw, /*line_to_req=*/false);
            functional_write_mem(req);
            CacheStatus st = ways[hw].status;
            lru_update(geom_, ways, hw, st, st);        // MRU bump
            ways[hw].dirty = true;
            cnt_wr_hit_++;
            req->inc_latency(wr_hit_lat);               // D2: store ack = winfo-FIFO latency
        } else {
            exchange_line_data(req, set, hw, /*line_to_req=*/true);
            CacheStatus st = ways[hw].status;
            lru_update(geom_, ways, hw, st, st);
            cnt_rd_hit_++;
            req->inc_latency(hit_latency_cycles_);
        }
        lat_sum_ += (uint64_t)req->get_full_latency();
        return vp::IO_REQ_OK;
    }

    // --- MISS (hit-pend/conflict/all-pend collapse to a miss here: single-outstanding, in-call) ---
    uint32_t vw = d.way;
    WayMeta &vline = ways[vw];
    if (vline.status == VALID && vline.dirty) {
        // copy the victim's bytes BEFORE the refill overwrites the line (controller.cpp:562-564).
        uint64_t old_line = ((uint64_t)vline.tag << (geom_.off_bits + geom_.depth_bits)) |
                            ((uint64_t)set << geom_.off_bits);
        memcpy(evict_data_buf_.data(),
               &data_[((size_t)set * num_ways_ + vw) * cache_line_bytes_], cache_line_bytes_);
        if (evict_itf_.is_bound()) {
            evict_req_.init();
            evict_req_.set_addr(l2_addr(old_line));
            evict_req_.set_size(cache_line_bytes_);
            evict_req_.set_is_write(true);
            evict_req_.set_data(evict_data_buf_.data());
            (void)evict_itf_.req(&evict_req_);          // fire-and-forget writeback
        }
        cnt_evict_++;
    }
    CacheStatus before = vline.status;
    const uint32_t old_tag = vline.tag;
    const bool old_dirty = vline.dirty;
    vline.tag = geom_.tag_of(addr);
    CacheStatus after = is_write ? WRITE_PEND : READ_PEND;
    lru_update(geom_, ways, vw, before, after);         // allocate — BEFORE the status write
    vline.status = after;
    vline.dirty = false;

    // Refill occupancy gate (calibration step 2): issue this refill no earlier than the previous line's
    // ready cycle (the previous miss's install+serve), so cold-miss throughput is ~1/(ML+pipeline) (RTL),
    // not ~1/ML. The refill is still issued to the store now for its DATA; the request's TIMING is stamped
    // from the controller-gated issue point using the store's nominal (ungated) latency.
    const int64_t issue = (sync_refill_busy_until_ > now) ? sync_refill_busy_until_ : now;

    refill_req_.init();
    refill_req_.set_addr(l2_addr(addr_line(addr)));
    refill_req_.set_size(cache_line_bytes_);
    refill_req_.set_is_write(false);
    refill_req_.set_data(refill_data_buf_.data());
    vp::IoReqStatus rst = refill_itf_.req(&refill_req_);
    if (rst == vp::IO_REQ_OK) {
        // The store's nominal (ungated) refill latency — the minimum seen (the store is ungated on the
        // first refill of a run). We use this for timing instead of the per-call gated full_latency.
        const int64_t full_lat = (int64_t)refill_req_.get_full_latency();
        if (ml_nominal_ < 0 || full_lat < ml_nominal_) ml_nominal_ = full_lat;
        // Install the refill DATA now (so D1-clamped followers read correct bytes after their wait) but
        // keep the line PEND with ready_cycle = the response cycle — the line is not VISIBLE as VALID
        // until the refill pipeline has logically delivered it (D1). The VALID install + its LRU
        // update happen lazily in the ready-cycle sweep.
        if (refill_req_.get_data() != nullptr)
            memcpy(&data_[((size_t)set * num_ways_ + vw) * cache_line_bytes_],
                   refill_req_.get_data(), cache_line_bytes_);
        // Response cycle = (controller-gated issue) + (store nominal latency) + (install pipeline).
        // The next refill-read waits for this line's ready cycle → serialized miss throughput.
        const int64_t resp_cycle = issue + ml_nominal_ + refill_bank_write_cycles_ + miss_penalty_cycles_;
        ways[vw].ready_cycle = resp_cycle;
        // The controller's refill pipeline stays busy an additional install_tail_cycles_ past the
        // response (RTL install-pipeline tail), so the NEXT refill-read waits for that too → cold-miss
        // throughput drops to the RTL ~1/(ML+17) rate.
        sync_refill_busy_until_ = resp_cycle + install_tail_cycles_;
        if (is_write) {
            // D2: the store ack comes from the winfo FIFO ~acceptance-time — the WRITE_PEND merge
            // with the refill happens inside the cache, invisible to the LSU. Apply the store data
            // (+ functional WT) now; the line stays WRITE_PEND+dirty until the ready-cycle sweep, so
            // a subsequent load correctly stalls for the rest of the refill.
            exchange_line_data(req, set, vw, /*line_to_req=*/false);
            functional_write_mem(req);
            ways[vw].dirty = true;
            cnt_wr_miss_++;
            req->inc_latency(wr_hit_lat);
        } else {
            exchange_line_data(req, set, vw, /*line_to_req=*/true);
            cnt_rd_miss_++;
            // The read misser waits for the refill: reported latency = resp_cycle - now.
            req->inc_latency(resp_cycle - now);
        }
        cnt_refill_++;
        lat_sum_ += (uint64_t)req->get_full_latency();
        return vp::IO_REQ_OK;
    }
    // The refill did not complete (rst != OK). The cluster L2 (wide_axi → SPM) answers synchronously, so
    // this is not expected — but the previous fallback was a real bug: it served the requester from the
    // NEVER-FILLED line (stale/zero bytes) and marked it VALID, so all later hits returned the same garbage
    // silently. Instead: undo the allocation (the dirty victim, if any, was already written back above, so
    // restoring its VALID state is safe — its data is in memory) and serve the requester directly from the
    // backing store with the original address/size. The line stays unallocated; a later access will miss
    // and retry the refill.
    lru_update(geom_, ways, vw, after, before);          // revert the allocate
    vline.tag = old_tag;
    vline.status = before;
    vline.dirty = old_dirty;
    // The wire needs the global (unrotated) address; the req itself stays in rotated space for any
    // later cache-side decode, so save/restore around the bypass call.
    const uint64_t req_addr_save = req->get_addr();
    req->set_addr(l2_addr(req_addr_save));
    vp::IoReqStatus fst = refill_itf_.req(req);          // bypass the cache: serve from backing memory
    req->set_addr(req_addr_save);
    if (fst == vp::IO_REQ_OK) {
        req->inc_latency((int64_t)req->get_full_latency() + miss_penalty_cycles_);
        lat_sum_ += (uint64_t)req->get_full_latency();
        return vp::IO_REQ_OK;
    }
    // Last resort (should be unreachable with a synchronous memory): keep the VLSU alive as before, but
    // WITHOUT marking the line VALID, and say so loudly.
    this->trace_.msg(vp::Trace::LEVEL_WARNING,
        "refill AND direct backing-store access both failed (set=%u addr=0x%lx) — serving unbacked data\n",
        set, (unsigned long)addr);
    if (is_write) { exchange_line_data(req, set, vw, false); functional_write_mem(req); }
    else          { exchange_line_data(req, set, vw, true); }
    req->inc_latency(refill_bank_write_cycles_ + miss_penalty_cycles_);
    return vp::IO_REQ_OK;
}

// ---------- per-cycle tick ----------
void InsituCacheCore::tick(vp::Block *__this, vp::ClockEvent *event)
{
    (void)event;
    InsituCacheCore *_this = static_cast<InsituCacheCore *>(__this);
    const int64_t now = _this->clock.get_cycles();
    _this->bank_.begin_cycle(now);          // reset the per-cycle bank write scoreboard

    _this->stage1_process();                // process the latched preread task
    _this->drain_outputs();                 // one beat per output FIFO
    _this->stage0_arbitrate();              // latch next preread task

    // Reschedule. If the only pending work is a sync-responder refill still in flight (its data
    // becomes ready at refill_ready_cycle_), jump straight to that cycle instead of spinning every
    // cycle — the refill latency EMERGES as the gap between issue and install.
    const int64_t now2 = _this->clock.get_cycles();
    const bool near_work = _this->preread_q_.valid || !_this->in_q_.empty() ||
        !_this->admission_stall_q_.empty() ||
        _this->refill_spill_valid_ || !_this->resp_fifo_.empty() || !_this->evic_fifo_.empty() ||
        (!_this->miss_fifo_.empty() && !_this->refill_pending_ && _this->refill_ready_cycle_ < 0);
    if (near_work) {
        _this->schedule_tick(1);
    } else if (_this->refill_ready_cycle_ > now2) {
        _this->schedule_tick(_this->refill_ready_cycle_ - now2);   // wake when the refill completes
    } else if (_this->any_work()) {
        _this->schedule_tick(1);
    }
}

void InsituCacheCore::stage1_process()
{
    // Refill install has priority for the single bank port (RTL core.sv:894-903 retr-room gate) and a
    // SEPARATE path from the request pipeline — so a stalled request in preread_q_ can never block the
    // refill that would unblock it (the deadlock when refills are deferred). If a refill installs this
    // tick, the latched request simply waits one more tick in preread_q_.
    if (maybe_install_refill()) return;
    if (!preread_q_.valid) return;
    if (process_request(preread_q_.req)) preread_q_.valid = false;   // done; a stall keeps it latched
}

// Install a refilled line if its data is ready (async resp set refill_spill_valid_, or the sync
// responder's stamped latency has elapsed at refill_ready_cycle_). Returns true if it installed.
bool InsituCacheCore::maybe_install_refill()
{
    const bool sync_ready = (refill_ready_cycle_ >= 0 && clock.get_cycles() >= refill_ready_cycle_);
    if (!refill_spill_valid_ && !sync_ready) return false;
    refill_spill_valid_ = false;
    refill_ready_cycle_ = -1;
    install_refill();
    return true;
}

bool InsituCacheCore::process_request(vp::IoReq *req)
{
    const uint64_t addr = req->get_addr();
    const bool is_write = req->get_is_write();
    const uint32_t set = geom_.set_index(addr);
    WayMeta *ways = set_ways(set);
    Decode d = decode_request(geom_, ways, addr, is_write);
    const int64_t now = clock.get_cycles();

    // VALID hit
    if (d.is_hit) {
        // bank read conflict: a write took this (way,bank-select) to a different row this cycle.
        if (!is_write && bank_.read_conflict(now, d.way, set)) {
            cnt_bank_conflict_++;
            return false;   // bank conflict → stall, retry next tick (stays in preread_q_)
        }
        if (is_write) {
            exchange_line_data(req, set, d.way, /*line_to_req=*/false);
            functional_write_mem(req);
            bank_.commit_write(now, d.way, set);
            CacheStatus before = ways[d.way].status;
            lru_update(geom_, ways, d.way, before, before);  // MRU bump
            ways[d.way].dirty = true;
            cnt_wr_hit_++;
        } else {
            exchange_line_data(req, set, d.way, /*line_to_req=*/true);
            lru_update(geom_, ways, d.way, ways[d.way].status, ways[d.way].status);
            cnt_rd_hit_++;
        }
        resp_fifo_.push_back(req);
        resp_done_cyc_.push_back(hit_ready_cyc(req));   // hit
        was_miss_[req] = false;
        return true;
    }

    // hit on a pending line (in-situ MSHR merge)
    if (d.is_hit_pend) {
        mshr_[(size_t)set*num_ways_ + d.way].push_back(req);
        cnt_mshr_merge_++;
        if (is_write) cnt_wr_miss_++; else cnt_rd_miss_++;
        return true;   // parked on the pending line's reader list
    }

    // conflict / all-pend → stall by retrying next tick (functional stall; full FSM enum is TODO)
    if (d.is_hit_conflit || d.is_all_pend) {
        return false;
    }

    // miss: allocate victim + refill
    if (miss_fifo_.size() >= miss_fifo_depth_ || retr_level_ >= retr_fifo_depth_) {
        return false;   // miss/retr FIFO full → stall, retry next tick
    }
    uint32_t vw = d.way;
    WayMeta &vline = ways[vw];
    if (vline.status == VALID && vline.dirty) {
        if (evic_fifo_.size() >= evic_fifo_depth_) {
            return false;   // eviction FIFO full → stall
        }
        uint64_t old_line = ((uint64_t)vline.tag << (geom_.off_bits + geom_.depth_bits)) |
                            ((uint64_t)set << geom_.off_bits);
        EvictEnt ent;
        ent.addr = old_line;
        const uint8_t *src = &data_[((size_t)set * num_ways_ + vw) * cache_line_bytes_];
        ent.data.assign(src, src + cache_line_bytes_);   // snapshot BEFORE the refill overwrites the way
        evic_fifo_.push_back(std::move(ent));
        cnt_evict_++;
    }
    CacheStatus before = vline.status;
    vline.tag = geom_.tag_of(addr);
    CacheStatus after = is_write ? WRITE_PEND : READ_PEND;
    lru_update(geom_, ways, vw, before, after);   // allocate (pre-write statuses) — call before status write
    vline.status = after;
    vline.dirty = false;
    mshr_[(size_t)set*num_ways_ + vw].push_back(req);
    retr_level_++;
    miss_fifo_.push_back(addr_line(addr));
    if (is_write) cnt_wr_miss_++; else cnt_rd_miss_++;
    return true;   // miss allocated, refill queued
}

// Cycle at which a HIT becomes eligible to respond.
//
// With hit_latency_floor_ > 0 this is a FLOOR measured from the request's arrival
// (accept_cyc_), so time the request already spent queueing counts toward it instead of being
// added to it. Falls back to the legacy tail-addend when the floor is disabled, or when the
// arrival stamp is missing (it is erased at drain, so a request can only be absent if it was
// never admitted through req_handler).
int64_t InsituCacheCore::hit_ready_cyc(vp::IoReq *req)
{
    const int64_t now = clock.get_cycles();
    if (hit_latency_floor_ <= 0) return now + resp_latency_cycles_;
    auto it = accept_cyc_.find(req);
    if (it == accept_cyc_.end()) return now + resp_latency_cycles_;
    const int64_t floor_cyc = it->second + hit_latency_floor_;
    return floor_cyc > now ? floor_cyc : now;
}

void InsituCacheCore::install_refill()
{
    // The refilled line is in refill_req_'s data buffer; re-decode set/way from the stashed addr.
    const uint64_t addr = pending_refill_addr_;
    const uint32_t set = geom_.set_index(addr);
    const uint64_t tag = geom_.tag_of(addr);
    WayMeta *ways = set_ways(set);
    int way = -1;
    for (uint32_t w = 0; w < num_ways_; w++)
        if ((ways[w].status == READ_PEND || ways[w].status == WRITE_PEND) && ways[w].tag == tag) { way = (int)w; break; }
    refill_pending_ = false; refill_spill_valid_ = false;
    if (way < 0) return;  // already installed/flushed

    // install the fetched bytes
    if (refill_req_.get_data() != nullptr) {
        memcpy(&data_[((size_t)set*num_ways_+(uint32_t)way)*cache_line_bytes_], refill_req_.get_data(), cache_line_bytes_);
        const uint64_t l2line = l2_addr(addr);
        if (dbg_watch() && (l2line >> 6) == (dbg_watch() >> 6))
            dbg_ev("refill", l2line + (dbg_watch() & 63), 4,
                   refill_req_.get_data() + (dbg_watch() & 63));
    }
    bank_.commit_write(clock.get_cycles(), (uint32_t)way, set);

    CacheStatus before = ways[way].status;
    lru_update(geom_, ways, (uint32_t)way, before, VALID);   // complete (pre-write)
    ways[way].status = VALID;
    cnt_refill_++;

    // drain the queued readers/writers of this line (in-situ MSHR)
    auto &q = mshr_[(size_t)set*num_ways_+(uint32_t)way];
    while (!q.empty()) {
        vp::IoReq *r = q.front(); q.pop_front();
        if (retr_level_ > 0) retr_level_--;
        if (r->get_is_write()) {
            exchange_line_data(r, set, way, /*line_to_req=*/false);
            functional_write_mem(r);
            ways[way].dirty = true;
        } else {
            exchange_line_data(r, set, way, /*line_to_req=*/true);
        }
        resp_fifo_.push_back(r);
        // waited on a refill: hit delay + the miss-side term
        resp_done_cyc_.push_back(clock.get_cycles() + resp_latency_cycles_ + miss_extra_cycles_);
        was_miss_[r] = true;
    }
}

void InsituCacheCore::drain_outputs()
{
    // one completed access resp() per tick
    if (!resp_fifo_.empty()) {
        // Structural response latency: the head is not eligible until resp_latency_cycles_ have
        // elapsed since it completed. Re-arm the tick so it is released on time.
        if (!resp_done_cyc_.empty()) {
            if (clock.get_cycles() < resp_done_cyc_.front()) { schedule_tick(); return; }
            resp_done_cyc_.pop_front();
        }
        vp::IoReq *r = resp_fifo_.front(); resp_fifo_.pop_front();
        {
            auto it = accept_cyc_.find(r);
            if (it != accept_cyc_.end()) {
                const int64_t lat = clock.get_cycles() - it->second;
                served_lat_sum_ += (uint64_t)lat; served_lat_n_++;
                auto mit = was_miss_.find(r);
                const bool miss = (mit != was_miss_.end()) && mit->second;
                if (mit != was_miss_.end()) was_miss_.erase(mit);
                if (miss) {
                    miss_lat_sum_ += (uint64_t)lat; miss_lat_n_++;
                    if (miss_lat_min_ < 0 || lat < miss_lat_min_) miss_lat_min_ = lat;
                } else {
                    hit_lat_sum_ += (uint64_t)lat; hit_lat_n_++;
                    if (hit_lat_min_ < 0 || lat < hit_lat_min_) hit_lat_min_ = lat;
                }
                accept_cyc_.erase(it);
            }
        }
        // The request's arg stack is untouched by us (see req_handler) — the requester's own arguments
        // are still on top, exactly where its resp callback expects them.
        //
        // Hand back the address the requester issued: the xbar rotates on the way in, and l2_addr() is
        // the same inverse applied on every L2 egress (identity when rotate_bits_==0). The synchronous
        // path gets this from the xbar instead, which can only un-rotate on an OK return. It matters
        // downstream: the L1 NoC's network interface re-derives routing from req->get_addr(), and a
        // rotated address parks the routing bits in the MSBs, outside every mapped window.
        r->set_addr(l2_addr(r->get_addr()));
        r->get_resp_port()->resp(r);
    }
    // one refill issue per tick, single-outstanding: gate on refill_pending_ (async in flight),
    // refill_spill_valid_ (ready, awaiting install) AND refill_ready_cycle_ (sync refill in flight) —
    // so a new line-refill cannot start until the current one installs (serialized miss throughput).
    if (!refill_pending_ && !refill_spill_valid_ && refill_ready_cycle_ < 0 && !miss_fifo_.empty()) {
        uint64_t line = miss_fifo_.front(); miss_fifo_.pop_front();
        pending_refill_addr_ = line;   // stays ROTATED — install_refill() re-decodes set/way from it
        refill_req_.init();
        refill_req_.set_addr(l2_addr(line));
        refill_req_.set_size(cache_line_bytes_);
        refill_req_.set_is_write(false);
        refill_req_.set_data(refill_data_buf_.data());
        refill_pending_ = true;
        vp::IoReqStatus st = refill_itf_.req(&refill_req_);
        if (st == vp::IO_REQ_OK) {
            // Synchronous responder: it returned the data now but stamped the access latency via
            // inc_latency. The refill's data is ready `lat` cycles from now — defer the install so the
            // cold-miss latency EMERGES as that gap (matches the RTL refill_mem_model). A 0-latency
            // responder installs next tick (lat<=0 → ready now).
            refill_pending_ = false;
            const int64_t lat = (int64_t)refill_req_.get_full_latency();
            const int64_t now = clock.get_cycles();
            refill_ready_cycle_ = now + (lat > 0 ? lat : 0);
        }
    }
    // one eviction issue per tick
    if (!evict_wb_pending_ && !evic_fifo_.empty()) {
        EvictEnt ent = std::move(evic_fifo_.front()); evic_fifo_.pop_front();
        evict_wb_buf_ = std::move(ent.data);   // must outlive the request when L2 answers PENDING
        evict_req_.init();
        evict_req_.set_addr(l2_addr(ent.addr));
        evict_req_.set_size(cache_line_bytes_);
        evict_req_.set_is_write(true);
        evict_req_.set_data(evict_wb_buf_.data());
        // DENIED counts as in flight, exactly like PENDING: the slave has taken ownership and will
        // grant + complete it later (FlooNoc's NI queues a denied burst and calls grant() when it
        // drains). Checking only for PENDING left the flag clear on DENIED, so the next tick re-issued
        // the SAME evict_req_ — the L2 refill mesh saw one write pointer resubmitted every cycle,
        // pushed onto the NI's denied queue each time, and every later drain incremented its
        // outstanding-burst count until it wedged at the cap with nothing left to decrement it.
        const vp::IoReqStatus est = evict_itf_.req(&evict_req_);
        if (est == vp::IO_REQ_PENDING || est == vp::IO_REQ_DENIED) evict_wb_pending_ = true;
    }
}

void InsituCacheCore::stage0_arbitrate()
{
    if (preread_q_.valid) return;   // stage-1 still holds a latched/stalled request
    // F1 on the ASYNC path: the flush walk gates all upstream traffic (l1d_busy_i), and it has to do
    // so STRUCTURALLY. run_flush() stamps the walk's duration with inc_latency() and run_request_sync()
    // stamps the remaining wait for anything arriving during it — both are discarded on this path, so
    // a flush cost nothing here and every flush-heavy kernel ran short (load-store_M16 -9.5%).
    if (clock.get_cycles() < flush_busy_until_) { schedule_tick(); return; }
    // (refill install is handled separately in maybe_install_refill — it does not use preread_q_.)
    // Re-admit parked requests as accept-queue space frees.
    while (!admission_stall_q_.empty() && in_q_.size() < in_q_cap_) {
        in_q_.push_back(admission_stall_q_.front());
        admission_stall_q_.pop_front();
    }
    if (!in_q_.empty()) {
        vp::IoReq *r = in_q_.front(); in_q_.pop_front();
        preread_q_.valid = true; preread_q_.is_refill = false; preread_q_.req = r;
        preread_q_.addr = r->get_addr(); preread_q_.is_write = r->get_is_write();
    }
}

// A writeback completed. Only evict_req_ is tracked — the flush path reuses evict_req_ synchronously and
// funcwr_req_ (functional write-through) also rides this port, so ignore anything that is not ours.
void InsituCacheCore::evict_resp_handler(vp::Block *__this, vp::IoReq *req)
{
    InsituCacheCore *_this = static_cast<InsituCacheCore *>(__this);
    if (req != &_this->evict_req_) return;
    _this->evict_wb_pending_ = false;
    _this->schedule_tick();
}

void InsituCacheCore::refill_resp_handler(vp::Block *__this, vp::IoReq *req)
{
    (void)req;
    InsituCacheCore *_this = static_cast<InsituCacheCore *>(__this);
    _this->refill_spill_valid_ = true;
    _this->refill_pending_ = false;
    _this->schedule_tick();
}

extern "C" vp::Component *gv_new(vp::ComponentConf &config)
{
    return new InsituCacheCore(config);
}
