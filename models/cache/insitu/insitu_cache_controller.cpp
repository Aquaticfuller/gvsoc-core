// SPDX-FileCopyrightText: 2026 ETH Zurich and University of Bologna
//
// SPDX-License-Identifier: Apache-2.0
//
// InSitu Cache controller — performance model.
//
// Single controller. One controller serves a subset of cache lines (address-interleaved at
// the cache interco). On hit it serves the request in-place with a fixed latency; on miss
// it triggers a refill and queues the request via an MSHR list until the line arrives.
//
// See prompt/insitu_cache_architecture.md §§6-7 for the RTL microarchitecture and
// prompt/insitu_cache_gvsoc_plan.md §§1-4 for modeling decisions.

#include <cstdint>
#include <cstring>
#include <cstdio>
#include <vector>
#include <deque>

#include <vp/vp.hpp>
#include <vp/queue.hpp>
#include <vp/itf/io.hpp>

namespace {

inline unsigned int ceil_log2(unsigned int n)
{
    if (n <= 1) return 0;
    return 32 - __builtin_clz(n - 1);
}

// Line state — mirrors RTL §7.6 (VALID / READ_PEND / WRITE_PEND).
enum class LineState : uint8_t
{
    INVALID = 0,
    VALID,
    READ_PEND,   // miss in flight; pending read reqs queued on MSHR
    WRITE_PEND,  // miss in flight for a write; merges on refill
};

struct CacheLine
{
    uint32_t tag = 0;
    LineState state = LineState::INVALID;
    bool dirty = false;
    // Cycle when this line becomes usable. Requests arriving before this pay the diff in
    // extra latency (captures the "refill just landed, banks still settling" window).
    int64_t ready_cycle = -1;
};

// One entry in the per-set MSHR queue. We track the arrival cycle explicitly so the
// request object's own latency bookkeeping stays clean across save()/restore().
struct MshrEntry
{
    vp::IoReq *req;
    int64_t arrival_cycle;
};

}  // namespace

class InsituCacheController : public vp::Component
{
public:
    explicit InsituCacheController(vp::ComponentConf &conf);

    void reset(bool active) override;

private:
    // ===== IoReq handlers =====
    static vp::IoReqStatus req_handler(vp::Block *__this, vp::IoReq *req);
    static void refill_resp_handler(vp::Block *__this, vp::IoReq *req);

    // ===== Internal helpers =====
    vp::IoReqStatus handle_request(vp::IoReq *req);
    CacheLine *lookup(uint32_t tag, uint32_t set, int *way_out);
    int pick_victim(uint32_t tag, uint32_t set);
    void issue_write_through(vp::IoReq *user_req);
    void issue_refill(uint32_t line_addr, uint32_t set, int way);
    void issue_eviction(uint32_t line_addr);
    void fsm_drain_mshr(uint32_t set);

    // Reserve one slot in the (near-serial) refill/writeback install pipeline. A completion
    // can occur no earlier than the pipe being free (`refill_drain_busy_until_ + step`) and
    // no earlier than `floor`; the pipe is then busy until that cycle. Returns the
    // completion cycle. Monotonic (never runs backwards). Used by both the refill-response
    // and the dirty-writeback paths so the drain invariant lives in one place.
    inline int64_t reserve_install_pipe(int64_t step, int64_t floor)
    {
        int64_t c = refill_drain_busy_until_ + step;
        if (floor > c) c = floor;
        refill_drain_busy_until_ = c;
        return c;
    }

    // ===== Address decomposition =====
    inline uint32_t addr_set_raw(uint64_t addr) const
    { return (addr >> line_bits_) & ((1u << set_bits_) - 1); }
    inline uint32_t addr_set(uint64_t addr) const {
        // Latest RTL's partition-flushable wrapper remaps incoming addresses so the
        // cache only ever sees sets in [bank_depth_for_spm_, num_sets_). We approximate
        // that here by folding the raw set into the cacheable region. SPM-region access
        // does not flow through this controller in the real system.
        if (!enable_spm_ || bank_depth_for_spm_ == 0) return addr_set_raw(addr);
        const uint32_t s = addr_set_raw(addr);
        return bank_depth_for_spm_ + (s % effective_num_sets_);
    }
    inline uint32_t addr_tag(uint64_t addr) const
    { return (uint32_t)(addr >> (line_bits_ + set_bits_)); }
    inline uint64_t addr_line(uint64_t addr) const
    { return addr & ~((1ULL << line_bits_) - 1); }

    // ===== Config =====
    uint32_t cache_line_bytes_;
    uint32_t num_ways_;
    uint32_t num_sets_;
    uint32_t tcdm_word_bytes_;
    uint32_t refill_beat_bytes_;
    bool     use_hash_way_select_;
    bool     use_forwarding_buffer_;   // RTL UseForwardingBuffer; informational — its
                                       // same-row-forward timing effect is absorbed into
                                       // hit_latency_cycles for now (Phase B: explicit).
    // Latest-RTL-tracking knobs (insitu_cache_architecture_v2.md):
    bool     write_through_mode_;       // false = pure write-back (latest RTL default)
    bool     enable_multi_read_pend_;   // false = legacy single-read MSHR per line
    bool     enable_spm_;
    uint32_t bank_depth_for_spm_;       // # bank rows reserved for SPM
    bool     enable_flush_;             // reserved; flush sequence not yet implemented
    int32_t  hit_latency_cycles_;
    int32_t  streaming_hit_latency_cycles_;  // <0 ⇒ OFF (flat hit_latency_cycles_ for every hit)
    int64_t  last_read_hit_cycle_ = -1;      // pipeline-warmth anchor (streaming-hit model)
    int32_t  bank_accept_cycles_;            // per-set bank accept interval (pipelined, ≈1)
    int32_t  scalar_bypass_port_;            // input port that bypasses the coalescer/bank (-1=none)
    int32_t  scalar_hit_latency_cycles_;     // read-hit latency on the scalar bypass port (<0=hit_latency)
    int32_t  write_hit_latency_cycles_;   // <0 ⇒ use hit_latency_cycles_
    int32_t  write_commit_cycles_;        // min cycles between accepted write hits
    int32_t  fwd_hit_latency_cycles_;     // read hit on the fwd-buffer line; <0 ⇒ hit_latency_cycles_
    int32_t  miss_penalty_cycles_;
    // Occupancy model (only active when defer_refills_): serialize refill/writeback
    // completions through the install pipeline at refill_drain_cycles_ apart.
    bool     defer_refills_;
    int32_t  refill_drain_cycles_;        // min cycles between completions (install rate)
    int32_t  refill_bank_write_cycles_;
    int32_t  folded_evict_penalty_cycles_;
    int32_t  mshr_drain_cycles_per_subarray_;
    uint32_t resp_fifo_depth_;
    uint32_t retr_fifo_depth_;
    uint32_t miss_fifo_depth_;
    uint32_t evic_fifo_depth_;
    uint32_t wt_fifo_depth_;
    // Derived: when enable_spm_, the number of *cacheable* sets is reduced.
    uint32_t effective_num_sets_;

    uint32_t line_bits_;
    uint32_t set_bits_;

    // ===== Ports =====
    vp::IoSlave  input_itf_;
    vp::IoMaster refill_itf_;
    vp::IoMaster evict_itf_;
    vp::IoMaster wt_itf_;

    // ===== State =====
    std::vector<CacheLine>               lines_;     // [num_sets * num_ways]
    std::vector<std::vector<uint8_t>>    lru_order_; // [num_sets][num_ways], front=MRU
    std::vector<std::deque<MshrEntry>>   mshr_;      // [num_sets]
    std::vector<bool>                    refill_in_flight_;  // [num_sets]
    std::vector<int64_t>                 set_busy_until_;    // [num_sets]

    // 1-entry forwarding buffer: the line whose row is currently cached in the access
    // controller's registers. A read hit on this line forwards combinationally (skips the
    // SRAM read) → fwd_hit_latency_cycles_. -1 == buffer empty.
    int64_t fwd_buffer_line_ = -1;
    // Controller-wide write-commit backpressure: earliest cycle a new write hit may be
    // accepted (models the write-info FIFO + per-way commit serialization).
    int64_t write_commit_busy_until_ = -1;
    // Occupancy: serializes refill completions into the cache at the install rate
    // (refill_drain_cycles_) so queued misses' completion latency inflates under load.
    int64_t refill_drain_busy_until_ = -1;

    // FIFO occupancy counters.
    uint32_t miss_fifo_level_ = 0;
    uint32_t evic_fifo_level_ = 0;
    uint32_t retr_fifo_level_ = 0;
    uint32_t resp_fifo_level_ = 0;

    // Owned IoReqs for outgoing traffic. Safe because the response path re-derives state
    // from the request's address, not from these fields.
    vp::IoReq refill_req_;
    vp::IoReq evict_req_;
    vp::IoReq wt_req_;
    // Scratch buffers backing each req's data pointer. The downstream memory model
    // performs a memcpy against the request's data pointer, so we must provide a real
    // buffer even though the perf model doesn't care about the byte values.
    std::vector<uint8_t> refill_data_buf_;
    std::vector<uint8_t> evict_data_buf_;
    std::vector<uint8_t> wt_data_buf_;

    // ===== Telemetry =====
    vp::Trace trace_;
    uint64_t cnt_rd_hit_ = 0;
    uint64_t cnt_rd_miss_ = 0;
    uint64_t cnt_wr_hit_ = 0;
    uint64_t cnt_wr_miss_ = 0;
    uint64_t cnt_mshr_merge_ = 0;
    uint64_t cnt_evict_ = 0;
    uint64_t cnt_wb_dirty_ = 0;
    uint64_t cnt_stall_miss_fifo_ = 0;
    uint64_t cnt_stall_evic_fifo_ = 0;
    uint64_t cnt_stall_mshr_full_ = 0;
    uint64_t cnt_refills_issued_ = 0;
    uint64_t cnt_writes_through_ = 0;
};

InsituCacheController::InsituCacheController(vp::ComponentConf &conf)
    : vp::Component(conf)
{
    auto *cfg = this->get_js_config();
    cache_line_bytes_              = cfg->get_child_int("cache_line_bytes");
    num_ways_                      = cfg->get_child_int("num_ways");
    num_sets_                      = cfg->get_child_int("num_sets");
    tcdm_word_bytes_               = cfg->get_child_int("tcdm_word_bytes");
    refill_beat_bytes_             = cfg->get_child_int("refill_beat_bytes");
    use_hash_way_select_           = cfg->get_child_bool("use_hash_way_select");
    use_forwarding_buffer_         = cfg->get_child_bool("use_forwarding_buffer");
    write_through_mode_            = cfg->get_child_bool("write_through_mode");
    enable_multi_read_pend_        = cfg->get_child_bool("enable_multi_read_pend");
    enable_spm_                    = cfg->get_child_bool("enable_spm");
    bank_depth_for_spm_            = cfg->get_child_int("bank_depth_for_spm");
    enable_flush_                  = cfg->get_child_bool("enable_flush");
    hit_latency_cycles_            = cfg->get_child_int("hit_latency_cycles");
    streaming_hit_latency_cycles_  = cfg->get_child_int("streaming_hit_latency_cycles");
    bank_accept_cycles_            = cfg->get_child_int("bank_accept_cycles");
    if (bank_accept_cycles_ < 1) bank_accept_cycles_ = 1;
    scalar_bypass_port_            = cfg->get_child_int("scalar_bypass_port");
    scalar_hit_latency_cycles_     = cfg->get_child_int("scalar_hit_latency_cycles");
    write_hit_latency_cycles_      = cfg->get_child_int("write_hit_latency_cycles");
    write_commit_cycles_           = cfg->get_child_int("write_commit_cycles");
    fwd_hit_latency_cycles_        = cfg->get_child_int("fwd_hit_latency_cycles");
    miss_penalty_cycles_           = cfg->get_child_int("miss_penalty_cycles");
    defer_refills_                 = cfg->get_child_bool("defer_refills");
    refill_drain_cycles_           = cfg->get_child_int("refill_drain_cycles");
    refill_bank_write_cycles_      = cfg->get_child_int("refill_bank_write_cycles");
    folded_evict_penalty_cycles_   = cfg->get_child_int("folded_evict_penalty_cycles");
    mshr_drain_cycles_per_subarray_= cfg->get_child_int("mshr_drain_cycles_per_subarray");
    resp_fifo_depth_               = cfg->get_child_int("resp_fifo_depth");
    retr_fifo_depth_               = cfg->get_child_int("retr_fifo_depth");
    miss_fifo_depth_               = cfg->get_child_int("miss_fifo_depth");
    evic_fifo_depth_               = cfg->get_child_int("evic_fifo_depth");
    wt_fifo_depth_                 = cfg->get_child_int("wt_fifo_depth");

    line_bits_ = ceil_log2(cache_line_bytes_);
    set_bits_  = ceil_log2(num_sets_);

    // SPM partitioning: shrink the cacheable set space (architecture_v2.md §5). The
    // RTL's wrapper rebases addresses so the cache only ever sees the upper sets; for
    // perf modelling we approximate this by reducing the effective number of sets used
    // for tag matching and victim selection. The bank arrays still cover all `num_sets_`
    // entries (so memory footprint is unchanged) — the cache simply never allocates
    // in the SPM-reserved sets.
    if (enable_spm_ && bank_depth_for_spm_ < num_sets_) {
        effective_num_sets_ = num_sets_ - bank_depth_for_spm_;
    } else {
        effective_num_sets_ = num_sets_;
    }

    lines_.assign(num_sets_ * num_ways_, CacheLine{});
    lru_order_.assign(num_sets_, std::vector<uint8_t>{});
    for (uint32_t s = 0; s < num_sets_; ++s) {
        lru_order_[s].resize(num_ways_);
        for (uint32_t w = 0; w < num_ways_; ++w) lru_order_[s][w] = (uint8_t)w;
    }
    mshr_.assign(num_sets_, std::deque<MshrEntry>{});
    refill_in_flight_.assign(num_sets_, false);
    set_busy_until_.assign(num_sets_, -1);
    refill_data_buf_.assign(cache_line_bytes_, 0);
    evict_data_buf_.assign(cache_line_bytes_, 0);
    wt_data_buf_.assign(cache_line_bytes_, 0);

    this->input_itf_.set_req_meth(&InsituCacheController::req_handler);
    this->new_slave_port("input", &this->input_itf_);

    this->refill_itf_.set_resp_meth(&InsituCacheController::refill_resp_handler);
    this->new_master_port("refill", &this->refill_itf_);

    this->new_master_port("evict", &this->evict_itf_);
    this->new_master_port("write_through", &this->wt_itf_);

    this->traces.new_trace("trace", &this->trace_, vp::DEBUG);

    this->trace_.msg(vp::Trace::LEVEL_INFO,
        "InsituCacheController instantiated (line=%u B, ways=%u, sets=%u, total=%u KB, "
        "hash_way=%d, fwd_buf=%d)\n",
        cache_line_bytes_, num_ways_, num_sets_,
        (cache_line_bytes_ * num_ways_ * num_sets_) >> 10,
        (int)use_hash_way_select_, (int)use_forwarding_buffer_);
}

void InsituCacheController::reset(bool active)
{
    if (active) {
        for (auto &line : lines_) {
            line.state = LineState::INVALID;
            line.tag = 0;
            line.dirty = false;
            line.ready_cycle = -1;
        }
        // refill_in_flight_ is std::vector<bool> (proxy elements) — assign by index.
        for (size_t i = 0; i < refill_in_flight_.size(); ++i) refill_in_flight_[i] = false;
        for (auto &v : set_busy_until_) v = -1;
        for (auto &q : mshr_) q.clear();
        miss_fifo_level_ = evic_fifo_level_ = retr_fifo_level_ = resp_fifo_level_ = 0;
        fwd_buffer_line_ = -1;
        last_read_hit_cycle_ = -1;
        write_commit_busy_until_ = -1;
        refill_drain_busy_until_ = -1;
    }
}

// ---------- request entry ----------

vp::IoReqStatus InsituCacheController::req_handler(vp::Block *__this, vp::IoReq *req)
{
    InsituCacheController *_this = static_cast<InsituCacheController *>(__this);
    _this->trace_.msg(vp::Trace::LEVEL_TRACE,
        "req addr=0x%lx size=%lu wr=%d\n",
        (unsigned long)req->get_addr(),
        (unsigned long)req->get_size(),
        (int)req->get_is_write());

    return _this->handle_request(req);
}

vp::IoReqStatus InsituCacheController::handle_request(vp::IoReq *req)
{
    const int64_t now = this->clock.get_cycles();
    const uint64_t addr = req->get_addr();
    const uint32_t tag = addr_tag(addr);
    const uint32_t set = addr_set(addr);
    const bool is_write = req->get_is_write();
    // Scalar bypass: the interco tags the request's initiator with its input-port index when
    // forward_initiator is on; the scalar port reads bypass the coalescer + bank contention.
    const bool is_scalar = (scalar_bypass_port_ >= 0) &&
                           (req->get_initiator() == scalar_bypass_port_);

    // Write-commit backpressure: the RTL write path serializes through the write-info
    // FIFO + per-way commit, so writes accept at most once per write_commit_cycles_.
    // Deny (back-pressure) a write that arrives while the commit slot is still busy; the
    // upstream retries on the next cycle. (No effect when write_commit_cycles_ <= 1.)
    if (is_write && write_commit_cycles_ > 1 && now < write_commit_busy_until_) {
        return vp::IO_REQ_DENIED;
    }
    if (is_write && write_commit_cycles_ > 1) {
        write_commit_busy_until_ = now + write_commit_cycles_;
    }

    int way = -1;
    CacheLine *line = lookup(tag, set, &way);

    // --- hit on VALID line ---
    if (line != nullptr && line->state == LineState::VALID) {
        // Base hit latency: writes ack faster than reads return (write-info FIFO push);
        // a read on the forwarding-buffer line forwards combinationally (skips SRAM read).
        bool forwarded = false;
        int64_t base = hit_latency_cycles_;
        if (is_write) {
            base = (write_hit_latency_cycles_ >= 0) ? write_hit_latency_cycles_
                                                    : hit_latency_cycles_;
        } else if (is_scalar && scalar_hit_latency_cycles_ >= 0) {
            // Scalar bypass read hit: short xbar path (RTL ≈ 3 cy), served outside the
            // coalescer/bank — does not contend for the per-set bank slot (treat as forwarded).
            base = scalar_hit_latency_cycles_;
            forwarded = true;
        } else if (use_forwarding_buffer_ &&
                   fwd_buffer_line_ == (int64_t)addr_line(addr)) {
            base = (fwd_hit_latency_cycles_ >= 0) ? fwd_hit_latency_cycles_
                                                  : hit_latency_cycles_;
            forwarded = true;
        } else if (streaming_hit_latency_cycles_ >= 0) {
            // Streaming read-hit pipelining. The RTL hit path has three decoupling registers
            // (coalescer req-spill, resp-spill, rsp_spliter/output-FIFO) that an isolated
            // access must fill in series (→ hit_latency_cycles_) but a back-to-back stream
            // keeps continuously occupied (→ streaming_hit_latency_cycles_). One register
            // drains per idle cycle, so the fill cost is the number drained since the last
            // read hit, capped at the depth difference — reproducing the RTL gap-sweep
            // (gap0→7, gap1→8, gap3→10). MemLatency-independent.
            const int64_t fill_max = (int64_t)hit_latency_cycles_ - streaming_hit_latency_cycles_;
            const int64_t gap = now - last_read_hit_cycle_ - 1;
            const int64_t drained = (gap <= 0) ? 0 : (gap < fill_max ? gap : fill_max);
            base = streaming_hit_latency_cycles_ + drained;
        }
        int64_t latency = base;
        if (line->ready_cycle > now) latency += (line->ready_cycle - now);
        // A forwarded read is served from the buffer registers, bypassing the SRAM bank, so it
        // does not contend for the per-set bank slot. Non-forwarded hits contend for the bank —
        // but the bank is PIPELINED: it accepts a new access to this set every bank_accept_cycles_
        // (≈1), each responding `latency` later, so back-to-back accesses to a hot/reused set
        // pipeline instead of serializing at the full latency. set_busy_until_[set] tracks the
        // cycle the bank is next free to ACCEPT; an access backed up beyond that pays the
        // queue-wait. (Advancing by the full latency — the pre-2026-06 behaviour — over-serialized
        // hot-set reuse and inflated real-kernel per-access latency ~6x.)
        if (!forwarded) {
            const int64_t accept_cycle = (set_busy_until_[set] > now) ? set_busy_until_[set] : now;
            latency += (accept_cycle - now);                       // queue-wait if bank backed up
            set_busy_until_[set] = accept_cycle + bank_accept_cycles_;
        }

        // This access caches its row in the 1-entry forwarding buffer.
        fwd_buffer_line_ = (int64_t)addr_line(addr);

        req->inc_latency(latency);

        if (!use_hash_way_select_) {
            auto &order = lru_order_[set];
            for (auto it = order.begin(); it != order.end(); ++it) {
                if (*it == way) { order.erase(it); break; }
            }
            order.insert(order.begin(), (uint8_t)way);
        }

        if (is_write) {
            line->dirty = true;
            cnt_wr_hit_++;
            // Write-through is only emitted in WriteThroughMode=1. In pure write-back
            // mode (the latest RTL default) the dirty bit alone records the write and
            // the line will be evicted later. See architecture_v2.md §0/§8.
            if (write_through_mode_) issue_write_through(req);
        } else {
            cnt_rd_hit_++;
            // A read hit keeps the hit pipeline's decoupling registers occupied; the next
            // read hit within the fill window inherits the streaming latency.
            last_read_hit_cycle_ = now;
        }
        return vp::IO_REQ_OK;
    }

    // --- hit on pending line (MSHR merge) ---
    if (line != nullptr && (line->state == LineState::READ_PEND ||
                             line->state == LineState::WRITE_PEND)) {
        // ENABLE_MULTI_READ_PEND: the latest RTL allows a per-line linked list of
        // pending reads (architecture_v2.md §7) — capacity is `num_ways_` per set, not
        // the retr_fifo. With the flag on we relax the gating so reads merging onto an
        // already-pending line bypass the global retr_fifo limit (they're cheap — just
        // a linked-list pointer update in the RTL). Writes still go through the normal
        // retr_fifo accounting because each write needs its dirty-merge byte tracking.
        const bool is_read_pend_merge =
            enable_multi_read_pend_ && !is_write && line->state == LineState::READ_PEND;
        if (!is_read_pend_merge && retr_fifo_level_ >= retr_fifo_depth_) {
            cnt_stall_mshr_full_++;
            return vp::IO_REQ_DENIED;
        }
        req->save();
        mshr_[set].push_back(MshrEntry{req, now});
        if (!is_read_pend_merge) retr_fifo_level_++;
        cnt_mshr_merge_++;
        if (is_write) cnt_wr_miss_++; else cnt_rd_miss_++;
        return vp::IO_REQ_PENDING;
    }

    // --- miss: allocate + refill ---
    if (miss_fifo_level_ >= miss_fifo_depth_) {
        cnt_stall_miss_fifo_++;
        return vp::IO_REQ_DENIED;
    }

    int victim_way = pick_victim(tag, set);
    CacheLine &vline = lines_[set * num_ways_ + victim_way];

    if (vline.state == LineState::VALID && vline.dirty) {
        if (evic_fifo_level_ >= evic_fifo_depth_) {
            cnt_stall_evic_fifo_++;
            return vp::IO_REQ_DENIED;
        }
        uint64_t old_line_addr =
            ((uint64_t)vline.tag << (line_bits_ + set_bits_)) |
            ((uint64_t)set       <<  line_bits_);
        issue_eviction((uint32_t)old_line_addr);
        cnt_evict_++;
        cnt_wb_dirty_++;
    }

    vline.tag = tag;
    vline.state = is_write ? LineState::WRITE_PEND : LineState::READ_PEND;
    vline.dirty = false;

    req->save();
    mshr_[set].push_back(MshrEntry{req, now});
    retr_fifo_level_++;
    miss_fifo_level_++;
    refill_in_flight_[set] = true;

    if (is_write) cnt_wr_miss_++; else cnt_rd_miss_++;

    issue_refill((uint32_t)addr_line(addr), set, victim_way);
    return vp::IO_REQ_PENDING;
}

// ---------- tag lookup ----------

CacheLine *InsituCacheController::lookup(uint32_t tag, uint32_t set, int *way_out)
{
    for (uint32_t w = 0; w < num_ways_; ++w) {
        CacheLine &ln = lines_[set * num_ways_ + w];
        if (ln.state != LineState::INVALID && ln.tag == tag) {
            *way_out = (int)w;
            return &ln;
        }
    }
    *way_out = -1;
    return nullptr;
}

// ---------- victim selection ----------

int InsituCacheController::pick_victim(uint32_t tag, uint32_t set)
{
    for (uint32_t w = 0; w < num_ways_; ++w) {
        if (lines_[set * num_ways_ + w].state == LineState::INVALID) return (int)w;
    }

    if (use_hash_way_select_) {
        uint32_t h = tag * 2654435761u;
        h ^= set * 0x9E3779B1u;
        return (int)(h % num_ways_);
    } else {
        return (int)lru_order_[set].back();
    }
}

// ---------- outgoing helpers ----------

void InsituCacheController::issue_write_through(vp::IoReq *user_req)
{
    if (!this->wt_itf_.is_bound()) return;
    wt_req_.init();
    wt_req_.set_addr(user_req->get_addr());
    wt_req_.set_size(user_req->get_size());
    wt_req_.set_is_write(true);
    // Downstream memory memcpys from the data pointer. Use our scratch buffer — the
    // perf model doesn't track byte values, so zeroed bytes are safe to forward.
    wt_req_.set_data(wt_data_buf_.data());
    (void)this->wt_itf_.req(&wt_req_);
    cnt_writes_through_++;
}

void InsituCacheController::issue_refill(uint32_t line_addr, uint32_t set, int way)
{
    (void)set; (void)way;
    refill_req_.init();
    refill_req_.set_addr(line_addr);
    refill_req_.set_size(cache_line_bytes_);
    refill_req_.set_is_write(false);
    // Downstream memory memcpys into the data pointer. Use our scratch buffer.
    refill_req_.set_data(refill_data_buf_.data());
    uint64_t beats = cache_line_bytes_ / refill_beat_bytes_;
    if (beats == 0) beats = 1;
    refill_req_.set_duration(beats);

    cnt_refills_issued_++;

    vp::IoReqStatus st = this->refill_itf_.req(&refill_req_);
    if (st == vp::IO_REQ_OK) {
        // Synchronous response path — the master's resp_meth was NOT invoked by the
        // target. Treat the same-cycle OK status as a refill completion right now.
        refill_resp_handler(this, &refill_req_);
    } else if (st != vp::IO_REQ_PENDING) {
        this->trace_.msg(vp::Trace::LEVEL_WARNING,
            "refill rejected (st=%d) addr=0x%x\n", (int)st, line_addr);
    }
}

void InsituCacheController::issue_eviction(uint32_t line_addr)
{
    if (!this->evict_itf_.is_bound()) return;
    evict_req_.init();
    evict_req_.set_addr(line_addr);
    evict_req_.set_size(cache_line_bytes_);
    evict_req_.set_is_write(true);
    evict_req_.set_data(evict_data_buf_.data());
    uint64_t beats = cache_line_bytes_ / refill_beat_bytes_;
    if (beats == 0) beats = 1;
    evict_req_.set_duration(beats);

    evic_fifo_level_++;
    (void)this->evict_itf_.req(&evict_req_);
    if (evic_fifo_level_ > 0) evic_fifo_level_--;  // single-slot model — phase 6 can refine

    // Occupancy: a dirty writeback shares the near-serial install/writeback pipeline with
    // refills, so it consumes a drain slot (refills + writebacks serialize on the same
    // resource). This is what makes a write-allocate-with-eviction stream (evict_dirty/
    // evict_wb) run at ~half the read-miss rate (RTL evict ≈ 0.177 vs cold_stream 0.243).
    // Folded victim read (folded_evict_penalty_cycles_) adds to the writeback's drain cost.
    if (defer_refills_ && refill_drain_cycles_ > 0) {
        const int64_t now = this->clock.get_cycles();
        const int64_t step = refill_drain_cycles_ + folded_evict_penalty_cycles_;
        (void)reserve_install_pipe(step, now + step);
    }
}

// ---------- refill response ----------

void InsituCacheController::refill_resp_handler(vp::Block *__this, vp::IoReq *req)
{
    InsituCacheController *_this = static_cast<InsituCacheController *>(__this);
    const int64_t now = _this->clock.get_cycles();

    // Recover set from the refill req's address. Find the pending way in that set
    // (exactly one way is in {READ_PEND, WRITE_PEND} with the matching tag).
    const uint32_t set = _this->addr_set(req->get_addr());
    const uint32_t tag = _this->addr_tag(req->get_addr());
    int pending_way = -1;
    for (uint32_t w = 0; w < _this->num_ways_; ++w) {
        CacheLine &ln = _this->lines_[set * _this->num_ways_ + w];
        if ((ln.state == LineState::READ_PEND || ln.state == LineState::WRITE_PEND) &&
            ln.tag == tag) {
            pending_way = (int)w;
            break;
        }
    }
    if (pending_way < 0) {
        _this->trace_.msg(vp::Trace::LEVEL_WARNING,
            "refill response with no matching pending line set=%u tag=0x%x\n", set, tag);
        return;
    }

    CacheLine &line = _this->lines_[set * _this->num_ways_ + pending_way];
    // Refill completion = memory latency (carried on the refill req) + bank-write +
    // the fixed cache-pipeline miss penalty (calibrated so a cold miss = MemLatency+17;
    // see prompt/insitu_cache_calib_report.md).
    int64_t refill_lat = req->get_full_latency() + _this->refill_bank_write_cycles_
                         + _this->miss_penalty_cycles_;
    // Occupancy: when refills pipeline (defer_refills_), the near-serial single-line
    // install pipeline drains at most one refill completion every refill_drain_cycles_.
    // Serialize completion cycles so a queued miss's completion (and thus the latency the
    // requester sees, t_resp = t_issue + full_latency) is pushed out under load — this is
    // what turns the flat-63 plateau into the RTL ramp + install-rate-bound throughput.
    // Head-of-line (isolated) miss is unaffected: the cyclestamp is in the past.
    if (_this->defer_refills_ && _this->refill_drain_cycles_ > 0) {
        int64_t completion = _this->reserve_install_pipe(_this->refill_drain_cycles_,
                                                         now + refill_lat);
        refill_lat = completion - now;
    }
    line.state = LineState::VALID;
    line.ready_cycle = now + refill_lat;
    _this->refill_in_flight_[set] = false;
    if (_this->miss_fifo_level_ > 0) _this->miss_fifo_level_--;

    _this->trace_.msg(vp::Trace::LEVEL_DEBUG,
        "refill done set=%u way=%d tag=0x%x ready_cycle=%ld\n",
        set, pending_way, tag, (long)line.ready_cycle);

    _this->fsm_drain_mshr(set);
}

// ---------- MSHR drain ----------

void InsituCacheController::fsm_drain_mshr(uint32_t set)
{
    auto &queue = mshr_[set];
    int subarray_idx = 0;
    int64_t prev_arrival = -1;   // same-cycle readers coalesce → share a drain subarray

    while (!queue.empty()) {
        MshrEntry entry = queue.front();
        queue.pop_front();
        if (retr_fifo_level_ > 0) retr_fifo_level_--;
        // The RTL par_coalescer merges same-cycle same-line reads into ONE entry, so they
        // retire together — only advance the per-subarray drain stagger for a reader that
        // arrived in a LATER cycle than the previous one (not per pending reader).
        if (prev_arrival >= 0 && entry.arrival_cycle != prev_arrival) subarray_idx++;
        prev_arrival = entry.arrival_cycle;

        vp::IoReq *req = entry.req;
        req->restore();

        // re-derive set/way now that state is valid
        uint64_t addr = req->get_addr();
        uint32_t s = addr_set(addr);
        uint32_t tag = addr_tag(addr);
        int way = -1;
        CacheLine *line = lookup(tag, s, &way);

        int64_t base_latency = 0;
        if (line != nullptr && line->ready_cycle > entry.arrival_cycle) {
            base_latency = line->ready_cycle - entry.arrival_cycle;
        } else {
            base_latency = hit_latency_cycles_;
        }
        base_latency += (int64_t)subarray_idx * mshr_drain_cycles_per_subarray_;

        req->inc_latency(base_latency);

        if (req->get_is_write() && line != nullptr) {
            line->dirty = true;
            // Same gating as on the synchronous write-hit path above: only emit on the
            // write-through path when WriteThroughMode=1 (see architecture_v2.md §0/§8).
            if (write_through_mode_) issue_write_through(req);
        }

        req->get_resp_port()->resp(req);
    }
}

// ---------- Module entry point ----------

extern "C" vp::Component *gv_new(vp::ComponentConf &config)
{
    return new InsituCacheController(config);
}
