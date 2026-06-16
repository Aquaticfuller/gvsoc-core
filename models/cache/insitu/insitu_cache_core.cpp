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
#include <cstring>

#include <vp/vp.hpp>
#include <vp/itf/io.hpp>

#include "insitu_cache_decode.hpp"
#include "insitu_cache_bank_array.hpp"

using namespace insitu;

class InsituCacheCore : public vp::Component
{
public:
    explicit InsituCacheCore(vp::ComponentConf &conf);
    void reset(bool active) override;

private:
    static vp::IoReqStatus req_handler(vp::Block *__this, vp::IoReq *req);
    static void refill_resp_handler(vp::Block *__this, vp::IoReq *req);
    static void tick(vp::Block *__this, vp::ClockEvent *event);

    void schedule_tick();
    void stage1_process();                 // decode + FSM + bank write
    void process_request(vp::IoReq *req);  // REQ_PROC body
    void install_refill();                 // refill block
    void drain_outputs();                  // one beat per output FIFO
    void stage0_arbitrate();               // pick next preread task
    bool any_work() const;

    // --- functional data (RETAINED from controller.cpp:118-130) ---
    inline void exchange_line_data(vp::IoReq *req, uint32_t set, int way, bool line_to_req) {
        if (req->get_data() == nullptr) return;
        uint32_t off = (uint32_t)(req->get_addr() & (cache_line_bytes_ - 1));
        uint32_t n = (uint32_t)req->get_size();
        if (off + n > cache_line_bytes_) n = cache_line_bytes_ - off;
        uint8_t *line = &data_[((size_t)set * num_ways_ + (uint32_t)way) * cache_line_bytes_ + off];
        if (line_to_req) memcpy(req->get_data(), line, n);
        else             memcpy(line, req->get_data(), n);
    }
    void functional_write_mem(vp::IoReq *user_req) {
        if (!functional_writethrough_ || !evict_itf_.is_bound() || user_req->get_data() == nullptr) return;
        funcwr_req_.init();
        funcwr_req_.set_addr(user_req->get_addr());
        funcwr_req_.set_size(user_req->get_size());
        funcwr_req_.set_is_write(true);
        funcwr_req_.set_data(user_req->get_data());
        (void)evict_itf_.req(&funcwr_req_);
    }

    uint64_t addr_line(uint64_t a) const { return a & ~((uint64_t)cache_line_bytes_ - 1); }
    WayMeta *set_ways(uint32_t set) { return &meta_[(size_t)set * num_ways_]; }

    // --- config / geometry ---
    uint32_t cache_line_bytes_, num_ways_, num_sets_;
    CacheGeom geom_;
    bool functional_writethrough_;
    int32_t miss_penalty_cycles_, refill_bank_write_cycles_;
    uint32_t retr_fifo_depth_, miss_fifo_depth_, evic_fifo_depth_;

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

    // stage-0→stage-1 pipeline register
    struct PrereadTask { bool valid=false; bool is_refill=false; vp::IoReq *req=nullptr;
                         uint64_t addr=0; bool is_write=false; } preread_q_;

    // in-situ MSHR: per pending line (set,way) the queued readers (+ a deferred write merge req)
    std::vector<std::deque<vp::IoReq*>> mshr_;   // [num_sets*num_ways]

    // output FIFOs (structural, capacity-bounded)
    std::deque<vp::IoReq*> resp_fifo_;            // completed reads/writes to resp() (retr+resp merged)
    std::deque<uint64_t>   miss_fifo_;            // line addrs needing a refill
    std::deque<uint64_t>   evic_fifo_;            // dirty line addrs to write back
    uint32_t retr_level_ = 0;

    // single-outstanding refill
    bool refill_pending_ = false;
    bool refill_spill_valid_ = false;
    uint64_t pending_refill_addr_ = 0;

    // ports
    vp::IoSlave  input_itf_;
    vp::IoMaster refill_itf_;
    vp::IoMaster evict_itf_;
    vp::IoSlave  flush_itf_;
    vp::IoReq    refill_req_, evict_req_, funcwr_req_;
    std::vector<uint8_t> refill_data_buf_, evict_data_buf_;

    vp::ClockEvent *tick_event_ = nullptr;
    vp::Trace trace_;
    // telemetry
    uint64_t cnt_rd_hit_=0, cnt_wr_hit_=0, cnt_rd_miss_=0, cnt_wr_miss_=0, cnt_mshr_merge_=0,
             cnt_refill_=0, cnt_evict_=0, cnt_bank_conflict_=0;
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
    flush_itf_.set_req_meth(&InsituCacheCore::req_handler);   // flush TODO (Step 6); accept-as-OK stub
    new_slave_port("flush", &flush_itf_);
    refill_itf_.set_resp_meth(&InsituCacheCore::refill_resp_handler);
    new_master_port("refill", &refill_itf_);
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
    resp_fifo_.clear(); miss_fifo_.clear(); evic_fifo_.clear();
    in_q_.clear(); preread_q_ = PrereadTask{};
    refill_pending_ = refill_spill_valid_ = false; retr_level_ = 0;
}

void InsituCacheCore::schedule_tick()
{
    if (!tick_event_->is_enqueued()) event_enqueue(tick_event_, 1);
}

bool InsituCacheCore::any_work() const
{
    return preread_q_.valid || !in_q_.empty() || refill_spill_valid_ ||
           !resp_fifo_.empty() || !miss_fifo_.empty() || !evic_fifo_.empty();
}

// ---------- stage-0 admission ----------
vp::IoReqStatus InsituCacheCore::req_handler(vp::Block *__this, vp::IoReq *req)
{
    InsituCacheCore *_this = static_cast<InsituCacheCore *>(__this);
    if (_this->in_q_.size() >= _this->in_q_cap_) return vp::IO_REQ_DENIED;  // accept queue full → backpressure
    req->save();
    _this->in_q_.push_back(req);
    _this->schedule_tick();
    return vp::IO_REQ_PENDING;
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

    if (_this->any_work()) _this->schedule_tick();
}

void InsituCacheCore::stage1_process()
{
    if (!preread_q_.valid) return;
    if (preread_q_.is_refill) install_refill();
    else process_request(preread_q_.req);
    preread_q_.valid = false;
}

void InsituCacheCore::process_request(vp::IoReq *req)
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
            preread_q_.valid = true; preread_q_.req = req; preread_q_.is_refill = false;  // retry next tick
            preread_q_.addr = addr; preread_q_.is_write = is_write;
            return;
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
        return;
    }

    // hit on a pending line (in-situ MSHR merge)
    if (d.is_hit_pend) {
        mshr_[(size_t)set*num_ways_ + d.way].push_back(req);
        cnt_mshr_merge_++;
        if (is_write) cnt_wr_miss_++; else cnt_rd_miss_++;
        return;
    }

    // conflict / all-pend → stall by retrying next tick (functional stall; full FSM enum is TODO)
    if (d.is_hit_conflit || d.is_all_pend) {
        preread_q_.valid = true; preread_q_.req = req; preread_q_.is_refill = false;
        preread_q_.addr = addr; preread_q_.is_write = is_write;
        return;
    }

    // miss: allocate victim + refill
    if (miss_fifo_.size() >= miss_fifo_depth_ || retr_level_ >= retr_fifo_depth_) {
        preread_q_.valid = true; preread_q_.req = req; preread_q_.is_refill = false;  // FIFO full → retry
        preread_q_.addr = addr; preread_q_.is_write = is_write;
        return;
    }
    uint32_t vw = d.way;
    WayMeta &vline = ways[vw];
    if (vline.status == VALID && vline.dirty) {
        if (evic_fifo_.size() >= evic_fifo_depth_) {
            preread_q_.valid = true; preread_q_.req = req; preread_q_.is_refill = false;
            preread_q_.addr = addr; preread_q_.is_write = is_write;
            return;
        }
        uint64_t old_line = ((uint64_t)vline.tag << (geom_.off_bits + geom_.depth_bits)) |
                            ((uint64_t)set << geom_.off_bits);
        evic_fifo_.push_back(old_line);
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
    if (refill_req_.get_data() != nullptr)
        memcpy(&data_[((size_t)set*num_ways_+(uint32_t)way)*cache_line_bytes_], refill_req_.get_data(), cache_line_bytes_);
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
    }
}

void InsituCacheCore::drain_outputs()
{
    // one completed access resp() per tick
    if (!resp_fifo_.empty()) {
        vp::IoReq *r = resp_fifo_.front(); resp_fifo_.pop_front();
        r->get_resp_port()->resp(r);
    }
    // one refill issue per tick, single-outstanding
    if (!refill_pending_ && !refill_spill_valid_ && !miss_fifo_.empty()) {
        uint64_t line = miss_fifo_.front(); miss_fifo_.pop_front();
        pending_refill_addr_ = line;
        refill_req_.init();
        refill_req_.set_addr(line);
        refill_req_.set_size(cache_line_bytes_);
        refill_req_.set_is_write(false);
        refill_req_.set_data(refill_data_buf_.data());
        refill_pending_ = true;
        vp::IoReqStatus st = refill_itf_.req(&refill_req_);
        if (st == vp::IO_REQ_OK) { refill_spill_valid_ = true; refill_pending_ = false; }  // sync responder
    }
    // one eviction issue per tick
    if (!evic_fifo_.empty()) {
        uint64_t line = evic_fifo_.front(); evic_fifo_.pop_front();
        evict_req_.init();
        evict_req_.set_addr(line);
        evict_req_.set_size(cache_line_bytes_);
        evict_req_.set_is_write(true);
        evict_req_.set_data(evict_data_buf_.data());
        (void)evict_itf_.req(&evict_req_);
    }
}

void InsituCacheCore::stage0_arbitrate()
{
    if (preread_q_.valid) return;   // stage-1 re-queued a stall this tick
    // refill spill has priority (install it next), if there is retr room
    if (refill_spill_valid_) {
        preread_q_.valid = true; preread_q_.is_refill = true; preread_q_.req = nullptr;
        return;
    }
    if (!in_q_.empty()) {
        vp::IoReq *r = in_q_.front(); in_q_.pop_front();
        preread_q_.valid = true; preread_q_.is_refill = false; preread_q_.req = r;
        preread_q_.addr = r->get_addr(); preread_q_.is_write = r->get_is_write();
    }
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
