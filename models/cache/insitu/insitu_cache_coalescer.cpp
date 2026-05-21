// SPDX-FileCopyrightText: 2026 ETH Zurich and University of Bologna
//
// SPDX-License-Identifier: Apache-2.0
//
// InSitu Cache write-through coalescer — performance model.
//
// Implements the 3-state FSM described in prompt/insitu_cache_architecture.md §10:
//
//   IDLE ──(first write)───► WRITE_COAL ──(same-tag write)──► WRITE_COAL (merge)
//                              │                               │
//                              │  (new-tag write, watchdog, or │
//                              │   read-snoop match)           │
//                              ▼                               │
//                           FLUSH ───(out bus ready)──► IDLE ◄──┘
//
// The flush emits a single wide (full-line) IoReq downstream and resets the FSM.
// Word-level merging is modeled only for counters — we don't actually carry data bytes.

#include <cstdint>
#include <cstdio>
#include <cstring>

#include <vp/vp.hpp>
#include <vp/itf/io.hpp>

enum class CoalState : uint8_t { IDLE, WRITE_COAL, FLUSH };

class InsituCacheCoalescer : public vp::Component
{
public:
    explicit InsituCacheCoalescer(vp::ComponentConf &conf);

    void reset(bool active) override;

private:
    static vp::IoReqStatus input_req_handler(vp::Block *__this, vp::IoReq *req);
    static vp::IoReqStatus snoop_req_handler(vp::Block *__this, vp::IoReq *req);
    static void watchdog_handler(vp::Block *__this, vp::ClockEvent *event);

    void flush_line();
    uint64_t line_base(uint64_t addr) const
    {
        return addr & ~((uint64_t)cache_line_bytes_ - 1);
    }

    // Config
    uint32_t cache_line_bytes_;
    int32_t  watchdog_cycles_;

    // Ports
    vp::IoSlave  input_itf_;
    vp::IoSlave  snoop_itf_;
    vp::IoMaster out_itf_;

    // FSM state
    CoalState state_ = CoalState::IDLE;
    uint64_t coal_tag_ = 0;                 // current coalesced line base address
    uint64_t coal_mask_ = 0;                // byte mask (OR of all merged writes)
    uint32_t coal_writes_ = 0;              // number of merged writes into this burst

    // Watchdog event (ticks every cycle while coalescing)
    vp::ClockEvent *watchdog_event_ = nullptr;
    int32_t watchdog_cnt_ = 0;

    // Outgoing req + scratch data buffer. Downstream memory memcpys from this pointer
    // on the wide flush write; we don't track byte values in the perf model so a zeroed
    // buffer is sufficient.
    vp::IoReq out_req_;
    std::vector<uint8_t> out_data_buf_;

    // Telemetry
    vp::Trace trace_;
    uint64_t cnt_writes_absorbed_ = 0;
    uint64_t cnt_flushes_new_tag_ = 0;
    uint64_t cnt_flushes_watchdog_ = 0;
    uint64_t cnt_flushes_snoop_ = 0;
    uint64_t cnt_merged_bursts_ = 0;
};

InsituCacheCoalescer::InsituCacheCoalescer(vp::ComponentConf &conf)
    : vp::Component(conf)
{
    auto *cfg = this->get_js_config();
    cache_line_bytes_ = cfg->get_child_int("cache_line_bytes");
    watchdog_cycles_  = cfg->get_child_int("watchdog_cycles");

    out_data_buf_.assign(cache_line_bytes_, 0);

    this->input_itf_.set_req_meth(&InsituCacheCoalescer::input_req_handler);
    this->new_slave_port("input", &this->input_itf_);

    this->snoop_itf_.set_req_meth(&InsituCacheCoalescer::snoop_req_handler);
    this->new_slave_port("read_snoop", &this->snoop_itf_);

    this->new_master_port("out", &this->out_itf_);

    this->traces.new_trace("trace", &this->trace_, vp::DEBUG);
    this->watchdog_event_ = this->event_new(InsituCacheCoalescer::watchdog_handler);

    this->trace_.msg(vp::Trace::LEVEL_INFO,
        "InsituCacheCoalescer instantiated (line=%u B, watchdog=%d cycles)\n",
        cache_line_bytes_, watchdog_cycles_);
}

void InsituCacheCoalescer::reset(bool active)
{
    if (active) {
        state_ = CoalState::IDLE;
        coal_tag_ = 0;
        coal_mask_ = 0;
        coal_writes_ = 0;
        watchdog_cnt_ = 0;
    }
}

// ---------- main input path ----------

vp::IoReqStatus InsituCacheCoalescer::input_req_handler(vp::Block *__this, vp::IoReq *req)
{
    InsituCacheCoalescer *_this = static_cast<InsituCacheCoalescer *>(__this);

    const uint64_t addr = req->get_addr();
    const uint64_t line = _this->line_base(addr);
    const bool is_write = req->get_is_write();

    _this->trace_.msg(vp::Trace::LEVEL_TRACE,
        "input addr=0x%lx wr=%d state=%d coal_tag=0x%lx\n",
        (unsigned long)addr, (int)is_write, (int)_this->state_,
        (unsigned long)_this->coal_tag_);

    if (!is_write) {
        // Read at the coalescer input is unusual (reads go through the cache itself). If
        // it happens, flush pending coalesce and forward. No performance credit.
        if (_this->state_ == CoalState::WRITE_COAL && line == _this->coal_tag_) {
            _this->cnt_flushes_snoop_++;
            _this->flush_line();
        }
        return _this->out_itf_.req_forward(req);
    }

    // Write path
    if (_this->state_ == CoalState::IDLE) {
        _this->state_ = CoalState::WRITE_COAL;
        _this->coal_tag_ = line;
        _this->coal_mask_ = 0;
        _this->coal_writes_ = 1;
        _this->cnt_writes_absorbed_++;
    } else if (line == _this->coal_tag_) {
        // Same line — merge.
        _this->coal_writes_++;
        _this->cnt_writes_absorbed_++;
    } else {
        // Different line — flush current, start a new coalesce.
        _this->cnt_flushes_new_tag_++;
        _this->flush_line();
        _this->state_ = CoalState::WRITE_COAL;
        _this->coal_tag_ = line;
        _this->coal_mask_ = 0;
        _this->coal_writes_ = 1;
        _this->cnt_writes_absorbed_++;
    }

    // Mark bytes touched (for reporting). Only valid if req->get_size() fits within the
    // line — we don't split requests that straddle a line boundary.
    uint64_t req_off = addr - line;
    uint64_t req_size = req->get_size();
    for (uint64_t b = 0; b < req_size && (req_off + b) < _this->cache_line_bytes_; ++b) {
        _this->coal_mask_ |= (1ULL << (req_off + b));
    }

    // (Re)start watchdog.
    _this->watchdog_cnt_ = _this->watchdog_cycles_;
    if (!_this->watchdog_event_->is_enqueued()) {
        _this->event_enqueue(_this->watchdog_event_, 1);
    }

    // Writes are fire-and-forget from the controller's POV — ack immediately with a small
    // per-merge latency to cap infinite batching.
    req->inc_latency(1);
    return vp::IO_REQ_OK;
}

// ---------- read-snoop path ----------

vp::IoReqStatus InsituCacheCoalescer::snoop_req_handler(vp::Block *__this, vp::IoReq *req)
{
    InsituCacheCoalescer *_this = static_cast<InsituCacheCoalescer *>(__this);
    const uint64_t addr = req->get_addr();
    const uint64_t line = _this->line_base(addr);

    if (_this->state_ == CoalState::WRITE_COAL && line == _this->coal_tag_) {
        _this->trace_.msg(vp::Trace::LEVEL_DEBUG,
            "snoop flush addr=0x%lx\n", (unsigned long)addr);
        _this->cnt_flushes_snoop_++;
        _this->flush_line();
    }
    // We don't forward the snoop — it's purely informational.
    return vp::IO_REQ_OK;
}

// ---------- watchdog tick ----------

void InsituCacheCoalescer::watchdog_handler(vp::Block *__this, vp::ClockEvent *event)
{
    (void)event;
    InsituCacheCoalescer *_this = static_cast<InsituCacheCoalescer *>(__this);

    if (_this->state_ != CoalState::WRITE_COAL) return;

    if (_this->watchdog_cnt_ > 0) _this->watchdog_cnt_--;

    if (_this->watchdog_cnt_ == 0) {
        _this->cnt_flushes_watchdog_++;
        _this->flush_line();
    } else {
        _this->event_enqueue(_this->watchdog_event_, 1);
    }
}

// ---------- flush ----------

void InsituCacheCoalescer::flush_line()
{
    if (coal_writes_ == 0) {
        state_ = CoalState::IDLE;
        return;
    }
    cnt_merged_bursts_++;

    out_req_.init();
    out_req_.set_addr(coal_tag_);
    out_req_.set_size(cache_line_bytes_);
    out_req_.set_is_write(true);
    out_req_.set_data(out_data_buf_.data());

    if (this->out_itf_.is_bound()) {
        (void)this->out_itf_.req(&out_req_);
    }

    trace_.msg(vp::Trace::LEVEL_TRACE,
        "flush burst (line=0x%lx, merged=%u writes, mask=0x%lx)\n",
        (unsigned long)coal_tag_, coal_writes_, (unsigned long)coal_mask_);

    state_ = CoalState::IDLE;
    coal_tag_ = 0;
    coal_mask_ = 0;
    coal_writes_ = 0;
    watchdog_cnt_ = 0;
}

extern "C" vp::Component *gv_new(vp::ComponentConf &config)
{
    return new InsituCacheCoalescer(config);
}
