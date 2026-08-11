// SPDX-FileCopyrightText: 2026 ETH Zurich and University of Bologna
// SPDX-License-Identifier: Apache-2.0
//
// InSitu cache — AMO / LR-SC shim (spatz_cache_amo), instanced on the scalar lane (j=4) of each cell.
//
// Wraps the validated `insitu_cache_amo.hpp` (ALU + LR/SC reservation) in an event shell that sits
// between the tile's scalar-lane crossbar and the cache core. The RTL puts spatz_cache_amo only on lane
// NrTCDMPortsPerCore-1 (cachepool_tile.sv:658), one per controller; the Spatz VLSU lanes bypass it.
//
// Handling by IoReqOpcode (engine io.hpp: READ=0 WRITE=1 LR=2 SC=3 SWAP=4 ADD=5 XOR=6 AND=7 OR=8 MIN=9
// MAX=10 MINU=11 MAXU=12):
//   - READ / WRITE  → pass straight through to the core (req_forward; response auto-routes upstream).
//                     A WRITE to the reserved address clears the reservation.
//   - LR            → set the reservation, present as a plain READ to the core (the load value returns).
//   - SC            → success (reservation valid + addr match): write the store to the core, then return
//                     0; failure: return 1 WITHOUT writing. (RISC-V SC semantics.)
//   - true AMO      → read-modify-write: read the line word from the core, compute amo_alu(op, old, b),
//                     write the result back, then return the OLD value. The lane is atomic: the scalar
//                     LSU is single-outstanding, so at most one AMO/SC is in flight (no queue needed).
//
// SCOPE: the open-loop calib issues only READ/WRITE, so this validates the PASS-THROUGH path
// (data_err=0, scalar lane unchanged). The LR/SC/AMO paths use the standalone-validated amo.hpp and are
// exercised closed-loop (Spatz). Cross-LANE reservation clearing (a VLSU write to the reserved addr,
// which bypasses this shim) and the DMA-vs-AMO conflict detector are deferred (the reservation here is
// per-bank: same-address LR/SC route to the same bank → same shim, so same-address LR/SC are correct).

#include <cstdint>
#include <cstring>

#include <cstdlib>
#include <deque>

#include <vp/vp.hpp>
#include <vp/itf/io.hpp>

#include "insitu_cache_amo.hpp"

using namespace insitu;

namespace {
// A/B switch for the concurrent-atomic park queue: INSITU_AMO_PARK=0 restores the pre-park
// behaviour (a second atomic overwrites the in-flight RMW state). Diagnostic only.
// INSITU_AMO_DEBUG=1 traces the first N RMW state transitions to stderr (async bring-up).
inline int amo_dbg()
{
    static const int v = [](){ const char *e = getenv("INSITU_AMO_DEBUG"); return e ? atoi(e) : 0; }();
    return v;
}
// The env value IS the line budget (INSITU_AMO_DEBUG=200000 to reach late cycles). A fixed budget
// here once made the shim look like it had gone silent after cycle 3886 when it had simply stopped
// printing.
inline int &amo_dbg_budget() { static int n = amo_dbg(); return n; }

inline bool park_enabled()
{
    static const bool v = [](){ const char *e = getenv("INSITU_AMO_PARK");
                                return !(e && e[0] == '0'); }();
    return v;
}

// engine IoReqOpcode → amo.hpp AmoOp (the two enums differ).
inline uint8_t opcode_to_amo(vp::IoReqOpcode op)
{
    switch (op) {
    case vp::LR:   return AMO_LR;
    case vp::SC:   return AMO_SC;
    case vp::SWAP: return AMO_SWAP;
    case vp::ADD:  return AMO_ADD;
    case vp::XOR:  return AMO_XOR;
    case vp::AND:  return AMO_AND;
    case vp::OR:   return AMO_OR;
    case vp::MIN:  return AMO_MIN;
    case vp::MAX:  return AMO_MAX;
    case vp::MINU: return AMO_MINU;
    case vp::MAXU: return AMO_MAXU;
    default:       return AMO_NONE;
    }
}
}

class InsituCacheAmo : public vp::Component
{
public:
    explicit InsituCacheAmo(vp::ComponentConf &conf);
    void stop() override {
        fprintf(stderr, "[INSITU-AMO %s] rmw=%lu lat_sum=%lu\n", this->get_path().c_str(),
                (unsigned long)n_rmw_, (unsigned long)lat_rmw_sum_);
        vp::Component::stop();
    }

private:
    static vp::IoReqStatus req_handler(vp::Block *__this, vp::IoReq *req);
    static void resp_handler(vp::Block *__this, vp::IoReq *req);
    // req_handler's body, callable again when a parked request is re-issued.
    static vp::IoReqStatus handle(InsituCacheAmo *_this, vp::IoReq *req, bool parked);
    static void drain_park(InsituCacheAmo *_this);
    // the occupancy window expired — release whatever queued behind it
    static void occ_handler(vp::Block *__this, vp::ClockEvent *event);

    Reservation res_;
    uint32_t word_bytes_;

    // ONE in-flight AMO/SC transaction — one RMW unit per bank, as in the RTL (spatz_cache_amo.sv holds
    // core_ready=0 for the whole DoAMO/WriteBackAMO/Wait window). With the synchronous-slave cache the
    // whole RMW resolves inside req_handler, so phase_ is always IDLE on entry and nothing ever queues.
    // With an ASYNC cache the sub-ops return PENDING and phase_ stays AMO_READ across ticks — a second
    // AMO arriving then overwrote phase_/orig_/scratch_/amo_addr_ and the two RMWs completed into each
    // other's result buffers (load-store_M16 reported `exp 0x3 got 0x2` plus its mirror image). Requests
    // that arrive on a busy lane are therefore parked here and re-issued when the lane frees, which is
    // also what the hardware does with core_ready.
    enum Phase { IDLE, AMO_READ, AMO_WRITE, SC_WRITE };
    Phase    phase_ = IDLE;
    std::deque<vp::IoReq*> park_q_;
    uint64_t dbg_watch_addr_ = 0;   // first atomic's address (debug)
    // Synchronous-slave cache (cachepool run_request_sync): the sub-read/sub-write to the core complete
    // in-call (req returns IO_REQ_OK, not PENDING), so the whole RMW/SC resolves INSIDE req_handler. In that
    // case the shim must return IO_REQ_OK (the result is already written into the upstream req's data) and
    // must NOT call resp() — calling resp() and then returning PENDING double-completes the request (SIGABRT).
    // On the async cache (Spatz controller) the sub-ops return PENDING and resp() fires later (PENDING path).
    bool     in_sync_call_ = false;
    bool     sync_completed_ = false;
    vp::IoReq *orig_ = nullptr;   // the held upstream AMO/SC request
    uint8_t  amo_op_ = AMO_NONE;
    uint64_t amo_addr_ = 0;
    uint32_t old_val_ = 0, b_val_ = 0;
    uint8_t  scratch_buf_[8];     // value buffer for the scratch read/write
    vp::IoReq scratch_;

    // B3 RMW lane occupancy (spatz_cache_amo.sv: core_ready=0 in DoAMO/WriteBackAMO/Wait — the
    // bank-shared scalar lane is HELD from RMW accept until the write-back drains, ~15-20 cy on a
    // hit, full refill time on a miss). rmw_busy_until_ = the cycle the window ends; any new request
    // on the lane (any opcode) waits it out. rmw_write_rtt_cycles_ = the write-back RTT knob.
    // structural_occupancy_: hold the lane in real simulated time for the whole RMW window instead of
    // stamping the wait on arrivals. With an async cache the stamp is discarded by the requester, so
    // only real blocking reproduces the RTL's core_ready=0 behaviour.
    bool     structural_occupancy_ = false;
    // With structural occupancy the lane is held for an ABSOLUTE window measured from RMW accept,
    // modelling the RTL's core_ready=0 span (spatz_cache_amo.sv, ~15-20 cycles on a hit). It must not
    // be additive: the sub-read and sub-write already consume real simulated time on the async path
    // (about 10 cycles each once resp_latency_cycles is calibrated), so adding a further tail on top
    // charged ~28 cycles per RMW and over-predicted spin-lock by 19.8%.
    int32_t  rmw_window_cycles_ = 18;
    int64_t  rmw_start_cyc_ = 0;
    vp::ClockEvent occ_event_;
    int64_t  rmw_busy_until_ = 0;
    int64_t  rmw_read_lat_ = 0;   // the scratch read's latency (captured before scratch_ re-init)
    int32_t  rmw_write_rtt_cycles_ = 8;

    vp::IoSlave  input_;
    vp::IoMaster output_;
    vp::Trace trace_;
    // diagnostics: op count + total stamped latency (dumped at stop())
    uint64_t n_rmw_ = 0, lat_rmw_sum_ = 0;
};

InsituCacheAmo::InsituCacheAmo(vp::ComponentConf &conf)
: vp::Component(conf), occ_event_(this, &InsituCacheAmo::occ_handler)
{
    auto *cfg = this->get_js_config();
    word_bytes_ = cfg->get_child_int("word_bytes");
    if (word_bytes_ < 1) word_bytes_ = 4;
    // B3: the write-back RTT of an RMW (the part after the scratch read). -1 → default 8
    // (read-hit ~10 + 1 + 8 ≈ 19 cy end-to-end, matching the RTL's ~15-20 cy).
    int32_t wrtt = cfg->get_child_int("amo_rmw_write_rtt_cycles");
    if (wrtt >= 0) rmw_write_rtt_cycles_ = wrtt;

    structural_occupancy_ = cfg->get("structural_occupancy")
        ? cfg->get_child_bool("structural_occupancy") : false;
    if (cfg->get("amo_rmw_window_cycles")) {
        const int32_t w = cfg->get_child_int("amo_rmw_window_cycles");
        if (w >= 0) rmw_window_cycles_ = w;
    }
    if (const char *e = getenv("INSITU_AMO_WINDOW")) rmw_window_cycles_ = atoi(e);

    input_.set_req_meth(&InsituCacheAmo::req_handler);
    this->new_slave_port("input", &input_);
    output_.set_resp_meth(&InsituCacheAmo::resp_handler);
    this->new_master_port("out", &output_);

    this->traces.new_trace("trace", &this->trace_, vp::DEBUG);
    this->trace_.msg(vp::Trace::LEVEL_INFO, "InsituCacheAmo word=%u\n", word_bytes_);
}

vp::IoReqStatus InsituCacheAmo::req_handler(vp::Block *__this, vp::IoReq *req)
{
    InsituCacheAmo *_this = static_cast<InsituCacheAmo *>(__this);
    // Lane busy with an RMW: EVERY new request waits, whatever its opcode. This is what makes the
    // read-modify-write atomic, and it is what `core_ready = 0` does in the RTL (spatz_cache_amo.sv
    // holds it for the whole DoAMO/WriteBackAMO/Wait window).
    //
    // Parking only atomics is not enough. A plain store that slips between an RMW's read and its
    // write-back is LOST, because the write-back then rewrites the pre-store value: the spin-lock
    // kernel deadlocked with every amoswap returning old=0x1 forever, since the holder's release
    // store (lock = 0) was overwritten by an in-flight swap's write-back (lock = 1) and the lock was
    // never released again. On a synchronous-slave cache the whole RMW resolves inside req_handler so
    // phase_ is always IDLE here and nothing can interleave; the async path made the window real.
    const bool lane_busy = (_this->phase_ != IDLE) ||
        (_this->structural_occupancy_ && _this->clock.get_cycles() < _this->rmw_busy_until_);
    if (lane_busy && park_enabled()) {
        _this->park_q_.push_back(req);
        if (_this->structural_occupancy_ && _this->phase_ == IDLE) {
            // nothing else will wake us: schedule the drain for when the window ends
            const int64_t d = _this->rmw_busy_until_ - _this->clock.get_cycles();
            if (!_this->occ_event_.is_enqueued() && d > 0) _this->occ_event_.enqueue(d);
        }
        return vp::IO_REQ_PENDING;
    }
    vp::IoReqStatus st = handle(_this, req, /*parked=*/false);
    // If that RMW resolved synchronously, release whatever queued behind it — resp_handler's drain
    // only runs on the async completion path.
    if (_this->phase_ == IDLE) drain_park(_this);
    return st;
}

// Drain parked requests once the lane is idle. We already answered PENDING upstream, so a request that
// now resolves synchronously (OK) is ours to resp(); a PENDING one is completed by the normal path.
void InsituCacheAmo::occ_handler(vp::Block *__this, vp::ClockEvent *event)
{
    InsituCacheAmo *_this = static_cast<InsituCacheAmo *>(__this);
    drain_park(_this);
}

void InsituCacheAmo::drain_park(InsituCacheAmo *_this)
{
    while (_this->phase_ == IDLE && !_this->park_q_.empty()) {
        vp::IoReq *r = _this->park_q_.front();
        _this->park_q_.pop_front();
        if (handle(_this, r, /*parked=*/true) == vp::IO_REQ_OK) r->get_resp_port()->resp(r);
    }
}

vp::IoReqStatus InsituCacheAmo::handle(InsituCacheAmo *_this, vp::IoReq *req, bool parked)
{
    const vp::IoReqOpcode op = req->get_opcode();
    const uint64_t addr = req->get_addr();
    const uint32_t core = (uint32_t)req->get_initiator();

    // B3: a new request on this lane (any opcode) waits out the previous RMW's occupancy window.
    // Skipped for a request that was parked: it already waited in real simulated time, so stamping
    // the window on top would charge the same serialization twice.
    const int64_t now = _this->clock.get_cycles();
    if (!parked && now < _this->rmw_busy_until_) req->inc_latency(_this->rmw_busy_until_ - now);

    // --- plain accesses: pass through ---
    if (op == vp::READ) {
        return _this->output_.req_forward(req);
    }
    if (op == vp::WRITE) {
        if (amo_dbg() && amo_dbg_budget() > 0 && addr == _this->dbg_watch_addr_) {
            amo_dbg_budget()--;
            uint32_t v = 0; if (req->get_data()) memcpy(&v, req->get_data(), req->get_size() < 4 ? req->get_size() : 4);
            fprintf(stderr, "[AMO %s] cyc=%ld PLAIN WRITE addr=0x%lx size=%u val=0x%x parked=%d\n",
                    _this->get_path().c_str(), (long)_this->clock.get_cycles(),
                    (unsigned long)addr, req->get_size(), v, (int)parked);
        }
        _this->res_.on_foreign_access(core, addr, AMO_NONE, /*is_write=*/true);
        return _this->output_.req_forward(req);
    }

    // --- LR: set reservation, present as a plain READ (the load value returns to the core) ---
    if (op == vp::LR) {
        _this->res_.on_lr(core, addr);
        req->set_opcode(vp::READ);
        return _this->output_.req_forward(req);
    }

    // --- SC: success → write + return 0; failure → return 1, no write ---
    if (op == vp::SC) {
        const bool ok = _this->res_.on_sc(core, addr);
        if (!ok) {
            uint8_t *dst = req->get_second_data() != nullptr ? req->get_second_data() : req->get_data();
            if (dst != nullptr) { uint32_t one = 1; memcpy(dst, &one, _this->word_bytes_ <= 4 ? _this->word_bytes_ : 4); }
            return vp::IO_REQ_OK;   // SC fail: no memory write, synchronous response (data=1)
        }
        // success: write the store to the core, then return 0 on the write-back response.
        _this->rmw_start_cyc_ = _this->clock.get_cycles();
        _this->phase_ = SC_WRITE;
        _this->orig_  = req;
        _this->scratch_.init();
        _this->scratch_.set_addr(addr);
        _this->scratch_.set_size(req->get_size());
        _this->scratch_.set_is_write(true);
        _this->scratch_.set_data(req->get_data());   // the SC store data
        _this->in_sync_call_ = true; _this->sync_completed_ = false;
        vp::IoReqStatus st = _this->output_.req(&_this->scratch_);
        if (st == vp::IO_REQ_OK) { resp_handler(_this, &_this->scratch_); }
        _this->in_sync_call_ = false;
        return _this->sync_completed_ ? vp::IO_REQ_OK : vp::IO_REQ_PENDING;
    }

    // --- true AMO: read-modify-write ---
    if (_this->dbg_watch_addr_ == 0) _this->dbg_watch_addr_ = addr;   // first atomic seen = the lock
    _this->rmw_start_cyc_ = _this->clock.get_cycles();
    _this->phase_   = AMO_READ;
    _this->orig_    = req;
    _this->amo_op_  = opcode_to_amo(op);
    _this->amo_addr_= addr;
    memcpy(&_this->b_val_, req->get_data(), 4);       // the store operand
    _this->scratch_.init();
    _this->scratch_.set_addr(addr);
    _this->scratch_.set_size(req->get_size());
    _this->scratch_.set_is_write(false);
    _this->scratch_.set_data(_this->scratch_buf_);
    _this->in_sync_call_ = true; _this->sync_completed_ = false;
    vp::IoReqStatus st = _this->output_.req(&_this->scratch_);
    if (amo_dbg() && amo_dbg_budget() > 0) {
        amo_dbg_budget()--;
        fprintf(stderr, "[AMO %s] cyc=%ld RMW issue op=%d addr=0x%lx sub_read_st=%d park=%zu\n",
                _this->get_path().c_str(), (long)_this->clock.get_cycles(), (int)op,
                (unsigned long)addr, (int)st, _this->park_q_.size());
    }
    if (st == vp::IO_REQ_OK) { resp_handler(_this, &_this->scratch_); }
    _this->in_sync_call_ = false;
    return _this->sync_completed_ ? vp::IO_REQ_OK : vp::IO_REQ_PENDING;
}

void InsituCacheAmo::resp_handler(vp::Block *__this, vp::IoReq *req)
{
    InsituCacheAmo *_this = static_cast<InsituCacheAmo *>(__this);
    if (amo_dbg() && amo_dbg_budget() > 0) {
        amo_dbg_budget()--;
        fprintf(stderr, "[AMO %s] cyc=%ld resp phase=%d mine=%d\n", _this->get_path().c_str(),
                (long)_this->clock.get_cycles(), (int)_this->phase_, (int)(req == &_this->scratch_));
    }
    if (req != &_this->scratch_) return;   // not our scratch (shouldn't happen)

    if (_this->phase_ == AMO_READ) {
        memcpy(&_this->old_val_, _this->scratch_buf_, 4);
        if (amo_dbg() && amo_dbg_budget() > 0) {
            amo_dbg_budget()--;
            fprintf(stderr, "[AMO %s] cyc=%ld RMW old=0x%x operand=0x%x -> new=0x%x addr=0x%lx initiator=%d\n",
                    _this->get_path().c_str(), (long)_this->clock.get_cycles(), _this->old_val_,
                    _this->b_val_, amo_alu(_this->amo_op_, _this->old_val_, _this->b_val_),
                    (unsigned long)_this->amo_addr_,
                    _this->orig_ ? _this->orig_->get_initiator() : -1);
        }
        _this->rmw_read_lat_ = _this->scratch_.get_full_latency();   // B3: before scratch_ re-init
        const uint32_t newv = amo_alu(_this->amo_op_, _this->old_val_, _this->b_val_);
        memcpy(_this->scratch_buf_, &newv, 4);
        _this->phase_ = AMO_WRITE;
        _this->scratch_.init();
        _this->scratch_.set_addr(_this->amo_addr_);
        _this->scratch_.set_size(4);
        _this->scratch_.set_is_write(true);
        _this->scratch_.set_data(_this->scratch_buf_);
        vp::IoReqStatus st = _this->output_.req(&_this->scratch_);
        if (st == vp::IO_REQ_OK) { resp_handler(_this, &_this->scratch_); }
        return;
    }

    // AMO_WRITE done → return the OLD value; SC_WRITE done → return 0 (success).
    // CRITICAL: the ISS AMO convention (lsu.cpp:547-549, mirrored by memory.cpp's atomics) is operand =
    // get_data(), RESULT = get_second_data() (which aliases the destination register). The old value MUST be
    // written to get_second_data() — writing it to get_data() (the previous behaviour) left rd unchanged, so
    // the core's beqz/bnez acquire check read stale register garbage and the spin lock never resolved.
    vp::IoReq *orig = _this->orig_;
    uint8_t *result_dst = (orig != nullptr && orig->get_second_data() != nullptr)
                          ? orig->get_second_data() : (orig != nullptr ? orig->get_data() : nullptr);
    if (result_dst != nullptr) {
        if (_this->phase_ == AMO_WRITE) memcpy(result_dst, &_this->old_val_, 4);
        else /* SC_WRITE */            { uint32_t zero = 0; memcpy(result_dst, &zero, 4); }
    }
    _this->phase_ = IDLE;
    _this->orig_ = nullptr;
    if (orig != nullptr) {
        // B3: the requester sees the full RMW round trip (scratch read + 1 + write-back), and the
        // lane stays busy for the same window (core_ready=0 until the write-back drains). A scratch
        // read MISS already carries the refill latency, so a missing RMW costs ~refill+write ≈ the
        // RTL's "full refill time on a miss". SC has no separate read: write + RTT. The window
        // CHAINS: it starts after the previous RMW's window (the wait stamped at entry) — setting
        // busy = now + total would let overlapping windows shrink the serialization.
        const int64_t now = _this->clock.get_cycles();
        if (_this->structural_occupancy_) {
            // The requester already lived through the RMW in real time, so no stamp (it would be
            // discarded anyway). Hold the lane out to the absolute end of the RTL's core_ready window,
            // measured from accept — never beyond it, and never a moment less.
            const int64_t end = _this->rmw_start_cyc_ + _this->rmw_window_cycles_;
            _this->rmw_busy_until_ = (end > now) ? end : now;
        } else {
            const int64_t total = (_this->rmw_read_lat_ > 0)
                ? (_this->rmw_read_lat_ + 1 + _this->scratch_.get_full_latency())
                : (_this->scratch_.get_full_latency() + _this->rmw_write_rtt_cycles_);
            orig->inc_latency(total);
            _this->rmw_busy_until_ = (_this->rmw_busy_until_ > now ? _this->rmw_busy_until_ : now) + total;
        }
        _this->rmw_read_lat_ = 0;
        _this->n_rmw_++; _this->lat_rmw_sum_ += (uint64_t)orig->get_full_latency();
    }
    if (_this->in_sync_call_) {
        // resolved inside req_handler (sync cache): tell it to return IO_REQ_OK; do NOT resp() (the result
        // is already in orig's data buffer). resp()+PENDING would double-complete the upstream request.
        _this->sync_completed_ = true;
    } else if (orig != nullptr) {
        orig->get_resp_port()->resp(orig);
        // Lane is idle again — let whatever queued behind this RMW through (async path only; on the
        // sync path nothing can be parked because phase_ never stays non-IDLE across a call).
        if (_this->structural_occupancy_) {
            const int64_t d = _this->rmw_busy_until_ - _this->clock.get_cycles();
            if (d > 0) { if (!_this->occ_event_.is_enqueued()) _this->occ_event_.enqueue(d); }
            else drain_park(_this);
        } else {
            drain_park(_this);
        }
    }
}

extern "C" vp::Component *gv_new(vp::ComponentConf &config)
{
    return new InsituCacheAmo(config);
}
