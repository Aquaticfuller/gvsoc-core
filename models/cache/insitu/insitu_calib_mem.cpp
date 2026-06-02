// SPDX-FileCopyrightText: 2026 ETH Zurich and University of Bologna
//
// SPDX-License-Identifier: Apache-2.0
//
// InSitu cache calibration — fixed-latency, serializing refill-memory model.
//
// This is the GVSoC twin of the RTL `refill_mem_model.sv` used by the standalone
// cache-calibration testbench (see ManyRVData_rebase/reports/cache_calib/
// CALIB_IMPLEMENTATION.md §2). It sits on the cache's L2 / refill port and answers
// line refills (reads) and writebacks (writes) with a deterministic, reproducible
// timing contract so the GVSoC perf model can be diffed 1:1 against the RTL.
//
// Timing contract (cycles), matching the RTL responder:
//   - MemLatency  : cycles from a request being *accepted* to its *first* response beat.
//   - BeatGap     : idle cycles between consecutive read-burst beats.
//   - AcceptEvery : min cycles between accepting two successive requests.
//   - Reads of a full line return `BurstLength = ceil(size/RefillDataWidth)` beats;
//     a writeback occupies the same envelope.
//
// **Serialization is the key behaviour to reproduce** (CALIB_IMPLEMENTATION.md §6): the
// RTL controller keeps at most ONE outstanding line-refill and serializes refill-reads
// vs. writeback-writes, so the memory only ever services one job at a time. We model
// that with a single `mem_busy_until_` cyclestamp instead of an async response FIFO:
//
//   service_start = max(now, mem_busy_until_)
//   occupancy     = MemLatency + (beats-1)*(1+BeatGap)
//   completion    = service_start + occupancy
//   latency_added = completion - now          // carried back on the IoReq
//   mem_busy_until_ = max(completion, service_start + AcceptEvery)
//
// Because the InSitu controller resolves refills on the synchronous-OK path
// (issue_refill -> refill_resp_handler inline, reading get_full_latency()), we return
// IO_REQ_OK with the latency accumulated via inc_latency(). The `mem_busy_until_`
// cyclestamp makes back-to-back distinct-line misses serialize even though the
// controller issues them on consecutive cycles — reproducing the RTL's
// memory-latency-bound, non-pipelined miss throughput.

#include <cstdint>
#include <cstring>
#include <cstdio>

#include <vp/vp.hpp>
#include <vp/itf/io.hpp>

class InsituCalibMem : public vp::Component
{
public:
    explicit InsituCalibMem(vp::ComponentConf &conf);

    void reset(bool active) override;
    void stop() override;

private:
    static vp::IoReqStatus req_handler(vp::Block *__this, vp::IoReq *req);

    inline uint64_t beats_for(uint64_t size) const
    {
        if (refill_beat_bytes_ == 0) return 1;
        uint64_t b = (size + refill_beat_bytes_ - 1) / refill_beat_bytes_;
        return (b == 0) ? 1 : b;
    }

    // ===== Config =====
    int64_t  mem_latency_;
    int64_t  beat_gap_;
    int64_t  accept_every_;
    uint32_t refill_beat_bytes_;
    uint32_t word_bytes_;
    bool     fill_pattern_;
    bool     writeback_overlap_;   // write (eviction) jobs don't serialize on mem_busy_until

    // ===== Ports =====
    vp::IoSlave input_itf_;

    // ===== State =====
    // Cycle at/after which the memory is free to begin servicing the next job. -1 means
    // idle (free immediately). One cyclestamp == one-job-at-a-time FIFO service, which
    // is exact for the DUT's single-outstanding-refill behaviour.
    int64_t mem_busy_until_;

    // ===== Telemetry =====
    vp::Trace trace_;
    uint64_t  cnt_read_jobs_  = 0;
    uint64_t  cnt_write_jobs_ = 0;
    uint64_t  sum_read_lat_   = 0;
    int64_t   max_read_lat_   = 0;
};

InsituCalibMem::InsituCalibMem(vp::ComponentConf &conf)
    : vp::Component(conf)
{
    auto *cfg = this->get_js_config();
    mem_latency_       = cfg->get_child_int("mem_latency");
    beat_gap_          = cfg->get_child_int("beat_gap");
    accept_every_      = cfg->get_child_int("accept_every");
    refill_beat_bytes_ = cfg->get_child_int("refill_beat_bytes");
    word_bytes_        = cfg->get_child_int("word_bytes");
    fill_pattern_      = cfg->get_child_bool("fill_pattern");
    writeback_overlap_ = cfg->get_child_bool("writeback_overlap");

    this->input_itf_.set_req_meth(&InsituCalibMem::req_handler);
    this->new_slave_port("input", &this->input_itf_);

    this->traces.new_trace("trace", &this->trace_, vp::DEBUG);
    this->trace_.msg(vp::Trace::LEVEL_INFO,
        "InsituCalibMem instantiated (mem_latency=%ld beat_gap=%ld accept_every=%ld "
        "refill_beat=%uB word=%uB fill_pattern=%d)\n",
        (long)mem_latency_, (long)beat_gap_, (long)accept_every_,
        refill_beat_bytes_, word_bytes_, (int)fill_pattern_);
}

void InsituCalibMem::reset(bool active)
{
    if (active) {
        mem_busy_until_ = -1;
        cnt_read_jobs_ = cnt_write_jobs_ = 0;
        sum_read_lat_ = 0;
        max_read_lat_ = 0;
    }
}

void InsituCalibMem::stop()
{
    // End-of-sim dump (block.cpp invokes stop() recursively when the sim closes).
    // Mirrors the RTL TB's per-phase mem_rd / mem_wr counters: reads = line refills
    // (misses), writes = writebacks (dirty evictions). The driver runs one trace per
    // sim, so these cumulative counts are that trace's memory traffic.
    fprintf(stderr,
        "[CALIB_MEM] mem_rd=%lu mem_wr=%lu read_lat_avg=%.1f read_lat_max=%ld\n",
        (unsigned long)cnt_read_jobs_, (unsigned long)cnt_write_jobs_,
        cnt_read_jobs_ ? (double)sum_read_lat_ / (double)cnt_read_jobs_ : 0.0,
        (long)max_read_lat_);
}

vp::IoReqStatus InsituCalibMem::req_handler(vp::Block *__this, vp::IoReq *req)
{
    InsituCalibMem *_this = static_cast<InsituCalibMem *>(__this);
    const int64_t  now   = _this->clock.get_cycles();
    const uint64_t size  = req->get_size();
    const bool     is_wr = req->get_is_write();

    const uint64_t beats     = _this->beats_for(size);
    const int64_t  occupancy = _this->mem_latency_ +
                               (int64_t)(beats - 1) * (1 + _this->beat_gap_);

    const bool overlap_wr = is_wr && _this->writeback_overlap_;

    // An overlapping writeback (dirty eviction) is serviced on a separate writeback path:
    // it does not wait behind the refill-read serialization and does not push it back —
    // it hides within the per-miss budget (matches the RTL: writeback adds no serial
    // stall, only a small per-miss latency adder modelled cache-side). It still takes
    // MemLatency to complete.
    const int64_t service_start = overlap_wr
        ? now
        : ((now > _this->mem_busy_until_) ? now : _this->mem_busy_until_);
    const int64_t completion = service_start + occupancy;
    int64_t latency = completion - now;
    if (latency < 0) latency = 0;

    if (!overlap_wr) {
        // Earliest the memory can begin the next read job: when this one finishes, but no
        // sooner than AcceptEvery cycles after this one's service start.
        int64_t next_free = completion;
        const int64_t accept_bound = service_start + _this->accept_every_;
        if (accept_bound > next_free) next_free = accept_bound;
        _this->mem_busy_until_ = next_free;
    }

    // Optional deterministic self-describing data (word @ byte-addr A holds value A),
    // matching the RTL responder so a read can be checked as data == addr. Off by
    // default: the InSitu perf model does not propagate refill bytes to the user
    // response, so calibration is timing-only (data_err = 0).
    if (_this->fill_pattern_ && !is_wr) {
        uint8_t *data = req->get_data();
        if (data != nullptr && _this->word_bytes_ > 0) {
            const uint64_t base = req->get_addr();
            for (uint64_t off = 0; off + _this->word_bytes_ <= size;
                 off += _this->word_bytes_) {
                uint64_t v = base + off;
                for (uint32_t b = 0; b < _this->word_bytes_; ++b) {
                    data[off + b] = (uint8_t)(v >> (8 * b));
                }
            }
        }
    }

    req->inc_latency((uint64_t)latency);

    if (is_wr) {
        _this->cnt_write_jobs_++;
    } else {
        _this->cnt_read_jobs_++;
        _this->sum_read_lat_ += (uint64_t)latency;
        if (latency > _this->max_read_lat_) _this->max_read_lat_ = latency;
    }

    _this->trace_.msg(vp::Trace::LEVEL_TRACE,
        "%s addr=0x%lx size=%lu beats=%lu now=%ld svc=%ld done=%ld lat=%ld busy_until=%ld\n",
        is_wr ? "WR" : "RD",
        (unsigned long)req->get_addr(), (unsigned long)size,
        (unsigned long)beats, (long)now, (long)service_start, (long)completion,
        (long)latency, (long)_this->mem_busy_until_);

    return vp::IO_REQ_OK;
}

extern "C" vp::Component *gv_new(vp::ComponentConf &config)
{
    return new InsituCalibMem(config);
}
