// SPDX-FileCopyrightText: 2026 ETH Zurich and University of Bologna
// SPDX-License-Identifier: Apache-2.0
//
// InSitu cache — structural per-cell par_coalescer (the 4 VLSU lanes → 1 wide cache beat).
//
// The RTL `par_coalescer` lives INSIDE `cachepool_cache_ctrl` (cachepool_cache_ctrl.sv:344): it merges
// the same-cycle, same-line, same-type narrow accesses of the 4 Spatz VLSU lanes into ONE wide 512b
// cache lookup, and splits the wide read response back to each merged port. This component wraps the
// validated `insitu_cache_coalesce.hpp` datapath (group-by-{is_write,line} + per-port word offsets +
// last-writer-wins merge + response split) in a per-cycle event-driven shell, sitting between the tile's
// lane crossbars and one cache core.
//
// SCOPE (Phase A2):
//   - READS coalesce: same-cycle same-line reads across the N input lanes collapse into ONE wide
//     line-read to the core; each merged reader is served its word from the returned line (rsp_spliter).
//     This reproduces "N narrow reads → 1 bank access" (the calib coal_warm/coal_cold effect).
//   - WRITES pass through individually (req_forward, response auto-routes upstream) — data-correct; the
//     RTL wide last-writer-wins write merge is a later refinement (writes never co-merge with reads, so
//     this does not affect read coalescing).
//   - The 1-cycle accumulation window (group requests by their arrival cycle, emit on the next tick) is
//     the CSHR window; its exact width + the watchdog are timing-calibration knobs.
// The scalar (Snitch/FPU) lane does NOT go through here — it bypasses to the core's second input port
// (the RTL 2:1 reqrsp_xbar inside the ctrl). Wiring is done by the tile.

#include <cstdint>
#include <cstring>
#include <map>
#include <string>
#include <vector>

#include <vp/vp.hpp>
#include <vp/itf/io.hpp>

#include "insitu_cache_coalesce.hpp"

using namespace insitu;

class InsituCacheCellCoalescer : public vp::Component
{
public:
    explicit InsituCacheCellCoalescer(vp::ComponentConf &conf);

private:
    static vp::IoReqStatus req_handler(vp::Block *__this, vp::IoReq *req, int input_id);
    static void resp_handler(vp::Block *__this, vp::IoReq *req);
    static void tick(vp::Block *__this, vp::ClockEvent *event);

    void schedule_tick();
    void split_and_resp(int gi);

    // one parked narrow read awaiting coalescing
    struct Pend { vp::IoReq *req; uint32_t port; uint64_t addr; uint32_t size; int64_t cyc; };
    std::vector<Pend> batch_;

    // one in-flight wide line-read group (fixed pool so &wide is stable for the resp map)
    struct Group {
        bool active = false;
        std::vector<vp::IoReq*> members;   // the merged reader reqs
        std::vector<uint32_t>   off;       // each member's byte offset within the line
        std::vector<uint32_t>   sz;        // each member's size
        std::vector<uint8_t>    line_buf;  // wide line scratch (line_bytes)
        vp::IoReq               wide;
    };
    std::vector<Group> pool_;
    std::vector<int>   free_;
    std::map<vp::IoReq*, int> wide2idx_;

    Coalescer coal_;
    uint32_t  num_inputs_;
    uint32_t  line_bytes_, word_bytes_;

    std::vector<vp::IoSlave *> inputs_;
    vp::IoMaster output_;
    vp::ClockEvent *tick_event_ = nullptr;
    vp::Trace trace_;
};

InsituCacheCellCoalescer::InsituCacheCellCoalescer(vp::ComponentConf &conf) : vp::Component(conf)
{
    auto *cfg = this->get_js_config();
    num_inputs_ = cfg->get_child_int("num_inputs");
    line_bytes_ = cfg->get_child_int("cache_line_bytes");
    word_bytes_ = cfg->get_child_int("word_bytes");
    if (word_bytes_ < 1) word_bytes_ = 4;
    coal_.init(line_bytes_, word_bytes_);

    const uint32_t kMaxGroups = 64;
    pool_.resize(kMaxGroups);
    for (uint32_t i = 0; i < kMaxGroups; i++) {
        pool_[i].line_buf.assign(line_bytes_, 0);
        free_.push_back((int)i);
        wide2idx_[&pool_[i].wide] = (int)i;
    }

    inputs_.resize(num_inputs_);
    for (uint32_t i = 0; i < num_inputs_; i++) {
        inputs_[i] = new vp::IoSlave();
        inputs_[i]->set_req_meth_muxed(&InsituCacheCellCoalescer::req_handler, (int)i);
        this->new_slave_port("in_" + std::to_string(i), inputs_[i]);
    }
    output_.set_resp_meth(&InsituCacheCellCoalescer::resp_handler);
    this->new_master_port("out", &output_);

    tick_event_ = this->event_new(&InsituCacheCellCoalescer::tick);
    this->traces.new_trace("trace", &this->trace_, vp::DEBUG);
    this->trace_.msg(vp::Trace::LEVEL_INFO, "InsituCacheCellCoalescer in=%u line=%u word=%u\n",
                     num_inputs_, line_bytes_, word_bytes_);
}

void InsituCacheCellCoalescer::schedule_tick()
{
    if (!tick_event_->is_enqueued()) this->event_enqueue(tick_event_, 1);
}

vp::IoReqStatus InsituCacheCellCoalescer::req_handler(vp::Block *__this, vp::IoReq *req, int input_id)
{
    InsituCacheCellCoalescer *_this = static_cast<InsituCacheCellCoalescer *>(__this);
    // Writes (and any non-read) pass straight through; the response auto-routes back upstream.
    if (req->get_is_write()) return _this->output_.req_forward(req);

    // Reads accumulate into the coalescing window; emitted on the next tick.
    _this->batch_.push_back(Pend{req, (uint32_t)input_id, req->get_addr(),
                                 (uint32_t)req->get_size(), _this->clock.get_cycles()});
    _this->schedule_tick();
    return vp::IO_REQ_PENDING;
}

void InsituCacheCellCoalescer::tick(vp::Block *__this, vp::ClockEvent *event)
{
    (void)event;
    InsituCacheCellCoalescer *_this = static_cast<InsituCacheCellCoalescer *>(__this);
    const int64_t now = _this->clock.get_cycles();

    // Window closes for accesses that arrived in a STRICTLY-earlier cycle; same-cycle arrivals wait.
    std::vector<Pend> ready, rest;
    for (auto &p : _this->batch_) { (p.cyc < now ? ready : rest).push_back(p); }
    _this->batch_.swap(rest);

    // Group the ready reads by arrival cycle, then coalesce same-line within each cycle.
    std::map<int64_t, std::vector<Pend>> by_cyc;
    for (auto &p : ready) by_cyc[p.cyc].push_back(p);

    for (auto &kv : by_cyc) {
        std::vector<NarrowAccess> acc;
        for (auto &p : kv.second) {
            NarrowAccess a; a.port = p.port; a.is_write = false; a.addr = p.addr; a.size = p.size;
            acc.push_back(a);
        }
        std::vector<CoalGroup> groups = _this->coal_.coalesce(acc);
        for (auto &g : groups) {
            if (_this->free_.empty()) {   // pool exhausted → defer this cycle's remainder
                for (auto &p : kv.second) _this->batch_.push_back(p);
                _this->schedule_tick();
                break;
            }
            const int gi = _this->free_.back(); _this->free_.pop_back();
            Group &grp = _this->pool_[gi];
            grp.active = true; grp.members.clear(); grp.off.clear(); grp.sz.clear();
            for (size_t k = 0; k < g.ports.size(); k++) {
                // map the merged port back to its parked Pend (same order as `acc`)
                const uint32_t port = g.ports[k];
                for (auto &p : kv.second) {
                    if (p.port == port) {
                        grp.members.push_back(p.req);
                        grp.off.push_back((uint32_t)(p.addr & (_this->line_bytes_ - 1)));
                        grp.sz.push_back(p.size);
                        break;
                    }
                }
            }
            grp.wide.init();
            grp.wide.set_addr(g.line);
            grp.wide.set_size(_this->line_bytes_);
            grp.wide.set_is_write(false);
            grp.wide.set_data(grp.line_buf.data());
            vp::IoReqStatus st = _this->output_.req(&grp.wide);
            if (st == vp::IO_REQ_OK) _this->split_and_resp(gi);
            // PENDING → resp_handler(grp.wide) splits + resp()s the members later.
        }
    }
    if (!_this->batch_.empty()) _this->schedule_tick();
}

void InsituCacheCellCoalescer::resp_handler(vp::Block *__this, vp::IoReq *req)
{
    InsituCacheCellCoalescer *_this = static_cast<InsituCacheCellCoalescer *>(__this);
    auto it = _this->wide2idx_.find(req);
    if (it == _this->wide2idx_.end()) return;   // not one of ours (shouldn't happen)
    _this->split_and_resp(it->second);
}

void InsituCacheCellCoalescer::split_and_resp(int gi)
{
    Group &grp = pool_[gi];
    for (size_t k = 0; k < grp.members.size(); k++) {
        vp::IoReq *m = grp.members[k];
        if (m->get_data() != nullptr) {
            uint32_t n = grp.sz[k];
            if (grp.off[k] + n > line_bytes_) n = line_bytes_ - grp.off[k];
            memcpy(m->get_data(), &grp.line_buf[grp.off[k]], n);   // rsp_spliter: each port gets its word
        }
        m->get_resp_port()->resp(m);
    }
    grp.active = false;
    grp.members.clear(); grp.off.clear(); grp.sz.clear();
    free_.push_back(gi);
}

extern "C" vp::Component *gv_new(vp::ComponentConf &config)
{
    return new InsituCacheCellCoalescer(config);
}
