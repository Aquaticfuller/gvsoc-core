// SPDX-FileCopyrightText: 2026 ETH Zurich and University of Bologna
// SPDX-License-Identifier: Apache-2.0
//
// InSitu cache — structural per-cell par_coalescer (the 4 VLSU lanes → 1 wide cache beat).
//
// The RTL `par_coalescer` lives INSIDE `cachepool_cache_ctrl` (cachepool_cache_ctrl.sv:344): it merges
// the same-cycle, same-part, same-type narrow accesses of the 4 Spatz VLSU lanes into ONE wide cache
// lookup, and splits the wide read response back to each merged port. This component wraps the
// validated `insitu_cache_coalesce.hpp` datapath (group-by-{is_write,part} + per-port word offsets +
// last-writer-wins merge + response split) in a per-cycle event-driven shell, sitting between the tile's
// lane crossbars and one cache core.
//
// SCOPE (C1, 2026-07-26):
//   - The coalescing key is the 16 B PART (production folds PartSplit=4 → CoalescerDataWidth =
//     CacheLineWidth/PartSplit = 512/4 = 128b, cachepool_cache_ctrl.sv:80-81), NOT the 64 B line —
//     `part_bytes` (= line/4 deployed; line = legacy A2 behaviour).
//   - READS coalesce: same-cycle same-part reads collapse into ONE wide part-read; each merged reader
//     is served its word from the returned part (rsp_spliter) and inherits the wide access's latency.
//   - WRITES coalesce last-writer-wins when the merged byte mask covers the WHOLE part (the unit-stride
//     store-stream case: 4 lanes × 4 B words = one 16 B beat = ONE bank access). Partial-coverage
//     groups fall back to forwarding each member individually (data-safe with the mask-less core).
//   - The 1-cycle accumulation window (group by arrival cycle, emit on the next tick) is the CSHR
//     window's minimal form; the cross-cycle watchdog/occupy-map FSM is C2 (deferred).
//   - The coalescer adds NO pipeline latency of its own: the ~4-cycle coalescer req/resp pipeline is
//     already folded into the calibrated structural hit (10 cy, C2 note), so members inherit exactly
//     the wide access's latency.
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

    // one parked narrow access awaiting coalescing
    struct Pend { vp::IoReq *req; uint32_t port; uint64_t addr; uint32_t size; int64_t cyc;
                  bool done = false; };   // done: already emitted this tick (pool-exhaustion safety)
    std::vector<Pend> batch_;

    // one in-flight wide part-access group (fixed pool so &wide is stable for the resp map)
    struct Group {
        bool active = false;
        bool is_write = false;
        std::vector<vp::IoReq*> members;   // the merged narrow reqs
        std::vector<uint32_t>   off;       // each member's byte offset within the part (reads)
        std::vector<uint32_t>   sz;        // each member's size
        std::vector<int64_t>    park_cyc;  // each member's arrival cycle (batch-slip correction)
        std::vector<uint8_t>    part_buf;  // wide part scratch (part_bytes)
        vp::IoReq               wide;
    };
    std::vector<Group> pool_;
    std::vector<int>   free_;
    std::map<vp::IoReq*, int> wide2idx_;

    Coalescer coal_;
    uint32_t  num_inputs_;
    uint32_t  line_bytes_, part_bytes_, word_bytes_;

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
    part_bytes_ = cfg->get_child_int("part_bytes");
    if (word_bytes_ < 1) word_bytes_ = 4;
    if (part_bytes_ < word_bytes_ || part_bytes_ > line_bytes_) part_bytes_ = line_bytes_;
    // The hpp "line" is our coalescing granule: the part.
    coal_.init(part_bytes_, word_bytes_);

    const uint32_t kMaxGroups = 64;
    pool_.resize(kMaxGroups);
    for (uint32_t i = 0; i < kMaxGroups; i++) {
        pool_[i].part_buf.assign(part_bytes_, 0);
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
    this->trace_.msg(vp::Trace::LEVEL_INFO, "InsituCacheCellCoalescer in=%u line=%u part=%u word=%u\n",
                     num_inputs_, line_bytes_, part_bytes_, word_bytes_);
}

void InsituCacheCellCoalescer::schedule_tick()
{
    if (!tick_event_->is_enqueued()) this->event_enqueue(tick_event_, 1);
}

vp::IoReqStatus InsituCacheCellCoalescer::req_handler(vp::Block *__this, vp::IoReq *req, int input_id)
{
    InsituCacheCellCoalescer *_this = static_cast<InsituCacheCellCoalescer *>(__this);
    const uint64_t addr = req->get_addr();
    const uint32_t size = (uint32_t)req->get_size();
    // Only word-granularity, part-contained accesses can coalesce (the RTL lanes are 32b). Anything
    // wider / part-spanning bypasses the window entirely; the response auto-routes upstream.
    const uint32_t part_off = (uint32_t)(addr & (_this->part_bytes_ - 1));
    if (size > _this->word_bytes_ || part_off + size > _this->part_bytes_)
        return _this->output_.req_forward(req);

    // Reads AND writes accumulate into the coalescing window (write merge = C1); emitted next tick.
    _this->batch_.push_back(Pend{req, (uint32_t)input_id, addr, size, _this->clock.get_cycles()});
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

    // Group the ready accesses by arrival cycle, then coalesce same-part within each cycle.
    std::map<int64_t, std::vector<Pend>> by_cyc;
    for (auto &p : ready) by_cyc[p.cyc].push_back(p);

    for (auto &kv : by_cyc) {
        std::vector<NarrowAccess> acc;
        for (auto &p : kv.second) {
            NarrowAccess a; a.port = p.port; a.is_write = p.req->get_is_write();
            a.addr = p.addr; a.size = p.size;
            if (a.is_write) a.wdata = p.req->get_data();   // wstrb=nullptr → all bytes of the word
            acc.push_back(a);
        }
        std::vector<CoalGroup> groups = _this->coal_.coalesce(acc);
        for (auto &g : groups) {
            if (_this->free_.empty()) {   // pool exhausted → defer this cycle's UNEMITTED remainder
                for (auto &p : kv.second) if (!p.done) _this->batch_.push_back(p);
                _this->schedule_tick();
                break;
            }
            // map the merged ports back to their parked Pend entries (same order as `acc`).
            // CRITICAL: match only !done entries — the coalescer input port is the PORT-CLASS, so
            // different cores' lane-j accesses arrive with the SAME port index (and one core can
            // also queue two same-cycle same-port bursts). Matching on port alone would put the
            // SAME req in two groups → double resp() → arg_pop on empty in the VLSU (SIGSEGV,
            // reproduced by M48 linked-list at 4-core).
            std::vector<Pend *> members;
            for (size_t k = 0; k < g.ports.size(); k++) {
                for (auto &p : kv.second) {
                    if (!p.done && p.port == g.ports[k]) { members.push_back(&p); break; }
                }
            }
            for (auto *mp : members) mp->done = true;

            if (g.is_write) {
                // Coverage of the merged byte mask: only a FULL part can ride one wide write (the
                // core has no byte-enables — a partial wide write would clobber the holes).
                uint32_t covered = 0;
                for (uint32_t b = 0; b < _this->part_bytes_; b++)
                    if (g.wmask[b >> 3] & (1u << (b & 7))) covered++;
                if (covered != _this->part_bytes_) {
                    // Partial coverage → forward each member individually (no merge, data-safe).
                    // req_forward (NOT req): req() would push our output's response context onto the
                    // parked req, and the upstream resp() would then loop back into our own
                    // resp_handler and be dropped as unknown. req_forward preserves the parked req's
                    // upstream context, so a sync OK resp()s upstream and an async completion
                    // auto-routes there — no bookkeeping.
                    for (auto *mp : members) {
                        vp::IoReqStatus st = _this->output_.req_forward(mp->req);
                        if (st == vp::IO_REQ_OK) mp->req->get_resp_port()->resp(mp->req);
                    }
                    continue;
                }
            }

            const int gi = _this->free_.back(); _this->free_.pop_back();
            Group &grp = _this->pool_[gi];
            grp.active = true; grp.is_write = g.is_write;
            grp.members.clear(); grp.off.clear(); grp.sz.clear(); grp.park_cyc.clear();
            for (auto *mp : members) {
                grp.members.push_back(mp->req);
                grp.off.push_back((uint32_t)(mp->addr & (_this->part_bytes_ - 1)));
                grp.sz.push_back(mp->size);
                grp.park_cyc.push_back(mp->cyc);
            }
            grp.wide.init();
            grp.wide.set_addr(g.line);                  // the coalescing key = part base
            grp.wide.set_size(_this->part_bytes_);
            grp.wide.set_is_write(g.is_write);
            if (g.is_write) memcpy(grp.part_buf.data(), g.wdata.data(), _this->part_bytes_);
            grp.wide.set_data(grp.part_buf.data());
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
    const int64_t lat = (int64_t)grp.wide.get_full_latency();
    const int64_t now = clock.get_cycles();
    for (size_t k = 0; k < grp.members.size(); k++) {
        vp::IoReq *m = grp.members[k];
        if (!grp.is_write && m->get_data() != nullptr) {
            uint32_t n = grp.sz[k];
            if (grp.off[k] + n > part_bytes_) n = part_bytes_ - grp.off[k];
            memcpy(m->get_data(), &grp.part_buf[grp.off[k]], n);   // rsp_spliter: each port its word
        }
        // C1: the member inherits the wide access's latency (one merged bank access serves all
        // members — THE point of the merge). Minus the batch-window slip (real cycles from the
        // member's arrival to the split): the coalescer's req/resp pipeline is already folded into
        // the calibrated core latency (the structural knobs reproduce the RTL ctrl's 10/67 END-TO-END,
        // coalescer included), so the park→emit cycle must not be charged a second time.
        const int64_t adj = lat - (now - grp.park_cyc[k]);
        m->inc_latency(adj > 0 ? adj : 0);
        m->get_resp_port()->resp(m);
    }
    grp.active = false;
    grp.members.clear(); grp.off.clear(); grp.sz.clear(); grp.park_cyc.clear();
    free_.push_back(gi);
}

extern "C" vp::Component *gv_new(vp::ComponentConf &config)
{
    return new InsituCacheCellCoalescer(config);
}
