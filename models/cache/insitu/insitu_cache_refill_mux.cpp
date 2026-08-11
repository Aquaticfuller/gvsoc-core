// SPDX-FileCopyrightText: 2026 ETH Zurich and University of Bologna
// SPDX-License-Identifier: Apache-2.0
//
// InSitu cache — group-level refill mux (v3-P3).
//
// N wide refill masters share ONE downstream port. In the intended design a group aggregates 16 bank
// refill ports (4 tiles x 4 banks) plus 1 L2 instruction-cache port = 17 inputs, and the single output
// goes to that group's L2 NoC router (v3-P4). The same component serves the 4->1 aggregation of the
// tiles' L1 instruction-cache refills into the group L2 I$.
//
// Arbitration (as specified for CachePool):
//   - INSTRUCTION inputs have STRICT PRIORITY over data inputs. They are the last `nb_prio` inputs,
//     so a 17-input mux with nb_prio=1 makes input 16 the instruction port.
//   - DATA inputs are served ROUND-ROBIN, resuming after the last input served so no input can be
//     starved by a busier neighbour.
//   - One request leaves per cycle. That is the whole point of a mux: it is a shared port, so the
//     contention it creates must be REAL simulated time rather than a latency stamp. Stamped latency
//     is discarded by an asynchronous requester (iss lsu.cpp data_response zeroes pending_latency),
//     which is the same trap that made the AMO lane and the flush walk cost nothing on that path.
//
// The response carries a requester id: set_initiator(input_id) stamps which input a request came from,
// the model's stand-in for the RTL's user field. GVSoC routes the response itself along the preserved
// resp-port chain, so the id is for observability and for downstream components that arbitrate on it —
// it is not what gets the data home.

#include <cstdint>
#include <cstdlib>
#include <deque>
#include <string>
#include <utility>
#include <vector>

#include <vp/vp.hpp>
#include <vp/itf/io.hpp>

class InsituCacheRefillMux : public vp::Component
{
public:
    explicit InsituCacheRefillMux(vp::ComponentConf &conf);

    void reset(bool active) override
    {
        if (active) {
            for (auto &q : queues_) q.clear();
            delayed_.clear();
            rr_ptr_ = 0;
        }
    }

    void stop() override
    {
        fprintf(stderr, "[INSITU-REFILL-MUX %s] fwd=%lu (instr=%lu data=%lu) max_q=%lu\n",
                this->get_path().c_str(), (unsigned long)n_fwd_, (unsigned long)n_fwd_prio_,
                (unsigned long)(n_fwd_ - n_fwd_prio_), (unsigned long)max_q_);
        vp::Component::stop();
    }

private:
    static vp::IoReqStatus req_handler(vp::Block *__this, vp::IoReq *req, int input_id);
    static void fsm_handler(vp::Block *__this, vp::ClockEvent *event);

    // next input to serve: instruction ports first (strict priority), then round-robin over data
    int pick_input();

    uint32_t num_inputs_ = 1;
    uint32_t nb_prio_ = 0;          // the LAST nb_prio_ inputs are instruction ports
    uint32_t rr_ptr_ = 0;           // round-robin cursor over the data inputs
    bool     forward_initiator_ = true;

    std::vector<vp::IoSlave *> inputs_;
    std::vector<std::deque<vp::IoReq *>> queues_;
    // Downstream latency must be spent as REAL TIME here. The backing memory answers a refill with
    // IO_REQ_OK plus an inc_latency() stamp, and the cache core turns that stamp into a deferred
    // install only on its own OK path. Once the mux sits in between and answers PENDING, responding in
    // the same cycle would throw the memory's latency away — refills would land ~50 cycles early,
    // which made kernels run FASTER with arbitration added and corrupted load-store's data.
    std::deque<std::pair<vp::IoReq *, int64_t>> delayed_;
    vp::IoMaster output_;
    vp::ClockEvent fsm_event_;
    vp::Trace trace_;

    uint64_t n_fwd_ = 0, n_fwd_prio_ = 0, max_q_ = 0;
};

InsituCacheRefillMux::InsituCacheRefillMux(vp::ComponentConf &conf)
: vp::Component(conf), fsm_event_(this, &InsituCacheRefillMux::fsm_handler)
{
    auto *cfg = this->get_js_config();
    num_inputs_ = cfg->get_child_int("num_inputs");
    if (num_inputs_ < 1) num_inputs_ = 1;
    nb_prio_ = cfg->get("nb_priority_inputs") ? cfg->get_child_int("nb_priority_inputs") : 0;
    if (nb_prio_ > num_inputs_) nb_prio_ = num_inputs_;
    forward_initiator_ = cfg->get("forward_initiator")
        ? cfg->get_child_bool("forward_initiator") : true;

    inputs_.resize(num_inputs_);
    queues_.resize(num_inputs_);
    for (uint32_t i = 0; i < num_inputs_; i++) {
        inputs_[i] = new vp::IoSlave();
        inputs_[i]->set_req_meth_muxed(&InsituCacheRefillMux::req_handler, (int)i);
        this->new_slave_port("in_" + std::to_string(i), inputs_[i]);
    }
    this->new_master_port("out", &output_);

    this->traces.new_trace("trace", &this->trace_, vp::DEBUG);
    this->trace_.msg(vp::Trace::LEVEL_INFO, "InsituCacheRefillMux inputs=%u prio=%u\n",
                     num_inputs_, nb_prio_);
}

// Queue and let the FSM forward one per cycle. Never DENY: an async master would wait forever for a
// resp() that never comes.
vp::IoReqStatus InsituCacheRefillMux::req_handler(vp::Block *__this, vp::IoReq *req, int input_id)
{
    InsituCacheRefillMux *_this = static_cast<InsituCacheRefillMux *>(__this);
    _this->queues_[input_id].push_back(req);
    if (_this->queues_[input_id].size() > _this->max_q_) _this->max_q_ = _this->queues_[input_id].size();
    if (!_this->fsm_event_.is_enqueued()) _this->fsm_event_.enqueue();
    return vp::IO_REQ_PENDING;
}

int InsituCacheRefillMux::pick_input()
{
    // instruction ports first, in order — strict priority, no round-robin among them
    for (uint32_t k = 0; k < nb_prio_; k++) {
        const uint32_t i = num_inputs_ - nb_prio_ + k;
        if (!queues_[i].empty()) return (int)i;
    }
    // then round-robin over the data ports, resuming after the last one served
    const uint32_t n_data = num_inputs_ - nb_prio_;
    for (uint32_t k = 0; k < n_data; k++) {
        const uint32_t i = (rr_ptr_ + k) % (n_data ? n_data : 1);
        if (!queues_[i].empty()) return (int)i;
    }
    return -1;
}

void InsituCacheRefillMux::fsm_handler(vp::Block *__this, vp::ClockEvent *event)
{
    InsituCacheRefillMux *_this = static_cast<InsituCacheRefillMux *>(__this);

    // release responses whose downstream latency has now elapsed
    {
        const int64_t now = _this->clock.get_cycles();
        for (auto it = _this->delayed_.begin(); it != _this->delayed_.end(); ) {
            if (it->second <= now) {
                vp::IoReq *r = it->first;
                it = _this->delayed_.erase(it);
                r->get_resp_port()->resp(r);
            } else {
                ++it;
            }
        }
    }

    const int sel = _this->pick_input();
    if (sel >= 0) {
        vp::IoReq *req = _this->queues_[sel].front();
        _this->queues_[sel].pop_front();

        const uint32_t n_data = _this->num_inputs_ - _this->nb_prio_;
        const bool is_prio = ((uint32_t)sel >= n_data);
        if (!is_prio && n_data) _this->rr_ptr_ = ((uint32_t)sel + 1) % n_data;

        // requester id (the RTL's user field); the response finds its own way back along the
        // preserved resp-port chain, so this is for observability and downstream arbitration.
        if (_this->forward_initiator_) req->set_initiator(sel);

        _this->trace_.msg(vp::Trace::LEVEL_TRACE, "forward in=%d addr=0x%lx size=%u %s\n",
                          sel, (unsigned long)req->get_addr(), req->get_size(),
                          is_prio ? "instr" : "data");

        _this->n_fwd_++;
        if (is_prio) _this->n_fwd_prio_++;

        // req_forward keeps the upstream resp port, so a PENDING downstream completes straight to the
        // original master. A synchronous OK is ours to answer, since we already returned PENDING —
        // after its stamped latency has actually elapsed.
        if (_this->output_.req_forward(req) == vp::IO_REQ_OK) {
            const int64_t lat = (int64_t)req->get_full_latency();
            if (lat > 0) _this->delayed_.push_back({req, _this->clock.get_cycles() + lat});
            else         req->get_resp_port()->resp(req);
        }
    }

    // one per cycle: come back while anything is still waiting to go out or to be released
    if (!_this->delayed_.empty()) { _this->fsm_event_.enqueue(); return; }
    for (auto &q : _this->queues_) {
        if (!q.empty()) { _this->fsm_event_.enqueue(); return; }
    }
}

extern "C" vp::Component *gv_new(vp::ComponentConf &config)
{
    return new InsituCacheRefillMux(config);
}
