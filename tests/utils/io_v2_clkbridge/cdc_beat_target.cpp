// SPDX-FileCopyrightText: 2026 ETH Zurich, University of Bologna and EssilorLuxottica SAS
//
// SPDX-License-Identifier: Apache-2.0
//
// Authors: Germain Haugou (germain.haugou@gmail.com)

/*
 * CDCBeatTarget — beat-plane io_v2 memory target stub for the clock-bridge
 * testbench, implementing the per-burst write-acknowledgement contract
 * (io_v2.hpp, "Write acknowledgement"):
 *
 *   - Every write beat is GRANTED, its payload consumed into the backing
 *     memory, and the beat freed to its pool — non-last beats get NO
 *     response at all. When the last beat of a burst lands, one DISTINCT
 *     data-less pool-backed ack (burst_id / initiator copied from the
 *     beats, the burst's final status) is streamed back via resp(); the
 *     initiator frees it.
 *   - A read burst request arrives data-less; the stub frees it and streams
 *     distinct allocator-backed response beats (one per beat_width, one per
 *     cycle) whose co-allocated payload carries the data.
 *
 * The stub never denies a request (no back-pressure of its own): the
 * admission back-pressure under test is the clock bridge's `depth` gate.
 */

#include <vp/vp.hpp>
#include <vp/itf/io_v2.hpp>

#include <algorithm>
#include <cstring>
#include <deque>
#include <vector>


class CDCBeatTarget : public vp::Component
{
public:
    CDCBeatTarget(vp::ComponentConf &conf);
    void reset(bool active) override;

private:
    static vp::IoReqStatus req_handler(vp::Block *__this, vp::IoReq *req);
    static void stream_handler(vp::Block *__this, vp::ClockEvent *event);

    vp::IoSlave in{&CDCBeatTarget::req_handler};
    vp::ClockEvent stream_event;
    vp::Trace trace;

    uint64_t size;
    int beat_width;
    std::vector<uint8_t> mem;

    // Pools: size-0 for the data-less burst acks, beat_width for the read
    // response beats (payload co-allocated).
    vp::IoReqAllocator *ack_pool = nullptr;
    vp::IoReqAllocator *beat_pool = nullptr;

    // The write burst currently accepting beats (beats of one burst never
    // interleave with another on a binding, so one open slot suffices).
    bool open_burst = false;
    uint64_t wr_base = 0;
    uint64_t wr_total = 0;
    int64_t wr_burst_id = -1;
    void *wr_initiator = nullptr;
    vp::IoRespStatus wr_status = vp::IO_RESP_OK;

    // Responses (burst acks + read beats) streamed upstream one per cycle.
    std::deque<vp::IoReq *> resp_queue;
};


CDCBeatTarget::CDCBeatTarget(vp::ComponentConf &config)
    : vp::Component(config),
      stream_event(this, &CDCBeatTarget::stream_handler)
{
    this->traces.new_trace("trace", &this->trace, vp::DEBUG);

    this->new_slave_port("input", &this->in);

    js::Config *cfg = this->get_js_config();
    this->size = (uint64_t)cfg->get_child_int("size");
    this->beat_width = cfg->get_child_int("beat_width");
    if (this->size == 0 || this->beat_width <= 0)
    {
        this->trace.fatal("invalid config (size=%lu, beat_width=%d)\n",
                          this->size, this->beat_width);
    }
    this->mem.resize(this->size, 0x55);

    this->ack_pool  = vp::IoReqAllocator::get(0);
    this->beat_pool = vp::IoReqAllocator::get(this->beat_width);
}


void CDCBeatTarget::reset(bool active)
{
    if (!active) return;

    // Undelivered responses (acks / read beats) are ours until accepted
    // upstream — return them to their pools.
    for (vp::IoReq *r : this->resp_queue)
    {
        r->free();
    }
    this->resp_queue.clear();
    this->stream_event.cancel();
    this->open_burst = false;
}


vp::IoReqStatus CDCBeatTarget::req_handler(vp::Block *__this, vp::IoReq *req)
{
    auto *self = static_cast<CDCBeatTarget *>(__this);

    if (req->get_opcode() == vp::WRITE)
    {
        // Per-burst ack contract: consume and free EVERY granted write beat;
        // respond exactly once per burst, on the last beat, with a distinct
        // data-less pool ack.
        self->traces.assert(req->allocator != nullptr,
            "write beat is not allocator-backed (req=%p) — unported master", req);

        if (req->is_first)
        {
            self->traces.assert(!self->open_burst,
                "write burst opened while another is still accepting beats");
            self->open_burst = true;
            self->wr_base = req->get_addr();
            self->wr_total = 0;
            self->wr_burst_id = req->burst_id;
            self->wr_initiator = req->initiator;
            self->wr_status = vp::IO_RESP_OK;
        }
        else
        {
            self->traces.assert(self->open_burst,
                "write-burst continuation without an open burst (req=%p)", req);
            self->traces.assert(self->wr_initiator == req->initiator,
                "write beats of one burst must carry the same initiator");
        }

        uint64_t addr = req->get_addr();
        uint64_t beat_size = req->get_size();
        if (addr + beat_size <= self->size && req->get_data() != nullptr)
        {
            memcpy(self->mem.data() + addr, req->get_data(), beat_size);
        }
        else if (beat_size > 0)
        {
            self->wr_status = vp::IO_RESP_INVALID;
        }
        self->wr_total += beat_size;

        bool last = req->is_last;
        // Consume the beat: freeing it is what releases the initiator's
        // buffer.
        req->free();

        if (last)
        {
            self->open_burst = false;
            vp::IoReq *ack = self->ack_pool->alloc();
            ack->prepare();
            // Size-0 pool: data is caller-managed — set it on EVERY
            // allocation (NULL for a data-less ack).
            ack->set_data(nullptr);
            ack->set_addr(self->wr_base);
            ack->set_size(self->wr_total);
            ack->set_opcode(vp::WRITE);
            ack->is_first = true;
            ack->is_last  = true;
            ack->burst_id  = self->wr_burst_id;
            ack->initiator = self->wr_initiator;
            ack->set_resp_status(self->wr_status);
            self->resp_queue.push_back(ack);
            self->stream_event.enqueue(1);
        }
        return vp::IO_REQ_GRANTED;
    }

    if (req->get_opcode() == vp::READ)
    {
        // Beat-plane read: the burst request is data-less and freed by us;
        // the payload goes back inside distinct response beats.
        self->traces.assert(req->get_data() == nullptr,
            "beat-plane read burst request must be data-less (req=%p)", req);

        uint64_t addr = req->get_addr();
        uint64_t total = req->get_size();
        int64_t burst_id = req->burst_id;
        void *initiator = req->initiator;
        bool in_range = addr + total <= self->size;
        req->free();

        uint64_t offset = 0;
        do
        {
            // A zero-size burst still gets a single zero-size beat.
            uint64_t beat = std::min<uint64_t>(total - offset,
                                               (uint64_t)self->beat_width);
            vp::IoReq *b = self->beat_pool->alloc();
            b->prepare();
            b->set_addr(addr + offset);
            b->set_size(beat);
            b->set_opcode(vp::READ);
            if (in_range)
            {
                memcpy(b->get_data(), self->mem.data() + addr + offset, beat);
            }
            else
            {
                b->set_resp_status(vp::IO_RESP_INVALID);
            }
            b->is_first = offset == 0;
            b->is_last  = offset + beat >= total;
            b->burst_id  = burst_id;
            b->initiator = initiator;
            self->resp_queue.push_back(b);
            offset += beat;
        } while (offset < total);

        self->stream_event.enqueue(1);
        return vp::IO_REQ_GRANTED;
    }

    self->trace.fatal("unsupported opcode %d\n", (int)req->get_opcode());
    return vp::IO_REQ_DONE;
}


void CDCBeatTarget::stream_handler(vp::Block *__this, vp::ClockEvent *)
{
    auto *self = static_cast<CDCBeatTarget *>(__this);

    if (!self->resp_queue.empty())
    {
        vp::IoReq *r = self->resp_queue.front();
        self->resp_queue.pop_front();
        // The clock bridge (the bound consumer) always accepts responses.
        vp::IoRespAck ack = self->in.resp(r);
        self->traces.assert(ack == vp::IO_RESP_ACCEPTED,
            "consumer denied a response but the stub has no resp_retry path");
    }

    if (!self->resp_queue.empty())
    {
        self->stream_event.enqueue(1);
    }
}


extern "C" vp::Component *gv_new(vp::ComponentConf &config)
{
    return new CDCBeatTarget(config);
}
