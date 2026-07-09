// SPDX-FileCopyrightText: 2026 ETH Zurich, University of Bologna and EssilorLuxottica SAS
//
// SPDX-License-Identifier: Apache-2.0
//
// Authors: Germain Haugou (germain.haugou@gmail.com)

/*
 * IoV2BeatToSyncAdapter — dedicated adapter auto-inserted by the gvrun2 systree
 * on io_v2 bindings whose master side declares signature IoV2Beat and whose
 * slave side declares signature IoV2Sync.
 *
 * It is a specialisation of IoV2BeatAdapter for the case where the downstream
 * slave honours the *sync* contract: every request completes inline with
 * IO_REQ_DONE (never IO_REQ_GRANTED + resp(), never IO_REQ_DENIED + retry()).
 * That lets the adapter be trivial: a write burst is forwarded to the slave
 * whole (inline DONE); a read burst — which carries no data under the beat
 * protocol — is served inline at submit time as beat-sized sub-reads, each
 * into a distinct allocator-backed beat whose co-allocated payload receives
 * the data. Either way every upstream beat is fully determined at submit and
 * pushed onto one flat queue; an FSM streams them back upstream at one resp()
 * per cycle. No async/deny bookkeeping, no latency-spread maths.
 *
 * Wire-protocol invariants preserved (identical to the general adapter for the
 * sync case):
 *   - The input port's req callback returns IO_REQ_GRANTED — never IO_REQ_DONE.
 *   - For each submission the upstream master receives exactly
 *     ceil(total_size / beat_width) resp() calls in byte order, is_first on the
 *     first, is_last on the last, with size/addr/burst_id/status set per beat.
 *     addr is the per-beat start address (burst_addr + emitted). Read beats are
 *     distinct allocator-backed objects the terminal master copies out of and
 *     frees; write acks round-trip the master's own request.
 *   - The first beat is delayed by the slave's get_full_latency(); beats then
 *     stream at one per cycle.
 *
 * Multi-outstanding: a second burst received while one is still streaming is
 * forwarded inline and queued behind the active one.
 *
 * The file is a private header for the component implementation; no model
 * includes it (the framework auto-inserts the component).
 */

#pragma once

#include <cstdint>
#include <deque>
#include <vp/vp.hpp>
#include <vp/itf/io_v2.hpp>
#include <vp/debug_mem.hpp>


class IoV2BeatToSyncAdapter : public vp::Component, public vp::DebugMemIf
{
public:
    IoV2BeatToSyncAdapter(vp::ComponentConf &config);
    void reset(bool active) override;

    // Backdoor debug path (vp/debug_mem.hpp): the adapter is invisible — both
    // calls are forwarded unchanged to the component bound downstream.
    vp::DebugMemIf *debug_mem_if() override { return this; }
    int debug_mem_access(uint64_t addr, uint8_t *data, uint64_t size,
        bool is_write) override;
    void debug_mem_regions(std::vector<vp::DebugMemRegion> &regions,
        uint64_t local_base, uint64_t window_size, uint64_t entry_base,
        int depth) override;

private:
    vp::DebugMemIf *resolve_debug_mem();

    static vp::IoReqStatus req_handler(vp::Block *__this, vp::IoReq *req);
    static vp::IoRespAck   resp_handler(vp::Block *__this, vp::IoReq *req);
    static void            retry_handler(vp::Block *__this, vp::IoRetryChannel channel);
    // Upstream master is ready to accept responses again: re-send the held beat
    // (response-path back-pressure) and resume streaming.
    static void            resp_retry_in_handler(vp::Block *__this, vp::IoRetryChannel channel);
    static void            fsm_handler(vp::Block *__this, vp::ClockEvent *event);

    // One queued upstream beat, fully determined at submit time. Read beats
    // are distinct allocator-backed objects whose payload was filled inline
    // at submit (beat != nullptr; the other fields are unused — the object
    // already carries them). Write acks round-trip the master's own request
    // (wreq), mutated per entry at emit time.
    struct StreamEntry
    {
        vp::IoReq *beat;
        vp::IoReq *wreq;
        uint64_t addr;
        uint8_t *data;      // master's buffer + offset
        uint64_t size;
        bool is_first;
        bool is_last;
        int64_t burst_id;
        vp::IoRespStatus status;
    };

    // Emit one entry. Returns false if the upstream master back-pressured the
    // beat (held for re-send from resp_retry_in_handler).
    bool emit_entry(const StreamEntry &e);

    int beat_width;
    // Shared pool serving beat-sized requests with their payload co-allocated;
    // read beats are drawn from it and freed by the terminal master.
    vp::IoReqAllocator *beat_allocator;
    vp::IoSlave in;
    vp::IoMaster out;
    vp::ClockEvent fsm_event;

    // Beats awaiting upstream delivery, in submission/byte order, drained at
    // one per cycle.
    std::deque<StreamEntry> entries;

    // Response-path back-pressure: the upstream master refused the beat we just
    // emitted. We hold that exact object and re-send it from
    // resp_retry_in_handler; nothing else is emitted until it is accepted.
    bool      resp_held = false;
    vp::IoReq *held_req = nullptr;

    vp::Trace trace;
};
