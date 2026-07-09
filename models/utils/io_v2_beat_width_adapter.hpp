// SPDX-FileCopyrightText: 2026 ETH Zurich, University of Bologna and EssilorLuxottica SAS
//
// SPDX-License-Identifier: Apache-2.0
//
// Authors: Germain Haugou (germain.haugou@gmail.com)

/*
 * IoV2BeatWidthAdapter — standalone component auto-inserted by the gvrun2
 * systree on io_v2 bindings whose master side declares signature
 * IoV2Beat(input_width) and whose slave side declares IoV2Beat(output_width)
 * with a different width. Both sides speak the beat sub-protocol; the adapter
 * only converts the beat granularity, repacking the per-beat streams in both
 * directions (N narrow beats <-> one wide beat, in cumulative byte order).
 *
 * Widths: the wider one must be an integer multiple of the narrower one.
 * Short terminal beats (burst size not a width multiple) are handled on both
 * sides, with beat boundaries derived from the cumulative offset within the
 * burst, per the io_v2.hpp per-beat addr convention.
 *
 * Timing / bandwidth: each side runs at its own width and at most one beat
 * per cycle per channel, so the two bindings show different per-cycle
 * occupancies — the narrow side streams every cycle while the wide side
 * carries the same bytes in fewer, wider beats (one every ratio cycles).
 * Aggregate byte bandwidth is bounded by the narrow side, exactly like a HW
 * bus-width up/down-sizer. Back-pressure keeps both sides honest:
 *
 *   - READ, output narrower: downstream beats arrive one per cycle and are
 *     packed; a wide upstream beat completes every ratio cycles.
 *   - READ, output wider: each wide downstream beat unpacks into ratio
 *     upstream beats (one per cycle); further downstream beats are refused
 *     (IO_RESP_DENIED + later resp_retry()) while the unpacked backlog is
 *     full, so the downstream is throttled to one beat per ratio cycles.
 *   - WRITE, output narrower: each upstream write request is chopped into
 *     one-per-cycle downstream beats; further upstream write requests are
 *     DENIED while the chop backlog is full and re-enabled via retry(WRITE).
 *   - WRITE, output wider: consecutive upstream write beats of one burst are
 *     packed (copied) into one wide downstream beat issued every ratio cycles.
 *
 * Both back-pressure points sit behind configurable FIFOs (read_fifo_depth /
 * write_fifo_depth, see the config): deepening them lets the adapter absorb
 * workload — a stalled upstream response channel, a burst of upstream writes —
 * before it starts denying. The auto defaults are the minimum depths that
 * sustain full streaming.
 *
 * Ownership follows the initiator-owned request convention:
 *   - The upstream burst request is never freed here; read response beats
 *     emitted upstream are distinct allocator-backed objects (input_width
 *     pool) the terminal master frees; write acks round-trip the upstream
 *     master's own request object.
 *   - The downstream read descriptor is our own allocator-backed (data-less)
 *     object, freed by us when its response stream completes; downstream
 *     read beats are freed by us as we consume them; downstream write beats
 *     are our own allocator-backed (output_width pool) objects whose payload
 *     carries a copy of the upstream data — they round-trip back to us as
 *     acks and we free them.
 *
 * No public API: private header of the component implementation. The
 * framework auto-inserts the component during the gvrun2 binding-collection
 * pass; no model includes this header.
 */

#pragma once

#include <cstdint>
#include <deque>
#include <vector>
#include <vp/vp.hpp>
#include <vp/itf/io_v2.hpp>
#include <vp/debug_mem.hpp>

#include <utils/io_v2_beat_width_adapter/io_v2_beat_width_adapter_config.hpp>


class IoV2BeatWidthAdapter : public vp::Component, public vp::DebugMemIf
{
public:
    IoV2BeatWidthAdapter(vp::ComponentConf &config);
    void reset(bool active) override;

    // Backdoor debug path: the adapter is invisible, forward to the downstream.
    vp::DebugMemIf *debug_mem_if() override { return this; }
    int debug_mem_access(uint64_t addr, uint8_t *data, uint64_t size,
        bool is_write) override;
    void debug_mem_regions(std::vector<vp::DebugMemRegion> &regions,
        uint64_t local_base, uint64_t window_size, uint64_t entry_base,
        int depth) override;

    IoV2BeatWidthAdapterConfig cfg;

private:
    // ---- READ path ---------------------------------------------------------
    // One in-flight upstream read burst. Downstream beats are correlated back
    // to it via initiator (copied by the downstream producer from our
    // descriptor onto every beat), payload bytes are packed into
    // input_width-sized upstream beats at cumulative-offset boundaries.
    struct ReadBurst
    {
        vp::IoReq *up_req;        // upstream descriptor (initiator-owned upstream)
        vp::IoReq *dn_req;        // our downstream descriptor (freed on last dn beat)
        uint64_t   burst_addr;
        uint64_t   total;
        int64_t    burst_id;
        void      *up_initiator;  // snapshot of up_req->initiator
        uint64_t   bytes_received = 0;
        // Upstream beat under construction: covers [cur_start, cur_start+cur_size)
        // of the burst; cur_fill bytes copied so far.
        vp::IoReq *cur_beat = nullptr;
        uint64_t   cur_start = 0;
        uint64_t   cur_size = 0;
        uint64_t   cur_fill = 0;
        vp::IoRespStatus status = vp::IO_RESP_OK;
    };

    // One upstream read beat scheduled for emission (fully packed).
    struct PendingRead
    {
        ReadBurst *burst;
        vp::IoReq *beat;          // allocator-backed (input_width pool)
        uint64_t   addr;
        uint64_t   size;
        bool       is_first;
        bool       is_last;
        int64_t    burst_id;
        vp::IoRespStatus status;
        int64_t    ready_cycle;
    };

    // ---- WRITE path --------------------------------------------------------
    // One upstream write request (big-packet burst or one beat of a beat-form
    // burst). Chopped/packed into output_width downstream chunks; acked
    // upstream at input_width boundaries once the covering chunks are acked.
    struct WriteJob
    {
        vp::IoReq *up_req;
        uint64_t   addr;
        uint64_t   size;
        uint8_t   *data;
        int64_t    burst_id;
        bool       up_first;      // framing snapshot (we mutate up_req for acks)
        bool       up_last;
        uint64_t   bytes_acked = 0;
        uint64_t   acks_scheduled = 0;   // bytes covered by scheduled acks
        bool       zero_ack_scheduled = false;
        vp::IoRespStatus status = vp::IO_RESP_OK;
    };

    // Byte range of one WriteJob covered by a downstream chunk.
    struct ChunkSeg
    {
        WriteJob *job;
        uint64_t  bytes;
    };

    // One downstream write beat: allocator-backed request (output_width pool)
    // whose co-allocated payload carries a copy of the covered upstream bytes.
    struct WriteChunk
    {
        vp::IoReq *req = nullptr;
        uint64_t   addr = 0;
        uint64_t   fill = 0;
        bool       is_first = false;
        bool       is_last = false;
        int64_t    burst_id = -1;
        std::vector<ChunkSeg> segs;
    };

    // One upstream write ack scheduled for emission (round-trips up_req).
    struct PendingAck
    {
        WriteJob *job;
        uint64_t  offset;
        uint64_t  size;
        bool      is_first;
        bool      is_last;
        bool      job_done;       // final ack of the job: release it after emit
        int64_t   ready_cycle;
    };

    static vp::IoReqStatus req_handler(vp::Block *__this, vp::IoReq *req);
    static vp::IoRespAck   resp_handler(vp::Block *__this, vp::IoReq *req);
    static void            retry_handler(vp::Block *__this, vp::IoRetryChannel channel);
    static void            resp_retry_in_handler(vp::Block *__this, vp::IoRetryChannel channel);
    static void            fsm_handler(vp::Block *__this, vp::ClockEvent *event);

    // Read path.
    vp::IoReqStatus submit_read(vp::IoReq *req);
    vp::IoRespAck   consume_read_beat(vp::IoReq *beat);
    void complete_read_inline(ReadBurst *burst, int64_t latency);
    void schedule_read_beat(ReadBurst *burst, vp::IoReq *beat, uint64_t offset,
                            uint64_t size, int64_t latency_cycles);
    void emit_read_beat(const PendingRead &pr);
    void retire_read_burst(ReadBurst *burst);

    // Write path.
    vp::IoReqStatus submit_write(vp::IoReq *req);
    void finish_chunk(bool is_last);
    void issue_pending_chunks();
    void ack_chunk(WriteChunk *chunk, vp::IoRespStatus status, int64_t latency_cycles);
    void schedule_job_acks(WriteJob *job, int64_t latency_cycles);
    void emit_ack(const PendingAck &ack);
    void release_job(WriteJob *job);
    void maybe_unblock_write();

    void reschedule_fsm();

    vp::DebugMemIf *resolve_debug_mem();

    // Tiny freelist pools (process-lifetime, like IoReqAllocator).
    ReadBurst  *alloc_read_burst();
    void        free_read_burst(ReadBurst *burst);
    WriteJob   *alloc_job();
    WriteChunk *alloc_chunk();
    void        free_chunk(WriteChunk *chunk);

    int input_width;
    int output_width;
    // Read response FIFO depth (config read_fifo_depth, in input_width
    // beats): repacked upstream read beats buffered before the downstream
    // producer is back-pressured. Auto (0) sizes it to 2 downstream beats'
    // worth (double-buffering) — the minimum for continuous streaming;
    // larger depths absorb workload (e.g. an upstream response stall)
    // without stalling the downstream.
    size_t read_pending_limit;
    // Write FIFO depth (config write_fifo_depth, in output_width chunks):
    // complete, un-issued downstream write chunks buffered before upstream
    // write requests are refused. Auto (0) = 2 — just enough for seamless
    // chunk streaming; larger depths absorb a burst of upstream writes at
    // the upstream rate before throttling the writer.
    size_t write_chunk_limit;

    // Allocator pools: input_width-sized upstream read beats, output_width-
    // sized downstream write chunks, data-less downstream read descriptors.
    vp::IoReqAllocator *in_beat_allocator;
    vp::IoReqAllocator *out_beat_allocator;
    vp::IoReqAllocator *desc_allocator;

    vp::IoSlave in;
    vp::IoMaster out;
    vp::ClockEvent fsm_event;

    // Read state.
    std::vector<ReadBurst *> live_reads;        // for reset cleanup
    std::deque<PendingRead> read_pending;
    int64_t read_cursor = -1;                   // <=1 upstream read beat / cycle
    bool dn_read_blocked = false;               // we denied a downstream beat

    // Write state.
    std::vector<WriteJob *> live_jobs;          // for reset cleanup
    WriteChunk *cur_chunk = nullptr;            // under construction
    std::deque<WriteChunk *> chunk_queue;       // complete, waiting to issue
    std::vector<WriteChunk *> chunks_in_flight; // issued, awaiting downstream ack
    WriteChunk *held_chunk = nullptr;           // DENIED downstream, re-send on retry
    int64_t chunk_issue_cursor = -1;            // <=1 downstream chunk / cycle
    bool up_write_blocked = false;              // we denied an upstream write
    std::deque<PendingAck> ack_pending;
    int64_t ack_cursor = -1;                    // <=1 upstream write ack / cycle

    // Response-path back-pressure from the upstream master: the refused beat is
    // held here and re-sent from resp_retry_in_handler; nothing else is emitted
    // until it is accepted.
    bool resp_held = false;
    vp::IoReq *held_req = nullptr;

    // Freelists.
    std::vector<ReadBurst *> read_burst_pool;
    std::vector<WriteJob *> job_pool;
    std::vector<WriteChunk *> chunk_pool;

    vp::Trace trace;
};
