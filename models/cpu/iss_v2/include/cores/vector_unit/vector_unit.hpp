/*
 * Copyright (C) 2020 SAS, ETH Zurich and University of Bologna
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * Authors: Germain Haugou (germain.haugou@gmail.com)
 */

#pragma once

#include <deque>
#include <queue>
#include <cpu/iss_v2/include/types.hpp>
#include <cpu/iss_v2/include/stats/insn_duration.hpp>
#include <vp/clock/clock_event.hpp>
#include <vp/register.hpp>

class Vu;
class IssWrapper;
class PendingInsn;

#define ISS_INSN_FLAGS_VECTOR (1<< 0)

class PendingInsn
{
public:
    InsnEntry *entry;
    uint64_t timestamp;
    uint64_t reg;
    uint64_t reg_2;
    uint64_t reg_3;
    bool valid;
    bool done;
    int id;
    bool in_can_be_chained;
    bool out_can_be_chained;
    int nb_bytes_done;
    float chaining_factor;
    float out_chaining_factor;
    int8_t inreg0_index;
    int8_t inreg1_index;
    int8_t inreg2_index;
    // Cycle at which the owning block really started executing this
    // instruction (not when it was enqueued/queued). -1 until then. Used to
    // measure per-label execution duration up to Vu::insn_end.
    int64_t exec_start_cycle;
    // FPU pipeline depth of this instruction (cycles between an operand
    // word entering the unit and its result word being written), derived
    // at issue time from the fpu_lat_class and the effective element
    // width. Dependent instructions trail this many extra cycles behind
    // (chaining gate) and the scoreboard entry is released this many
    // cycles after the last chunk, without blocking the issue of the
    // next instruction in the block.
    int pipeline_latency;
    // Cycle at which the last chunk of this instruction was executed
    // (all bytes committed), -1 while still executing. Used by the
    // chaining gate to time the consumer's trailing chunks against the
    // producer's pipeline drain instead of its scoreboard release.
    int64_t commit_done_cycle;
    // A reduction is a serial chain through the FPU (one element per round
    // trip), not a lane-wide operation -- see the compute FSM.
    bool is_reduction;
    // Earliest cycle at which this instruction may execute its next chunk,
    // accumulated from the pipeline drain of its (possibly already
    // released) input dependencies. Carried on the consumer because the
    // scoreboard entry of the producer is released before its pipeline has
    // drained (the release also gates WAW/WAR, which the RTL frees
    // per-word).
    int64_t chain_release_cycle;
};

// This represents a generic HW block where vector instructions can be forwarded
class VuBlock : public vp::Block
{
public:
    VuBlock(Block *parent, std::string name) : vp::Block(parent, name) {}
    // False if the block can accept a new instruction
    virtual bool is_full() = 0;
    // Enqueue a new instruction to the block
    virtual void enqueue_insn(PendingInsn *pending_insn) = 0;
    // Initialize ISA decoding tree. Used to attach special handlers to instructions
    virtual void isa_init() {}
};

// Generic block for computation. Used for FPU and sliding to dispatch instructions in parallel
// and assign cost based on vector length, number of lanes and lmul
class VuCompute : public VuBlock
{
public:
    VuCompute(Vu &vu, std::string name);
    void reset(bool active) override;
    bool is_full() override { return this->insns.size() == 4; }
    void enqueue_insn(PendingInsn *pending_insn) override;

private:
    // Handler for internal FSM
    static void fsm_handler(vp::Block *__this, vp::ClockEvent *event);

    Vu &vu;
    // TEMPORARY: which reduction algorithm the unit implements. True = serial
    // chain through a single FU, one element per round trip (the MemPool-Spatz
    // fork). False = reduce across the lanes, as upstream Spatz and Ara do,
    // which runs at the generic lane-wide chunk rate instead. Drop the serial
    // path once the fork realigns with upstream.
    bool reduction_is_serial;
    // Cycles per element of a serial reduction. Unused when not serial.
    int reduction_step_latency;
    // Used for this block system traces
    vp::Trace trace;
    // Event for active state
    vp::Trace event_active;
    // Event for PC of instruction being processed
    vp::Trace event_pc;
    // Event for label of instruction being processed
    vp::Trace event_label;
    // Clock event used for scheduling FSM handler when at least one instruction has to be processed
    vp::ClockEvent fsm_event;
    // Queue of pending instructions to be processed by this block
    // The block process them in-order
    std::queue<PendingInsn *> insns;
    // Instructions whose chunks have all been executed but which are still
    // draining through the FPU pipeline (plus their completion latency).
    // They do not block the issue of the next instruction; their
    // scoreboard entry is released once their timestamp has passed.
    std::vector<PendingInsn *> draining;
    // When the instruction is chained, this indicates the minimum cyclestamp where the instruction
    // can finished, based on operation duration.
    int64_t end_cyclestamp;
    int total_size;
    int vstart;
    int vend;
};

class VuLsuPendingInsn
{
public:
    // Cycle at which the instruction entered the block queue. Used to
    // discount the dispatch latency already absorbed while waiting
    // behind the previous instruction's request stream.
    int64_t enqueue_cycle = 0;
public:
    PendingInsn *insn;
    // Number of pending bursts. This is used to detect when the instruction is fully done.
    int nb_pending_bursts;
    // Used by some blocks to flag the termination
    bool done;
    // Bursts of the operation not yet committed to the vector register file. Loads
    // only: store bursts commit one by one on their response.
    int nb_remaining_bursts;
};

#if defined(CONFIG_GVSOC_ISS_USE_SPATZ)

#if defined(CONFIG_GVSOC_ISS_VLSU_V2)

// Block processing load/store vector instructions, io_v2 variant.
//
// Functionally mirrors the v1 ``VuLsu`` (same FSM, same instruction handling)
// but talks to the TCDM through master ports built on ``vp/itf/io_v2.hpp``:
// status codes are ``IO_REQ_DONE``/``GRANTED``/``DENIED``, error reporting
// rides on the response status (``IO_RESP_OK``/``IO_RESP_INVALID``), and the
// retry/resp callbacks are mandatory (passed at port construction time).
//
// All three downstream forms are supported, so the VLSU ports can face a
// deny/retry arbiter such as ``interco.log_ico_v2``:
//   - DONE: completion is scheduled ``get_full_latency()`` cycles later
//     through the delayed-bursts priority queue.
//   - GRANTED: completion arrives through the resp() callback.
//   - DENIED: the request is parked on its port and re-issued synchronously
//     inside the retry() callback (mandatory io_v2 contract — the log ico
//     keeps its accept window open only for the duration of the retry call).
//     Only the denied port stalls; the other ports keep streaming.
class VuLsu : public VuBlock
{
public:

    void reset(bool active) override;
    void start() override;
    VuLsu(Vu &vu, Iss &iss);
    bool is_full() override { return this->nb_pending_insn.get() == VuLsu::queue_size; }
    void enqueue_insn(PendingInsn *pending_insn) override;
    void isa_init() override;

private:
    // Handler for internal FSM
    static void fsm_handler(vp::Block *__this, vp::ClockEvent *event);
    // Handler called when a load instruction starts to be processed, in order to initialize the FSM
    // for a read burst
    static void handle_insn_load(VuLsu *vlsu, iss_insn_t *insn);
    // Handler called when a write instruction starts to be processed, in order to initialize the FSM
    // for a write burst
    static void handle_insn_store(VuLsu *vlsu, iss_insn_t *insn);
    // Handler called when a stride load instruction starts to be processed, in order to initialize
    // the FSM for a read burst
    static void handle_insn_load_strided(VuLsu *vlsu, iss_insn_t *insn);
    // Handler called when a strided write instruction starts to be processed, in order to
    // initialize the FSM for a write burst
    static void handle_insn_store_strided(VuLsu *vlsu, iss_insn_t *insn);
    // Handler called when an indexed load instruction starts to be processed, in order to initialize
    // the FSM for a read burst
    static void handle_insn_load_indexed(VuLsu *vlsu, iss_insn_t *insn);
    // Handler called when an indexed write instruction starts to be processed, in order to
    // initialize the FSM for a write burst
    static void handle_insn_store_indexed(VuLsu *vlsu, iss_insn_t *insn);

    void handle_access(iss_insn_t *insn, bool is_write, int reg, bool do_stride=false, iss_reg_t stride=0, int reg_indexed=-1);

    // io_v2 master callbacks — pure ready/valid signal (no request) on retry,
    // and response notification on resp.
    static void port_retry_muxed(vp::Block *__this, int id, vp::IoRetryChannel);
    static vp::IoRespAck port_resp_muxed(vp::Block *__this, vp::IoReq *req, int id);

    // Advance the issue bookkeeping of a burst submitted to its port. denied_reqs
    // models the RTL output spill, so a denied burst has already left the
    // instruction and advances at submission, not when its retry succeeds.
    void burst_issued(vp::IoReq *req, int port);
    // Called when a request has been accepted with DONE: complete it now or
    // push it to the delayed-bursts queue for get_full_latency() cycles.
    void handle_done(vp::IoReq *req);
    // Terminate one burst: commit its elements to the VRF and retire it.
    void burst_done(vp::IoReq *req);

    // Number of instruction that can be enqueued at the same time
    static constexpr int queue_size = 4;

    Vu &vu;
    vp::Trace trace;
    // Event for active state
    vp::Trace event_active;
    // Event for address of current AXI burst
    std::vector<vp::Trace> event_addr;
    // Event for size of current AXI burst
    std::vector<vp::Trace> event_size;
    // Event for write or read opcode of current AXI burst
    std::vector<vp::Trace> event_is_write;
    // Event for PC of enqueued instructions
    vp::Trace event_queue;
    // Event for PC of instruction being processed
    vp::Trace event_pc;
    // Event for label of instruction being processed
    vp::Event event_label;
    // Clock event used for scheduling FSM handler when at least one instruction has to be processed
    vp::ClockEvent fsm_event;
    // Queue of pending instructions to be processed by this block
    // The block process them in-order
    std::vector<VuLsuPendingInsn> insns;
    // Base address of the current memory operation
    iss_addr_t pending_addr;
    // Total size of the current load/store operation, fixed during execution
    iss_addr_t pending_size;
    // Size of one burst of the current load/store operation. One element for a strided
    // or indexed operation, one lane otherwise.
    iss_addr_t burst_size;
    // Remaining size of the current load/store operation
    iss_addr_t remaining_size;
    // Write or read of the current load/store operation
    bool pending_is_write;
    // Base pointer in the vector register file for the current memory operation
    uint8_t *pending_velem;
    // Thsi indicates the vector register involved in the load/sotre operation.
    // Used for vector chaining to commit elements to correct register.
    int pending_vreg;
    // First valid instruction in the queue.
    int insn_first;
    // First valid instruction in the queue waiting to be started
    int insn_first_waiting;
    // Index in the queue where the next instruction should be pushed.
    int insn_last;
    // Number of enqueued instructions
    vp::Register<uint8_t> nb_pending_insn;
    // Number of instructions waiting to be started
    int nb_waiting_insn;
    // Ports to the TCDM, used by VLSU for vector load and store operations
    std::vector<vp::IoMaster> ports;
    // Number of TCDM ports
    int nb_ports;
    iss_reg_t stride;
    bool strided;
    int elem_size;
    int reg_indexed;
    int inst_elem_size;
    int64_t op_timestamp;
    bool prev_is_write;
    bool started;
    int vstart;

    // Independent static issue: global burst k belongs to port k % nb_ports.
    // Each port keeps only its submitted-burst count and DENIED state; burst
    // count/size derive from the transfer size and access mode.
    std::vector<int> port_burst;
    std::vector<bool> port_stalled;
    // Per-port request denied by the downstream and waiting for its retry()
    // signal. io_v2 requires the re-issue to happen inside the retry callback,
    // so the request itself must be kept, not just the stalled flag.
    std::vector<vp::IoReq *> denied_reqs;

    // Instruction currently active in the VLSU. pending_insn->timestamp is reused across phases:
    // 1. as an enqueue-cycle guard, 2. as the request issuing start time after instruction latency,
    // and 3. for memory-response/retirement timing. Keeping this index separate from insn_first_waiting
    // makes the phase explicit and prevents queued instructions from consuming their instruction latency
    // before they become active.
    int insn_ongoing;
    // Bursts which have been handled synchronously with a delay. There are hold here until their
    // delay has elapsed
    struct DelayedBurst
    {
        vp::IoReq *req;
        uint64_t timestamp;
    };

    struct DelayedBurstCompare
    {
        bool operator()(const DelayedBurst &a, const DelayedBurst &b) const
        {
            return a.timestamp > b.timestamp;
        }
    };

    std::priority_queue<DelayedBurst, std::vector<DelayedBurst>, DelayedBurstCompare> delayed_bursts;

    // A burst in flight. Both loads and stores take their request from the same pool,
    // and the request describes the elements it covers, so that it can be committed
    // when it completes: through its ROB entry for a load, directly for a store.
    struct VlsuReq
    {
        vp::IoReq req;
        // Copy of the vector elements to be stored. The register file is read when the
        // request is sent, but the request is only applied when it reaches the target,
        // when a younger instruction may already have overwritten the register. Unused
        // by loads, which write the register file from the response data.
        std::vector<uint8_t> data;
        // Instruction slot which issued the request
        VuLsuPendingInsn *slot = nullptr;
        // Port on which the request was sent
        int port = 0;
        // Vector register and range of elements covered by the request
        int vreg = 0;
        int vstart = 0;
        int nb_elem = 0;
    };

    // Reorder Buffer for mempool configuration. It only tracks the completion order of
    // the loads, the requests themselves being described by VlsuReq.
    struct VlsuRobEntry
    {
        // Port and id
        int port = 0;
        int rob_id = 0;

        // If the entry is allocated to a request
        bool allocated = false;

        // If the response is valid
        bool valid = false;

        // Request itself
        VlsuReq *req = nullptr;
    };

    // Reorder buffer
    std::vector<std::vector<VlsuRobEntry>> rob;
    // Next available entry in the ROB for each port
    std::vector<int> rob_next;
    // The first entry in the ROB which is waiting for response for each port
    std::vector<int> rob_first;
    // Number of allocated entries in the ROB for each port
    std::vector<int> rob_count;

    // Requests, allocated on demand and recycled once their response is received. The
    // pool puts no limit on the number of outstanding bursts, like RTL where the number
    // of loads is limited by the ROB and the number of stores by nothing. A deque keeps
    // the requests already in flight at a stable address while the pool grows.
    std::deque<VlsuReq> reqs;
    // Requests which are not in flight
    std::vector<VlsuReq *> reqs_free;
    // Number of store bursts waiting for their response
    int nb_pending_stores;
};

#else

// Block processing load/store vector instructions
class VuLsu : public VuBlock
{
public:

    void reset(bool active) override;
    void start() override;
    VuLsu(Vu &vu, Iss &iss);
    bool is_full() override { return this->nb_pending_insn.get() == VuLsu::queue_size; }
    void enqueue_insn(PendingInsn *pending_insn) override;
    void isa_init() override;

private:
    // Handler for internal FSM
    static void fsm_handler(vp::Block *__this, vp::ClockEvent *event);
    // Handler called when a load instruction starts to be processed, in order to initialize the FSM
    // for a read burst
    static void handle_insn_load(VuLsu *vlsu, iss_insn_t *insn);
    // Handler called when a write instruction starts to be processed, in order to initialize the FSM
    // for a write burst
    static void handle_insn_store(VuLsu *vlsu, iss_insn_t *insn);
    // Handler called when a stride load instruction starts to be processed, in order to initialize
    // the FSM for a read burst
    static void handle_insn_load_strided(VuLsu *vlsu, iss_insn_t *insn);
    // Handler called when a strided write instruction starts to be processed, in order to
    // initialize the FSM for a write burst
    static void handle_insn_store_strided(VuLsu *vlsu, iss_insn_t *insn);
    // Handler called when an indexed load instruction starts to be processed, in order to initialize
    // the FSM for a read burst
    static void handle_insn_load_indexed(VuLsu *vlsu, iss_insn_t *insn);
    // Handler called when an indexed write instruction starts to be processed, in order to
    // initialize the FSM for a write burst
    static void handle_insn_store_indexed(VuLsu *vlsu, iss_insn_t *insn);

    void handle_access(iss_insn_t *insn, bool is_write, int reg, bool do_stride=false, iss_reg_t stride=0, int reg_indexed=-1);

    // Handler for asynchronous burst grants
    static void data_grant(vp::Block *__this, vp::IoReq *req);
    // Handler for asynchronous burst responses
    static void data_response(vp::Block *__this, vp::IoReq *req);

    // Number of instruction that can be enqueued at the same time
    static constexpr int queue_size = 4;

    Vu &vu;
    vp::Trace trace;
    // Event for active state
    vp::Trace event_active;
    // Event for address of current AXI burst
    std::vector<vp::Trace> event_addr;
    // Event for size of current AXI burst
    std::vector<vp::Trace> event_size;
    // Event for write or read opcode of current AXI burst
    std::vector<vp::Trace> event_is_write;
    // Event for PC of enqueued instructions
    vp::Trace event_queue;
    // Event for PC of instruction being processed
    vp::Trace event_pc;
    // Event for label of instruction being processed
    vp::Event event_label;
    // Clock event used for scheduling FSM handler when at least one instruction has to be processed
    vp::ClockEvent fsm_event;
    // Queue of pending instructions to be processed by this block
    // The block process them in-order
    std::vector<VuLsuPendingInsn> insns;
    // Base address of the current memory operation
    iss_addr_t pending_addr;
    // Total size of the current load/store operation, fixed during execution
    iss_addr_t pending_size;
    // Burst size of the current operation: one element for a strided or indexed
    // access, one lane otherwise.
    iss_addr_t burst_size;
    // Remaining size of the current load/store operation
    iss_addr_t remaining_size;
    // Write or read of the current load/store operation
    bool pending_is_write;
    // Base pointer in the vector register file for the current memory operation
    uint8_t *pending_velem;
    // Thsi indicates the vector register involved in the load/sotre operation.
    // Used for vector chaining to commit elements to correct register.
    int pending_vreg;
    // First valid instruction in the queue.
    int insn_first;
    // First valid instruction in the queue waiting to be started
    int insn_first_waiting;
    // Index in the queue where the next instruction should be pushed.
    int insn_last;
    // Number of enqueued instructions
    vp::Register<uint8_t> nb_pending_insn;
    // Number of instructions waiting to be started
    int nb_waiting_insn;
    // Ports to the TCDM, used by VLSU for vector load and store operations
    std::vector<vp::IoMaster> ports;
    // Number of TCDM ports
    int nb_ports;
    iss_reg_t stride;
    bool strided;
    int elem_size;
    int reg_indexed;
    int inst_elem_size;
    int64_t op_timestamp;
    bool prev_is_write;
    bool started;
    int vstart;

    // Independent static issue: global burst k belongs to port k % nb_ports.
    // Each port keeps only its accepted-burst count and DENIED state; burst
    // count/size are derived from the immutable transfer size and access mode.
    std::vector<int> port_burst;
    std::vector<bool> port_stalled;

    // Instruction currently active in the VLSU. pending_insn->timestamp is reused across phases:
    // 1. as an enqueue-cycle guard, 2. as the request issuing start time after instruction latency,
    // and 3. for memory-response/retirement timing. Keeping this index separate from insn_first_waiting
    // makes the phase explicit and prevents queued instructions from consuming their instruction latency
    // before they become active.
    int insn_ongoing;
    // Bursts which have been handled synchronously with a delay. There are hold here until their
    // delay has elapsed
    struct DelayedBurst
    {
        vp::IoReq *req;
        uint64_t timestamp;
    };

    struct DelayedBurstCompare
    {
        bool operator()(const DelayedBurst &a, const DelayedBurst &b) const
        {
            return a.timestamp > b.timestamp;
        }
    };

    std::priority_queue<DelayedBurst, std::vector<DelayedBurst>, DelayedBurstCompare> delayed_bursts;

    // A burst in flight. Both loads and stores take their request from the same pool,
    // and the request describes the elements it covers, so that it can be committed
    // when it completes: through its ROB entry for a load, directly for a store.
    struct VlsuReq
    {
        vp::IoReq req;
        // Copy of the vector elements to be stored. The register file is read when the
        // request is sent, but the request is only applied when it reaches the target,
        // when a younger instruction may already have overwritten the register. Unused
        // by loads, which write the register file from the response data.
        std::vector<uint8_t> data;
        // Instruction slot which issued the request
        VuLsuPendingInsn *slot = nullptr;
        // Port on which the request was sent
        int port = 0;
        // Vector register and range of elements covered by the request
        int vreg = 0;
        int vstart = 0;
        int nb_elem = 0;
    };

    // Reorder Buffer for mempool configuration. It only tracks the completion order of
    // the loads, the requests themselves being described by VlsuReq.
    struct VlsuRobEntry
    {
        // Port and id
        int port = 0;
        int rob_id = 0;

        // If the entry is allocated to a request
        bool allocated = false;

        // If the response is valid
        bool valid = false;

        // Request itself
        VlsuReq *req = nullptr;
    };

    // Reorder buffer
    std::vector<std::vector<VlsuRobEntry>> rob;
    // Next available entry in the ROB for each port
    std::vector<int> rob_next;
    // The first entry in the ROB which is waiting for response for each port
    std::vector<int> rob_first;
    // Number of allocated entries in the ROB for each port
    std::vector<int> rob_count;

    // Requests, allocated on demand and recycled once their response is received. The
    // pool puts no limit on the number of outstanding bursts, like RTL where the number
    // of loads is limited by the ROB and the number of stores by nothing. A deque keeps
    // the requests already in flight at a stable address while the pool grows.
    std::deque<VlsuReq> reqs;
    // Requests which are not in flight
    std::vector<VlsuReq *> reqs_free;
    // Number of store bursts waiting for their response
    int nb_pending_stores;
};

#endif // CONFIG_GVSOC_ISS_VLSU_V2

#else

// Block processing load/store vector instructions
class VuLsu : public VuBlock
{
public:

    void reset(bool active) override;
    VuLsu(Vu &vu, Iss &iss);
    bool is_full() override { return this->nb_pending_insn.get() == VuLsu::queue_size; }
    void enqueue_insn(PendingInsn *pending_insn) override;
    void isa_init() override;

private:
    // Handler for internal FSM
    static void fsm_handler(vp::Block *__this, vp::ClockEvent *event);
    // Handler called when a load instruction starts to be processed, in order to initialize the FSM
    // for a read burst
    static void handle_insn_load(VuLsu *vlsu, iss_insn_t *insn);
    // Handler called when a write instruction starts to be processed, in order to initialize the FSM
    // for a write burst
    static void handle_insn_store(VuLsu *vlsu, iss_insn_t *insn);
    // Handler for asynchronous burst grants
    static void data_grant(vp::Block *__this, vp::IoReq *req);
    // Handler for asynchronous burst responses
    static void data_response(vp::Block *__this, vp::IoReq *req);
    void handle_burst_end(vp::IoReq *req);

    // Number of bursts which can be sent at the same time
    static constexpr int nb_burst = 8;
    // Number of instruction that can be enqueued at the same time
    static constexpr int queue_size = 4;

    Vu &vu;
    vp::Trace trace;
    // Event for active state
    vp::Trace event_active;
    // Event for address of current AXI burst
    vp::Trace event_addr;
    // Event for size of current AXI burst
    vp::Trace event_size;
    // Event for write or read opcode of current AXI burst
    vp::Trace event_is_write;
    // Event for PC of enqueued instructions
    vp::Trace event_queue;
    // Event for PC of instruction being processed
    vp::Trace event_pc;
    // Event for label of instruction being processed
    vp::Event event_label;
    // Clock event used for scheduling FSM handler when at least one instruction has to be processed
    vp::ClockEvent fsm_event;
    // Queue of pending instructions to be processed by this block
    // The block process them in-order
    std::vector<VuLsuPendingInsn> insns;
    // Address of the next burst to be sent
    iss_addr_t pending_addr;
    // Remaining size of the current load/store operation
    iss_addr_t pending_size;
    // Write or read of the current load/store operation
    bool pending_is_write;
    // Pointer to vector register file where next burst should read or written
    uint8_t *pending_velem;
    // Queue of available bursts for load/store
    std::queue<vp::IoReq *> free_bursts;
    // Memory interface for load/store bursts
    vp::IoMaster io_itf;
    // True if one burst was not granted. Once it is true, the block can not send any burst
    // anymore until the last one is granted
    bool stalled;
    // Bursts which have been handled synchronously with a delay. There are hold here until their
    // delay has elapsed
    std::queue<vp::IoReq *> delayed_bursts;
    // Timestamps where burst in delayed_bursts can be released
    std::queue<int64_t> delayed_bursts_timestamps;
    // First valid instruction in the queue.
    int insn_first;
    // First valid instruction in the queue waiting to be started
    int insn_first_waiting;
    // Index in the queue where the next instruction should be pushed.
    int insn_last;
    // Number of enqueued instructions
    vp::Register<uint8_t> nb_pending_insn;
    // Number of instructions waiting to be started
    int nb_waiting_insn;
    int elem_size;
    int vstart;

    // Instruction currently active in the VLSU. It is either waiting for its instruction latency to elapse 
    // or already actively issuing memory requests. This is kept separate from insn_first_waiting for the following reason. 
    // pending_insn->timestamp has multiple purposes:
    // 1. At the instruction enqueue time, it is set to (enqueue cycle + 1) to avoid starting to execute the instruction immediately in the same cycle;
    // 2. At the instruction execution start, it is set to (current cycle + instruction latency), to prevent the instruction from issuing memory requests before its instruction latency elapses;
    // 3. During the instruction execution, it is used to track the synchronous memory access latency (current cycle + request latency) and instruction retirement time.
    // In order to distinguish between the first two cases in the FSM, and ensure that instruction latency is only modeled once,
    // we seperate index insn_ongoing (with timestamp of purpose 2 or 3) from insn_first_waiting (with timestamp of purpose 1).
    // This also prevents the younger instructions waiting in the queue from consuming its latency while the older instruction is still active.
    int insn_ongoing;
};

#endif

// Vu top block
class Vu : public vp::Block
{
    friend class VuCompute;
    friend class VuLsu;

public:
    // List of sub-blocks processing instructions
    typedef enum
    {
        vlsu_id,
        vfpu_id,
        vslide_id,
        nb_blocks
    } blocks_e;

    Vu(Iss &iss);

    void reset(bool reset);
    // Called by ISS to initialize vector instructions in the ISA decoding tree to attach needed
    // handlers
    void isa_init();
    // Called by sub-blocks to notify the end of processing of an instruction. The instruction
    // The instruction handler is called, the instruction is removed ffrom the sequencer, and the
    // registers involved is updated
    void insn_end(PendingInsn *insn);
    void insn_commit(PendingInsn *pending_insn, int nb_elems);
    bool insn_ready(PendingInsn *insn);
    // Same, for an instruction about to read the elements up to next_read rather than
    // the chunk after the ones it already processed.
    bool insn_ready(PendingInsn *insn, int next_read);
    // Called by the ISS to offload an instruction to vu. The instruction is pushed to the queue
    // of pending instruction, analyzed, and push to a processing block for executing once
    // dependencies are resolved. Can be called only when queue is not full.
    void insn_enqueue(InsnEntry *insn);
    // Return true when queue if full and vu can not accept new instructions
    bool queue_is_full() { return this->queue_full.get(); }
    bool queue_is_empty() { return this->nb_pending_insn == 0; }
    // Return the CVA6 register value associated to the instruction being executed
    inline uint64_t current_insn_reg_get() { return current_insn_reg; }
    inline uint64_t current_insn_reg_2_get() { return current_insn_reg_2; }

    void dump_regs_to_trace(iss_insn_t *insn, PendingInsn *pending_insn, int nb_elem, bool is_out);
    inline void exec_insn_chunk(iss_insn_t *insn, PendingInsn *pending_insn, int vstart,
        int vend, int nb_elem);

    // Access to upper ISS
    Iss &iss;
    // Number of <lane_width> bits lanes in vu.
    int nb_lanes;
    // Width in bits of one lane
    int lane_width;

    // Number of pending vector loads and stores in spatz. Use to synchronize with snitch memory
    // accesses
    int nb_pending_vaccess;
    // Number of pending vector stores in spatz. Use to synchronize with snitch memory accesses
    int nb_pending_vstore;

    int vstart;
    int vend;
    vp::Trace trace;
    uint64_t saved_value;
    void insn_handle_reduction();

private:
    void insn_commit(PendingInsn *pending_insn);
    static iss_reg_t vector_insn_stub_handler(Iss *iss, iss_insn_t *insn, iss_reg_t pc);

    static iss_reg_t load_store_handler(Iss *iss, iss_insn_t *insn, iss_reg_t pc);
    // Handler for internal FSM
    static void fsm_handler(vp::Block *__this, vp::ClockEvent *event);
    // Allocate a slot for an instruction being enqueued. This is used to duplicate the cva6
    // pending instruction to commit it early so that cva6 can use the slot for another instruction.
    PendingInsn *pending_insn_alloc(InsnEntry *entry);
    inline int alloc_id();
    inline void free_id(int id);

    // Size of the queue holding pending instructions. Once full, vu can not accept instructions
    // from CVA6 anymore. Spatz can handle 4 instructions at a time.
    static constexpr int queue_size = 4;

    // Event for active state
    vp::Trace event_active;
    // Event for PC of instruction being enqueued
    vp::Trace event_pc;
    // Event for PC of enqueued instructions
    vp::Trace event_queue;
    // Event for label of instruction being enqueued
    vp::Trace event_label;
    // Clock event used for scheduling FSM handler when at least one instruction has to be processed
    vp::ClockEvent fsm_event;
    // Number of instructions currently being processed by vu. This is increased when an
    // instruction is enqueued, and decreased when it ends
    vp::Register<uint8_t> nb_pending_insn;
    // List of instructions currently being processed by vu. This contains both the instructions
    // waiting for the resolution of their dependencies, and the instructions already dispatched
    // to the processing blocks.
    std::vector<PendingInsn> pending_insns;
    // True if the queue is full and vu can not accept any other instruction
    vp::Register<bool> queue_full;
    // List of processing blocks
    std::vector<VuBlock *> blocks;
    // CVA6 register associated to the instruction being executed. This is used by instruction
    // handlers when they are executed. This needs to be buffered because CVA6 might have executed
    // following instructions overriding the register
    uint64_t current_insn_reg;
    uint64_t current_insn_reg_2;
    uint64_t insn_id_table;

    std::queue<PendingInsn *> stalled_insns;
    std::vector<uint64_t> insns_in_deps;
    std::vector<uint64_t> insns_out_deps;
    uint64_t writing_insns[32];
    uint64_t reading_insns[32];
    int insn_latency;

#ifdef CONFIG_GVSOC_STATS_ACTIVE
    // Per-label execution-duration statistics for vector instructions, dumped
    // under the "vinsn_duration" group. A vector instruction waits in the Vu
    // queue and in its block before really executing, so duration is measured
    // from the block's real start (exec_start_cycle) to Vu::insn_end.
    bool stats_enabled = false;
    InsnDurationStats insn_durations;
#endif
};

inline int Vu::alloc_id()
{
    int id = __builtin_ctzll(this->insn_id_table);
    this->insn_id_table &= ~(1ULL << id);
    return id;
}

inline void Vu::free_id(int id)
{
    this->insn_id_table |= (1ULL << id);
}

inline void Vu::exec_insn_chunk(iss_insn_t *insn, PendingInsn *pending_insn, int vstart,
    int vend, int nb_elem)
{
    this->vstart = vstart;
    this->vend = vend;

#ifdef VP_TRACE_ACTIVE
    this->dump_regs_to_trace(insn, pending_insn, nb_elem, false);
#endif

    insn->stub_handler(&this->iss, insn, insn->addr);

#ifdef VP_TRACE_ACTIVE
    this->dump_regs_to_trace(insn, pending_insn, nb_elem, true);
#endif
}

inline void Vu::insn_handle_reduction()
{
    // Called once per CHUNK from rv32v_timed.hpp's vfred*/vred* handlers (which
    // exec_insn_chunk reaches through insn->stub_handler), so this is a flat
    // per-chunk cost, not a per-element rate. A unit which reduces serially
    // overrides it in VuCompute::fsm_handler with reduction_step_latency.
    this->insn_latency = 3;
}
