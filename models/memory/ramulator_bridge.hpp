// SPDX-FileCopyrightText: 2026 ETH Zurich, University of Bologna and EssilorLuxottica SAS
//
// SPDX-License-Identifier: Apache-2.0
//
// Authors: Germain Haugou (germain.haugou@gmail.com)
//
// Bridge between Ramulator's per-command stream (the data its LiveTraceStreamer
// / CmdTraceRecorder plugins feed to the Ramulator visualizer) and the GVSoC
// memory.ramulator wrapper, so DRAM commands can be drawn on the gvsoc-gui3
// timeline.
//
// The GvsocBridge controller plugin (ramulator_bridge.cpp) reports every issued
// DRAM command to an IDramCommandSink implemented by the wrapper. Plugin and
// wrapper are compiled into the same model .so, so this is a plain in-process
// C++ interface (no ABI/dlsym concerns). This header is deliberately free of
// Ramulator headers so the wrapper can include it alongside the GVSoC headers.
//
// Binding: a Ramulator instance is built synchronously inside the wrapper
// constructor; the wrapper publishes itself as the "pending sink" right before
// connecting the memory system (where the plugin's setup() runs) and clears it
// after. The plugin claims the pending sink in setup(). Single-threaded
// construction makes this safe even with several memory.ramulator instances.

#pragma once

#include <string>
#include <vector>

namespace ramulator_bridge
{

// One issued DRAM command, forwarded from the plugin to the wrapper. Pointers
// are valid only for the duration of the on_dram_command() call.
struct DramCommand
{
    long long clk;              // controller clock cycle (Ramulator m_clk)
    int channel;                // controller channel id
    int command_id;             // Ramulator command id
    const char *command_name;   // resolved name, e.g. "ACT" / "RD" / "PREpb"
    const int *addr_vec;        // per-level address (AddrVec_t), -1 if unused
    int level_count;
    int type_id;                // 0 read / 1 write / -1 maintenance
    int source_id;
};

// Implemented by the GVSoC wrapper.
class IDramCommandSink
{
public:
    virtual ~IDramCommandSink() = default;

    // Called once, when the bridge plugin is set up, with the DRAM address
    // layout for this controller/channel (names + per-level sizes, as in
    // DRAMSpec::level_names and Organization::level_sizes).
    virtual void on_dram_setup(int channel,
        const std::vector<std::string> &level_names,
        const std::vector<int> &level_sizes) = 0;

    // Called for every issued DRAM command (Ramulator IControllerPlugin::on_issue).
    virtual void on_dram_command(const DramCommand &cmd) = 0;
};

// Pending-sink handshake (see file header). set_pending_sink() is called by the
// wrapper before building/connecting its Ramulator instance; take_pending_sink()
// is called once by the plugin's setup() and returns (and clears) it.
void set_pending_sink(IDramCommandSink *sink);
IDramCommandSink *take_pending_sink();

}  // namespace ramulator_bridge
