// SPDX-FileCopyrightText: 2026 ETH Zurich, University of Bologna and EssilorLuxottica SAS
//
// SPDX-License-Identifier: Apache-2.0
//
// Authors: Germain Haugou (germain.haugou@gmail.com)
//
// GvsocBridge — a Ramulator IControllerPlugin that forwards every issued DRAM
// command to the GVSoC memory.ramulator wrapper (via IDramCommandSink), so the
// command stream can be drawn on the gvsoc-gui3 timeline. Same data as
// Ramulator's LiveTraceStreamer / CmdTraceRecorder plugins.
//
// Compiled INTO the memory.ramulator model .so (a direct model source, so its
// static RAMULATOR_REGISTER_IMPLEMENTATION registrar is retained). It links the
// same libramulator as the wrapper, so it registers into the shared global
// Factory registry and is resolved by name ("GvsocBridge") from the YAML
// `controller_plugins` list — no patch to libramulator.

#include "ramulator_bridge.hpp"

#include "ramulator/base/base.h"
#include "ramulator/base/request.h"
#include "ramulator/controller/controller_base.h"
#include "ramulator/controller/plugin/i_controller_plugin.h"
#include "ramulator/dram/dram_spec.h"
#include "ramulator/frontend/i_frontend.h"
#include "ramulator/memory_system/i_memory_system.h"

namespace ramulator_bridge
{

// Pending-sink handshake storage (see ramulator_bridge.hpp). Single-threaded
// construction; no locking needed.
static IDramCommandSink *s_pending_sink = nullptr;

void set_pending_sink(IDramCommandSink *sink) { s_pending_sink = sink; }

IDramCommandSink *take_pending_sink()
{
    IDramCommandSink *s = s_pending_sink;
    s_pending_sink = nullptr;
    return s;
}

}  // namespace ramulator_bridge


namespace Ramulator
{

class GvsocBridge : public IControllerPlugin, public Implementation
{
    RAMULATOR_REGISTER_IMPLEMENTATION(IControllerPlugin, GvsocBridge, "GvsocBridge")

public:
    void init() override {}

    void setup(IFrontEnd *frontend, IMemorySystem *memory_system) override
    {
        m_ctrl = cast_parent<ControllerBase>();
        m_sink = ramulator_bridge::take_pending_sink();
        if (m_sink != nullptr)
        {
            const auto &spec = *m_ctrl->m_device.m_spec;
            m_sink->on_dram_setup(m_ctrl->m_channel_id, spec.level_names,
                                  spec.organization.level_sizes);
        }
    }

    void on_issue(const Request &req) override
    {
        if (m_sink == nullptr) return;
        const auto &spec = *m_ctrl->m_device.m_spec;
        ramulator_bridge::DramCommand cmd{
            (long long)m_ctrl->m_clk,
            m_ctrl->m_channel_id,
            req.command,
            spec.command_names[req.command].c_str(),
            req.addr_vec.data(),
            (int)req.addr_vec.size(),
            req.type_id,
            req.source_id,
        };
        m_sink->on_dram_command(cmd);
    }

private:
    ControllerBase *m_ctrl = nullptr;
    ramulator_bridge::IDramCommandSink *m_sink = nullptr;
};

}  // namespace Ramulator
