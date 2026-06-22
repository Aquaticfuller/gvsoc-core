/*
 * Copyright (C) 2026 ETH Zurich and University of Bologna.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 */

/*
 * CachePool fake-UART byte sink.
 *
 * The CachePool snRuntime printf path (`_putchar`, software/snRuntime/src/printf.c) does a single-byte
 * store to the linker-absolute symbol `fake_uart = 0xC0010000` (common.ld = RTL `UartAddr`,
 * cachepool_pkg.sv:91). The RTL models this with `axi_uart.sv` (an always-ready write-only sink that
 * forwards each byte to the testbench `$write`). This is the ENTIRE stdout path for the CachePool
 * benchmarks — they do NOT use the HTIF putc syscall variant (that #define is disabled).
 *
 * This model is the GVSoC twin: a write-only, always-ready MMIO device. Every written byte is emitted to
 * the simulator's stdout (fd 1), mirroring axi_uart.sv:108-126. Reads return 0.
 */

#include <vp/vp.hpp>
#include <vp/itf/io.hpp>
#include <unistd.h>
#include <string.h>

class CachePoolUart : public vp::Component
{
public:
    CachePoolUart(vp::ComponentConf &conf);

private:
    static vp::IoReqStatus req_handler(vp::Block *__this, vp::IoReq *req);
    vp::IoSlave input_itf_;
    vp::Trace   trace_;
};

CachePoolUart::CachePoolUart(vp::ComponentConf &conf) : vp::Component(conf)
{
    this->input_itf_.set_req_meth(&CachePoolUart::req_handler);
    this->new_slave_port("input", &this->input_itf_);
    this->traces.new_trace("trace", &this->trace_, vp::DEBUG);
}

vp::IoReqStatus CachePoolUart::req_handler(vp::Block *__this, vp::IoReq *req)
{
    CachePoolUart *_this = static_cast<CachePoolUart *>(__this);
    uint8_t *data = req->get_data();
    if (req->get_is_write())
    {
        if (data != nullptr)
        {
            // Emit each written byte to stdout (the RTL axi_uart forwards to the TB $write).
            for (uint64_t i = 0; i < req->get_size(); i++)
            {
                char c = (char)data[i];
                if (::write(1, &c, 1) != 1) { /* best-effort, ignore */ }
            }
        }
    }
    else if (data != nullptr)
    {
        // Always-ready, status-zero reads (the snRuntime path never reads the UART).
        memset(data, 0, req->get_size());
    }
    return vp::IO_REQ_OK;
}

extern "C" vp::Component *gv_new(vp::ComponentConf &config)
{
    return new CachePoolUart(config);
}
