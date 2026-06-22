#
# Copyright (C) 2026 ETH Zurich and University of Bologna.
#
# Licensed under the Apache License, Version 2.0 (the "License").
#

import gvsoc.systree


class CachePoolUart(gvsoc.systree.Component):
    """CachePool fake-UART byte sink (RTL `axi_uart.sv` / `fake_uart` @ 0xC0010000).

    A write-only, always-ready MMIO device: every written byte is emitted to the simulator stdout.
    This is the entire stdout path for the CachePool snRuntime printf (`_putchar` stores a byte to
    `fake_uart`). Map it at the CachePool `UartAddr` (0xC0010000) on the SoC narrow AXI.
    """

    def __init__(self, parent: gvsoc.systree.Component, name: str):
        super().__init__(parent, name)
        self.add_sources(['devices/uart/cachepool_uart.cpp'])

    def i_INPUT(self) -> gvsoc.systree.SlaveItf:
        return gvsoc.systree.SlaveItf(self, 'input', signature='io')
