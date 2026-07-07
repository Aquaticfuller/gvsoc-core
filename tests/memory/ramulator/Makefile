# SPDX-FileCopyrightText: 2026 ETH Zurich, University of Bologna and EssilorLuxottica SAS
#
# SPDX-License-Identifier: Apache-2.0
#
# Authors: Germain Haugou (germain.haugou@gmail.com)
GVSOC_ROOT ?= ../../../..
TARGET = test
CASE ?= read_basic
MEM ?= ramulator
REPEAT ?= 1
TARGET := $(TARGET):case=$(CASE):mem=$(MEM):repeat=$(REPEAT)

include $(GVSOC_CORE)/tests/common.mk
