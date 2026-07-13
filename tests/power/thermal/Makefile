# SPDX-FileCopyrightText: 2026 ETH Zurich, University of Bologna and EssilorLuxottica SAS
#
# SPDX-License-Identifier: Apache-2.0
#
# Authors: Germain Haugou (germain.haugou@gmail.com)
GVSOC_ROOT ?= ../../../..
TARGET = test
CASE ?= heating
TARGET := $(TARGET):case=$(CASE)

# The thermal model is only active when power modeling is enabled
runner_args += --power

include $(GVSOC_CORE)/tests/common.mk
