/*
 * Copyright (C) 2022 GreenWaves Technologies, SAS, ETH Zurich and
 *                    University of Bologna
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
 * Authors: Germain Haugou, GreenWaves Technologies (germain.haugou@greenwaves-technologies.com)
 */

#pragma once

#include <stdint.h>
#include "vp/itf/wire.hpp"

class TrafficReceiverConfig
{
public:
    bool is_start = false;
    int bandwidth;
    // Pacing rate is <bandwidth> bytes per <bandwidth_period> cycles, so
    // sub-byte-per-cycle and fractional rates are expressible (e.g. one 8-byte
    // beat every 3.5 cycles is bandwidth=16, bandwidth_period=7). The v2
    // receiver accumulates the remainder across requests instead of rounding
    // per request.
    int bandwidth_period = 1;
    // Fixed response delay in cycles, applied to every request on top of the
    // pacing. Pipelined: delays the response without holding back the pacing
    // of the following requests.
    int latency = 0;
};


class TrafficReceiverConfigMaster : public vp::WireMaster<TrafficReceiverConfig>
{
public:
    inline void start(int bandwidth, int bandwidth_period=1, int latency=0);
};


inline void TrafficReceiverConfigMaster::start(int bandwidth, int bandwidth_period, int latency)
{
    TrafficReceiverConfig config = { .is_start=true, .bandwidth=bandwidth,
        .bandwidth_period=bandwidth_period, .latency=latency };
    this->sync(config);
}
