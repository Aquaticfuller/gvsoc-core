/*
 * Copyright (C) 2020 GreenWaves Technologies, SAS, ETH Zurich and
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

#include <vp/vp.hpp>
#include <cpu/iss_v2/include/types.hpp>

/*
 * ISS-side entry points of the memcheck semihosting calls. The runtime allocator
 * declares its allocations here; they are registered in the engine-level registry
 * (vp::MemCheck) which hands back a buffer ID, and the ID gets attached to the
 * register receiving the pointer (a0) so the regfile and LSU propagate it
 * alongside the pointer.
 */
class Memcheck
{
public:
    Memcheck(Iss &iss);
    // Semihosting 0x114: register an allocated buffer, taint a0 with its ID.
    // The pointer is returned unchanged.
    iss_reg_t mem_alloc(iss_reg_t mem_id, iss_reg_t ptr, iss_reg_t size);
    // Semihosting 0x115: mark the buffer as freed. The pointer is returned
    // unchanged.
    iss_reg_t mem_free(iss_reg_t mem_id, iss_reg_t ptr, iss_reg_t size);

private:
    Iss &iss;
    vp::Trace trace;
};
