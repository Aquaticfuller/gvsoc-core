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

#include "cpu/iss_v2/include/iss.hpp"
#include "vp/memcheck.hpp"


Memcheck::Memcheck(Iss &iss)
: iss(iss)
{
    iss.traces.new_trace("memcheck", &this->trace, vp::DEBUG);
}

iss_reg_t Memcheck::mem_alloc(iss_reg_t mem_id, iss_reg_t ptr, iss_reg_t size)
{
    if (this->iss.traces.get_trace_engine()->is_memcheck_enabled())
    {
        uint64_t pc = this->iss.exec.current_insn;
        uint64_t ra = this->iss.regfile.get_reg_untimed(1);

        uint32_t id = this->iss.get_memcheck()->alloc(mem_id, ptr, size, pc, ra);

        if (id == 0)
        {
            this->trace.force_warning(
                "Allocating from an undeclared memcheck region (mem_id: %d, ptr: 0x%lx)\n",
                (int)mem_id, (uint64_t)ptr);
            return ptr;
        }

        this->trace.msg(vp::Trace::LEVEL_INFO,
            "Memory alloc (mem_id: %d, ptr: 0x%lx, size: 0x%lx, buffer_id: %u)\n",
            (int)mem_id, (uint64_t)ptr, (uint64_t)size, id);

        // Attach the new buffer to the register receiving the returned pointer
        this->iss.regfile.memcheck_set_valid(10, true);
        this->iss.regfile.memcheck_set_id(10, id);
    }

    return ptr;
}

iss_reg_t Memcheck::mem_free(iss_reg_t mem_id, iss_reg_t ptr, iss_reg_t size)
{
    if (this->iss.traces.get_trace_engine()->is_memcheck_enabled())
    {
        uint64_t pc = this->iss.exec.current_insn;
        uint64_t ra = this->iss.regfile.get_reg_untimed(1);

        uint32_t id = this->iss.get_memcheck()->free(mem_id, ptr, size, pc, ra);

        if (id == 0)
        {
            this->trace.force_warning(
                "Freeing an unknown buffer, double free or invalid free "
                "(mem_id: %d, ptr: 0x%lx)\n", (int)mem_id, (uint64_t)ptr);
            return ptr;
        }

        this->trace.msg(vp::Trace::LEVEL_INFO,
            "Memory free (mem_id: %d, ptr: 0x%lx, size: 0x%lx, buffer_id: %u)\n",
            (int)mem_id, (uint64_t)ptr, (uint64_t)size, id);

        this->iss.regfile.memcheck_set_valid(10, true);
    }

    return ptr;
}
