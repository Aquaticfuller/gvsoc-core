// SPDX-FileCopyrightText: 2026 ETH Zurich, University of Bologna and EssilorLuxottica SAS
//
// SPDX-License-Identifier: Apache-2.0
//
// Authors: Germain Haugou (germain.haugou@gmail.com)
//
// Memcheck region declaration component.
//
// A pure declaration carrier: it has no ports. At construction it reads the
// allocator-backed regions from its typed config and declares them to the
// memcheck registry, so that allocations can be attached to a named region and
// reports can name the region a faulty access lands in. The checks themselves
// live in the core models (which fold aliases through the trace engine) and in
// vp::MemCheck; this component only feeds the table.

#include <vp/vp.hpp>
#include <vp/memcheck.hpp>
#include <utils/memcheck_regions/memcheck_regions_config.hpp>

class MemcheckRegions : public vp::Component
{
public:
    MemcheckRegions(vp::ComponentConf &config);

private:
    MemcheckRegionsConfig cfg;
};

MemcheckRegions::MemcheckRegions(vp::ComponentConf &config)
    : vp::Component(config, this->cfg)
{
    for (size_t i = 0; i < this->cfg.regions_count; i++)
    {
        const MemcheckRegion &region = this->cfg.regions[i];
        this->get_memcheck()->declare_region((int)region.mem_id, region.name,
            (uint64_t)region.base, (uint64_t)region.size);
    }
}

extern "C" vp::Component *gv_new(vp::ComponentConf &config)
{
    return new MemcheckRegions(config);
}
