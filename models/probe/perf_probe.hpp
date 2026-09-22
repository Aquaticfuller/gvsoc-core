/*
 * Copyright (C) 2026 ETH Zurich and University of Bologna
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
 * Performance-probe interface. See prompt/perf_probe_design.md.
 *
 * A model that wants to be sampled every time slice derives from probe::Source, keeps its
 * counters CUMULATIVE, and calls probe::attach() from its start(). The collector
 * (perf_probe_collector.cpp) finds itself through the engine's service registry: models are
 * separate .so files loaded with RTLD_DEEPBIND, so a global variable or a dynamic_cast across
 * them is not safe, but vp::Block::new_service() propagates a pointer to every ancestor and
 * get_service() climbs back up to it. A system without a collector costs one map lookup per
 * component at start and nothing afterwards -- attach() simply returns false.
 *
 * Header-only on purpose: every model .so gets its own copy and there is nothing to link.
 */

#pragma once

#include <vp/vp.hpp>
#include <cstdint>
#include <string>
#include <vector>

namespace probe
{

enum ColKind
{
    COUNTER = 0,    // monotonic; the CSV carries the per-slice delta
    GAUGE   = 1,    // point-in-time; the CSV carries the sampled value
};

struct Column
{
    const char *name;
    ColKind kind;
};

class Source
{
public:
    virtual ~Source() {}
    // Short kind name, one CSV file per kind: "core", "spatz", "cache", "noc_router", ...
    virtual const char *probe_kind() const = 0;
    // Column list, fixed for the life of the source and identical for every source of a kind.
    virtual void probe_columns(std::vector<Column> &cols) const = 0;
    // Current cumulative values, same order as probe_columns(). `now` is the sampling cycle so
    // occupancy integrals can be brought up to date before they are read.
    virtual void probe_sample(int64_t now, std::vector<uint64_t> &vals) = 0;
};

class Collector
{
public:
    virtual ~Collector() {}
    // `name` is appended to the owner's path in meta.json; empty = the path alone.
    virtual void attach(vp::Block *owner, Source *src, const std::string &name) = 0;
    virtual int64_t slice_cycles() const = 0;
    // One software probe store (offset into the probe window, 32-bit value). Called by a core's
    // LSU directly so the store costs the core nothing beyond its issue slot.
    virtual void sw_store(uint64_t off, uint32_t value) = 0;
};

static const char *const SERVICE_NAME = "perf_probe";

inline Collector *get(vp::Block *b)
{
    return (Collector *)b->get_service(SERVICE_NAME);
}

// Register `src` with the collector if one exists. Call from start(), not from a constructor:
// components are constructed in tree order and the collector may not exist yet.
inline bool attach(vp::Block *owner, Source *src, const std::string &name = "")
{
    Collector *c = get(owner);
    if (c == nullptr)
    {
        return false;
    }
    c->attach(owner, src, name);
    return true;
}

// Occupancy integrator: keeps Σ(level · cycles) exactly, updated only when the level changes.
// The analysis divides the per-slice delta by the slice length to get the average occupancy.
struct Occupancy
{
    uint64_t integral = 0;
    int64_t  last     = 0;
    int64_t  level    = 0;

    inline void set(int64_t now, int64_t new_level)
    {
        if (now > this->last && this->level > 0)
        {
            this->integral += (uint64_t)this->level * (uint64_t)(now - this->last);
        }
        if (now > this->last) this->last = now;
        this->level = new_level;
    }
    inline void add(int64_t now, int64_t delta) { this->set(now, this->level + delta); }
    // Bring the integral up to `now` without changing the level, then read it.
    inline uint64_t read(int64_t now) { this->set(now, this->level); return this->integral; }
    inline void reset(int64_t now = 0) { this->integral = 0; this->last = now; this->level = 0; }
};

// Stall-interval accumulator: begin()/end() bracket one interval attributed to one reason.
struct Interval
{
    int64_t start = -1;
    inline void begin(int64_t now) { this->start = now; }
    // Returns the elapsed cycles (0 if no interval was open) and closes it.
    inline uint64_t end(int64_t now)
    {
        if (this->start < 0) return 0;
        uint64_t d = now > this->start ? (uint64_t)(now - this->start) : 0;
        this->start = -1;
        return d;
    }
    inline bool open() const { return this->start >= 0; }
};

} // namespace probe
