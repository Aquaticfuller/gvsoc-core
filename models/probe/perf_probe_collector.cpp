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
 * Performance-probe collector. See prompt/perf_probe_design.md.
 *
 * One instance per system. It registers itself as the "perf_probe" service; every probe::Source
 * in the tree attaches from its start(). A ClockEvent fires every `slice_cycles`: each source's
 * cumulative counters are read, differenced against the previous sample, and written as one row
 * of the kind's CSV. The io slave `input` is the software probe port: a 32-bit store whose
 * address encodes (hart, event) and whose data is the value; stores are binned into the slice in
 * progress and written to sw.csv / sw_entity.csv.
 *
 * Output files (all under `out_dir`):
 *   meta.json     slice length, source table (id -> kind, path), column dictionaries, event names
 *   <kind>.csv    slice,cycle,cycles,src,<columns...>   deltas for COUNTER, values for GAUGE
 *   sw.csv        slice,cycle,hart,event,count,sum
 *   sw_entity.csv slice,cycle,entity,event,count,sum,last
 *   summary.json  cumulative totals per source at the end of the run
 */

#include <vp/vp.hpp>
#include <vp/itf/io.hpp>

#include <cstdio>
#include <cstring>
#include <map>
#include <set>
#include <string>
#include <vector>
#include <sys/stat.h>
#include <sys/types.h>

#include "probe/perf_probe.hpp"
#include "probe/perf_probe_events.h"

class PerfProbeCollector : public vp::Component, public probe::Collector
{
public:
    PerfProbeCollector(vp::ComponentConf &conf);

    void reset(bool active) override;
    void stop() override;

    // probe::Collector
    void attach(vp::Block *owner, probe::Source *src, const std::string &name) override;
    int64_t slice_cycles() const override { return this->slice_; }

private:
    struct Entry
    {
        int id;
        probe::Source *src;
        std::string kind;
        std::string path;
        std::vector<probe::Column> cols;
        std::vector<uint64_t> prev;
        std::vector<uint64_t> cur;
    };
    struct KindFile
    {
        FILE *f = nullptr;
        std::vector<probe::Column> cols;
        uint64_t rows = 0;
    };
    struct SwBin  { uint64_t count = 0, sum = 0; };
    struct SwEnt  { uint64_t count = 0, sum = 0, last = 0; };

    static void slice_handler(vp::Block *__this, vp::ClockEvent *event);
    static vp::IoReqStatus sw_req(vp::Block *__this, vp::IoReq *req);

    void sample(int64_t now);
    void flush_sw(int64_t now);
    void write_meta();
    void write_summary(int64_t now);
    FILE *open_csv(const std::string &name, const std::vector<probe::Column> &cols, bool sw_hdr);
    static void mkdir_p(const std::string &path);
    static std::string json_escape(const std::string &s);

    vp::Trace trace_;
    vp::IoSlave sw_in_;
    vp::ClockEvent slice_event_;

    int64_t slice_ = 1000;
    std::string out_dir_;
    std::set<std::string> kinds_filter_;   // empty = every kind
    int64_t slice_start_ = 0;
    int64_t slice_idx_ = 0;
    bool meta_written_ = false;

    std::vector<Entry> entries_;
    std::map<std::string, KindFile> files_;

    std::map<std::pair<int, int>, SwBin> sw_hart_;   // (hart, event)
    std::map<std::pair<int, int>, SwEnt> sw_ent_;    // (entity, event)
    FILE *sw_f_ = nullptr;
    FILE *sw_ent_f_ = nullptr;
    uint64_t sw_stores_ = 0;
    uint64_t sw_dropped_ = 0;
    int max_hart_seen_ = -1;
};


PerfProbeCollector::PerfProbeCollector(vp::ComponentConf &conf)
    : vp::Component(conf), slice_event_(this, &PerfProbeCollector::slice_handler)
{
    this->traces.new_trace("trace", &this->trace_, vp::DEBUG);

    js::Config *cfg = this->get_js_config();
    this->slice_ = cfg->get_child_int("slice_cycles");
    if (this->slice_ <= 0) this->slice_ = 1000;
    this->out_dir_ = cfg->get_child_str("out_dir");
    if (this->out_dir_.empty()) this->out_dir_ = "perf_probe";
    std::string kinds = cfg->get("kinds") ? cfg->get("kinds")->get_str() : "";
    size_t pos = 0;
    while (pos < kinds.size())
    {
        size_t next = kinds.find(',', pos);
        if (next == std::string::npos) next = kinds.size();
        std::string k = kinds.substr(pos, next - pos);
        if (!k.empty()) this->kinds_filter_.insert(k);
        pos = next + 1;
    }

    this->sw_in_.set_req_meth(&PerfProbeCollector::sw_req);
    this->new_slave_port("input", &this->sw_in_);

    // Visible from every block in the tree: new_service() propagates to the ancestors and
    // get_service() climbs to them. The pointer stored is the INTERFACE, which is what the
    // sources cast back to.
    this->new_service(probe::SERVICE_NAME, (void *)static_cast<probe::Collector *>(this));

    mkdir_p(this->out_dir_);
}


void PerfProbeCollector::reset(bool active)
{
    if (!active)
    {
        this->slice_start_ = this->clock.get_cycles();
        this->slice_idx_ = 0;
        if (!this->slice_event_.is_enqueued())
        {
            this->slice_event_.enqueue(this->slice_);
        }
    }
}


void PerfProbeCollector::attach(vp::Block *owner, probe::Source *src, const std::string &name)
{
    Entry e;
    e.id = (int)this->entries_.size();
    e.src = src;
    e.kind = src->probe_kind();
    e.path = owner->get_path();
    if (!name.empty()) e.path += "/" + name;
    src->probe_columns(e.cols);
    e.prev.assign(e.cols.size(), 0);
    e.cur.assign(e.cols.size(), 0);

    if (!this->kinds_filter_.empty() && this->kinds_filter_.count(e.kind) == 0)
    {
        return;     // filtered out: not sampled, not in meta
    }

    KindFile &kf = this->files_[e.kind];
    if (kf.f == nullptr)
    {
        kf.cols = e.cols;
        kf.f = this->open_csv(e.kind, e.cols, false);
    }
    else if (kf.cols.size() != e.cols.size())
    {
        this->trace_.fatal("perf_probe: source %s has %zu columns but kind %s was registered "
                           "with %zu\n", e.path.c_str(), e.cols.size(), e.kind.c_str(),
                           kf.cols.size());
    }

    // A source attaching after the first slice starts from its current value, not from zero.
    src->probe_sample(this->clock.get_cycles(), e.prev);
    this->entries_.push_back(e);
}


FILE *PerfProbeCollector::open_csv(const std::string &name, const std::vector<probe::Column> &cols,
                                   bool sw_hdr)
{
    std::string path = this->out_dir_ + "/" + name + ".csv";
    FILE *f = fopen(path.c_str(), "w");
    if (f == nullptr)
    {
        this->trace_.fatal("perf_probe: cannot open %s\n", path.c_str());
        return nullptr;
    }
    setvbuf(f, nullptr, _IOFBF, 1 << 20);
    if (!sw_hdr)
    {
        fprintf(f, "slice,cycle,cycles,src");
        for (const probe::Column &c : cols) fprintf(f, ",%s", c.name);
        fprintf(f, "\n");
    }
    return f;
}


void PerfProbeCollector::slice_handler(vp::Block *__this, vp::ClockEvent *event)
{
    PerfProbeCollector *_this = (PerfProbeCollector *)__this;
    _this->sample(_this->clock.get_cycles());
    _this->slice_event_.enqueue(_this->slice_);
}


void PerfProbeCollector::sample(int64_t now)
{
    if (now <= this->slice_start_) return;
    const int64_t len = now - this->slice_start_;

    if (!this->meta_written_)
    {
        this->write_meta();     // early copy so a run can be inspected while it is still going
    }

    for (Entry &e : this->entries_)
    {
        e.src->probe_sample(now, e.cur);
        FILE *f = this->files_[e.kind].f;
        fprintf(f, "%lld,%lld,%lld,%d", (long long)this->slice_idx_, (long long)this->slice_start_,
                (long long)len, e.id);
        for (size_t i = 0; i < e.cols.size(); i++)
        {
            uint64_t v = e.cur[i];
            if (e.cols[i].kind == probe::COUNTER)
            {
                // A source that reset its counters would go backwards; clamp rather than wrap.
                v = (e.cur[i] >= e.prev[i]) ? (e.cur[i] - e.prev[i]) : 0;
            }
            fprintf(f, ",%llu", (unsigned long long)v);
        }
        fprintf(f, "\n");
        this->files_[e.kind].rows++;
        e.prev = e.cur;
    }

    this->flush_sw(now);

    this->slice_start_ = now;
    this->slice_idx_++;
}


void PerfProbeCollector::flush_sw(int64_t now)
{
    const int64_t len = now - this->slice_start_;
    (void)len;
    if (!this->sw_hart_.empty())
    {
        if (this->sw_f_ == nullptr)
        {
            this->sw_f_ = this->open_csv("sw", {}, true);
            fprintf(this->sw_f_, "slice,cycle,hart,event,count,sum\n");
        }
        for (auto &kv : this->sw_hart_)
        {
            fprintf(this->sw_f_, "%lld,%lld,%d,%d,%llu,%llu\n", (long long)this->slice_idx_,
                    (long long)this->slice_start_, kv.first.first, kv.first.second,
                    (unsigned long long)kv.second.count, (unsigned long long)kv.second.sum);
        }
        this->sw_hart_.clear();
    }
    if (!this->sw_ent_.empty())
    {
        if (this->sw_ent_f_ == nullptr)
        {
            this->sw_ent_f_ = this->open_csv("sw_entity", {}, true);
            fprintf(this->sw_ent_f_, "slice,cycle,entity,event,count,sum,last\n");
        }
        for (auto &kv : this->sw_ent_)
        {
            fprintf(this->sw_ent_f_, "%lld,%lld,%d,%d,%llu,%llu,%llu\n",
                    (long long)this->slice_idx_, (long long)this->slice_start_,
                    kv.first.first, kv.first.second, (unsigned long long)kv.second.count,
                    (unsigned long long)kv.second.sum, (unsigned long long)kv.second.last);
        }
        this->sw_ent_.clear();
    }
}


vp::IoReqStatus PerfProbeCollector::sw_req(vp::Block *__this, vp::IoReq *req)
{
    PerfProbeCollector *_this = (PerfProbeCollector *)__this;

    // Write-only port. Reads return 0 so a stray load does not fault the core.
    if (!req->get_is_write())
    {
        if (req->get_data() != nullptr) memset(req->get_data(), 0, req->get_size());
        return vp::IO_REQ_OK;
    }

    const uint64_t off = req->get_addr();
    const int hart = (int)(off >> PERF_PROBE_HART_SHIFT);
    const int evt  = (int)((off >> 2) & (PERF_PROBE_NB_EVENTS - 1));
    uint32_t value = 0;
    if (req->get_data() != nullptr)
    {
        memcpy(&value, req->get_data(), req->get_size() < 4 ? req->get_size() : 4);
    }
    if (evt >= PROBE_EVT_NB_DEFINED)
    {
        _this->sw_dropped_++;
        return vp::IO_REQ_OK;
    }
    _this->sw_stores_++;
    if (hart > _this->max_hart_seen_) _this->max_hart_seen_ = hart;

    const bool per_entity = (PERF_PROBE_ENTITY_EVENTS >> evt) & 1u;
    const bool gauge      = (PERF_PROBE_GAUGE_EVENTS  >> evt) & 1u;
    const uint32_t v      = per_entity ? (value & PERF_PROBE_VALUE_MASK) : value;

    SwBin &b = _this->sw_hart_[std::make_pair(hart, evt)];
    b.count++;
    b.sum += v;

    if (per_entity)
    {
        const int entity = (int)(value >> PERF_PROBE_ENTITY_SHIFT);
        SwEnt &e = _this->sw_ent_[std::make_pair(entity, evt)];
        e.count++;
        e.sum += v;
        e.last = v;
    }
    else if (gauge)
    {
        // A hart-level gauge (role, phase, mm_live): the analysis reads `sum/count` per slice
        // as the mean level, `last` is not tracked per hart to keep sw.csv narrow.
        (void)gauge;
    }

    return vp::IO_REQ_OK;
}


void PerfProbeCollector::stop()
{
    const int64_t now = this->clock.get_cycles();
    this->sample(now);          // the final partial slice, with its real length in `cycles`
    this->write_meta();
    this->write_summary(now);
    for (auto &kv : this->files_)
    {
        if (kv.second.f) fclose(kv.second.f);
        kv.second.f = nullptr;
    }
    if (this->sw_f_) { fclose(this->sw_f_); this->sw_f_ = nullptr; }
    if (this->sw_ent_f_) { fclose(this->sw_ent_f_); this->sw_ent_f_ = nullptr; }
    fprintf(stderr, "[PERF-PROBE] %zu sources, %lld slices of %lld cycles, %llu sw stores "
                    "(%llu dropped) -> %s/\n",
            this->entries_.size(), (long long)this->slice_idx_, (long long)this->slice_,
            (unsigned long long)this->sw_stores_, (unsigned long long)this->sw_dropped_,
            this->out_dir_.c_str());
    vp::Component::stop();
}


std::string PerfProbeCollector::json_escape(const std::string &s)
{
    std::string o;
    for (char c : s)
    {
        if (c == '"' || c == '\\') { o += '\\'; o += c; }
        else if (c == '\n') o += "\\n";
        else o += c;
    }
    return o;
}


void PerfProbeCollector::write_meta()
{
    std::string path = this->out_dir_ + "/meta.json";
    FILE *f = fopen(path.c_str(), "w");
    if (f == nullptr) return;
    fprintf(f, "{\n  \"slice_cycles\": %lld,\n", (long long)this->slice_);
    fprintf(f, "  \"freq_hz\": %lld,\n", (long long)this->clock.get_frequency());
    fprintf(f, "  \"nb_harts_sw\": %d,\n", this->max_hart_seen_ + 1);

    fprintf(f, "  \"columns\": {\n");
    bool first = true;
    for (auto &kv : this->files_)
    {
        fprintf(f, "%s    \"%s\": [", first ? "" : ",\n", kv.first.c_str());
        for (size_t i = 0; i < kv.second.cols.size(); i++)
        {
            fprintf(f, "%s{\"name\": \"%s\", \"type\": \"%s\"}", i ? ", " : "",
                    kv.second.cols[i].name,
                    kv.second.cols[i].kind == probe::COUNTER ? "counter" : "gauge");
        }
        fprintf(f, "]");
        first = false;
    }
    fprintf(f, "\n  },\n");

    fprintf(f, "  \"sources\": [\n");
    for (size_t i = 0; i < this->entries_.size(); i++)
    {
        const Entry &e = this->entries_[i];
        fprintf(f, "    {\"id\": %d, \"kind\": \"%s\", \"path\": \"%s\"}%s\n", e.id,
                e.kind.c_str(), json_escape(e.path).c_str(),
                i + 1 < this->entries_.size() ? "," : "");
    }
    fprintf(f, "  ],\n");

    static const char *names[] = PERF_PROBE_EVENT_NAMES;
    fprintf(f, "  \"events\": {");
    for (int i = 0; i < PROBE_EVT_NB_DEFINED; i++)
    {
        fprintf(f, "%s\"%d\": {\"name\": \"%s\", \"entity\": %s, \"gauge\": %s}", i ? ", " : "",
                i, names[i], ((PERF_PROBE_ENTITY_EVENTS >> i) & 1u) ? "true" : "false",
                ((PERF_PROBE_GAUGE_EVENTS >> i) & 1u) ? "true" : "false");
    }
    fprintf(f, "},\n");
    fprintf(f, "  \"sw_port\": {\"base\": %u, \"hart_shift\": %d, \"nb_events\": %d}\n",
            PERF_PROBE_BASE, PERF_PROBE_HART_SHIFT, PERF_PROBE_NB_EVENTS);
    fprintf(f, "}\n");
    fclose(f);
    this->meta_written_ = true;
}


void PerfProbeCollector::write_summary(int64_t now)
{
    std::string path = this->out_dir_ + "/summary.json";
    FILE *f = fopen(path.c_str(), "w");
    if (f == nullptr) return;
    fprintf(f, "{\n  \"end_cycle\": %lld,\n  \"slices\": %lld,\n  \"sw_stores\": %llu,\n",
            (long long)now, (long long)this->slice_idx_, (unsigned long long)this->sw_stores_);
    fprintf(f, "  \"cumulative\": [\n");
    for (size_t i = 0; i < this->entries_.size(); i++)
    {
        Entry &e = this->entries_[i];
        e.src->probe_sample(now, e.cur);
        fprintf(f, "    {\"id\": %d, \"kind\": \"%s\", \"values\": {", e.id, e.kind.c_str());
        for (size_t c = 0; c < e.cols.size(); c++)
        {
            fprintf(f, "%s\"%s\": %llu", c ? ", " : "", e.cols[c].name,
                    (unsigned long long)e.cur[c]);
        }
        fprintf(f, "}}%s\n", i + 1 < this->entries_.size() ? "," : "");
    }
    fprintf(f, "  ]\n}\n");
    fclose(f);
}


void PerfProbeCollector::mkdir_p(const std::string &path)
{
    std::string cur;
    for (size_t i = 0; i < path.size(); i++)
    {
        cur += path[i];
        if (path[i] == '/' || i + 1 == path.size())
        {
            mkdir(cur.c_str(), 0775);
        }
    }
}


extern "C" vp::Component *gv_new(vp::ComponentConf &config)
{
    return new PerfProbeCollector(config);
}
