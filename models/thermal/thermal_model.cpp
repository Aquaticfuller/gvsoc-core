// SPDX-FileCopyrightText: 2026 ETH Zurich, University of Bologna and EssilorLuxottica SAS
//
// SPDX-License-Identifier: Apache-2.0
//
// Authors: Germain Haugou (germain.haugou@gmail.com)
//
// Closed-loop thermal model.
//
// Periodically samples the power consumed at each configured sync point
// (a component path whose top power trace covers the whole subtree),
// hands the powers to a thermal simulator (the built-in RC model for
// now, see thermal_simulator.hpp), and applies the returned temperatures
// back onto the power sources so that temperature-dependent power tables
// are re-evaluated.
//
// Power is sampled through the never-reset total energy counters of the
// power traces, by taking deltas, so any number of consumers (this model,
// --power report captures, proxy reports) can observe power without
// interfering with each other.

#include <memory>
#include <string>
#include <vector>

#include <vp/vp.hpp>
#include <thermal/thermal_model/thermal_model_config.hpp>

#include "thermal_simulator.hpp"

class ThermalModel : public vp::Component
{
public:
    ThermalModel(vp::ComponentConf &config);

    ThermalModelConfig cfg;

private:
    struct SyncPoint
    {
        std::string name;
        std::string path;
        vp::Block *block = NULL;
        vp::Trace temp_trace;
        double prev_energy = 0.0;
    };

    void start() override;
    void reset(bool active) override;
    static void event_handler(vp::Block *__this, vp::TimeEvent *event);

    static std::vector<std::string> split_path(const std::string &path);

    vp::TimeEvent event;
    vp::Trace trace;
    std::unique_ptr<ThermalSimulator> simulator;
    std::vector<SyncPoint *> sync_points;
    std::vector<double> power_w;
    std::vector<double> temp_c;
    // False when power modeling is disabled, in which case the model stays
    // fully inert (no periodic event)
    bool enabled = false;
};



ThermalModel::ThermalModel(vp::ComponentConf &config)
    : vp::Component(config, this->cfg), event(this, &ThermalModel::event_handler)
{
    traces.new_trace("trace", &this->trace, vp::DEBUG);

    std::vector<double> rth, tau;
    for (size_t i = 0; i < this->cfg.sync_points_count; i++)
    {
        const ThermalSyncPointConfig &point_cfg = this->cfg.sync_points[i];

        SyncPoint *point = new SyncPoint();
        point->name = point_cfg.name;
        point->path = point_cfg.component_path;
        traces.new_trace_event_real("temp_" + point->name, &point->temp_trace);
        point->temp_trace.event_real(this->cfg.temp_init);
        this->sync_points.push_back(point);

        rth.push_back(point_cfg.rth);
        tau.push_back(point_cfg.tau);
    }

    this->simulator = std::make_unique<RcThermalSimulator>(
        this->cfg.temp_ambient, rth, tau);
    this->power_w.resize(this->sync_points.size());
    this->temp_c.assign(this->sync_points.size(), this->cfg.temp_init);
}



void ThermalModel::start()
{
    // Stay silently dormant when no sync point is configured, so that
    // targets can embed a thermal model which is only activated when a
    // sync points file is given through the 'file' parameter
    if (this->sync_points.empty())
    {
        return;
    }

    this->enabled = this->power.get_engine()->is_enabled();
    if (!this->enabled)
    {
        this->trace.force_warning_no_error(
            "Thermal model is inactive since power modeling is disabled (use --power)\n");
        return;
    }

    vp::Block *top = this;
    while (top->get_parent())
    {
        top = top->get_parent();
    }

    for (SyncPoint *point : this->sync_points)
    {
        point->block = top->get_block_from_path(split_path(point->path));
        if (point->block == NULL)
        {
            this->trace.fatal("Could not resolve thermal sync point (path: %s)\n",
                point->path.c_str());
            return;
        }

        point->block->power.temperature_set_all(this->cfg.temp_init);

        double dynamic, leakage;
        point->block->power.get_total_energy(&dynamic, &leakage);
        point->prev_energy = dynamic + leakage;
    }
}



void ThermalModel::reset(bool active)
{
    if (!active && this->time.get_time() == 0 && this->enabled)
    {
        this->event.enqueue(this->cfg.period);
    }
}



void ThermalModel::event_handler(vp::Block *__this, vp::TimeEvent *event)
{
    ThermalModel *_this = (ThermalModel *)__this;

    // First sample every sync point, so that applying a temperature (which
    // re-injects background/leakage power) cannot disturb the reading of
    // another point
    for (size_t i = 0; i < _this->sync_points.size(); i++)
    {
        SyncPoint *point = _this->sync_points[i];

        double dynamic, leakage;
        point->block->power.get_total_energy(&dynamic, &leakage);
        double energy = dynamic + leakage;

        // Energy counters are in W.ps, dividing the delta by the period in
        // picoseconds gives the average power in W
        _this->power_w[i] = (energy - point->prev_energy) / _this->cfg.period;
        point->prev_energy = energy;
    }

    _this->simulator->update((double)_this->cfg.period * 1e-12, _this->power_w,
        _this->temp_c);

    for (size_t i = 0; i < _this->sync_points.size(); i++)
    {
        SyncPoint *point = _this->sync_points[i];

        point->block->power.temperature_set_all(_this->temp_c[i]);
        point->temp_trace.event_real(_this->temp_c[i]);

        _this->trace.msg(vp::TraceLevel::DEBUG,
            "Updated sync point (name: %s, power: %f W, temp: %f C)\n",
            point->name.c_str(), _this->power_w[i], _this->temp_c[i]);

        if (_this->cfg.verbose)
        {
            printf("[%ld] thermal %s power_w=%.9f temp_c=%.3f\n",
                _this->time.get_time(), point->name.c_str(), _this->power_w[i],
                _this->temp_c[i]);
        }
    }

    _this->event.enqueue(_this->cfg.period);
}



std::vector<std::string> ThermalModel::split_path(const std::string &path)
{
    std::vector<std::string> result;
    std::string current;
    for (char c : path)
    {
        if (c == '/')
        {
            if (!current.empty())
            {
                result.push_back(current);
                current.clear();
            }
        }
        else
        {
            current += c;
        }
    }
    if (!current.empty())
    {
        result.push_back(current);
    }
    return result;
}



extern "C" vp::Component *gv_new(vp::ComponentConf &config)
{
    return new ThermalModel(config);
}
