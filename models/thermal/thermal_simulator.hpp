// SPDX-FileCopyrightText: 2026 ETH Zurich, University of Bologna and EssilorLuxottica SAS
//
// SPDX-License-Identifier: Apache-2.0
//
// Authors: Germain Haugou (germain.haugou@gmail.com)

#pragma once

#include <vector>

/**
 * @brief Thermal simulator interface
 *
 * The thermal model component periodically hands the average power consumed
 * at each sync point over the elapsed interval to a thermal simulator, which
 * returns the new temperature of each sync point. The built-in RC model below
 * is a stand-in for a future connection to an external thermal simulator
 * speaking the same interface.
 */
class ThermalSimulator
{
public:
    virtual ~ThermalSimulator() = default;

    /**
     * @brief Advance the thermal state by one interval
     *
     * @param dt      Interval duration, in seconds
     * @param power_w Average power consumed at each sync point over the
     *                interval, in W
     * @param temp_c  Temperature of each sync point, in celsius. Contains the
     *                previous temperatures on entry, must be updated with the
     *                new ones.
     */
    virtual void update(double dt, const std::vector<double> &power_w,
        std::vector<double> &temp_c) = 0;
};

/**
 * @brief Fake built-in thermal simulator
 *
 * Models each sync point as an independent first-order RC network to ambient:
 *
 *     T += dt/tau * (P*Rth + T_amb - T)
 *
 * so each point converges exponentially towards ambient plus its own
 * dissipation, with no thermal coupling between points.
 */
class RcThermalSimulator : public ThermalSimulator
{
public:
    RcThermalSimulator(double temp_ambient, std::vector<double> rth,
        std::vector<double> tau)
        : temp_ambient(temp_ambient), rth(std::move(rth)), tau(std::move(tau))
    {
    }

    void update(double dt, const std::vector<double> &power_w,
        std::vector<double> &temp_c) override
    {
        for (size_t i = 0; i < power_w.size(); i++)
        {
            double target = power_w[i] * this->rth[i] + this->temp_ambient;
            double factor = dt / this->tau[i];
            // Keep the discretized exponential stable when the update period
            // is longer than the time constant
            if (factor > 1.0)
            {
                factor = 1.0;
            }
            temp_c[i] += factor * (target - temp_c[i]);
        }
    }

private:
    double temp_ambient;
    std::vector<double> rth;
    std::vector<double> tau;
};
