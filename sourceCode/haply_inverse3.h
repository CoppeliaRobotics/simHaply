// Copyright 2024 Coppelia Robotics AG. All rights reserved.
// marc@coppeliarobotics.com
// www.coppeliarobotics.com
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
// ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// -------------------------------------------------------------------
// Authors:
// Federico Ferri <federico.ferri.it at gmail dot com>
// -------------------------------------------------------------------

#pragma once

#include <array>
#include <mutex>
#include <optional>
#include <string>

#include "stubs.h"
#include "haply_device.h"
#include "control_loop.h"

#include "HardwareAPI.h"

struct Haply_Inverse3 : public Haply_Device<Haply::HardwareAPI::Devices::Inverse3, 1>, public ControlLoop
{
    Haply_Inverse3(const std::string &port);
    void tick();
    void setTargetPosition(const std::array<double, 3> &target_position);
    void setTargetForce(const std::array<double, 3> &target_force);
    void setConstraint(const std::array<double, 3> &p, const std::array<double, 3> &n);
    void setAttractor(const std::array<double, 3> &p);
    void setForceParams(double kf, double maxf);
    void setGravityCompensation(bool enabled, double scale_factor);
    std::array<double, 3> getPosition();
    std::array<double, 3> getVelocity();

private:
    simhaply_mode mode{simhaply_mode_force_ctrl};
    std::optional<std::pair<bool, double>> set_gravity_compensation;
    std::optional<std::array<double, 3>> target_position;
    std::optional<std::array<double, 3>> target_force;
    std::array<double, 3> p;
    std::array<double, 3> n;
    std::array<double, 3> position;
    std::array<double, 3> velocity;
    double kf{10.0};
    double maxf{10.0};
};
