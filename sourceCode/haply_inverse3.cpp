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

#include "haply_inverse3.h"

Haply_Inverse3::Haply_Inverse3(const std::string &port)
    : Haply_Device<Haply::HardwareAPI::Devices::Inverse3, 1>("Inverse3", port),
      ControlLoop("Inverse3")
{
}

template<typename T>
static void readPositionAndVelocity(const T &in, std::array<double, 3> &position, std::array<double, 3> &velocity)
{
    position[0] = in.position[0];
    position[1] = in.position[1];
    position[2] = in.position[2];
    velocity[0] = in.velocity[0];
    velocity[1] = in.velocity[1];
    velocity[2] = in.velocity[2];
}

void Haply_Inverse3::tick()
{
    simhaply_mode m;
    {
        std::lock_guard<std::mutex> lock(mtx);

        m = mode;

        if(set_gravity_compensation.has_value())
        {
            Haply::HardwareAPI::Devices::Inverse3::GravityCompensationPayload req;
            const auto &gc = set_gravity_compensation.value();
            req.enabled = gc.first;
            req.gravity_scale_factor = gc.second;
            Haply::HardwareAPI::Devices::Inverse3::GravityCompensationPayload resp = device->SetGravityCompensation(req);
        }
    }
    switch(m)
    {
        case simhaply_mode_position_ctrl:
        {
            Haply::HardwareAPI::Devices::Inverse3::EndEffectorPositionRequest req;
            {
                std::lock_guard<std::mutex> lock(mtx);
                if(!target_position) return;
                const auto &p = target_position.value();
                req.position[0] = p[0];
                req.position[1] = p[1];
                req.position[2] = p[2];
                target_position.reset();
            }
            Haply::HardwareAPI::Devices::Inverse3::EndEffectorStateResponse resp = device->EndEffectorPosition(req);
            {
                std::lock_guard<std::mutex> lock(mtx);
                readPositionAndVelocity(resp, position, velocity);
            }
        }
        break;
        case simhaply_mode_force_ctrl:
        {
            Haply::HardwareAPI::Devices::Inverse3::EndEffectorForceRequest req;
            {
                std::lock_guard<std::mutex> lock(mtx);
                if(!target_force) return;
                const auto &f = target_force.value();
                req.force[0] = f[0];
                req.force[1] = f[1];
                req.force[2] = f[2];
                target_force.reset();
            }
            Haply::HardwareAPI::Devices::Inverse3::EndEffectorStateResponse resp = device->EndEffectorForce(req, true);
            {
                std::lock_guard<std::mutex> lock(mtx);
                readPositionAndVelocity(resp, position, velocity);
            }
        }
        break;
        case simhaply_mode_constraint:
        {
            Haply::HardwareAPI::Devices::Inverse3::EndEffectorForceRequest req;
            {
                std::lock_guard<std::mutex> lock(mtx);
                double norm = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
                double unx = n[0] / norm;
                double uny = n[1] / norm;
                double unz = n[2] / norm;
                double dx = position[0] - p[0];
                double dy = position[1] - p[1];
                double dz = position[2] - p[2];
                double sdf = std::min(maxf, std::max(-maxf, kf * std::min(0., dx * unx + dy * uny + dz * unz)));
                if(tick_num > 0)
                {
                    req.force[0] = -unx * sdf;
                    req.force[1] = -uny * sdf;
                    req.force[2] = -unz * sdf;
                }
            }
            Haply::HardwareAPI::Devices::Inverse3::EndEffectorStateResponse resp = device->EndEffectorForce(req, true);
            {
                std::lock_guard<std::mutex> lock(mtx);
                readPositionAndVelocity(resp, position, velocity);
            }
        }
        break;
        case simhaply_mode_attractor:
        {
            Haply::HardwareAPI::Devices::Inverse3::EndEffectorForceRequest req;
            {
                std::lock_guard<std::mutex> lock(mtx);
                double dx = position[0] - p[0];
                double dy = position[1] - p[1];
                double dz = position[2] - p[2];
                double dnorm = std::sqrt(dx * dx + dy * dy + dz * dz);
                dx /= dnorm;
                dy /= dnorm;
                dz /= dnorm;
                dnorm = std::min(maxf, std::max(-maxf, -kf * dnorm));
                if(tick_num > 0)
                {
                    req.force[0] = dnorm * dx;
                    req.force[1] = dnorm * dy;
                    req.force[2] = dnorm * dz;
                }
            }
            Haply::HardwareAPI::Devices::Inverse3::EndEffectorStateResponse resp = device->EndEffectorForce(req, true);
            {
                std::lock_guard<std::mutex> lock(mtx);
                readPositionAndVelocity(resp, position, velocity);
            }
        }
        break;
    }
}

void Haply_Inverse3::setTargetPosition(const std::array<double, 3> &target_position)
{
    std::lock_guard<std::mutex> lock(mtx);
    this->mode = simhaply_mode_position_ctrl;
    this->target_position = target_position;
}

void Haply_Inverse3::setTargetForce(const std::array<double, 3> &target_force)
{
    std::lock_guard<std::mutex> lock(mtx);
    this->mode = simhaply_mode_force_ctrl;
    this->target_force = target_force;
}

void Haply_Inverse3::setConstraint(const std::array<double, 3> &p, const std::array<double, 3> &n)
{
    std::lock_guard<std::mutex> lock(mtx);
    this->mode = simhaply_mode_constraint;
    this->p = p;
    this->n = n;
}

void Haply_Inverse3::setAttractor(const std::array<double, 3> &p)
{
    std::lock_guard<std::mutex> lock(mtx);
    this->mode = simhaply_mode_attractor;
    this->p = p;
    this->n = {0, 0, 1};
}

void Haply_Inverse3::setForceParams(double kf, double maxf)
{
    std::lock_guard<std::mutex> lock(mtx);
    this->kf = kf;
    this->maxf = maxf;
}

void Haply_Inverse3::setGravityCompensation(bool enabled, double scale_factor)
{
    std::lock_guard<std::mutex> lock(mtx);
    set_gravity_compensation = std::make_pair(enabled, scale_factor);
}

std::array<double, 3> Haply_Inverse3::getPosition()
{
    std::lock_guard<std::mutex> lock(mtx);
    return position;
}

std::array<double, 3> Haply_Inverse3::getVelocity()
{
    std::lock_guard<std::mutex> lock(mtx);
    return velocity;
}
