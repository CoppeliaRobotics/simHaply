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

void Haply_Inverse3::tick()
{
    std::lock_guard<std::mutex> lock(mtx);

    if(gravity_compensation.has_value())
    {
        device->SetGravityCompensation(gravity_compensation.value());
        gravity_compensation.reset();
    }

    Eigen::Vector3d position = Eigen::Map<Eigen::Vector3f>(end_effector_state.position).cast<double>();
    Eigen::Vector3d velocity = Eigen::Map<Eigen::Vector3f>(end_effector_state.velocity).cast<double>();

    switch(mode)
    {
        case simhaply_mode_constraint:
            {
                if(tick_num > 0)
                {
                    double sdf = std::min(maxf, std::max(-maxf, kf * std::min(0.0, (position - p).dot(n))));
                    end_effector_force.emplace();
                    end_effector_force->force[0] = -n(0) * sdf;
                    end_effector_force->force[1] = -n(1) * sdf;
                    end_effector_force->force[2] = -n(2) * sdf;
                }
            }
            break;
        case simhaply_mode_attractor:
            {
                if(tick_num > 0)
                {
                    Eigen::Vector3d d = position - p;
                    double dnorm = d.norm();
                    d.normalize();
                    dnorm = std::min(maxf, std::max(-maxf, -kf * dnorm));
                    end_effector_force.emplace();
                    end_effector_force->force[0] = dnorm * d(0);
                    end_effector_force->force[1] = dnorm * d(1);
                    end_effector_force->force[2] = dnorm * d(2);
                }
            }
            break;
        default:
            break;
    }

    if(end_effector_force.has_value())
    {
        end_effector_state = device->EndEffectorForce(end_effector_force.value(), true);
        end_effector_force.reset();
    }
    else if(end_effector_position.has_value())
    {
        end_effector_state = device->EndEffectorPosition(end_effector_position.value());
        end_effector_position.reset();
    }
}

void Haply_Inverse3::setPosition(const Eigen::Vector3d &position)
{
    std::lock_guard<std::mutex> lock(mtx);
    mode = simhaply_mode_position_ctrl;
    end_effector_position.emplace();
    end_effector_position->position[0] = position(0);
    end_effector_position->position[1] = position(1);
    end_effector_position->position[2] = position(2);
    end_effector_force.reset();
}

void Haply_Inverse3::setForce(const Eigen::Vector3d &force)
{
    std::lock_guard<std::mutex> lock(mtx);
    mode = simhaply_mode_force_ctrl;
    end_effector_force.emplace();
    end_effector_force->force[0] = force(0);
    end_effector_force->force[1] = force(1);
    end_effector_force->force[2] = force(2);
    end_effector_position.reset();
}

void Haply_Inverse3::setConstraint(const Eigen::Vector3d &p, const Eigen::Vector3d &n)
{
    std::lock_guard<std::mutex> lock(mtx);
    mode = simhaply_mode_constraint;
    this->p = p;
    this->n = n.normalized();
}

void Haply_Inverse3::setAttractor(const Eigen::Vector3d &p)
{
    std::lock_guard<std::mutex> lock(mtx);
    mode = simhaply_mode_attractor;
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
    gravity_compensation.emplace();
    gravity_compensation->enabled = enabled;
    gravity_compensation->gravity_scale_factor = scale_factor;
}

Eigen::Vector3d Haply_Inverse3::getPosition()
{
    std::lock_guard<std::mutex> lock(mtx);
    return {
        end_effector_state.position[0],
        end_effector_state.position[1],
        end_effector_state.position[2]
    };
}

Eigen::Vector3d Haply_Inverse3::getVelocity()
{
    std::lock_guard<std::mutex> lock(mtx);
    return {
        end_effector_state.velocity[0],
        end_effector_state.velocity[1],
        end_effector_state.velocity[2]
    };
}
