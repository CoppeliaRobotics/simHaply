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

#include "haply_handle.h"

Haply_Handle::Haply_Handle(const std::string &port)
    : Haply_Device<Haply::HardwareAPI::Devices::Handle, 2>(port),
      ControlLoop("Handle")
{
}

void Haply_Handle::tick()
{
    Haply::HardwareAPI::Devices::Handle::VersegripStatusResponse resp = device->GetVersegripStatus();

    if(resp.error_flag)
    {
        return;
    }

    {
        std::lock_guard<std::mutex> lock(mtx);
        for(int i = 0; i < 4; i++)
            quaternion[i] = resp.quaternion[i];
        buttons = resp.buttons;
        battery_level = resp.battery_level;
    }
}

std::array<double, 4> Haply_Handle::getQuaternion()
{
    std::lock_guard<std::mutex> lock(mtx);
    return quaternion;
}

int Haply_Handle::getButtons()
{
    std::lock_guard<std::mutex> lock(mtx);
    return buttons;
}

float Haply_Handle::getBatteryLevel()
{
    std::lock_guard<std::mutex> lock(mtx);
    return battery_level;
}
