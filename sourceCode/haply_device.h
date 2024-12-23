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

#include <algorithm>
#include <iostream>
#include <string>

#include "HardwareAPI.h"

template<typename T, int initMode>
struct Haply_Device
{
    Haply_Device(const std::string &devType, const std::string &port)
        : devType(devType), port(port)
    {
        std::cout << "Haply_" << devType << "[" << port << "]: opening" << std::endl;

        serial_stream = new Haply::HardwareAPI::IO::SerialStream(port.c_str());
        device = new T(serial_stream);
        if constexpr(initMode == 1)
        {
            Haply::HardwareAPI::Devices::Inverse3::DeviceInfoResponse resp = device->DeviceWakeup();
        }
        else if constexpr(initMode == 2)
        {
            device->SendDeviceWakeup();
            device->RequestStatus();
        }
    }

    virtual ~Haply_Device()
    {
        delete device;
        delete serial_stream;

        std::cout << "Haply_" << devType << "[" << port << "]: closed" << std::endl;
    }

protected:
    std::string devType;
    std::string port;
    Haply::HardwareAPI::IO::SerialStream *serial_stream;
    T *device;
};
