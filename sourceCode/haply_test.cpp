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

#include <string>
#include <vector>
#include <memory>
#include <chrono>
#include <thread>
#include <atomic>

#include "HardwareAPI.h"

using namespace std;

int main()
{
    string portInv3;
    string portHandle;
    constexpr double targetHz = 1000;

#if 1
    portInv3 = "/dev/cu.usbmodem163718101";
    portHandle = "/dev/cu.usbmodem22401";
#else
    for(const string &s : Haply::HardwareAPI::Devices::DeviceDetection::DetectInverse3s())
        cout << "found Inverse3 at " << (portInv3 = s) << endl;
    for(const string &s : Haply::HardwareAPI::Devices::DeviceDetection::DetectHandles())
        cout << "found Handle at " << (portHandle = s) << endl;
    for(const string &s : Haply::HardwareAPI::Devices::DeviceDetection::DetectWiredHandles())
        cout << "found wired Handle at " << (portHandle = s) << endl;
    for(const string &s : Haply::HardwareAPI::Devices::DeviceDetection::DetectWirelessHandles())
        cout << "found wireless Handle at " << (portHandle = s) << endl;
#endif

    Haply::HardwareAPI::IO::SerialStream ssInv3(portInv3.c_str());
    Haply::HardwareAPI::Devices::Inverse3 devInv3(&ssInv3);

    Haply::HardwareAPI::Devices::Inverse3::DeviceInfoResponse resp = devInv3.DeviceWakeup();
    cout << "Device ID: " << resp.device_id << endl;

    Haply::HardwareAPI::IO::SerialStream ssHandle(portHandle.c_str());
    Haply::HardwareAPI::Devices::Handle devHandle(&ssHandle);
    devHandle.SendDeviceWakeup();
    devHandle.RequestStatus();

    using clock = std::chrono::steady_clock;
    constexpr std::chrono::microseconds target_duration(long(1000000. / targetHz));
    auto next_tick = clock::now();

    while(true)
    {
        next_tick += target_duration;
        auto now = clock::now();

        {
            Haply::HardwareAPI::Devices::Inverse3::EndEffectorForceRequest req;
            req.force[0] = 0.0;
            req.force[1] = 0.0;
            req.force[2] = 0.0;
            Haply::HardwareAPI::Devices::Inverse3::EndEffectorStateResponse resp = devInv3.EndEffectorForce(req, true);
            cout << " pos=[" << resp.position[0] << ", " << resp.position[1] << ", " << resp.position[2] << "]";
        }
        {
            Haply::HardwareAPI::Devices::Handle::VersegripStatusResponse resp = devHandle.GetVersegripStatus();
            cout << " btns=" << int(resp.buttons);
        }
        cout << endl;

        if(next_tick > now)
            std::this_thread::sleep_until(next_tick);
    }

    return 0;
}
