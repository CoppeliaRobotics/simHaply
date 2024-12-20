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

    while(true)
    {
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
    }

    return 0;
}
