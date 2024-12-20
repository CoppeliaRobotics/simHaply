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

#include "config.h"
#include "plugin.h"
#include <simPlusPlus/Plugin.h>
#include <simPlusPlus/Handles.h>
#include "stubs.h"

struct Haply_Inverse3
{
    Haply::HardwareAPI::IO::SerialStream *serial_stream;
    Haply::HardwareAPI::Devices::Inverse3 *device;
};

struct Haply_Handle
{
    Haply::HardwareAPI::IO::SerialStream *serial_stream;
    Haply::HardwareAPI::Devices::Handle *device;
};

struct HaplyControlLoop : public Haply_Inverse3
{
    void tick()
    {
        double norm = std::sqrt(nx * nx + ny * ny + nz * nz);
        double unx = nx / norm;
        double uny = ny / norm;
        double unz = nz / norm;
        double dx = x - px;
        double dy = y - py;
        double dz = z - pz;
        double sdf = std::min(maxf, std::max(-maxf, kf * std::min(0., dx * unx + dy * uny + dz * unz)));
        Haply::HardwareAPI::Devices::Inverse3::EndEffectorForceRequest req;
        if(!first_tick)
        {
            req.force[0] = -unx * sdf;
            req.force[1] = -uny * sdf;
            req.force[2] = -unz * sdf;
        }
        first_tick = false;
        Haply::HardwareAPI::Devices::Inverse3::EndEffectorStateResponse resp = device->EndEffectorForce(req, true);
        x = resp.position[0];
        y = resp.position[1];
        z = resp.position[2];
    }

    void run()
    {
        using clock = std::chrono::steady_clock;
        constexpr std::chrono::microseconds target_duration(1000); // 1 kHz = 1000 microseconds

        auto next_tick = clock::now();

        while(running.load())
        {
            next_tick += target_duration;

            tick();

            auto now = clock::now();
#if 0
            std::cout << std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count() << "us  x: " << x << "  y: " << y << "  z: " << z << std::endl;
#endif

            if(next_tick > now)
            {
                std::this_thread::sleep_until(next_tick);
            }
            else
            {
                // Optional: Handle overrun (e.g., logging or adjusting next_tick).
            }
        }
    }

    void start()
    {
        running.store(true);
        control_thread = std::thread(&HaplyControlLoop::run, this);
    }

    void stop()
    {
        running.store(false);
        if(control_thread.joinable())
        {
            control_thread.join();
        }
    }

    ~HaplyControlLoop()
    {
        stop();
    }

    double kf{10.0};
    double maxf{10.0};
    double x{0}, y{0}, z{0};
    double px{0}, py{0}, pz{0}, nx{0}, ny{0}, nz{1};
    bool first_tick{true};

private:
    std::atomic<bool> running{false};
    std::thread control_thread;
};

class Plugin : public sim::Plugin
{
public:
    void onInit()
    {
        if(!registerScriptStuff())
            throw std::runtime_error("script stuff initialization failed");

        setExtVersion("Haply's HardwareAPI plugin");
        setBuildDate(BUILD_DATE);
    }

    void onScriptStateAboutToBeDestroyed(int scriptHandle, long long scriptUid)
    {
        for(auto dev : inverse3Handles.find(scriptHandle))
        {
            closeInverse3_in argin;
            argin.handle = inverse3Handles.toHandle(dev);
            closeInverse3_out argout;
            closeInverse3(&argin, &argout);
        }
        for(auto dev : handleHandles.find(scriptHandle))
        {
            closeHandle_in argin;
            argin.handle = handleHandles.toHandle(dev);
            closeHandle_out argout;
            closeHandle(&argin, &argout);
        }
    }

    void detectInverse3s(detectInverse3s_in *in, detectInverse3s_out *out)
    {
        out->ports = Haply::HardwareAPI::Devices::DeviceDetection::DetectInverse3s();
    }

    void detectHandles(detectHandles_in *in, detectHandles_out *out)
    {
        out->ports = Haply::HardwareAPI::Devices::DeviceDetection::DetectHandles();
    }

    void detectWiredHandles(detectWiredHandles_in *in, detectWiredHandles_out *out)
    {
        out->ports = Haply::HardwareAPI::Devices::DeviceDetection::DetectWiredHandles();
    }

    void detectWirelessHandles(detectWirelessHandles_in *in, detectWirelessHandles_out *out)
    {
        out->ports = Haply::HardwareAPI::Devices::DeviceDetection::DetectWirelessHandles();
    }

    template<typename T>
    T * get(const std::string &h)
    {
        throw std::runtime_error("invalid template arg");
    }

    template<>
    Haply_Inverse3 * get(const std::string &h)
    {
        auto *item = inverse3Handles.get(h);
        if(!item)
            throw std::runtime_error("invalid Inverse3 handle");
        return item;
    }

    template<>
    Haply_Handle * get(const std::string &h)
    {
        auto *item = handleHandles.get(h);
        if(!item)
            throw std::runtime_error("invalid Handle handle");
        return item;
    }

    template<>
    HaplyControlLoop * get(const std::string &h)
    {
        auto *item = controlLoopHandles.get(h);
        if(!item)
            throw std::runtime_error("invalid HaplyControlLoop handle");
        return item;
    }

    void openInverse3(openInverse3_in *in, openInverse3_out *out)
    {
        auto *item = new Haply_Inverse3;
        item->serial_stream = new Haply::HardwareAPI::IO::SerialStream(in->port.c_str());
        auto *dev = item->device = new Haply::HardwareAPI::Devices::Inverse3(item->serial_stream);
        out->handle = inverse3Handles.add(item, in->_.scriptID);

        Haply::HardwareAPI::Devices::Inverse3::DeviceInfoResponse resp = dev->DeviceWakeup();
        out->deviceId = resp.device_id;
    }

    void openHandle(openHandle_in *in, openHandle_out *out)
    {
        auto *item = new Haply_Handle;
        item->serial_stream = new Haply::HardwareAPI::IO::SerialStream(in->port.c_str());
        auto *dev = item->device = new Haply::HardwareAPI::Devices::Handle(item->serial_stream);
        out->handle = handleHandles.add(item, in->_.scriptID);

        dev->SendDeviceWakeup();
        dev->RequestStatus();
        out->deviceId = -1;
    }

    void closeInverse3(closeInverse3_in *in, closeInverse3_out *out)
    {
        auto *item = get<Haply_Inverse3>(in->handle);
        delete item->device;
        delete item->serial_stream;
        delete inverse3Handles.remove(item);
    }

    void closeHandle(closeHandle_in *in, closeHandle_out *out)
    {
        auto *item = get<Haply_Handle>(in->handle);
        delete item->device;
        delete item->serial_stream;
        delete handleHandles.remove(item);
    }

    void firmwareVersionExtQuery(firmwareVersionExtQuery_in *in, firmwareVersionExtQuery_out *out)
    {
        auto *dev = get<Haply_Inverse3>(in->handle)->device;
        Haply::HardwareAPI::Devices::Inverse3::FirmwareVersionExtResponse resp = dev->FirmwareVersionExtQuery();
        out->uuid = std::string(reinterpret_cast<const char*>(&resp.firmware_version_id.bytes[0]), resp.firmware_version_id.SIZE);
    }

    void jointTorques(jointTorques_in *in, jointTorques_out *out)
    {
        auto *dev = get<Haply_Inverse3>(in->handle)->device;
        Haply::HardwareAPI::Devices::Inverse3::JointTorquesRequest req;
        int n = std::min(in->torques.size(), size_t(Haply::HardwareAPI::Devices::Inverse3::NUM_JOINTS));
        for(size_t i = 0; i < n; i++) req.torques[i] = in->torques[i];
        Haply::HardwareAPI::Devices::Inverse3::JointStatesResponse resp = dev->JointTorques(req);
        out->angles.resize(n);
        for(size_t i = 0; i < n; i++) out->angles[i] = resp.angles[i];
        out->angularVelocities.resize(n);
        for(size_t i = 0; i < n; i++) out->angularVelocities[i] = resp.angularVelocities[i];
    }

    void jointAngles(jointAngles_in *in, jointAngles_out *out)
    {
        auto *dev = get<Haply_Inverse3>(in->handle)->device;
        Haply::HardwareAPI::Devices::Inverse3::JointAnglesRequest req;
        int n = std::min(in->angles.size(), size_t(Haply::HardwareAPI::Devices::Inverse3::NUM_JOINTS));
        for(size_t i = 0; i < n; i++) req.angles[i] = in->angles[i];
        Haply::HardwareAPI::Devices::Inverse3::JointStatesResponse resp = dev->JointAngles(req);
        out->angles.resize(n);
        for(size_t i = 0; i < n; i++) out->angles[i] = resp.angles[i];
        out->angularVelocities.resize(n);
        for(size_t i = 0; i < n; i++) out->angularVelocities[i] = resp.angularVelocities[i];
    }

    void endEffectorForce(endEffectorForce_in *in, endEffectorForce_out *out)
    {
        auto *dev = get<Haply_Inverse3>(in->handle)->device;
        Haply::HardwareAPI::Devices::Inverse3::EndEffectorForceRequest req;
        int n = std::min(in->force.size(), size_t(Haply::HardwareAPI::VECTOR_SIZE));
        for(size_t i = 0; i < n; i++) req.force[i] = in->force[i];
        Haply::HardwareAPI::Devices::Inverse3::EndEffectorStateResponse resp = dev->EndEffectorForce(req, in->onboard);
        out->position.resize(n);
        for(size_t i = 0; i < n; i++) out->position[i] = resp.position[i];
        out->velocity.resize(n);
        for(size_t i = 0; i < n; i++) out->velocity[i] = resp.velocity[i];
    }

    void endEffectorPosition(endEffectorPosition_in *in, endEffectorPosition_out *out)
    {
        auto *dev = get<Haply_Inverse3>(in->handle)->device;
        Haply::HardwareAPI::Devices::Inverse3::EndEffectorPositionRequest req;
        int n = std::min(in->position.size(), size_t(Haply::HardwareAPI::VECTOR_SIZE));
        for(size_t i = 0; i < n; i++) req.position[i] = in->position[i];
        Haply::HardwareAPI::Devices::Inverse3::EndEffectorStateResponse resp = dev->EndEffectorPosition(req);
        out->position.resize(n);
        for(size_t i = 0; i < n; i++) out->position[i] = resp.position[i];
        out->velocity.resize(n);
        for(size_t i = 0; i < n; i++) out->velocity[i] = resp.velocity[i];
    }

    void deviceOrientationQuery(deviceOrientationQuery_in *in, deviceOrientationQuery_out *out)
    {
        auto *dev = get<Haply_Inverse3>(in->handle)->device;
        Haply::HardwareAPI::Devices::Inverse3::DeviceOrientationResponse resp = dev->DeviceOrientationQuery();
        int n = Haply::HardwareAPI::QUATERNION_SIZE;
        out->quaternion.resize(n);
        for(size_t i = 0; i < n; i++) out->quaternion[i] = resp.quaternion[i];
    }

    void devicePowerQuery(devicePowerQuery_in *in, devicePowerQuery_out *out)
    {
        auto *dev = get<Haply_Inverse3>(in->handle)->device;
        Haply::HardwareAPI::Devices::Inverse3::DevicePowerResponse resp = dev->DevicePowerQuery();
        out->powered = resp.powered;
    }

    void deviceTemperatureQuery(deviceTemperatureQuery_in *in, deviceTemperatureQuery_out *out)
    {
        auto *dev = get<Haply_Inverse3>(in->handle)->device;
        Haply::HardwareAPI::Devices::Inverse3::DeviceTemperatureResponse resp = dev->DeviceTemperatureQuery();
        out->temperature = resp.temperature;
    }

    void motorCurrentsQuery(motorCurrentsQuery_in *in, motorCurrentsQuery_out *out)
    {
        auto *dev = get<Haply_Inverse3>(in->handle)->device;
        Haply::HardwareAPI::Devices::Inverse3::MotorCurrentsResponse resp = dev->MotorCurrentsQuery();
        int n = Haply::HardwareAPI::Devices::Inverse3::NUM_JOINTS;
        out->currents.resize(n);
        for(size_t i = 0; i < n; i++) out->currents[i] = resp.currents[i];
    }

    void getDeviceHandedness(getDeviceHandedness_in *in, getDeviceHandedness_out *out)
    {
        auto *dev = get<Haply_Inverse3>(in->handle)->device;
        Haply::HardwareAPI::Devices::Inverse3::DeviceHandednessPayload resp = dev->GetDeviceHandedness();
        out->handedness = resp.handedness;
    }

    void setDeviceHandedness(setDeviceHandedness_in *in, setDeviceHandedness_out *out)
    {
        auto *dev = get<Haply_Inverse3>(in->handle)->device;
        Haply::HardwareAPI::Devices::Inverse3::DeviceHandednessPayload req;
        req.handedness = in->handedness;
        Haply::HardwareAPI::Devices::Inverse3::DeviceHandednessPayload resp = dev->SetDeviceHandedness(req);
        out->handedness = resp.handedness;
    }

    void getTorqueScaling(getTorqueScaling_in *in, getTorqueScaling_out *out)
    {
        auto *dev = get<Haply_Inverse3>(in->handle)->device;
        Haply::HardwareAPI::Devices::Inverse3::TorqueScalingPayload resp = dev->GetTorqueScaling();
        out->enabled = resp.enabled;
    }

    void setTorqueScaling(setTorqueScaling_in *in, setTorqueScaling_out *out)
    {
        auto *dev = get<Haply_Inverse3>(in->handle)->device;
        Haply::HardwareAPI::Devices::Inverse3::TorqueScalingPayload req;
        req.enabled = in->enabled;
        Haply::HardwareAPI::Devices::Inverse3::TorqueScalingPayload resp = dev->SetTorqueScaling(req);
        out->enabled = resp.enabled;
    }

    void getGravityCompensation(getGravityCompensation_in *in, getGravityCompensation_out *out)
    {
        auto *dev = get<Haply_Inverse3>(in->handle)->device;
        Haply::HardwareAPI::Devices::Inverse3::GravityCompensationPayload resp = dev->GetGravityCompensation();
        out->enabled = resp.enabled;
        out->gravity_scale_factor = resp.gravity_scale_factor;
    }

    void setGravityCompensation(setGravityCompensation_in *in, setGravityCompensation_out *out)
    {
        auto *dev = get<Haply_Inverse3>(in->handle)->device;
        Haply::HardwareAPI::Devices::Inverse3::GravityCompensationPayload req;
        req.enabled = in->enabled;
        req.gravity_scale_factor = in->gravity_scale_factor;
        Haply::HardwareAPI::Devices::Inverse3::GravityCompensationPayload resp = dev->SetGravityCompensation(req);
        out->enabled = resp.enabled;
        out->gravity_scale_factor = resp.gravity_scale_factor;
    }

    void saveConfig(saveConfig_in *in, saveConfig_out *out)
    {
        auto *dev = get<Haply_Inverse3>(in->handle)->device;
        Haply::HardwareAPI::Devices::Inverse3::SaveConfigResponse resp = dev->SaveConfig();
        out->parameters_updated = resp.parameters_updated;
    }

    void getVersegripStatus(getVersegripStatus_in *in, getVersegripStatus_out *out)
    {
        auto *dev = get<Haply_Handle>(in->handle)->device;
        Haply::HardwareAPI::Devices::Handle::VersegripStatusResponse resp = dev->GetVersegripStatus();
        out->device_id = resp.device_id;
        for(int i = 0; i < 4; i++) out->quaternion.push_back(resp.quaternion[i]);
        out->error_flag = resp.error_flag;
        out->hall_effect_sensor_level = resp.hall_effect_sensor_level;
        out->buttons = resp.buttons;
        out->battery_level = resp.battery_level;
        out->q.push_back(resp.q.x);
        out->q.push_back(resp.q.y);
        out->q.push_back(resp.q.z);
        out->q.push_back(resp.q.w);
    }

    void startControlLoop(startControlLoop_in *in, startControlLoop_out *out)
    {
        auto *item = new HaplyControlLoop;
        item->serial_stream = new Haply::HardwareAPI::IO::SerialStream(in->port.c_str());
        auto *dev = item->device = new Haply::HardwareAPI::Devices::Inverse3(item->serial_stream);
        out->handle = controlLoopHandles.add(item, in->_.scriptID);

        Haply::HardwareAPI::Devices::Inverse3::DeviceInfoResponse resp = dev->DeviceWakeup();

        item->start();
    }

    void stopControlLoop(stopControlLoop_in *in, stopControlLoop_out *out)
    {
        auto *item = get<HaplyControlLoop>(in->handle);

        item->stop();

        delete item->device;
        delete item->serial_stream;
        delete controlLoopHandles.remove(item);
    }

    void setControlLoopParams(setControlLoopParams_in *in, setControlLoopParams_out *out)
    {
        auto *item = get<HaplyControlLoop>(in->handle);
        item->kf = in->kf;
        item->maxf = in->maxf;
        item->px = in->p[0];
        item->py = in->p[1];
        item->pz = in->p[2];
        item->nx = in->n[0];
        item->ny = in->n[1];
        item->nz = in->n[2];
        out->tip.push_back(item->x);
        out->tip.push_back(item->y);
        out->tip.push_back(item->z);
    }

private:
    sim::Handles<Haply_Inverse3*> inverse3Handles{"Haply_Inverse3"};
    sim::Handles<Haply_Handle*> handleHandles{"Haply_Handle"};
    sim::Handles<HaplyControlLoop*> controlLoopHandles{"HaplyControlLoop"};
};

SIM_PLUGIN(Plugin)
#include "stubsPlusPlus.cpp"
