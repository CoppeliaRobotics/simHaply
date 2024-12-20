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
#include <mutex>
#include <condition_variable>

#include "HardwareAPI.h"

#include "config.h"
#include "plugin.h"
#include <simPlusPlus/Plugin.h>
#include <simPlusPlus/Handles.h>
#include "stubs.h"

template<typename T, size_t N>
std::vector<T> toVector(const std::array<T, N> &a)
{
    std::vector<T> v;
    for(size_t i = 0; i < N; i++)
        v.push_back(a[i]);
    return v;
}

template<typename T, size_t N>
std::array<T, N> toArray(const std::vector<T> &v)
{
    std::array<T, N> a;
    for(size_t i = 0; i < std::min(v.size(), N); i++)
        a[i] = v[i];
    return a;
}

template<typename T, int initMode>
struct Haply_Device
{
    Haply_Device(const std::string &port)
        : port(port)
    {
        std::cout << "Haply_Device[" << port << "]: opening" << std::endl;

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

        std::cout << "Haply_Device[" << port << "]: closed" << std::endl;
    }

protected:
    std::string port;
    Haply::HardwareAPI::IO::SerialStream *serial_stream;
    T *device;
};

struct ControlLoop
{
    ControlLoop(const std::string &name)
        : name(name)
    {
        start();
    }

    virtual ~ControlLoop()
    {
        stop();
    }

    virtual void tick()
    {
    }

    void run()
    {
        std::cout << "ControlLoop[" << name << "] starting" << std::endl;

        using clock = std::chrono::steady_clock;
        constexpr std::chrono::microseconds target_duration(1000); // 1 kHz = 1000 microseconds
        auto next_tick = clock::now();

        while(running.load())
        {
            next_tick += target_duration;

            auto start = std::chrono::steady_clock::now();
            tick();
            auto end = std::chrono::steady_clock::now();
            int elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
            if(elapsed_ms > 100)
                std::cout << "ControlLoop[" << name << "] warning: executing a tick took " << elapsed_ms << "ms" << std::endl;

            tick_num++;

            auto now = clock::now();
            if(next_tick > now)
            {
                std::this_thread::sleep_until(next_tick);
            }
            else
            {
                // Handle overrun
            }
        }

        std::cout << "ControlLoop[" << name << "] terminated" << std::endl;
    }

private:
    void start()
    {
        tick_num = 0;
        running.store(true);
        control_thread = std::thread(&ControlLoop::run, this);
    }

    void stop()
    {
        running.store(false);
        if(control_thread.joinable()) control_thread.join();
    }

protected:
    int tick_num;
    std::mutex mtx;

private:
    std::string name;
    std::atomic<bool> running{false};
    std::thread control_thread;
};

struct Haply_Inverse3 : public Haply_Device<Haply::HardwareAPI::Devices::Inverse3, 1>, public ControlLoop
{
    Haply_Inverse3(const std::string &port)
        : Haply_Device<Haply::HardwareAPI::Devices::Inverse3, 1>(port),
          ControlLoop("Inverse3")
    {
    }

    void tick()
    {
        simhaply_mode m;
        {
            std::lock_guard<std::mutex> lock(mtx);
            m = mode;
        }
        switch(m)
        {
            case simhaply_mode_free:
            {
                Haply::HardwareAPI::Devices::Inverse3::EndEffectorForceRequest req;
                {
                    std::lock_guard<std::mutex> lock(mtx);
                    req.force[0] = 0;
                    req.force[1] = 0;
                    req.force[2] = 0;
                }
                Haply::HardwareAPI::Devices::Inverse3::EndEffectorStateResponse resp = device->EndEffectorForce(req, true);
                {
                    std::lock_guard<std::mutex> lock(mtx);
                    position[0] = resp.position[0];
                    position[1] = resp.position[1];
                    position[2] = resp.position[2];
                    velocity[0] = resp.velocity[0];
                    velocity[1] = resp.velocity[1];
                    velocity[2] = resp.velocity[2];
                }
            }
            break;
            case simhaply_mode_position_ctrl:
            {
            }
            break;
            case simhaply_mode_force_ctrl:
            {
                Haply::HardwareAPI::Devices::Inverse3::EndEffectorForceRequest req;
                {
                    std::lock_guard<std::mutex> lock(mtx);
#if 1
                    req.force[0] = 0;
                    req.force[1] = 0;
                    req.force[2] = 0;
#else
                    req.force[0] = target_force[0];
                    req.force[1] = target_force[1];
                    req.force[2] = target_force[2];
#endif
                }
                Haply::HardwareAPI::Devices::Inverse3::EndEffectorStateResponse resp = device->EndEffectorForce(req, true);
                {
                    std::lock_guard<std::mutex> lock(mtx);
                    position[0] = resp.position[0];
                    position[1] = resp.position[1];
                    position[2] = resp.position[2];
                    velocity[0] = resp.velocity[0];
                    velocity[1] = resp.velocity[1];
                    velocity[2] = resp.velocity[2];
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
                    position[0] = resp.position[0];
                    position[1] = resp.position[1];
                    position[2] = resp.position[2];
                    velocity[0] = resp.velocity[0];
                    velocity[1] = resp.velocity[1];
                    velocity[2] = resp.velocity[2];
                }
            }
            break;
        }
    }

    void setFree()
    {
        std::lock_guard<std::mutex> lock(mtx);
        this->mode = simhaply_mode_free;
    }

    void setTargetPosition(const std::array<double, 3> &target_position)
    {
        std::lock_guard<std::mutex> lock(mtx);
        this->mode = simhaply_mode_position_ctrl;
        this->target_position = target_position;
    }

    void setTargetForce(const std::array<double, 3> &target_force)
    {
        std::lock_guard<std::mutex> lock(mtx);
        this->mode = simhaply_mode_force_ctrl;
        this->target_force = target_force;
    }

    void setConstraint(const std::array<double, 3> &p, const std::array<double, 3> &n)
    {
        std::lock_guard<std::mutex> lock(mtx);
        this->mode = simhaply_mode_constraint;
        this->p = p;
        this->n = n;
    }

    void setConstraintForce(double kf, double maxf)
    {
        std::lock_guard<std::mutex> lock(mtx);
        this->mode = simhaply_mode_constraint;
        this->kf = kf;
        this->maxf = maxf;
    }

    std::array<double, 3> getPosition()
    {
        std::lock_guard<std::mutex> lock(mtx);
        return position;
    }

    std::array<double, 3> getVelocity()
    {
        std::lock_guard<std::mutex> lock(mtx);
        return velocity;
    }

private:
    simhaply_mode mode{simhaply_mode_free};
    std::array<double, 3> target_position;
    std::array<double, 3> target_force;
    std::array<double, 3> p;
    std::array<double, 3> n;
    std::array<double, 3> position;
    std::array<double, 3> velocity;
    double kf{10.0};
    double maxf{10.0};
};

struct Haply_Handle : public Haply_Device<Haply::HardwareAPI::Devices::Handle, 2>, public ControlLoop
{
    Haply_Handle(const std::string &port)
        : Haply_Device<Haply::HardwareAPI::Devices::Handle, 2>(port),
          ControlLoop("Handle")
    {
    }

    void tick()
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

    std::array<double, 4> getQuaternion()
    {
        return quaternion;
    }

    int getButtons()
    {
        return buttons;
    }

    float getBatteryLevel()
    {
        return battery_level;
    }

private:
    std::array<double, 4> quaternion;
    int buttons;
    float battery_level;
};

class Plugin : public sim::Plugin
{
public:
    template<typename T> T * get(const std::string &h) { return nullptr; }

    void onInit()
    {
        if(!registerScriptStuff())
            throw std::runtime_error("script stuff initialization failed");

        setExtVersion("Haply's HardwareAPI plugin");
        setBuildDate(BUILD_DATE);
    }

    void onScriptStateAboutToBeDestroyed(int scriptHandle, long long scriptUid)
    {
        cleanupInverse3Handles(scriptHandle);
        cleanupHandleHandles(scriptHandle);
    }

    template<>
    Haply_Inverse3 * get(const std::string &h)
    {
        auto *item = inverse3Handles.get(h);
        if(!item) throw std::runtime_error("invalid Inverse3 handle");
        return item;
    }

    void detectInverse3s(detectInverse3s_in *in, detectInverse3s_out *out)
    {
        out->ports = Haply::HardwareAPI::Devices::DeviceDetection::DetectInverse3s();
    }

    void openInverse3(openInverse3_in *in, openInverse3_out *out)
    {
        auto *item = new Haply_Inverse3(in->port);
        out->handle = inverse3Handles.add(item, in->_.scriptID);
    }

    void closeInverse3(closeInverse3_in *in, closeInverse3_out *out)
    {
        auto *item = get<Haply_Inverse3>(in->handle);
        delete inverse3Handles.remove(item);
    }

    void cleanupInverse3Handles(int scriptHandle)
    {
        for(auto dev : inverse3Handles.find(scriptHandle))
        {
            closeInverse3_in argin;
            argin.handle = inverse3Handles.toHandle(dev);
            closeInverse3_out argout;
            closeInverse3(&argin, &argout);
        }
    }

    void setInverse3Free(setInverse3Free_in *in, setInverse3Free_out *out)
    {
        auto *item = get<Haply_Inverse3>(in->handle);
        item->setFree();
    }

    void setInverse3TargetPosition(setInverse3TargetPosition_in *in, setInverse3TargetPosition_out *out)
    {
        auto *item = get<Haply_Inverse3>(in->handle);
        item->setTargetPosition(toArray<double, 3>(in->target_position));
    }

    void setInverse3TargetForce(setInverse3TargetForce_in *in, setInverse3TargetForce_out *out)
    {
        auto *item = get<Haply_Inverse3>(in->handle);
        item->setTargetForce(toArray<double, 3>(in->target_force));
    }

    void setInverse3Constraint(setInverse3Constraint_in *in, setInverse3Constraint_out *out)
    {
        auto *item = get<Haply_Inverse3>(in->handle);
        item->setConstraint(toArray<double, 3>(in->p), toArray<double, 3>(in->n));
    }

    void setInverse3ConstraintForce(setInverse3ConstraintForce_in *in, setInverse3ConstraintForce_out *out)
    {
        auto *item = get<Haply_Inverse3>(in->handle);
        item->setConstraintForce(in->kf, in->maxf);
    }

    void getInverse3Position(getInverse3Position_in *in, getInverse3Position_out *out)
    {
        auto *item = get<Haply_Inverse3>(in->handle);
        out->position = toVector<double, 3>(item->getPosition());
    }

    void getInverse3Velocity(getInverse3Velocity_in *in, getInverse3Velocity_out *out)
    {
        auto *item = get<Haply_Inverse3>(in->handle);
        out->velocity = toVector<double, 3>(item->getVelocity());
    }

    template<>
    Haply_Handle * get(const std::string &h)
    {
        auto *item = handleHandles.get(h);
        if(!item) throw std::runtime_error("invalid Handle handle");
        return item;
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

    void openHandle(openHandle_in *in, openHandle_out *out)
    {
        auto *item = new Haply_Handle(in->port);
        out->handle = handleHandles.add(item, in->_.scriptID);
    }

    void closeHandle(closeHandle_in *in, closeHandle_out *out)
    {
        auto *item = get<Haply_Handle>(in->handle);
        delete handleHandles.remove(item);
    }

    void cleanupHandleHandles(int scriptHandle)
    {
        for(auto dev : handleHandles.find(scriptHandle))
        {
            closeHandle_in argin;
            argin.handle = handleHandles.toHandle(dev);
            closeHandle_out argout;
            closeHandle(&argin, &argout);
        }
    }

    void getHandleQuaternion(getHandleQuaternion_in *in, getHandleQuaternion_out *out)
    {
        auto *item = get<Haply_Handle>(in->handle);
        out->quaternion = toVector<double, 4>(item->getQuaternion());
    }

    void getHandleButtons(getHandleButtons_in *in, getHandleButtons_out *out)
    {
        auto *item = get<Haply_Handle>(in->handle);
        out->buttons = item->getButtons();
    }

    void getHandleBatteryLevel(getHandleBatteryLevel_in *in, getHandleBatteryLevel_out *out)
    {
        auto *item = get<Haply_Handle>(in->handle);
        out->battery_level = item->getBatteryLevel();
    }

private:
    sim::Handles<Haply_Inverse3*> inverse3Handles{"Haply_Inverse3"};
    sim::Handles<Haply_Handle*> handleHandles{"Haply_Handle"};
};

SIM_PLUGIN(Plugin)
#include "stubsPlusPlus.cpp"
