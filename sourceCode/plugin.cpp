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

#include <simPlusPlus/Plugin.h>
#include <simPlusPlus/Handles.h>

#include "config.h"
#include "plugin.h"
#include "stubs.h"
#include "utils.h"
#include "haply_inverse3.h"
#include "haply_handle.h"

#include "HardwareAPI.h"

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

    void setInverse3GravityCompensation(setInverse3GravityCompensation_in *in, setInverse3GravityCompensation_out *out)
    {
        auto *item = get<Haply_Inverse3>(in->handle);
        item->setGravityCompensation(in->enabled, in->scale_factor);
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

    void setInverse3Attractor(setInverse3Attractor_in *in, setInverse3Attractor_out *out)
    {
        auto *item = get<Haply_Inverse3>(in->handle);
        item->setAttractor(toArray<double, 3>(in->p));
    }

    void setInverse3ForceParams(setInverse3ForceParams_in *in, setInverse3ForceParams_out *out)
    {
        auto *item = get<Haply_Inverse3>(in->handle);
        item->setForceParams(in->kf, in->maxf);
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
