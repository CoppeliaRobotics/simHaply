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

#include "control_loop.h"

ControlLoop::ControlLoop(const std::string &name)
    : name(name)
{
    start();
}

ControlLoop::~ControlLoop()
{
    stop();
}

void ControlLoop::tick()
{
}

void ControlLoop::run()
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

void ControlLoop::start()
{
    tick_num = 0;
    running.store(true);
    control_thread = std::thread(&ControlLoop::run, this);
}

void ControlLoop::stop()
{
    running.store(false);
    if(control_thread.joinable()) control_thread.join();
}
