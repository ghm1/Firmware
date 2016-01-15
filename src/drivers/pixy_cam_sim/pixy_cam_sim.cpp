/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file pixy_cam_sim.cpp
 * Pixycamera sim driver. Receives pixy_cam_pts from gazebo simulation.
 *
 * @author Michael Göttlicher <michael.goettlicher@bfh.ch>
 */

//#include <px4_config.h>
//#include <px4_tasks.h>
//#include <px4_posix.h>
//#include <unistd.h>
//#include <stdio.h>
//#include <poll.h>
//#include <string.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>

#include <px4_config.h>
//#include <nuttx/sched.h>

#include <systemlib/systemlib.h>
#include <systemlib/err.h>
#include "PixyCamSim.hpp"


extern "C" __EXPORT int pixy_cam_sim_main(int argc, char *argv[]);

namespace pixy_cam_sim {
    //global instance
    PixyCamSim* pixy_instance;
}

int pixy_cam_sim_main(int argc, char *argv[])
{
    if (argc < 2) {
        warnx("usage: pixy_cam_sim {start|stop|status}");
        return 1;
    }

    if (!strcmp(argv[1], "start")) {

        if (pixy_cam_sim::pixy_instance != nullptr) {
            warnx("already running");
            return 1;
        }

        pixy_cam_sim::pixy_instance = new pixy_cam_sim::PixyCamSim;

        if (pixy_cam_sim::pixy_instance == nullptr) {
            warnx("alloc failed");
            return 1;
        }

        if (OK != pixy_cam_sim::pixy_instance->start()) {
            delete pixy_cam_sim::pixy_instance;
            pixy_cam_sim::pixy_instance = nullptr;
            warnx("start failed");
            return 1;
        }

        return 0;
    }

    if (!strcmp(argv[1], "stop")) {
        if (pixy_cam_sim::pixy_instance == nullptr) {
            warnx("not running");
            return 1;
        }

        delete pixy_cam_sim::pixy_instance;
        pixy_cam_sim::pixy_instance = nullptr;
        return 0;
    }

    if (!strcmp(argv[1], "status")) {
        if (pixy_cam_sim::pixy_instance) {
            warnx("running");
            return 0;

        } else {
            warnx("not running");
            return 1;
        }
    }

    warnx("unrecognized command");
    return 1;
}

