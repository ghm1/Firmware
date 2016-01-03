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
 * @file target_shift_estimator.cpp
 * This application estimates the vehicle position relative to a detected target
 * by the pixy camera
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


//#include <uORB/uORB.h>
//#include <uORB/topics/sensor_combined.h>
//#include <uORB/topics/vehicle_attitude.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>

#include <px4_config.h>
#include <nuttx/sched.h>

#include <systemlib/systemlib.h>
#include <systemlib/err.h>

#include "TargetShiftEstimator.hpp"

extern "C" __EXPORT int target_shift_estimator_main(int argc, char *argv[]);
bool makeTest();

namespace shift_estimator {
    //global instance
    TargetShiftEstimator* instance;
}

bool makeTest()
{
    struct camera_pixy5pts_s testPts;
    testPts.count = 4;
    testPts.timestamp = hrt_absolute_time();
    //add points
//    testPts.x_coord[0] = -0.3006;
//    testPts.y_coord[0] = -0.2003;
//    testPts.x_coord[1] = -0.1000;
//    testPts.y_coord[1] = -0.2000;
//    testPts.x_coord[2] = -0.3005;
//    testPts.y_coord[2] = -0.0000;
//    testPts.x_coord[3] = -0.4998;
//    testPts.y_coord[3] = -0.1998;
    testPts.x_coord[0] = -0.1671;
    testPts.y_coord[0] = -0.1075;
    testPts.x_coord[1] = 0.0238;
    testPts.y_coord[1] = -0.1098;
    testPts.x_coord[2] = -0.1719;
    testPts.y_coord[2] = 0.0850;
    testPts.x_coord[3] = -0.3500;
    testPts.y_coord[3] = -0.1052;

    struct control_state_s test_ctrl_state;
    //math::Quaternion q(1, 0, 0, 0);
    math::Quaternion q(0.99718, 0.043538, -0.06099, -0.0026629);
    test_ctrl_state.q[0] = q(0);
    test_ctrl_state.q[1] = q(1);
    test_ctrl_state.q[2] = q(2);
    test_ctrl_state.q[3] = q(3);

    bool success = shift_estimator::instance->initTest(testPts, test_ctrl_state);

    return success;
}

int target_shift_estimator_main(int argc, char *argv[])
{
    if (argc < 2) {
        warnx("usage: target_shift_estimator {start|stop|status|test}");
        return 1;
    }

    if (!strcmp(argv[1], "start")) {

        if (shift_estimator::instance != nullptr) {
            warnx("already running");
            return 1;
        }

        shift_estimator::instance = new shift_estimator::TargetShiftEstimator;

        if (shift_estimator::instance == nullptr) {
            warnx("alloc failed");
            return 1;
        }

        if (OK != shift_estimator::instance->start()) {
            delete shift_estimator::instance;
            shift_estimator::instance = nullptr;
            warnx("start failed");
            return 1;
        }

        return 0;
    }

    if (!strcmp(argv[1], "stop")) {
        if (shift_estimator::instance == nullptr) {
            warnx("not running");
            return 1;
        }

        delete shift_estimator::instance;
        shift_estimator::instance = nullptr;
        return 0;
    }

    if (!strcmp(argv[1], "status")) {
        if (shift_estimator::instance) {
            warnx("running");
            return 0;

        } else {
            warnx("not running");
            return 1;
        }
    }

    if (!strcmp(argv[1], "test")) {
        if (shift_estimator::instance) {
            warnx("already running, please stop first for test");
            return 0;

        } else {
            shift_estimator::instance = new shift_estimator::TargetShiftEstimator;

            if (shift_estimator::instance == nullptr) {
                warnx("alloc failed");
                return 1;
            }

            makeTest();
            if (OK != shift_estimator::instance->start()) {
                delete shift_estimator::instance;
                shift_estimator::instance = nullptr;
                warnx("start failed");
                return 1;
            }

            return 1;
        }
    }

    warnx("unrecognized command");
    return 1;
}

