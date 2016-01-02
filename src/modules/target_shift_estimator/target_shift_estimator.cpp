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

#include <px4_config.h>
#include <nuttx/sched.h>

#include <systemlib/systemlib.h>
#include <systemlib/err.h>

static bool thread_should_exit_target_shift = false;     /**< daemon exit flag */
static bool thread_running_target_shift = false;         /**< daemon status flag */
static int daemon_task_target_shift;                     /**< Handle of daemon task / thread */

extern "C" __EXPORT int target_shift_estimator_main(int argc, char *argv[]);

class TargetShiftEstimator
{
public:
    /**
     * Constructor
     */
    TargetShiftEstimator();

    /**
     * Destructor, also kills task.
     */
    ~TargetShiftEstimator();

    /**
     * Start task.
     *
     * @return		OK on success.
     */
    int		start();

private:
    bool		_task_should_exit;		/**< if true, task should exit */
};

/**
 * Mainloop of daemon.
 */
int target_shift_estimator_thread_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);

static void
usage(const char *reason)
{
    if (reason) {
        warnx("%s\n", reason);
    }

    warnx("usage: target_shift_estimator {start|stop|status} [-p <additional params>]\n\n");
}


int target_shift_estimator_main(int argc, char *argv[])
{
    if (argc < 2) {
        usage("missing command");
        return 1;
    }

    if (!strcmp(argv[1], "start")) {

        if (thread_running_target_shift) {
            warnx("target_shift_estimator already running\n");
            /* this is not an error */
            return 0;
        }

        thread_should_exit_target_shift = false;
        daemon_task_target_shift = px4_task_spawn_cmd("target_shift_estimator",
                         SCHED_DEFAULT,
                         SCHED_PRIORITY_DEFAULT,
                         2000,
                         target_shift_estimator_thread_main,
                         (argv) ? (char *const *)&argv[2] : (char *const *)NULL);
        return 0;
    }

    if (!strcmp(argv[1], "stop")) {
        thread_should_exit_target_shift = true;
        return 0;
    }

    if (!strcmp(argv[1], "status")) {
        if (thread_running_target_shift) {
            warnx("\trunning\n");

        } else {
            warnx("\tnot started\n");
        }

        return 0;
    }

    usage("unrecognized command");
    return 1;
}


int target_shift_estimator_thread_main(int argc, char *argv[])
{

    warnx("[target_shift_estimator] starting\n");

    thread_running_target_shift = true;

    while (!thread_should_exit_target_shift) {
        warnx("Hello target_shift_estimator!\n");
        sleep(10);
    }

    warnx("[target_shift_estimator] exiting.\n");

    thread_running_target_shift = false;

    return 0;
}
