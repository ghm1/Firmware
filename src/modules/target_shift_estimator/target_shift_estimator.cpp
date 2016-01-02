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
    int         _control_task;			/**< task handle for task */

    /**
     * Shim for calling task_main from task_create.
     */
    static void	task_main_trampoline(int argc, char *argv[]);

    /**
     * Main sensor collection task.
     */
    void		task_main();
};

namespace shift_estimator
{
    /* oddly, ERROR is not defined for c++ */
    #ifdef ERROR
    # undef ERROR
    #endif
    static const int ERROR = -1;

    TargetShiftEstimator	*instance;
}



TargetShiftEstimator::TargetShiftEstimator()
    : _task_should_exit(false),
      _control_task(-1)
{

}

TargetShiftEstimator::~TargetShiftEstimator()
{
    if (_control_task != -1) {
        /* task wakes up every 100ms or so at the longest */
        _task_should_exit = true;

        /* wait for a second for the task to quit at our request */
        unsigned i = 0;

        do {
            /* wait 20ms */
            usleep(20000);

            /* if we have given up, kill it */
            if (++i > 50) {
                px4_task_delete(_control_task);
                break;
            }
        } while (_control_task != -1);
    }

    shift_estimator::instance = nullptr;
}

int
TargetShiftEstimator::start()
{
    ASSERT(_control_task == -1);

    /* start the task */
    _control_task = px4_task_spawn_cmd("target_shift_estimator",
                       SCHED_DEFAULT,
                       SCHED_PRIORITY_DEFAULT,
                       4096,
                       (px4_main_t)&TargetShiftEstimator::task_main_trampoline,
                       nullptr);

    if (_control_task < 0) {
        warn("task start failed");
        return -errno;
    }

    return OK;
}

void
TargetShiftEstimator::task_main_trampoline(int argc, char *argv[])
{
    shift_estimator::instance->task_main();
}

void
TargetShiftEstimator::task_main()
{
    warnx("[target_shift_estimator] starting\n");

    while (!_task_should_exit) {
        warnx("Hello target_shift_estimator!\n");
        sleep(20);

        //parameter update
        //poll subscribtions
        //main loop while schleife
    }

    warnx("[target_shift_estimator] exiting.\n");

    _control_task = -1;
}


int target_shift_estimator_main(int argc, char *argv[])
{
    if (argc < 2) {
        warnx("usage: target_shift_estimator {start|stop|status}");
        return 1;
    }

    if (!strcmp(argv[1], "start")) {

        if (shift_estimator::instance != nullptr) {
            warnx("already running");
            return 1;
        }

        shift_estimator::instance = new TargetShiftEstimator;

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

    warnx("unrecognized command");
    return 1;
}

