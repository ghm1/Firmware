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

using namespace shift_estimator;

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

//global instance
TargetShiftEstimator* instance;

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
