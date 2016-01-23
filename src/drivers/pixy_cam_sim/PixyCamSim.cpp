#include <px4_config.h>
#include <px4_defines.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <functional>
#include <drivers/drv_hrt.h>
#include <arch/board/board.h>
#include <vector>
#include <queue>
#include <algorithm>
#include <px4_config.h>
//#include <nuttx/sched.h>
#include <math.h>

#include <systemlib/systemlib.h>
#include <mathlib/mathlib.h>
#include <lib/geo/geo.h>
#include <mavlink/mavlink_log.h>
#include <platforms/px4_defines.h>

#include "PixyCamSim.hpp"

//#define PIXY_CAM_CENTER_X				160.44f			// the x-axis center pixel position
//#define PIXY_CAM_CENTER_Y				100.75f		// the y-axis center pixel position
//#define PIXY_CAM_FOCAL_X                323.70584f       // focal length in x-direction
//#define PIXY_CAM_FOCAL_Y                324.26201f       // focal length in y-direction

#define PIXY_CAM_SIM_FOCAL 322.5f
#define PIXY_CAM_SIM_CENTER_X 159.0f
#define PIXY_CAM_SIM_CENTER_Y 99.0f
//NEU
#define PIXY_CAM_SIM_NO_PTS 4
using namespace pixy_cam_sim;

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

//global pixy_instance
//PixyCamSim* pixy_instance;

PixyCamSim::PixyCamSim()
    : _task_should_exit(false),
      _control_task(-1),
      _pixy_cam_pts_sub(-1),
      _camera_norm_coords_pub(nullptr)
{
    memset(&_pixy_cam_pts_s, 0, sizeof(_pixy_cam_pts_s));
}

PixyCamSim::~PixyCamSim()
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

    pixy_cam_sim::pixy_instance = nullptr;
}

int
PixyCamSim::start()
{
    ASSERT(_control_task == -1);

    /* start the task */    _control_task = px4_task_spawn_cmd("pixy_cam_sim",
                       SCHED_DEFAULT,
                       SCHED_PRIORITY_DEFAULT,
                       4096,
                       (px4_main_t)&PixyCamSim::task_main_trampoline,
                       nullptr);

    if (_control_task < 0) {
        warn("task start failed");
        return -errno;
    }

    return OK;
}

void
PixyCamSim::task_main_trampoline(int argc, char *argv[])
{
    pixy_cam_sim::pixy_instance->task_main();
}

void
PixyCamSim::task_main()
{
    warnx("[pixy_cam_sim] starting\n");

    //for false point rejection
    std::vector<float> ptAreas(PIXY_CAM_SIM_NO_PTS);
    bool checkPtsAreas = true;

    //subscribe to topics an make a first poll
    make_subscriptions();
    poll_subscriptions();

    /* wakeup source */
    px4_pollfd_struct_t fds[1];

    fds[0].fd = _pixy_cam_pts_sub;
    fds[0].events = POLLIN;

    while (!_task_should_exit)
    {
        /* wait for up to 500ms for data */
        int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 500);

        /* timed out - periodic check for _task_should_exit */
        if (pret == 0) {
            continue;
        }

        /* this is undesirable but not much we can do */
        if (pret < 0) {
            warn("poll error %d, %d", pret, errno);
            continue;
        }

        //check if we have new messages on topics
        poll_subscriptions();

//        if(_pixy_cam_pts_s.count == 0)
//            continue;

        //todo:
        //make intrinsic transformation
        int count = _pixy_cam_pts_s.count;
        _camera_norm_coords_s.count = count;

//neu
        //we need four points, there is no optimization for error point extraction
        if(count != PIXY_CAM_SIM_NO_PTS) {
            continue;
        }

        if( checkPtsAreas )
        {
            //check point size
            for( int i=0; i < PIXY_CAM_SIM_NO_PTS; ++i )
            {
                ptAreas[i] = _pixy_cam_pts_s.width[i] * _pixy_cam_pts_s.heigth[i];
            }

            std::sort(ptAreas.begin(), ptAreas.end());
            //if the largest point is more than three times the size of the smallest this is no valid target
            if(ptAreas[3] > 3*ptAreas[0]) {
                continue;
            }
        }
//neu ende

        for( int i=0; i<PIXY_CAM_SIM_NO_PTS; ++i )
        {
            //intrinsic transform
            float x = (_pixy_cam_pts_s.x[i] - PIXY_CAM_SIM_CENTER_X) / PIXY_CAM_SIM_FOCAL;
            float y = (_pixy_cam_pts_s.y[i] - PIXY_CAM_SIM_CENTER_Y) / PIXY_CAM_SIM_FOCAL;

            //undistortion not required in simulation

            //todo: reject points depending on width / heigth

            //rotate camera about -90 degree around z (x becomes -y and y becomes x)
            //x becomes -y
            _camera_norm_coords_s.x_coord[i] = -y;
            //y becomes x
            _camera_norm_coords_s.y_coord[i] = x;
        }

        //send new target land position over uorb
        if (_camera_norm_coords_pub == nullptr) {
            _camera_norm_coords_pub = orb_advertise(ORB_ID(camera_norm_coords), &_camera_norm_coords_s);

        } else {
            /* publish it */
            orb_publish(ORB_ID(camera_norm_coords), _camera_norm_coords_pub, &_camera_norm_coords_s);
        }
    }


    warnx("[pixy_cam_sim] exiting.\n");
    _control_task = -1;
}

void
PixyCamSim::make_subscriptions()
{
    _pixy_cam_pts_sub = orb_subscribe(ORB_ID(pixy_cam_pts));
}

void
PixyCamSim::poll_subscriptions()
{
    bool updated = false;

    orb_check(_pixy_cam_pts_sub, &updated);

    if (updated) {
        //warnx("[PixyCamSim] updated");
        orb_copy(ORB_ID(pixy_cam_pts), _pixy_cam_pts_sub, &_pixy_cam_pts_s);
    }
}
