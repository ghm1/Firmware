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
#include <algorithm>
#include <px4_config.h>
#include <nuttx/sched.h>
#include <math.h>

#include <systemlib/systemlib.h>
#include <mathlib/mathlib.h>
#include <lib/geo/geo.h>
#include <mavlink/mavlink_log.h>
#include <platforms/px4_defines.h>

#include "TargetShiftEstimator.hpp"

#define TARGET_DISTANCE_L_R 0.3f

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
      _control_task(-1),
      /* subscriptions */
      _local_pos_sub(-1),
      _pixy5pts_sub(-1),
      _ctrl_state_sub(-1)
{
    memset(&_ctrl_state, 0, sizeof(_ctrl_state));
    memset(&_local_pos, 0, sizeof(_local_pos));
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
    //subscribe to topics an make a first poll
    make_subscriptions();
    poll_subscriptions();

    while (!_task_should_exit) {
        //check if we have new messages on topics
        poll_subscriptions();
        calculateTargetToCameraShift();

        //ouput example values
        //warnx("local position x: %.2f", (double)_local_pos.x );
        //warnx("roll: %.2f", (double)euler_angles(0) );
        warnx("pixy5pts: %d", _pixy5pts.count );
        warnx("pixy5pts: %.2f", (double)_pixy5pts.x_coord[0] );
        warnx("pixy5pts: %.2f", (double)_pixy5pts.y_coord[0] );

        sleep(1);

        //warnx("[target_shift_estimator] running\n");
    }


    warnx("[target_shift_estimator] exiting.\n");
    _control_task = -1;
}

void
TargetShiftEstimator::make_subscriptions()
{
    //attitude from attitude controller
    _ctrl_state_sub = orb_subscribe(ORB_ID(control_state));
    _pixy5pts_sub = orb_subscribe(ORB_ID(camera_pixy5pts));
    _local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));
}

void
TargetShiftEstimator::poll_subscriptions()
{
    bool updated = false;

    orb_check(_ctrl_state_sub, &updated);

    if (updated) {
        //warnx("control_state updated");
        orb_copy(ORB_ID(control_state), _ctrl_state_sub, &_ctrl_state);
    }

    orb_check(_local_pos_sub, &updated);

    if (updated) {
        //warnx("vehicle_local_position updated");
        orb_copy(ORB_ID(vehicle_local_position), _local_pos_sub, &_local_pos);
    }

    orb_check(_pixy5pts_sub, &updated);

    if (updated) {
        warnx("camera_pixy5pts updated");
        orb_copy(ORB_ID(camera_pixy5pts), _pixy5pts_sub, &_pixy5pts);
    }
}

void
TargetShiftEstimator::calculateTargetToCameraShift()
{
    if(_pixy5pts.count == 4)
    {
        //rotate points into orthogonal camera (timestamps should be similar)
        math::Quaternion q_att(_ctrl_state.q[0], _ctrl_state.q[1], _ctrl_state.q[2], _ctrl_state.q[3]);
        math::Matrix<3, 3> R = q_att.to_dcm();

        std::vector<math::Vector<3>> rotPts(4);
        math::Vector<3> pt;
        for(unsigned i=0; i<_pixy5pts.count; ++i)
        {
            pt(0) = _pixy5pts.x_coord[i];
            pt(1) = _pixy5pts.y_coord[i];
            pt(2) = 1.0;
            //1. todo: ??? ist die rotation korrekt oder matrix invertieren????
            pt = R * pt;
            rotPts.push_back(pt);
        }

        //identify points by sorting
        identifyTargetPoints(rotPts);

        if(_target.valid)
        {
            //calculate heigth above target
            float heightAboveTarget = TARGET_DISTANCE_L_R / _target.distLR;
            //calculate x/y-position offset to target
            math::Vector<3> pos_xy = _target.M * heightAboveTarget;
        }
    }
}

void
TargetShiftEstimator::identifyTargetPoints( std::vector<math::Vector<3>>& rotPts )
{
    //We search for a target with the following shape: L = left, M = middle, R = right, F = direction point
    //************************************************
    //******************* F **************************
    //************************************************
    //************************************************
    //********* L ******* M ******* R ****************
    //************************************************
    //We try to identify the correct target by building a line between two of the four points assuming that these two
    //points are the targets L and R points. For the real target M lies on the line between L and R. Among all target
    //candidates it should be sufficient to identify the target candidate, where M projects between L and R and the
    //orthogonal distance of M to line LR is the shortest among all target candidates.
    //So it is a necessary condition for a valid target candidate, that F and M project
    //on the line segment between R and L. Moreover we calculate the orthogonal distance of M to L-R;
    //Finally the target candidates are sorted by their M to Line-LR distance. The searched target
    //is the one with the shortest distance.

    std::vector<Target> targetCandidates;
    //try to find target candidates and add them to targetCandidates vector
    findTargetCandidate( rotPts.at(0), rotPts.at(1), rotPts.at(2), rotPts.at(3), targetCandidates );
    findTargetCandidate( rotPts.at(0), rotPts.at(2), rotPts.at(1), rotPts.at(3), targetCandidates );
    findTargetCandidate( rotPts.at(0), rotPts.at(3), rotPts.at(1), rotPts.at(2), targetCandidates );
    findTargetCandidate( rotPts.at(1), rotPts.at(2), rotPts.at(0), rotPts.at(3), targetCandidates );
    findTargetCandidate( rotPts.at(1), rotPts.at(3), rotPts.at(0), rotPts.at(2), targetCandidates );
    findTargetCandidate( rotPts.at(2), rotPts.at(3), rotPts.at(0), rotPts.at(1), targetCandidates );

    //sort list according to smallest M to L-R distance
    std::sort(targetCandidates.begin(), targetCandidates.end(), targetCompare);

    if( targetCandidates.size())
    {
        _target = targetCandidates.at(0);
        _target.valid = true;
        //calculate distance between left and right target point
        _target.distLR = ptDistance( _target.L, _target.R );
    }
}

void
TargetShiftEstimator::findTargetCandidate(const math::Vector<3>& L1, const math::Vector<3>& L2,
                                          const math::Vector<3>& P1, const math::Vector<3>& P2,
                                          std::vector<Target>& targetCandidates )
{
    //direction vector u from L1 to L2
    math::Vector<3> u = L2 - L1;
    float uSq = u * u;

    //projection of first point
    float lambda = (P1 - L1) * u / uSq;
    //test, if projected point is on line between L1 and L2 (L and R)
    if( lambda > 1.0f || lambda < 0.0f )
        return;
    //calculate projected point
    math::Vector<3> projP1 = L1 + u * lambda;
    //calculate line to point distance
    float distP1 = ptDistance(projP1, P1);

    //projection of second point
    lambda = (P2 - L1) * u / uSq;
    //test, if projected point is on line between L1 and L2 (L and R)
    if( lambda > 1.0f || lambda < 0.0f )
        return;
    //calculate projected point
    math::Vector<3> projP2 = L1 + u * lambda;
    //calculate line to point distance
    float distP2 = ptDistance(projP2, P2);

    Target cand;
    //find middle and direction point
    if(distP1 < distP2)
    {
        //P1 lies nearer to line and could be M in an valid target
        cand.M = P1;
        cand.distM = distP1;
        cand.F = P2;
        cand.distF = distP2;
    }
    else
    {
        //P2 lies nearer to line and could be M in an valid target
        cand.M = P2;
        cand.distM = distP2;
        cand.F = P1;
        cand.distF = distP1;
    }

    //find left and right point (under the precondition, that the other points are ordered correctly)
    float t = (L1(0)-cand.M(0))*(cand.F(1)-cand.M(1)) - (L1(1)-cand.M(1))*(cand.F(0)-cand.M(0));
    if( t > 0.0f )
    {
        cand.L = L1;
        cand.R = L2;
    }
    else
    {
        cand.L = L2;
        cand.R = L1;
    }

    targetCandidates.push_back(cand);
}

float TargetShiftEstimator::ptDistance(const math::Vector<3> &pt1, const math::Vector<3> &pt2)
{
    math::Vector<3> diff = pt1-pt2;
    return sqrt(diff(0)*diff(0) + diff(1)*diff(1) +  diff(2)*diff(2));
}

bool
TargetShiftEstimator::targetCompare(const Target& lhs, const Target& rhs)
{
    return lhs.distM < rhs.distM;
}



