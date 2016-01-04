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

//test diff #define TARGET_DISTANCE_L_R 2.0f
//test results: -1.5, -1.0, 5.0
#define TARGET_DISTANCE_L_R 0.25f


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
      _ctrl_state_sub(-1),
      _test(false)
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
    if(!_test)
    {
        warnx("[target_shift_estimator] starting\n");

        //subscribe to topics an make a first poll
        make_subscriptions();
        poll_subscriptions();

        /* wakeup source */
        px4_pollfd_struct_t fds[1];

        fds[0].fd = _pixy5pts_sub;
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

//            int count = _pixy5pts.count;
//            if(count == 4)
//            {
//                //warnx("unrotated pts: %d", count);
//                for(unsigned i=0; i < count; i++ )
//                {
//                    warnx("Pt %d: x= %.2f, y= %.2f", i, (double)_pixy5pts.x_coord[i], (double)_pixy5pts.y_coord[i] );
//                }
//            }

            //do calculation
            calculateTargetToCameraShift();
        }
    }
    else
    {
        warnx("[target_shift_estimator] test\n");

        //do calculation
        calculateTargetToCameraShift();

        warnx("Rotated points: %d", _rotPts.size());
        for( unsigned i=0; i < _rotPts.size(); ++i )
        {
            math::Vector<3> pt = _rotPts.at(i);
            warnx("Pt %d: x= %.2f, y= %.2f, z=%.2f ", i, (double)pt(0), (double)pt(1), (double)pt(3) );
        }

        //ouput target candidates
        warnx("Target candidates: %d", _targetCandidates.size());
        for( unsigned i=0; i < _targetCandidates.size(); ++i )
        {
            Target t = _targetCandidates.at(i);
            warnx("Candidate %d:", i);
            warnx("L: x= %.2f, y= %.2f, z=%.2f ", (double)t.L(0), (double)t.L(1), (double)t.L(3) );
            warnx("R: x= %.2f, y= %.2f, z=%.2f ", (double)t.R(0), (double)t.R(1), (double)t.R(3) );
            warnx("M: x= %.2f, y= %.2f, z=%.2f ", (double)t.M(0), (double)t.M(1), (double)t.M(3) );
            warnx("F: x= %.2f, y= %.2f, z=%.2f ", (double)t.F(0), (double)t.F(1), (double)t.F(3) );
            warnx("distM: %.2f", (double)t.distM);
            warnx("distF: %.2f", (double)t.distF);
            warnx("distLR: %.2f", (double)t.distLR);
            warnx("valid: %.2f", (double)t.valid);
        }

        warnx("target.valid: %d", (int)_target.valid );
        warnx("distLR: %.2f", (double)_target.distLR);
        warnx("_shift_xyz: x= %.2f, y= %.2f, z=%.2f ", (double)_shift_xyz(0), (double)_shift_xyz(1), (double)_shift_xyz(2) );

        _task_should_exit = true;

        sleep(1);
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
        //warnx("camera_pixy5pts updated");
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
        //warnx("%.2f %.2f %.2f", (double)R(0,0), (double)R(0,1), (double)R(0,2));
        //warnx("%.2f %.2f %.2f", (double)R(1,0), (double)R(1,1), (double)R(1,2));
        //warnx("%.2f %.2f %.2f", (double)R(2,0), (double)R(2,1), (double)R(2,2));
//        math::Vector<3> euler_angles;
//        euler_angles = R.to_euler();
//        warnx("euler: %.2f %.2f %.2f", (double)euler_angles(0), (double)euler_angles(1), (double)euler_angles(2));

//        warnx("rotated points");
        _rotPts.clear();
        math::Vector<3> pt;
        for(unsigned i=0; i<_pixy5pts.count; ++i)
        {
            pt(0) = _pixy5pts.x_coord[i];
            pt(1) = _pixy5pts.y_coord[i];
            pt(2) = 1.0;
            pt = R * pt;
            //to normalized image coordinates
            pt(0) = pt(0) / pt(2);
            pt(1) = pt(1) / pt(2);
            pt(2) = 1.0;

            _rotPts.push_back(pt);
//            warnx("Pt %.2f %.2f %.2f", (double)pt(0), (double)pt(1), (double)pt(2));
        }

        //identify points by sorting
        identifyTargetPoints();

        if(_target.valid)
        {
            //calculate heigth above target
            _shift_xyz(2) = TARGET_DISTANCE_L_R / _target.distLR;
            //calculate x/y-position offset to target
            _shift_xyz(0) = _target.M(0) * _shift_xyz(2);
            _shift_xyz(1) = _target.M(1) * _shift_xyz(2);
            _target.valid = false;

            //warnx("distLR: %.2f", (double)_target.distLR);
            warnx("_shift_xyz: x= %.2f, y= %.2f, z=%.2f ", (double)_shift_xyz(0), (double)_shift_xyz(1), (double)_shift_xyz(2) );
        }
    }
}

void
TargetShiftEstimator::identifyTargetPoints()
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

    _targetCandidates.clear();
    //try to find target candidates and add them to targetCandidates vector
    findTargetCandidate( _rotPts.at(0), _rotPts.at(1), _rotPts.at(2), _rotPts.at(3) );
    findTargetCandidate( _rotPts.at(0), _rotPts.at(2), _rotPts.at(1), _rotPts.at(3) );
    findTargetCandidate( _rotPts.at(0), _rotPts.at(3), _rotPts.at(1), _rotPts.at(2) );
    findTargetCandidate( _rotPts.at(1), _rotPts.at(2), _rotPts.at(0), _rotPts.at(3) );
    findTargetCandidate( _rotPts.at(1), _rotPts.at(3), _rotPts.at(0), _rotPts.at(2) );
    findTargetCandidate( _rotPts.at(2), _rotPts.at(3), _rotPts.at(0), _rotPts.at(1) );

    //sort list according to smallest M to L-R distance
    std::sort(_targetCandidates.begin(), _targetCandidates.end(), targetCompare );

    if( _targetCandidates.size())
    {
        _target = _targetCandidates.at(0);
        _target.valid = true;
        //calculate distance between left and right target point
        _target.distLR = ptDistance( _target.L, _target.R );
    }
}

void
TargetShiftEstimator::findTargetCandidate(const math::Vector<3>& L1, const math::Vector<3>& L2,
                                          const math::Vector<3>& P1, const math::Vector<3>& P2 )
{
    //direction vector u from L1 to L2
    math::Vector<3> u = L2 - L1;
    float uSq = u * u;

    //projection of first point
    float lambda = (P1 - L1) * u / uSq;
    //warnx("lambda: %.2f", (double)lambda);
    //test, if projected point is on line between L1 and L2 (L and R)
    if( lambda > 1.0f || lambda < 0.0f )
        return;
    //calculate projected point
    math::Vector<3> projP1 = L1 + u * lambda;
    //calculate line to point distance
    float distP1 = ptDistance(projP1, P1);

    //projection of second point
    lambda = (P2 - L1) * u / uSq;
    //warnx("lambda: %.2f", (double)lambda);
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

    _targetCandidates.push_back(cand);
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

bool
TargetShiftEstimator::initTest( const struct camera_pixy5pts_s& testPts,
                                const struct control_state_s& test_ctrl_state )
{
    _test = true;
    _pixy5pts = testPts;
    _ctrl_state = test_ctrl_state;

    return OK;
}


