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

#include "TargetLandPosEstimator.hpp"

//test diff #define TARGET_DISTANCE_L_R 2.0f
//test results: -1.5, -1.0, 5.0
#define TARGET_DISTANCE_L_R 0.25f


using namespace target_land_pos_estimator;

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

//global instance
TargetLandPosEstimator* instance;

TargetLandPosEstimator::TargetLandPosEstimator()
    : _task_should_exit(false),
      _control_task(-1),
      /* subscriptions */
      _local_pos_sub(-1),
      _pixy5pts_sub(-1),
      _ctrl_state_sub(-1),
      _target_land_position_pub(nullptr),
      _test1(false),
      _test2(false),
      _test3(true)
{
    memset(&_ctrl_state, 0, sizeof(_ctrl_state));
    memset(&_local_pos, 0, sizeof(_local_pos));
    memset(&_target_land_position, 0, sizeof(_target_land_position));

    memset(&_ref_pos, 0, sizeof(_ref_pos));
}

TargetLandPosEstimator::~TargetLandPosEstimator()
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

    target_land_pos_estimator::instance = nullptr;
}

int
TargetLandPosEstimator::start()
{
    ASSERT(_control_task == -1);

    /* start the task */
    _control_task = px4_task_spawn_cmd("target_land_pos_estimator",
                       SCHED_DEFAULT,
                       SCHED_PRIORITY_DEFAULT,
                       4096,
                       (px4_main_t)&TargetLandPosEstimator::task_main_trampoline,
                       nullptr);

    if (_control_task < 0) {
        warn("task start failed");
        return -errno;
    }

    return OK;
}

void
TargetLandPosEstimator::task_main_trampoline(int argc, char *argv[])
{
    target_land_pos_estimator::instance->task_main();
}

void
TargetLandPosEstimator::task_main()
{
    if(_test1) {
        warnx("[target_land_pos_estimator] test1\n");
        //we make test calculations from test points and output result. no usage of global position.
        test1();
    }
    else if(_test2) {
        warnx("[target_land_pos_estimator] test2\n");
        //we publish and define public land position to test landing state machine in simulation
        test2();
    }
    else if(_test3) {
        warnx("[shift_estimator] test3\n");
        //we use the home position plus an offset plus noise to check reaction of copter to scattering position estimations
        test3();
    }
    else {
        warnx("[shift_estimator] starting\n");

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
                //warnx("[shift_estimator] poll timeout");
                continue;
            }

            /* this is undesirable but not much we can do */
            if (pret < 0) {
                warn("poll error %d, %d", pret, errno);
                continue;
            }

            //check if we have new messages on topics
            poll_subscriptions();

            //if we dont have valid local position reference continue
            if( _local_pos.xy_valid == true && _local_pos.z_valid )
                continue;

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

            //if we dont have a valid target continue
            if( !_target.valid )
                continue;

            //calculate yaw
            float yaw = atan2(_target.F(1), _target.F(0));

            //Add shift to local position. As we are in a Body-NED frame it is just a summation.
            _targetPosGlobal(0) = _shift_xyz(0) + _local_pos.x;
            _targetPosGlobal(1) = _shift_xyz(1) + _local_pos.y;
            _targetPosGlobal(2) = _shift_xyz(2) + _local_pos.z;  //we are in NED, z is negative

            /* update local projection reference */
            map_projection_init(&_ref_pos, _local_pos.ref_lat, _local_pos.ref_lon);

            //project target position to global frame
            double est_lat, est_lon;
            map_projection_reproject(&_ref_pos, _targetPosGlobal(0), _targetPosGlobal(1), &est_lat, &est_lon);
            float est_alt = -(_targetPosGlobal(2) - _local_pos.ref_alt);

            //assign target_land_position
            //ghm1todo
            _target_land_position.timestamp = hrt_absolute_time();
            _target_land_position.lat = est_lat;
            _target_land_position.lon = est_lon;
            _target_land_position.alt = est_alt;
            _target_land_position.x = 0.0f;
            _target_land_position.y = 0.0f;
            _target_land_position.z = 0.0f;
            _target_land_position.yaw = yaw;
            _target_land_position.direction_x = 0.0f;
            _target_land_position.direction_y = 0.0f;
            _target_land_position.direction_z = 0.0f;

            //send new target land position over uorb
            if (_target_land_position_pub == nullptr) {
                _target_land_position_pub = orb_advertise(ORB_ID(target_land_position), &_target_land_position);

            } else {
                /* publish it */
                orb_publish(ORB_ID(target_land_position), _target_land_position_pub, &_target_land_position);
            }
        }
    }


    warnx("[shift_estimator] exiting.\n");
    _control_task = -1;
}

void
TargetLandPosEstimator::make_subscriptions()
{
    //attitude from attitude controller
    _ctrl_state_sub = orb_subscribe(ORB_ID(control_state));
    _pixy5pts_sub = orb_subscribe(ORB_ID(camera_norm_coords));
    _local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));
}

void
TargetLandPosEstimator::poll_subscriptions()
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
        //warnx("camera_norm_coords updated");
        orb_copy(ORB_ID(camera_norm_coords), _pixy5pts_sub, &_pixy5pts);
    }
}

void
TargetLandPosEstimator::calculateTargetToCameraShift()
{
    if(_pixy5pts.count == 4)
    {
        //rotate points into NED frame (orthogonal camera rotated about z-axis)  (timestamps should be similar)
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
TargetLandPosEstimator::identifyTargetPoints()
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
TargetLandPosEstimator::findTargetCandidate(const math::Vector<3>& L1, const math::Vector<3>& L2,
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

float TargetLandPosEstimator::ptDistance(const math::Vector<3> &pt1, const math::Vector<3> &pt2)
{
    math::Vector<3> diff = pt1-pt2;
    return sqrt(diff(0)*diff(0) + diff(1)*diff(1) +  diff(2)*diff(2));
}

bool
TargetLandPosEstimator::targetCompare(const Target& lhs, const Target& rhs)
{
    return lhs.distM < rhs.distM;
}

bool
TargetLandPosEstimator::test1()
{
    struct camera_norm_coords_s testPts;
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

    _pixy5pts = testPts;
    _ctrl_state = test_ctrl_state;

    while(!_task_should_exit)
    {
        //time to calculate, simulates poll
        usleep(20000); //20ms

        calculateTargetToCameraShift();

        //do calculation
        //calculateTargetToCameraShift();

//        warnx("Rotated points: %d", _rotPts.size());
//        for( unsigned i=0; i < _rotPts.size(); ++i )
//        {
//            math::Vector<3> pt = _rotPts.at(i);
//            warnx("Pt %d: x= %.2f, y= %.2f, z=%.2f ", i, (double)pt(0), (double)pt(1), (double)pt(3) );
//        }

//        //ouput target candidates
//        warnx("Target candidates: %d", _targetCandidates.size());
//        for( unsigned i=0; i < _targetCandidates.size(); ++i )
//        {
//            Target t = _targetCandidates.at(i);
//            warnx("Candidate %d:", i);
//            warnx("L: x= %.2f, y= %.2f, z=%.2f ", (double)t.L(0), (double)t.L(1), (double)t.L(3) );
//            warnx("R: x= %.2f, y= %.2f, z=%.2f ", (double)t.R(0), (double)t.R(1), (double)t.R(3) );
//            warnx("M: x= %.2f, y= %.2f, z=%.2f ", (double)t.M(0), (double)t.M(1), (double)t.M(3) );
//            warnx("F: x= %.2f, y= %.2f, z=%.2f ", (double)t.F(0), (double)t.F(1), (double)t.F(3) );
//            warnx("distM: %.2f", (double)t.distM);
//            warnx("distF: %.2f", (double)t.distF);
//            warnx("distLR: %.2f", (double)t.distLR);
//            warnx("valid: %.2f", (double)t.valid);
//        }

//        warnx("target.valid: %d", (int)_target.valid );
//        warnx("distLR: %.2f", (double)_target.distLR);
        warnx("_shift_xyz: x= %.2f, y= %.2f, z=%.2f ", (double)_shift_xyz(0), (double)_shift_xyz(1), (double)_shift_xyz(2) );

    }

    return OK;
}

bool
TargetLandPosEstimator::test2()
{
    while(!_task_should_exit)
    {
        //time to calculate, simulates poll
        usleep(20000); //20ms

        //assign target_land_position
        //ghm1todo
        _target_land_position.timestamp = hrt_absolute_time();
        _target_land_position.lat = 47.3668;
        _target_land_position.lon = 8.55;
        _target_land_position.alt = 0.054;
        _target_land_position.x = 0.0f;
        _target_land_position.y = 0.0f;
        _target_land_position.z = 0.0f;

        _target_land_position.yaw = - 3.14;
        _target_land_position.direction_x = 0.0f;
        _target_land_position.direction_y = 0.0f;
        _target_land_position.direction_z = 0.0f;

        //send new target land position over uorb
        if (_target_land_position_pub == nullptr) {
            _target_land_position_pub = orb_advertise(ORB_ID(target_land_position), &_target_land_position);

        } else {
            /* publish it */
            orb_publish(ORB_ID(target_land_position), _target_land_position_pub, &_target_land_position);
        }
    }
    return OK;
}

bool
TargetLandPosEstimator::test3()
{
    //we add a constant offset to home position with an additional random noise
    //lets see how the copter reacts

    //subsrcibe to home pos
    struct home_position_s home_pos;
    memset(&home_pos, 0, sizeof(home_pos));
    int home_position_sub = orb_subscribe( ORB_ID(home_position) );
    //subscribe to local position for reprojection
    _local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));

    //offset to home -> target_land_position
    float x_offs = 1.0f;
    float y_offs = 2.0f;
    float z_offs = 0.0f;
    float yaw_offs = .0f;

    /* wakeup source */
    px4_pollfd_struct_t fds[2];

    fds[0].fd = home_position_sub;
    fds[0].events = POLLIN;

    bool homepos_valid = false;
    while(!homepos_valid)
    {
        /* wait for up to 500ms for data */
        int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 500);

        /* timed out - periodic check for _task_should_exit */
        if (pret == 0) {
            //warnx("[target_land_pos_estimator] poll timeout");
            continue;
        }

        /* this is undesirable but not much we can do */
        if (pret < 0) {
            //warn("poll error %d, %d", pret, errno);
            continue;
        }

        bool updated = false;

        orb_check(home_position_sub, &updated);
        if (updated) {
            warnx("[target_land_pos_estimator] home position updated");
            orb_copy(ORB_ID(home_position), home_position_sub, &home_pos);
        }

        warnx("[target_land_pos_estimator] homepos: lat= %.5f, lon= %.5f, alt= %.5f", home_pos.lat, home_pos.lon, (double)home_pos.alt );

        homepos_valid = true;
    }


    fds[0].fd = _local_pos_sub;
    fds[0].events = POLLIN;

    bool localpos_valid = false;
    while(!localpos_valid)
    {
        /* wait for up to 500ms for data */
        int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 500);

        /* timed out - periodic check for _task_should_exit */
        if (pret == 0) {
            //warnx("[target_land_pos_estimator] poll timeout");
            continue;
        }

        /* this is undesirable but not much we can do */
        if (pret < 0) {
            //warn("poll error %d, %d", pret, errno);
            continue;
        }

        bool updated = false;

        orb_check(_local_pos_sub, &updated);
        if (updated) {
            warnx("[target_land_pos_estimator] local position updated");
            orb_copy(ORB_ID(vehicle_local_position), _local_pos_sub, &_local_pos);
        }

        localpos_valid = true;
    }

    //get reference from local position (the reference is always the same)
    //reproject position in lokal frame to global frame
    /* update local projection reference */
    map_projection_init( &_ref_pos, _local_pos.ref_lat, _local_pos.ref_lon );
    float ref_alt = _local_pos.ref_alt;

    //project homeposition to local frame
    float x_home, y_home;
    map_projection_project(&_ref_pos, home_pos.lat, home_pos.lon, &x_home, &y_home);
    float z_home = -(home_pos.alt - ref_alt);

    //add constant offsets
    float x_target = x_home + x_offs;
    float y_target = y_home + y_offs;
    float z_target = z_home + z_offs;
    float yaw_target = home_pos.yaw + yaw_offs;

    while (!_task_should_exit)
    {
//        //add random noise from -0.05 ... +0.05 to detected offsets
//        float x_n = ((float)( rand() % 20 - 9) / 100 );
//        float y_n = ((float)( rand() % 20 - 9) / 100 );
//        float z_n = ((float)( rand() % 20 - 9) / 100 );
//        float yaw_n = ((float)( rand() % 20 - 9) / 100 );

//        float x = x_target + x_n;
//        float y = y_target + y_n;
//        float z = z_target + z_n;
//        float yaw = yaw_target + yaw_n;

        float x = x_target;
        float y = y_target;
        float z = z_target;
        float yaw = yaw_target;

        //todo: we want to see it for some time before publish (mean over fifo-buffer)

        //reproject target position to global frame
        double est_lat, est_lon;
        map_projection_reproject(&_ref_pos, x, y, &est_lat, &est_lon);
        float est_alt = ref_alt - z;

//        //test: project, to see if the small changes are still the same
//        float x_rep, y_rep;
//        map_projection_project(&_ref_pos, est_lat, est_lon, &x_rep, &y_rep);
//        float z_rep = -(est_alt - ref_alt);

//        //output difference
//        float x_diff = x_rep - x_target;
//        float y_diff = y_rep - y_target;
//        float z_diff = z_rep - z_target;
//        warnx("x_n rep: %.5f %.5f", (double)x_diff, (double)x_n );
//        warnx("y_n rep: %.5f %.5f", (double)y_diff, (double)y_n );
//        warnx("z_n rep: %.5f %.5f", (double)z_diff, (double)z_n );

        //assignment of final global position
        _target_land_position.timestamp = hrt_absolute_time();
        _target_land_position.lat = est_lat;
        _target_land_position.lon = est_lon;
        _target_land_position.alt = est_alt;
        _target_land_position.x = 0.0f;
        _target_land_position.y = 0.0f;
        _target_land_position.z = 0.0f;
        _target_land_position.yaw = yaw;
        _target_land_position.direction_x = 0.0f;
        _target_land_position.direction_y = 0.0f;
        _target_land_position.direction_z = 0.0f;

        //warnx("[target_land_pos_estimator] tarlpos: x= %.5f, y= %.5f, z= %.5f", _target_land_position.lat, _target_land_position.lon, (double)_target_land_position.alt );

        //send new target land position over uorb
        if (_target_land_position_pub == nullptr) {
            _target_land_position_pub = orb_advertise(ORB_ID(target_land_position), &_target_land_position);

        } else {
            /* publish it */
            orb_publish(ORB_ID(target_land_position), _target_land_position_pub, &_target_land_position);
        }

        usleep(20000); //20ms
    }

    return OK;
}


