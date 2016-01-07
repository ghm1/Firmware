/****************************************************************************
 *
 *   Copyright (c) 2013-2014 PX4 Development Team. All rights reserved.
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
 * @file target_land.cpp
 * Helper class to access TARGET_LAND
 * related to rtl helper class
 * @author Michael Göttlicher <michael.goettlicher@bfh.ch>
 */

#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <fcntl.h>

#include <mavlink/mavlink_log.h>
#include <systemlib/err.h>
#include <geo/geo.h>

#include <uORB/uORB.h>
#include <navigator/navigation.h>
#include <uORB/topics/home_position.h>

#include "navigator.h"
#include "target_land.h"

#define DELAY_SIGMA	0.01f

TargetLand::TargetLand(Navigator *navigator, const char *name) :
	MissionBlock(navigator, name),
    _target_land_state(TARGET_LAND_STATE_NONE),
    _target_land_start_lock(false),
    _param_return_alt(this, "TARGET_LAND_RETURN_ALT", false),
    _param_descend_alt(this, "TARGET_LAND_DESCEND_ALT", false),
    _param_land_delay(this, "TARGET_LAND_LAND_DELAY", false)
{
	/* load initial params */
	updateParams();
	/* initial reset */
	on_inactive();
}

TargetLand::~TargetLand()
{
}

void
TargetLand::on_inactive()
{
    /* reset TARGET_LAND state only if setpoint moved */
	if (!_navigator->get_can_loiter_at_sp()) {
        _target_land_state = TARGET_LAND_STATE_NONE;
	}
}

void
TargetLand::on_activation()
{
    /* decide where to enter the TARGET_LAND procedure when we switch into it */
    if (_target_land_state == TARGET_LAND_STATE_NONE) {
        /* for safety reasons don't go into TARGET_LAND if landed */
		if (_navigator->get_vstatus()->condition_landed) {
            _target_land_state = TARGET_LAND_STATE_LANDED;
            warnx("[TargetLand] on_activation: TARGET_LAND_STATE_LANDED");
            mavlink_log_critical(_navigator->get_mavlink_fd(), "no TARGET_LAND when landed");

		/* if lower than return altitude, climb up first */
        } else if (_navigator->get_global_position()->alt < _navigator->get_target_land_position()->alt
			   + _param_return_alt.get()) {
            _target_land_state = TARGET_LAND_STATE_CLIMB;
            warnx("[TargetLand] on_activation: TARGET_LAND_STATE_CLIMB");
            _target_land_start_lock = false;

		/* otherwise go straight to return */
		} else {
			/* set altitude setpoint to current altitude */
            _target_land_state = TARGET_LAND_STATE_RETURN;
            warnx("[TargetLand] on_activation: TARGET_LAND_STATE_RETURN");
			_mission_item.altitude_is_relative = false;
			_mission_item.altitude = _navigator->get_global_position()->alt;
            _target_land_start_lock = false;
		}

	}

    set_target_land_item();
}

void
TargetLand::on_active()
{
    if (_target_land_state != TARGET_LAND_STATE_LANDED && is_mission_item_reached()) {
        advance_target_land();
        set_target_land_item();
	}
}

void
TargetLand::set_target_land_item()
{
	struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

	/* make sure we have the latest params */
	updateParams();

    if (!_target_land_start_lock) {
		set_previous_pos_setpoint();
	}

	_navigator->set_can_loiter_at_sp(false);

    switch (_target_land_state) {
    case TARGET_LAND_STATE_CLIMB: {
        //we climb at the current position to our target heigth
        float climb_alt = _navigator->get_target_land_position()->alt + _param_return_alt.get();

		_mission_item.lat = _navigator->get_global_position()->lat;
		_mission_item.lon = _navigator->get_global_position()->lon;
		_mission_item.altitude_is_relative = false;
		_mission_item.altitude = climb_alt;
		_mission_item.yaw = NAN;
		_mission_item.loiter_radius = _navigator->get_loiter_radius();
		_mission_item.loiter_direction = 1;
		_mission_item.nav_cmd = NAV_CMD_WAYPOINT;
		_mission_item.acceptance_radius = _navigator->get_acceptance_radius();
		_mission_item.time_inside = 0.0f;
		_mission_item.pitch_min = 0.0f;
		_mission_item.autocontinue = true;
		_mission_item.origin = ORIGIN_ONBOARD;

        mavlink_log_critical(_navigator->get_mavlink_fd(), "TARGET_LAND: climb to %d m (%d m above home)",
			(int)(climb_alt),
            (int)(climb_alt - _navigator->get_target_land_position()->alt));
		break;
	}

    case TARGET_LAND_STATE_RETURN: {
        //we are at target height. lets fly to our target lat/lon position
        _mission_item.lat = _navigator->get_target_land_position()->lat;
        _mission_item.lon = _navigator->get_target_land_position()->lon;
		 // don't change altitude

		 if (pos_sp_triplet->previous.valid) {
		 	/* if previous setpoint is valid then use it to calculate heading to home */
		 	_mission_item.yaw = get_bearing_to_next_waypoint(
		 	        pos_sp_triplet->previous.lat, pos_sp_triplet->previous.lon,
		 	        _mission_item.lat, _mission_item.lon);

		 } else {
		 	/* else use current position */
		 	_mission_item.yaw = get_bearing_to_next_waypoint(
		 	        _navigator->get_global_position()->lat, _navigator->get_global_position()->lon,
		 	        _mission_item.lat, _mission_item.lon);
		 }
		_mission_item.loiter_radius = _navigator->get_loiter_radius();
		_mission_item.loiter_direction = 1;
		_mission_item.nav_cmd = NAV_CMD_WAYPOINT;
		_mission_item.acceptance_radius = _navigator->get_acceptance_radius();
		_mission_item.time_inside = 0.0f;
		_mission_item.pitch_min = 0.0f;
		_mission_item.autocontinue = true;
		_mission_item.origin = ORIGIN_ONBOARD;

        mavlink_log_critical(_navigator->get_mavlink_fd(), "TARGET_LAND: return at %d m (%d m above home)",
			(int)(_mission_item.altitude),
            (int)(_mission_item.altitude - _navigator->get_target_land_position()->alt));

        _target_land_start_lock = true;
		break;
	}

    case TARGET_LAND_STATE_DESCEND: {
        //we previously reached our target lat/lon position. Now we descent, but still correct x/y position.
        _mission_item.lat = _navigator->get_target_land_position()->lat;
        _mission_item.lon = _navigator->get_target_land_position()->lon;
		_mission_item.altitude_is_relative = false;
        _mission_item.altitude = _navigator->get_target_land_position()->alt + _param_descend_alt.get();
        _mission_item.yaw = _navigator->get_target_land_position()->yaw;
		_mission_item.loiter_radius = _navigator->get_loiter_radius();
		_mission_item.loiter_direction = 1;
		_mission_item.nav_cmd = NAV_CMD_LOITER_TIME_LIMIT;
		_mission_item.acceptance_radius = _navigator->get_acceptance_radius();
		_mission_item.time_inside = 0.0f;
		_mission_item.pitch_min = 0.0f;
		_mission_item.autocontinue = false;
		_mission_item.origin = ORIGIN_ONBOARD;

        mavlink_log_critical(_navigator->get_mavlink_fd(), "TARGET_LAND: descend to %d m (%d m above home)",
			(int)(_mission_item.altitude),
            (int)(_mission_item.altitude - _navigator->get_target_land_position()->alt));
		break;
	}

    case TARGET_LAND_STATE_LOITER: {
		bool autoland = _param_land_delay.get() > -DELAY_SIGMA;

        _mission_item.lat = _navigator->get_target_land_position()->lat;
        _mission_item.lon = _navigator->get_target_land_position()->lon;
		_mission_item.altitude_is_relative = false;
        _mission_item.altitude = _navigator->get_target_land_position()->alt + _param_descend_alt.get();
        _mission_item.yaw = _navigator->get_target_land_position()->yaw;
		_mission_item.loiter_radius = _navigator->get_loiter_radius();
		_mission_item.loiter_direction = 1;
		_mission_item.nav_cmd = autoland ? NAV_CMD_LOITER_TIME_LIMIT : NAV_CMD_LOITER_UNLIMITED;
		_mission_item.acceptance_radius = _navigator->get_acceptance_radius();
		_mission_item.time_inside = _param_land_delay.get() < 0.0f ? 0.0f : _param_land_delay.get();
		_mission_item.pitch_min = 0.0f;
		_mission_item.autocontinue = autoland;
		_mission_item.origin = ORIGIN_ONBOARD;

		_navigator->set_can_loiter_at_sp(true);

		if (autoland) {
            mavlink_log_critical(_navigator->get_mavlink_fd(), "TARGET_LAND: loiter %.1fs", (double)_mission_item.time_inside);

		} else {
            mavlink_log_critical(_navigator->get_mavlink_fd(), "TARGET_LAND: completed, loiter");
		}
		break;
	}

    case TARGET_LAND_STATE_LAND: {
        _mission_item.lat = _navigator->get_target_land_position()->lat;
        _mission_item.lon = _navigator->get_target_land_position()->lon;
		_mission_item.altitude_is_relative = false;
        _mission_item.altitude = _navigator->get_target_land_position()->alt;
        _mission_item.yaw = _navigator->get_target_land_position()->yaw;
		_mission_item.loiter_radius = _navigator->get_loiter_radius();
		_mission_item.loiter_direction = 1;
		_mission_item.nav_cmd = NAV_CMD_LAND;
		_mission_item.acceptance_radius = _navigator->get_acceptance_radius();
		_mission_item.time_inside = 0.0f;
		_mission_item.pitch_min = 0.0f;
		_mission_item.autocontinue = true;
		_mission_item.origin = ORIGIN_ONBOARD;

        mavlink_log_critical(_navigator->get_mavlink_fd(), "TARGET_LAND: land at target");
		break;
	}

    case TARGET_LAND_STATE_LANDED: {
        _mission_item.lat = _navigator->get_target_land_position()->lat;
        _mission_item.lon = _navigator->get_target_land_position()->lon;
		_mission_item.altitude_is_relative = false;
        _mission_item.altitude = _navigator->get_target_land_position()->alt;
		// Do not change / control yaw in landed
		_mission_item.yaw = NAN;
		_mission_item.loiter_radius = _navigator->get_loiter_radius();
		_mission_item.loiter_direction = 1;
		_mission_item.nav_cmd = NAV_CMD_IDLE;
		_mission_item.acceptance_radius = _navigator->get_acceptance_radius();
		_mission_item.time_inside = 0.0f;
		_mission_item.pitch_min = 0.0f;
		_mission_item.autocontinue = true;
		_mission_item.origin = ORIGIN_ONBOARD;

        mavlink_log_critical(_navigator->get_mavlink_fd(), "TARGET_LAND: completed, landed");
		break;
	}

	default:
		break;
	}

	reset_mission_item_reached();

	/* convert mission item to current position setpoint and make it valid */
	mission_item_to_position_setpoint(&_mission_item, &pos_sp_triplet->current);
	pos_sp_triplet->next.valid = false;

    //ghm1: this makes the navigator publish the new position setpoint triplet
	_navigator->set_position_setpoint_triplet_updated();
}

void
TargetLand::advance_target_land()
{
    //Wir benötigen noch einen mechanismus, der sinken verlangsamt je näher wir dem target sind.
    //Darin evtl. auch sinken stoppen, wenn Abweichung vom Target zu gross.
    switch (_target_land_state) {
    case TARGET_LAND_STATE_CLIMB:
        _target_land_state = TARGET_LAND_STATE_RETURN;
        warnx("[TargetLand] advance_target_land: TARGET_LAND_STATE_RETURN");
		break;

    case TARGET_LAND_STATE_RETURN:
        _target_land_state = TARGET_LAND_STATE_DESCEND;
        warnx("[TargetLand] advance_target_land: TARGET_LAND_STATE_DESCEND");
		break;

    case TARGET_LAND_STATE_DESCEND:
		/* only go to land if autoland is enabled */
		if (_param_land_delay.get() < -DELAY_SIGMA || _param_land_delay.get() > DELAY_SIGMA) {
            _target_land_state = TARGET_LAND_STATE_LOITER;
            warnx("[TargetLand] advance_target_land: TARGET_LAND_STATE_LOITER");
        } else {
            _target_land_state = TARGET_LAND_STATE_LAND;
            warnx("[TargetLand] advance_target_land: TARGET_LAND_STATE_LAND");
		}
		break;

    case TARGET_LAND_STATE_LOITER:
        _target_land_state = TARGET_LAND_STATE_LAND;
        warnx("[TargetLand] advance_target_land: TARGET_LAND_STATE_LAND");
		break;

    case TARGET_LAND_STATE_LAND:
        _target_land_state = TARGET_LAND_STATE_LANDED;
        warnx("[TargetLand] advance_target_land: TARGET_LAND_STATE_LANDED");
		break;

	default:
		break;
	}
}
