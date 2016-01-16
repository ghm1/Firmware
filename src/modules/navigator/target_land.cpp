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
    _target_land_state_changed(false),
    _param_return_alt(this, "TL_RETURN_ALT", false),
    _param_descend_alt(this, "TL_DESCEND_ALT", false),
    _param_land_delay(this, "TL_LAND_DELAY", false),
    _param_acc_radius_at_alt(this, "TL_ACCEPTRAD_ALT", false),
    _param_acc_radius_over_target(this, "TL_ACCEPTRAD_TAR", false)
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
//    /* reset TARGET_LAND state only if setpoint moved */
//	if (!_navigator->get_can_loiter_at_sp()) {
//        _target_land_state = TARGET_LAND_STATE_NONE;
//	}

    //ghm1proof: always reset
    _target_land_state = TARGET_LAND_STATE_NONE;
}


bool
TargetLand::target_visible()
{
    hrt_abstime now = hrt_absolute_time();
    hrt_abstime itemTimeStamp = _navigator->get_target_land_position()->timestamp;
    if( ( now - itemTimeStamp ) > (hrt_abstime)1e6f ) {
        //ghm1proof
 //       warnx("ghm1proof: target invisible");
        return false;
    }
    else {
  //      warnx("ghm1proof: target visible");
        return true;
    }
}

void
TargetLand::on_activation()
{

    /* decide where to enter the TARGET_LAND procedure when we switch into it */
    if (_target_land_state == TARGET_LAND_STATE_NONE) {
        /* for safety reasons don't go into TARGET_LAND if landed */
        if (_navigator->get_vstatus()->condition_landed)
        {
            _target_land_state = TARGET_LAND_STATE_LANDED;
            warnx("[TargetLand] on_activation: TARGET_LAND_STATE_LANDED");
            mavlink_log_critical(_navigator->get_mavlink_fd(), "no TARGET_LAND when landed");
        }
        /* if lower than return altitude, climb up first */
        //ghm1 and !target_visible (compares timestamp of target_land_poisiton with current time)
        else if (_navigator->get_global_position()->alt < _navigator->get_target_land_position()->alt
               + _param_return_alt.get()
                 && !target_visible())
        {
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

        //in every case the state has changed
        _target_land_state_changed = true;

	}

    set_target_land_item();
}

void
TargetLand::on_active()
{
    //ghm1: if the mission item is reached AND we have not finished landing then call the statemachine,
    //define the next mission item and set it as next setpoint in position setpoint triplet (in navigator)

    if(_target_land_state != TARGET_LAND_STATE_LANDED)
    {
        if( _target_land_state != TARGET_LAND_STATE_LOITER_LOW )
        {
            //make standard position check
            //advance statemachine if nessessary
            if (is_mission_item_reached())
            {
                //warnx("mission item reached");
                advance_target_land();
                reset_mission_item_reached(); //resets conditions, that must be reached for mission_item_reached condition
            }
        }
        else
        {
            //in this case we want to be more precise:
            //we check if we are at the desired position with desired yaw for some time
            //time starts as soon as we enter position and is resetted, if we loose hold
//todo: parameter for yaw
            if (is_mission_item_reached_precisely(0.02))
            {
                //warnx("mission item reached precisely");
                advance_target_land();
                reset_mission_item_reached(); //resets conditions, that must be reached for mission_item_reached condition
            }
        }

//        //during landing phase we dont correct
//        if( _target_land_state != TARGET_LAND_STATE_LAND )
//        {
            //problem: wir haben zwar den state geändert, aber target_land_pos wurde nicht upgedated
            //deshalb immer updaten, wenn state changed

            //always update position setpoint if target land position has updated
            if( _target_land_state_changed )
            {
                set_target_land_item();
                _target_land_state_changed = false;
            }
            else if( _navigator->target_land_pos_updated())
            {
                set_target_land_item();
            }
            else
            {
                //check if we are in LOITER_LOW state and target was out of FOV for a longer time
                //then go to LOITER_HIGH
                if( !target_visible() && _target_land_state == TARGET_LAND_STATE_LOITER_LOW )
                {
                    _target_land_state = TARGET_LAND_STATE_GOTO_POS_HIGH;
                    set_target_land_item();
                }
            }
//        }
    }
}

void
TargetLand::set_target_land_item()
{
	struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

	/* make sure we have the latest params */
	updateParams();

    //ghm1todo: wofür ist des?
    if (!_target_land_start_lock) {
		set_previous_pos_setpoint();
	}

     //flags if current position SP can be used to loiter
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
        _mission_item.time_inside = 1.0f;
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

         //todo: das funktioniert so nicht, wenn wir immer target_land position updaten:
         // wir müssen uns climb lat lon merken
         //wo merken:

        //ausserdem macht das mit dem yaw anpassen turbulenzen
//hier
//         if (pos_sp_triplet->previous.valid) {
//            /* if previous setpoint is valid then use it to calculate heading to home */
//            _mission_item.yaw = get_bearing_to_next_waypoint(
//                    pos_sp_triplet->previous.lat, pos_sp_triplet->previous.lon,
//                    _mission_item.lat, _mission_item.lon);

//         } else {
		 	/* else use current position */

        //erstmal testen
		 	_mission_item.yaw = get_bearing_to_next_waypoint(
		 	        _navigator->get_global_position()->lat, _navigator->get_global_position()->lon,
		 	        _mission_item.lat, _mission_item.lon);
//         }

		_mission_item.loiter_radius = _navigator->get_loiter_radius();
		_mission_item.loiter_direction = 1;
		_mission_item.nav_cmd = NAV_CMD_WAYPOINT;
        _mission_item.acceptance_radius = _navigator->get_acceptance_radius();
        _mission_item.time_inside = 1.0f;
		_mission_item.pitch_min = 0.0f;
		_mission_item.autocontinue = true;
		_mission_item.origin = ORIGIN_ONBOARD;

        mavlink_log_critical(_navigator->get_mavlink_fd(), "TARGET_LAND: return at %d m (%d m above home)",
			(int)(_mission_item.altitude),
            (int)(_mission_item.altitude - _navigator->get_target_land_position()->alt));

        _target_land_start_lock = true;
		break;
	}



    //ghm1todo: diesen state brauchen wir eigentlich nicht, da ja erst auf jeden setpoint gewartet wird

    case TARGET_LAND_STATE_ADJUST_YAW: {
        //we are at target positino and return_alt height. now
        _mission_item.lat = _navigator->get_target_land_position()->lat;
        _mission_item.lon = _navigator->get_target_land_position()->lon;
         // don't change altitude, but new yaw
        _mission_item.yaw = _navigator->get_target_land_position()->yaw;
        _mission_item.loiter_radius = _navigator->get_loiter_radius();
        _mission_item.loiter_direction = 1;
        _mission_item.nav_cmd = NAV_CMD_LOITER_TIME_LIMIT;
        _mission_item.acceptance_radius = _param_acc_radius_at_alt.get();
        _mission_item.time_inside = 1.0f;
        _mission_item.pitch_min = 0.0f;
        _mission_item.autocontinue = true;
        _mission_item.origin = ORIGIN_ONBOARD;

        mavlink_log_critical(_navigator->get_mavlink_fd(), "TARGET_LAND: adjust yaw to %.2f",
            (double)(_mission_item.yaw));

        _target_land_start_lock = true;
        break;
    }

    case TARGET_LAND_STATE_GOTO_POS_HIGH: {
        //we previously reached our target lat/lon position. Now we descent, but still correct x/y position.
        _mission_item.lat = _navigator->get_target_land_position()->lat;
        _mission_item.lon = _navigator->get_target_land_position()->lon;
		_mission_item.altitude_is_relative = false;
        //go down to _param_descend_alt above our target

        //todo: param
        _mission_item.altitude = _navigator->get_target_land_position()->alt + 5.0f;
                //_navigator->get_target_land_position()->alt + _param_descend_alt.get();


        _mission_item.yaw = _navigator->get_target_land_position()->yaw;
        _mission_item.loiter_radius = _navigator->get_loiter_radius(); //FW
        _mission_item.loiter_direction = 1; //FW
        //todo ? time limit?
        //_mission_item.nav_cmd = NAV_CMD_LOITER_TIME_LIMIT;
        _mission_item.nav_cmd = NAV_CMD_LOITER_TIME_LIMIT;
        _mission_item.acceptance_radius = _param_acc_radius_at_alt.get();
        _mission_item.time_inside = 3.0f;
		_mission_item.pitch_min = 0.0f;
		_mission_item.autocontinue = false;
		_mission_item.origin = ORIGIN_ONBOARD;

        mavlink_log_critical(_navigator->get_mavlink_fd(), "TARGET_LAND: descend to %.2f m (%.2f m above home)",
            (double)(_mission_item.altitude),
            (double)(_mission_item.altitude - _navigator->get_target_land_position()->alt));
		break;
	}

    case TARGET_LAND_STATE_GOTO_LOITER_LOW: {
        //we previously reached our target lat/lon position. Now we descent, but still correct x/y position.
        _mission_item.lat = _navigator->get_target_land_position()->lat;
        _mission_item.lon = _navigator->get_target_land_position()->lon;
        _mission_item.altitude_is_relative = false;
        //go down to _param_descend_alt above our target

        //todo: param
        _mission_item.altitude = _navigator->get_target_land_position()->alt + _param_descend_alt.get();
                //_navigator->get_target_land_position()->alt + _param_descend_alt.get();

//todo: Geschwindigkeit beim Landen erforschen -> warum dort langsam, wird da was im regler gemacht?

        _mission_item.yaw = _navigator->get_target_land_position()->yaw;
        _mission_item.loiter_radius = _navigator->get_loiter_radius(); //FW
        _mission_item.loiter_direction = 1; //FW
        //todo ? time limit?
        //_mission_item.nav_cmd = NAV_CMD_LOITER_TIME_LIMIT;
        _mission_item.nav_cmd = NAV_CMD_LAND;
        _mission_item.acceptance_radius = _param_acc_radius_at_alt.get();
        _mission_item.time_inside = 3.0f;
        _mission_item.pitch_min = 0.0f;
        _mission_item.autocontinue = false;
        _mission_item.origin = ORIGIN_ONBOARD;

        mavlink_log_critical(_navigator->get_mavlink_fd(), "TARGET_LAND: descend to %.2f m (%.2f m above home)",
            (double)(_mission_item.altitude),
            (double)(_mission_item.altitude - _navigator->get_target_land_position()->alt));

        break;
    }

    case TARGET_LAND_STATE_LOITER_LOW: {
        //wieso brauchen wir loiter low: wir wollen uns knapp über dem Target positionieren,
        //dort, wo das grosse target aufgrund des ungünstigen bildsensors nicht mehr sichtbar ist.
        //gleichzeitig brauchen wir das grosse target, um in grosser höhe ein target zu erkennen

        bool autoland = _param_land_delay.get() > -DELAY_SIGMA;

//todo: yaw und position müssen hier genauer abgeprüft werden: nein nicht hier, sondern bei der prüfung ob mission item reached

        _mission_item.lat = _navigator->get_target_land_position()->lat;
        _mission_item.lon = _navigator->get_target_land_position()->lon;
        _mission_item.altitude_is_relative = false;
        _mission_item.altitude = _navigator->get_target_land_position()->alt + _param_descend_alt.get();
        _mission_item.yaw = _navigator->get_target_land_position()->yaw;
        _mission_item.loiter_radius = _navigator->get_loiter_radius(); //FW
        _mission_item.loiter_direction = 1;
        _mission_item.nav_cmd = autoland ? NAV_CMD_LOITER_TIME_LIMIT : NAV_CMD_LOITER_UNLIMITED;
        //_mission_item.nav_cmd = NAV_CMD_LOITER_TIME_LIMIT;
        //HIER STELLEN WIR GENAUER EIN
        _mission_item.acceptance_radius = _param_acc_radius_over_target.get();

        //todo: param
        _mission_item.time_inside = 2.0f; //_param_land_delay.get() < 0.0f ? 0.0f : _param_land_delay.get();

        _mission_item.pitch_min = 0.0f;
        _mission_item.autocontinue = autoland;
        _mission_item.origin = ORIGIN_ONBOARD;

        //flags if current position SP can be used to loiter
        _navigator->set_can_loiter_at_sp(true);

        if (autoland) {
            mavlink_log_critical(_navigator->get_mavlink_fd(), "TARGET_LAND: loiter %.1fs", (double)_mission_item.time_inside);

        } else {
            mavlink_log_critical(_navigator->get_mavlink_fd(), "TARGET_LAND: completed, loiter");
        }
        break;
    }

    case TARGET_LAND_STATE_LAND: {
//todo: we dont correct position anymore is this good, perhaps we need a flag to test
        _mission_item.lat = _navigator->get_target_land_position()->lat;
        _mission_item.lon = _navigator->get_target_land_position()->lon;
        _mission_item.altitude_is_relative = false;
        //now adjust target altitude
        _mission_item.altitude = _navigator->get_target_land_position()->alt;
        _mission_item.yaw = _navigator->get_target_land_position()->yaw;
        _mission_item.loiter_radius = _navigator->get_loiter_radius();
        _mission_item.loiter_direction = 1;
        _mission_item.nav_cmd = NAV_CMD_LAND;
        _mission_item.acceptance_radius = _param_acc_radius_over_target.get();
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
        _mission_item.acceptance_radius = _param_acc_radius_at_alt.get();
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


    //achtung
    //todo: wir haben hier ein etwas anderes verhalten, da wir immer neue setpoints setzen
    //wir werden das mission item nie erreichen, wenn wir es immer zurücksetzen
    //-> wir brauchen eigene bedingungen für das loiter low
//    if( _target_land_state_changed )
//    {
//        warnx("reset mission item reached");
//        //call only if state changed in advance_target_land
//        reset_mission_item_reached(); //resets conditions, that must be reached for mission_item_reached condition
//        _target_land_state_changed = false;
//    }

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

    //init true by default and reset it if we did not change
    //_target_land_state_changed = true;

    switch (_target_land_state) {
    case TARGET_LAND_STATE_CLIMB:
        _target_land_state = TARGET_LAND_STATE_RETURN; //orig (return brauchen wir immer nur die frage, ob wir yaw ausrichten.
        _target_land_state_changed = true;
        warnx("[TargetLand] advance_target_land: TARGET_LAND_STATE_RETURN");        
		break;

    case TARGET_LAND_STATE_RETURN:
        _target_land_state = TARGET_LAND_STATE_ADJUST_YAW;
        _target_land_state_changed = true;
        warnx("[TargetLand] advance_target_land: TARGET_LAND_STATE_ADJUST_YAW");
		break;

    case TARGET_LAND_STATE_ADJUST_YAW:
        _target_land_state = TARGET_LAND_STATE_GOTO_POS_HIGH;
        _target_land_state_changed = true;
        warnx("[TargetLand] advance_target_land: TARGET_LAND_STATE_GOTO_POS_HIGH");
        break;

//    case TARGET_LAND_STATE_DESCEND:
//		/* only go to land if autoland is enabled */
//        //ghm1: if we have a delay TARGET_LAND_LAND_DELAY parameterized,
//        //then we go to loiter state where time_inside will be parameterized to TARGET_LAND_LAND_DELAY
//        //and descend altitude will be parameterized to targetland alt + TARGET_LAND_DESCEND_ALT.

//        //As long as the Target is in field of view of the camera the relative heigth above the target
//        //should be correct if there are no fast height estimation drifts. We could additionally advertise
//        //the height above the target on the distance sensor topic (what happens, if it is not always valid).

//        //If we loose the target from the field of view we have to takeoff again until we can see it.


////		if (_param_land_delay.get() < -DELAY_SIGMA || _param_land_delay.get() > DELAY_SIGMA) {
////            _target_land_state = TARGET_LAND_STATE_LOITER_HIGH;
////            warnx("[TargetLand] advance_target_land: TARGET_LAND_STATE_LOITER");
////        } else {
////            _target_land_state = TARGET_LAND_STATE_LAND;
////            warnx("[TargetLand] advance_target_land: TARGET_LAND_STATE_LAND");
////		}
//        _target_land_state = TARGET_LAND_STATE_LOITER_HIGH;
//        warnx("[TargetLand] advance_target_land: TARGET_LAND_STATE_LOITER_HIGH");
//		break;

    case TARGET_LAND_STATE_GOTO_POS_HIGH:
        //we go down an try to hold view of the small target
        _target_land_state = TARGET_LAND_STATE_GOTO_LOITER_LOW;
        _target_land_state_changed = true;
        warnx("[TargetLand] advance_target_land: TARGET_LAND_STATE_GOTO_LOITER_LOW");
        break;

    case TARGET_LAND_STATE_GOTO_LOITER_LOW:
        _target_land_state = TARGET_LAND_STATE_LOITER_LOW;
        _target_land_state_changed = true;
        warnx("[TargetLand] advance_target_land: TARGET_LAND_STATE_LOITER_LOW");
        break;

    case TARGET_LAND_STATE_LOITER_LOW:
        _target_land_state = TARGET_LAND_STATE_LAND;
        _target_land_state_changed = true;
        warnx("[TargetLand] advance_target_land: TARGET_LAND_STATE_LAND");
		break;

    case TARGET_LAND_STATE_LAND:
        _target_land_state = TARGET_LAND_STATE_LANDED;
        _target_land_state_changed = true;
        warnx("[TargetLand] advance_target_land: TARGET_LAND_STATE_LANDED");
		break;

	default:
        //_target_land_state_changed = false;
		break;
	}
}
