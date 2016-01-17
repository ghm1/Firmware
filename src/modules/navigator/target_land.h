/***************************************************************************
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
 * @file target_land.h
 * Helper class for automatic precise land on target
 * related to rtl helper class
 *
 * @author Michael Göttlicher <michael.goettlicher@bfh.ch>
 */

#ifndef NAVIGATOR_TARGET_LAND_H
#define NAVIGATOR_TARGET_LAND_H

#include <controllib/blocks.hpp>
#include <controllib/block/BlockParam.hpp>

#include <navigator/navigation.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/vehicle_global_position.h>

#include "navigator_mode.h"
#include "mission_block.h"

class Navigator;

//Ist das gespeicherte lande target zu alt, dann steigen auf alt, queren und landen.
//Ist das target hingegen aktuell, nicht steigen, sondern nur queren und anschliessend landen.
//Was auch cool wäre, wär wenn man während dem descending die sinkgeschwindigkeit beinflussen könnte.
class TargetLand : public MissionBlock
{
public:
    TargetLand(Navigator *navigator, const char *name);

    ~TargetLand();

	virtual void on_inactive();

	virtual void on_activation();

	virtual void on_active();

private:
	/**
     * Set the TARGET_LAND item
	 */
    void		set_target_land_item();

	/**
     * Move to next TARGET_LAND item
	 */
    void		advance_target_land();

    /**
     * Check if target was visible in the last timeSec seconds
     */
    bool        target_visible( float timeSec );

    enum TARGET_LANDState {
        TARGET_LAND_STATE_NONE = 0,
        TARGET_LAND_STATE_CLIMB,
        TARGET_LAND_STATE_RETURN,
        TARGET_LAND_STATE_ADJUST_YAW,
        TARGET_LAND_STATE_GOTO_POS_HIGH,
        TARGET_LAND_STATE_GOTO_LOITER_LOW,
        TARGET_LAND_STATE_LOITER_LOW,
        TARGET_LAND_STATE_LAND,
        TARGET_LAND_STATE_LANDED,
    } _target_land_state;

    bool _target_land_start_lock;
    bool _target_land_state_changed;

	control::BlockParamFloat _param_return_alt;
    control::BlockParamFloat _param_highpos_alt;
    control::BlockParamFloat _param_loiterlow_alt;
    control::BlockParamFloat _param_accept_radius_at_highpos;
    control::BlockParamFloat _param_accep_radius_loiterlow;
    control::BlockParamFloat _param_yaw_error_loiterlow;
    control::BlockParamFloat _param_acceptance_time;
    control::BlockParamFloat _param_invisible_timeout;
};

#endif
