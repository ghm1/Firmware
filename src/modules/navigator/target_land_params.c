/****************************************************************************
 *
 *   Copyright (c) 2014 PX4 Development Team. All rights reserved.
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
 * AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
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
 * @file target_land_params.c
 *
 * Parameters for TARGET_LAND
 *
 * @author Michael Göttlicher <michael.goettlicher@bfh.ch>
 */

/*
 * TARGET_LAND parameters, accessible via MAVLink
 */

/**
 * TARGET_LAND return altitude
 *
 * Altitude to fly back in TARGET_LAND in meters
 *
 * @unit meters
 * @min 0
 * @max 150
 * @group Target Land
 */
PARAM_DEFINE_FLOAT(TL_RETURN_ALT, 20);


/**
 * TARGET_LAND hight position altitude
 *
 * This is a position over target at altitude, where the target can be seen very well.
 * From this goto loiter low position.
 *
 * @unit meters
 * @min 1.0
 * @max 10
 * @group Target Land
 */
PARAM_DEFINE_FLOAT(TL_HIGHPOS_ALT, 5);


/**
 * TARGET_LAND loiter low altitude
 *
 * Stay at this altitude above target position after TARGET_LAND descending.
 * Land (i.e. slowly descend) from this altitude if autolanding allowed.
 *
 * @unit meters
 * @min 0.5
 * @max 5
 * @group Target Land
 */
PARAM_DEFINE_FLOAT(TL_LOITERLOW_ALT, 1);



/**
 * TARGET_LAND position error radius, that is allowed for statetransition to land state.
 *
 *
 * @unit meter
 * @min 0.05
 * @max 2.0
 * @group Target Land
 */
PARAM_DEFINE_FLOAT(TL_RADI_HIGHPOS, 0.5f);


/**
 * TARGET_LAND position error radius, that is allowed for statetransition to land state.
 *
 *
 * @unit meter
 * @min 0.05
 * @max 1.0
 * @group Target Land
 */
PARAM_DEFINE_FLOAT(TL_RADI_LOITRLOW, 0.05f);


/**
 * TARGET_LAND max yaw error, that is allowed for statetransition to land state.
 *
 *
 * @unit meter
 * @min 0.005
 * @max 0.5
 * @group Target Land
 */
PARAM_DEFINE_FLOAT(TL_YAWERRLOITLOW, 0.03f);


/**
 * TARGET_LAND time, the vehicle has to be inside TL_RADI_LOITRLOW and TL_YAWERRLOITLOW, before statetransition to land state.
 *
 * If set to -1, then vehicle will never land, but loiter over target at low altitude
 *
 * @unit seconds
 * @min -1
 * @max 300
 * @group Target Land
 */
PARAM_DEFINE_FLOAT(TL_ACCEPT_TIME, 2.0f);





/**
 * TARGET_LAND timeout for invisibility of target in loiter low state
 *
 * If target is invisible for more than TL_INVIS_TIMEOUT seconds, the vehicle goes back to TL_HIGHPOS_ALT
 *
 * @unit seconds
 * @min 0
 * @max 300
 * @group Target Land
 */
PARAM_DEFINE_FLOAT(TL_INVIS_TIMEOUT, 10.0f);


