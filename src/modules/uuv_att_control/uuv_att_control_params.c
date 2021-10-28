/****************************************************************************
 *
 *   Copyright (c) 2013-2020 PX4 Development Team. All rights reserved.
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
 * @file uuv_att_control_params.c
 *
 * Parameters defined by the attitude control task for unmanned underwater vehicles (UUVs)
 *
 * This is a modification of the fixed wing/ground rover params and it is designed for ground rovers.
 * It has been developed starting from the fw  module, simplified and improved with dedicated items.
 *
 * All the ackowledgments and credits for the fw wing/rover app are reported in those files.
 *
 * @author Daniel Duecker <daniel.duecker@tuhh.de>
 */

/*
 * Controller parameters, accessible via MAVLink
 *
 */

// Roll gains
/**
 * Roll proportional gain
 *
 * @group UUV Attitude Control
 */
PARAM_DEFINE_FLOAT(UUV_ROLL_P, 1.0f);

/**
 * Roll differential gain
 *
 * @group UUV Attitude Control
 */
PARAM_DEFINE_FLOAT(UUV_ROLL_D, 0.2f);

/**
 * Roll integral gain
 *
 * @group UUV Attitude Control
 */
PARAM_DEFINE_FLOAT(UUV_ROLL_I, 0.2f);

// Pitch gains
/**
 * Pitch proportional gain
 *
 * @group UUV Attitude Control
 */
PARAM_DEFINE_FLOAT(UUV_PITCH_P, 1.0f);

/**
 * Pitch differential gain
 *
 * @group UUV Attitude Control
 */
PARAM_DEFINE_FLOAT(UUV_PITCH_D, 0.2f);

/**
 * Pitch integral gain
 *
 * @group UUV Attitude Control
 */
PARAM_DEFINE_FLOAT(UUV_PITCH_I, 0.2f);

// Yaw gains
/**
 * Yawh proportional gain
 *
 * @group UUV Attitude Control
 */
PARAM_DEFINE_FLOAT(UUV_YAW_P, 1.0f);

/**
 * Yaw differential gain
 *
 * @group UUV Attitude Control
 */
PARAM_DEFINE_FLOAT(UUV_YAW_D, 0.2f);

/**
 * Yaw integral gain
 *
 * @group UUV Attitude Control
 */
PARAM_DEFINE_FLOAT(UUV_YAW_I, 0.0f);

// Input Modes
/**
 * Select Input Mode
 *
 * @value 0 use Attitude Setpoints
 * @value 1 Direct Feedthrough
 * @group UUV Attitude Control
 */
PARAM_DEFINE_INT32(UUV_INPUT_MODE, 0);

/**
 * Skip the controller
 *
 * @value 0 use the module's controller
 * @value 1 skip the controller and feedthrough the setpoints
 */
PARAM_DEFINE_INT32(UUV_SKIP_CTRL, 0);

/**
 * Direct roll input
 *
 * @group UUV Attitude Control
 */
PARAM_DEFINE_FLOAT(UUV_DIRCT_ROLL, 0.0f);

/**
 * Direct pitch input
 *
 * @group UUV Attitude Control
 */
PARAM_DEFINE_FLOAT(UUV_DIRCT_PITCH, 0.0f);

/**
 * Direct yaw input
 *
 * @group UUV Attitude Control
 */
PARAM_DEFINE_FLOAT(UUV_DIRCT_YAW, 0.0f);

/**
 * Direct thrust input
 *
 * @group UUV Attitude Control
 */
PARAM_DEFINE_FLOAT(UUV_DIRCT_THRUST, 0.0f);

/**
 * manual P-Control height
 *
 * @group UUV Manual Control
 */
PARAM_DEFINE_FLOAT(UUV_M_HGT_P_CTRL, 1.0f);

/**
 * manual P-Control height
 *
 * @group UUV Manual Control
 */
PARAM_DEFINE_FLOAT(UUV_M_HGT_I_CTRL, 0.2f);

/**
 * manual P-Control height
 *
 * @group UUV Manual Control
 */
PARAM_DEFINE_FLOAT(UUV_M_HGT_D_CTRL, 0.2f);

/**
 * manual P-Control height
 *
 * @group UUV Manual Control
 */
PARAM_DEFINE_FLOAT(UUV_M_YAW_SPEED, 0.1f);

/**
 * manual P-Control height
 *
 * @group UUV Manual Control
 */
PARAM_DEFINE_FLOAT(UUV_M_HGT_SPEED, 0.1f);

/**
 * control yaw vs direct feedthrough
 *
 * @group UUV Manual Control
 * @value 0 use direct feedthrough for yaw
 * @value 1 control yaw by controller(dependent on internal yaw measurement)
 */
PARAM_DEFINE_INT32(UUV_M_YAW_CTRL, 0);
/**
 *
 * @group windup effect correction roll
 */
PARAM_DEFINE_FLOAT(UUV_WIND_ROLL, 0.2);
/**
 *
 * @group windup effect correction pitch
 */
PARAM_DEFINE_FLOAT(UUV_WIND_PITCH, 0.2);
/**
 *
 * @group windup effect correction yaw
 */
PARAM_DEFINE_FLOAT(UUV_WIND_YAW, 0.2);
/**
 *
 * @group windup effect correction hgt
 */
PARAM_DEFINE_FLOAT(UUV_WIND_HGT, 0.2);
/**
 * how fast is manual control
 *
 * @group x y manual control speed
 */
PARAM_DEFINE_FLOAT(UUV_M_XY_SPEED, 1.0);

