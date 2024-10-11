/*
 * Copyright (C) 2024 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 */

#ifndef SETPOINT_H
#define SETPOINT_H

#include "xil_types.h"
#include "stabilizer_types.h"
#include "arm_math.h"


/* defined by remote control */
#define CHANNEL_THRUST  0
#define CHANNEL_YAW     3
#define CHANNEL_ROLL    1
#define CHANNEL_PITCH   2
#define CHANNEL_APU_TAKES_OVER  5
#define CHANNEL_MIN_VALUE   172.0f
#define CHANNEL_MID_VALUE   992.0f
#define CHANNEL_HALF_RANGE  819.0f
#define CHANNEL_FULL_RANGE  1639.0f

/* user defined thrust */
#define THRUST_MAX          UINT16_MAX //note: max. possible is 16 bit unsigned...  

/* user defined maximum euler angles in degrees (positive/negative) */
#define ROLL_ANGLE_MAX      45.0f 
#define PITCH_ANGLE_MAX     45.0f
#define YAW_ANGLE_MAX       45.0f


void initSetpoint(setpoint_t *setpoint);
void getSetpoint(setpoint_t *setpoint);
bool remoteControlIsActive(void);



#endif // SETPOINT_H
