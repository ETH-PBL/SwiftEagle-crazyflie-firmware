/*
 * Copyright (C) 2024 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 */

#include "xil_io.h"
#include "sbus.h"
#include "setpoint.h"
#include "ipi.h"

static bool setpointWarningDisplayed = false;

void initSetpoint(setpoint_t *setpoint) {
    setpoint->mode.x = modeDisable;
    setpoint->mode.y = modeDisable;
    setpoint->mode.z = modeDisable;
    setpoint->mode.roll = modeAbs;
    setpoint->mode.pitch = modeAbs;
    setpoint->mode.yaw = modeAbs;
    setpoint->mode.quat = modeDisable;
}

void getSetpoint(setpoint_t *setpoint) {
    uint32_t ch;

    /* If corresponding switch on remote control is in neutral position, use remote control to drive setpoint. 
       Else, APU drives setpoint. */
    ch = getChannelData(CHANNEL_APU_TAKES_OVER);
    if ((ch > getChannelThresholdLow()) && (ch < getChannelThresholdHigh())) {
        /* first check if setpoint valid */
        if (getStatusCtrl() & STATUS_CTRL_REG_DATA_VALID_BIT) {
            /* get it from remote control over s-bus */
            setpoint->thrust = (getChannelData(CHANNEL_THRUST) - CHANNEL_MIN_VALUE) / CHANNEL_FULL_RANGE * THRUST_MAX;
            setpoint->attitude.roll = (getChannelData(CHANNEL_ROLL) - CHANNEL_MID_VALUE) / CHANNEL_HALF_RANGE * ROLL_ANGLE_MAX;
            setpoint->attitude.pitch = (getChannelData(CHANNEL_PITCH) - CHANNEL_MID_VALUE) / CHANNEL_HALF_RANGE * PITCH_ANGLE_MAX;
            setpoint->attitude.yaw = (getChannelData(CHANNEL_YAW) - CHANNEL_MID_VALUE) / CHANNEL_HALF_RANGE * YAW_ANGLE_MAX;
        }
        else {
            if (!setpointWarningDisplayed) {
                xil_printf("error: no valid setpoint from remote control available, motors are shut down\r\n");
                setpointWarningDisplayed = true;
            }
        }
    } 
    else {
        /* get it from shared memory (which is populated by APU) */
        ipi_getSetpoint(setpoint);
    }
}

bool remoteControlIsActive(void) {
    uint32_t status;
    
    status = getStatusCtrl();
    return (status & STATUS_CTRL_REG_DATA_VALID_BIT);
}
