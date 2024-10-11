/*
 * Copyright (C) 2024 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 */

#include "xparameters.h"
#include "xil_types.h"
#include "xil_io.h"
#include "sleep.h"
#include "motors.h"

#define ESC_MIN_THRUST 3000.0f
#define THRUST_SCALE 1.0f // uint16 output and input.... // debugging: 1024.0f

static uint32_t motor_ratios[] = {0, 0, 0, 0};  // actual PWM signals

void motorsReset() {
    // not implemented in fpga yet
}

void motorsArm() {
    uint32_t curr;

    // note: On arming ESC there is a low beep followed by high beep

    xil_printf("arming motors ...");
    for (int i=0; i<4; i++) {
        Xil_Out32(MOTORS_REG_OFFSET + MOTORS_REG_THROTTLE_0 + i*4, 0);
    }
    Xil_Out32(MOTORS_REG_OFFSET + MOTORS_REG_CONFIG, 0x0F);
    usleep(3*1000*1000);
    xil_printf("\rarming motors ... done\r\n");
}

void motorsDisarm() {
    xil_printf("disarming motors ...");

    for (int i=0; i<4; i++) {
        Xil_Out32(MOTORS_REG_OFFSET + MOTORS_REG_THROTTLE_0 + i*4, 0);
    }
    Xil_Out32(MOTORS_REG_OFFSET + MOTORS_REG_CONFIG, 0x00);
    usleep(1000);
    xil_printf("\rdisarming motors ... done\r\n");
}

void motorsStop(uint32_t motor_number) {
    xil_printf("stopping motor %d ...", motor_number);

    Xil_Out32(MOTORS_REG_OFFSET + MOTORS_REG_THROTTLE_0 + motor_number*4, 48);
    usleep(1000);
    xil_printf("\rstopping motor %d ... done\r\n", motor_number);
}

void motorsNormalDirection(uint32_t motor_number) {
    Xil_Out32(MOTORS_REG_OFFSET + MOTORS_REG_TLM, 1 << motor_number);
    Xil_Out32(MOTORS_REG_OFFSET + MOTORS_REG_THROTTLE_0 + motor_number*4, MOTORS_CMD_NORMAL_DIR);
    /* delay ensures command is sent at least the required 6 times */ 
    usleep(1000);
    Xil_Out32(MOTORS_REG_OFFSET + MOTORS_REG_TLM, 0);
}

void motorsReverseDirection(uint32_t motor_number) {
    Xil_Out32(MOTORS_REG_OFFSET + MOTORS_REG_TLM, 1 << motor_number);
    Xil_Out32(MOTORS_REG_OFFSET + MOTORS_REG_THROTTLE_0 + motor_number*4, MOTORS_CMD_REVERSE_DIR);
    /* delay ensures command is sent at least the required 6 times */ 
    usleep(1000);
    Xil_Out32(MOTORS_REG_OFFSET + MOTORS_REG_TLM, 0);
}

void motorsBeep1(uint32_t motor_number) {
    Xil_Out32(MOTORS_REG_OFFSET + MOTORS_REG_TLM, 1 << motor_number);
    Xil_Out32(MOTORS_REG_OFFSET + MOTORS_REG_THROTTLE_0 + motor_number*4, MOTORS_CMD_BEEP1);
    usleep(1000);
    Xil_Out32(MOTORS_REG_OFFSET + MOTORS_REG_TLM, 0);
}

void motorsBeep2(uint32_t motor_number) {
    Xil_Out32(MOTORS_REG_OFFSET + MOTORS_REG_TLM, 1 << motor_number);
    Xil_Out32(MOTORS_REG_OFFSET + MOTORS_REG_THROTTLE_0 + motor_number*4, MOTORS_CMD_BEEP2);
    usleep(1000);
    Xil_Out32(MOTORS_REG_OFFSET + MOTORS_REG_TLM, 0);
}

void motorsBeep3(uint32_t motor_number) {
    Xil_Out32(MOTORS_REG_OFFSET + MOTORS_REG_TLM, 1 << motor_number);
    Xil_Out32(MOTORS_REG_OFFSET + MOTORS_REG_THROTTLE_0 + motor_number*4, MOTORS_CMD_BEEP3);
    usleep(1000);
    Xil_Out32(MOTORS_REG_OFFSET + MOTORS_REG_TLM, 0);
}

void motorsBeep4(uint32_t motor_number) {
    Xil_Out32(MOTORS_REG_OFFSET + MOTORS_REG_TLM, 1 << motor_number);
    Xil_Out32(MOTORS_REG_OFFSET + MOTORS_REG_THROTTLE_0 + motor_number*4, MOTORS_CMD_BEEP4);
    usleep(1000);
    Xil_Out32(MOTORS_REG_OFFSET + MOTORS_REG_TLM, 0);
}

void motorsBeep5(uint32_t motor_number) {
    Xil_Out32(MOTORS_REG_OFFSET + MOTORS_REG_TLM, 1 << motor_number);
    Xil_Out32(MOTORS_REG_OFFSET + MOTORS_REG_THROTTLE_0 + motor_number*4, MOTORS_CMD_BEEP5);
    usleep(1000);
    Xil_Out32(MOTORS_REG_OFFSET + MOTORS_REG_TLM, 0);
}

void motorsSet(uint32_t motor_number, uint16_t value) {
    float thrust;
    uint16_t thrust_esc;
    motor_ratios[motor_number] = value;

    /* normalize thrust to suitable range for ESC */
    thrust = ESC_MIN_THRUST + (float)value / THRUST_SCALE;

    //note: the ESC output is 11 bit, the FPGA hardware takes the LSB,so make sure to have the right bits...
    thrust_esc = ((uint16_t)thrust >> 5) & 0x07FF;

    Xil_Out32(MOTORS_REG_OFFSET + MOTORS_REG_THROTTLE_0 + motor_number*4, thrust_esc);
}

int motorsGetRatio(uint32_t motor_number)
{
  return motor_ratios[motor_number];
}