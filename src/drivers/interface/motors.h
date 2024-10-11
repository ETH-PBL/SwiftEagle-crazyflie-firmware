/*
 * Copyright (C) 2024 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 */

#ifndef MOTORS_H
#define MOTORS_H

#include "xil_types.h"

/* base address */
#define MOTORS_REG_OFFSET XPAR_MOTORS_OUT_AXI_TO_16_REGISTERS_0_S00_AXI_BASEADDR //XPAR_MOTORS_OUT_AXI_TO_16_REGISTERS_0_DEVICE_ID

/* register addresses */
#define MOTORS_REG_CONFIG       0x00
#define MOTORS_REG_THROTTLE_0   0x10
#define MOTORS_REG_THROTTLE_1   0x14
#define MOTORS_REG_THROTTLE_2   0x18
#define MOTORS_REG_THROTTLE_3   0x1c
#define MOTORS_REG_TLM          0x20

/* commands */
#define MOTORS_CMD_NORMAL_DIR   20
#define MOTORS_CMD_REVERSE_DIR  21
#define MOTORS_CMD_BEEP1        1
#define MOTORS_CMD_BEEP2        2
#define MOTORS_CMD_BEEP3        3
#define MOTORS_CMD_BEEP4        4
#define MOTORS_CMD_BEEP5        5

/* contants for drone frame hw rev4*/
#define MOTOR_0    1
#define MOTOR_1    0
#define MOTOR_2    2
#define MOTOR_3    3

#define NBR_OF_MOTORS 4

void motorsReset();
void motorsArm();
void motorsDisarm();
void motorsStop(uint32_t motor_number);
void motorsNormalDirection(uint32_t motor_number);
void motorsReverseDirection(uint32_t motor_number);
void motorsBeep1(uint32_t motor_number);
void motorsBeep2(uint32_t motor_number);
void motorsBeep3(uint32_t motor_number);
void motorsBeep4(uint32_t motor_number);
void motorsBeep5(uint32_t motor_number);
void motorsSet(uint32_t motor_number, uint16_t value);
int motorsGetRatio(uint32_t motor_number);


#endif // MOTORS_H