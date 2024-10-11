/*
 * Copyright (C) 2024 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 */


/*
 *  S-Bus register address mapping:
 *
 *   what                       | offset (words)  | default value
 *  ============================================================= 
 *   status / control           | 0               | 0
 *   channel threshold low      | 1               | 300
 *   channel threshold high     | 2               | 1700
 *   channel inactive value     | 3               | 8
 *   max. number of lost frames | 4               | 100
 *   channel 0                  | 5               | 0
 *   channel 1                  | 6               | 0
 *   channel 2                  | 7               | 0
 *   channel 3                  | 8               | 0
 *   channel 4                  | 9               | 0
 *   channel 5                  | 10              | 0
 *   channel 6                  | 11              | 0
 *   channel 7                  | 12              | 0
 *   channel 8                  | 13              | 0
 *   channel 9                  | 14              | 0
 *   channel 10                 | 15              | 0
 *   channel 11                 | 16              | 0
 *   channel 12                 | 17              | 0
 *   channel 13                 | 18              | 0
 *   channel 14                 | 19              | 0
 *   channel 15                 | 20              | 0
 *   flags                      | 21              | 0
 */


#ifndef SBUS_H
#define SBUS_H

#include "xil_types.h"
#include "xparameters.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "xscugic.h"

/* base address */
#define SBUS_REG_OFFSET XPAR_SBUS_TOP_0_BASEADDR

/* registers */
#define STATUS_CTRL_REG_OFFSET 0
#define CH_THRESHOLD_LOW_REG_OFFSET 4
#define CH_THRESHOLD_HIGH_REG_OFFSET 8
#define CH_INACTIVE_VALUE_REG_OFFSET 12
#define MAX_NBR_LOST_FRAMES_REG_OFFSET 16
#define CHANNEL_0_REG_OFFSET 20
#define FLAGS_REG_OFFSET 84

/* bits */
#define STATUS_CTRL_REG_DATA_VALID_BIT 1
#define STATUS_REG_ACK_INT_BIT 2

/* interrupt initialization */
void sbus_irq_init(XScuGic *pInterruptController);
extern xSemaphoreHandle interruptFromSbus;

/* interrupt acknowledge */
void sbus_irq_ack(void);

/* get */
uint32_t getChannelThresholdLow(void);
uint32_t getChannelThresholdHigh(void);
uint32_t getChannelInactiveValue(void);
uint32_t getMaxNumberLostFrames(void);
uint32_t getStatusCtrl(void);
uint32_t getChannelData(uint32_t channel);
uint32_t getFlags(void);

/* set */
void setChannelThresholdLow(uint32_t value);
void setChannelThresholdHigh(uint32_t value);
void setChannelInactiveValue(uint32_t value);
void setMaxNumberLostFrames(uint32_t value);


#endif // SBUS_H
