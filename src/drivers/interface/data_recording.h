/*
 * Copyright (C) 2024 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 */


/*
 *  data recorder register address mapping:
 *
 *   what                       | offset (bytes)  | default value
 *  ============================================================= 
 *   status                     | 0               | 0
 *   control                    | 4               | 0
 *   destination address        | 8               | 0x60000000
 *   recording time (ticks)     | 12              | 100000000
 *   bytes transferred          | 16              | 0
 */


#ifndef SRC_EVT_RECORDING_H_
#define SRC_EVT_RECORDING_H_

#include "xscugic.h"

/* --- dvs camera --- */
#define EVT_RECORDING_DESTINATION_ADDR 0x30000000
#define EVT_RECORDING_TICKS 500000000   // 5 seconds @100MHz

/* base address */
#define EVT_RECORDING_BASEADDR XPAR_EVENT_RECORDER_0_BASEADDR
#define EVTD_RECORDING_BASEADDR XPAR_EVENT_RECORDER_1_BASEADDR

/* registers */
#define EVT_RECORDING_STATUS_REG_OFFSET 0
#define EVT_RECORDING_CONTROL_REG_OFFSET 4
#define EVT_RECORDING_DEST_ADDR_REG_OFFSET 8
#define EVT_RECORDING_TIME_REG_OFFSET 12
#define EVT_BYTES_TRANSFERRED_REG_OFFSET 16

/* bits */
#define EVT_RECORDING_START_BIT 1


/* --- frame based rgb camera --- */
#define RGB_RECORDING_DESTINATION_ADDR 0x40000000
#define RGBD_RECORDING_DESTINATION_ADDR 0x50000000
#define RGB_RECORDING_TICKS 500000000   // 5 seconds @100MHz

/* base address */
#define RGB_RECORDING_BASEADDR XPAR_FRAME_RECORDER_0_BASEADDR
#define RGBD_RECORDING_BASEADDR XPAR_FRAME_RECORDER_1_BASEADDR

/* registers */
#define RGB_RECORDING_STATUS_REG_OFFSET 0
#define RGB_RECORDING_CONTROL_REG_OFFSET 4
#define RGB_RECORDING_DEST_ADDR_REG_OFFSET 8
#define RGB_RECORDING_TIME_REG_OFFSET 12
#define RGB_BYTES_TRANSFERRED_REG_OFFSET 16

/* bits */
#define RGB_RECORDING_START_BIT 1


int data_recording_interrupt_init(XScuGic *pInterruptController);
void data_recording_init(void);

#endif /* SRC_EVT_RECORDING_H_ */
