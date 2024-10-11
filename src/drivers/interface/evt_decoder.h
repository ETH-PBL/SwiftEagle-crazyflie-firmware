
/*
 * Copyright (C) 2024 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 */

/*
 *  evt decoder register address mapping:
 *
 *   what                       | offset (bytes)  | default value
 *  ============================================================= 
 *   status                     | 0               | 0
 *   control                    | 4               | 0         
 */


#ifndef EVT_DECODER_H
#define EVT_DECODER_H

#include "xil_types.h"
#include "xparameters.h"

/* base address */
#define EVT_REG_BASEADDR XPAR_EVT_TOP_WRAPPER_0_BASEADDR

/* registers */
#define EVT_STATUS_REG_OFFSET 0
#define EVT_CONTROL_REG_OFFSET 4

/* bits */
#define EVT_STATUS_REG_ERROR_BIT 1
#define EVT_CONTROL_REG_START_EXTTRIG_BIT 2


/* get */
uint32_t evt_decoder_getStatus(void);
uint32_t evt_decoder_getControl(void);
/* set */
void evt_decoder_setControl(uint32_t control);


#endif // EVT_DECODER_H
