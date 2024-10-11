/*
 * Copyright (C) 2024 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 */

#include "xil_types.h"
#include "xil_io.h"
#include "evt_decoder.h"


/* get */
uint32_t evt_decoder_getStatus(void) {

    return Xil_In32(EVT_REG_BASEADDR + EVT_STATUS_REG_OFFSET);
}

/* get */
uint32_t evt_decoder_getControl(void) {

    return Xil_In32(EVT_REG_BASEADDR + EVT_CONTROL_REG_OFFSET);
}

/* set */
void evt_decoder_setControl(uint32_t control) {

    Xil_Out32(EVT_REG_BASEADDR + EVT_CONTROL_REG_OFFSET, control);
}
