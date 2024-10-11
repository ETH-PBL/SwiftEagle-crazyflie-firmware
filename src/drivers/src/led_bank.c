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
#include "led_bank.h"


void led_on(uint8_t led, uint8_t red, uint8_t green, uint8_t blue) {

    Xil_Out32(XPAR_LED_CONTROLLER_0_BASEADDR + 8 + led*4, (blue << 16) | (red << 8) | (green << 0));
}

void led_off(uint8_t led) {

    Xil_Out32(XPAR_LED_CONTROLLER_0_BASEADDR + 8 + led*4, 0x0);
}
