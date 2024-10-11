
/*
 * Copyright (C) 2024 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 */


#ifndef SRC_LED_BANK_H_
#define SRC_LED_BANK_H_

#define LED_BANK_BLUE 0xFF0000

#include "xil_types.h"


void led_on(uint8_t led, uint8_t red, uint8_t green, uint8_t blue);
void led_off(uint8_t led);

#endif /* SRC_LED_BANK_H_ */
