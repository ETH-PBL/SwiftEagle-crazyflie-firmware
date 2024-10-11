/*
 * Copyright (C) 2024 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 */

//Adapted from Metavision SDC (open source on prophesee git)

#ifndef __COMMON_CAMERA_genx320_H__
#define __COMMON_CAMERA_genx320_H__

#include "iic_mipi.h"

int32_t genx320_init(uint8_t expander_channel);
int32_t genx320_open(uint8_t expander_channel);
int32_t genx320_on(uint8_t expander_channel);
int32_t genx320_off(uint8_t expander_channel);

#endif
