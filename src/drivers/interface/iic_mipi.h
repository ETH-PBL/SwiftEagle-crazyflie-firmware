/*
 * Copyright (C) 2024 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 */


#ifndef __IIC_MIPI_H__
#define __IIC_MIPI_H__

#include "xscugic.h"

#define IIC_EXPANDER_DEV_ADDR 0x70

#define IIC_EXPANDER_CHANNEL_0 0x01
#define IIC_EXPANDER_CHANNEL_1 0x02
#define IIC_EXPANDER_CHANNEL_2 0x04
#define IIC_EXPANDER_CHANNEL_3 0x08


typedef struct {
    uint8_t     expander_channel;
    uint8_t     address;
    uint32_t    buffer_length;
} mipi_interface;


int32_t iic_mipi_init(void);
void iic_mipi_interrupt_init(XScuGic *pInterruptController);
int32_t iic_mipi_write(mipi_interface *mipi, uint16_t reg, uint8_t *buf);
int32_t iic_mipi_read(mipi_interface *mipi, uint16_t reg, uint8_t *buf);


#endif /* __IIC_MIPI_H__ */
