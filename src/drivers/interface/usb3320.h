/*
 * Copyright (C) 2024 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 */

#ifndef SRC_USB_H_
#define SRC_USB_H_

/* bit masks */
#define GUSB2PHYACC_ULPI_NEWREGREQ_BIT_MASK     0x02000000
#define GUSB2PHYACC_ULPI_VSTSBSY_BIT_MASK       0x00800000
#define GUSB2PHYACC_ULPI_REGWR_BIT_MASK         0x00400000
#define GUSB2PHYACC_ULPI_REGDATA_BIT_MASK       0x008000FF
/* bits position */
#define GUSB2PHYACC_ULPI_REGWR_BITS_SHIFT       16

/* USB3320 registers */
#define USB3320_VENDOR_ID_LOW_REG       0x00
#define USB3320_VENDOR_ID_HIGH_REG      0x01
#define USB3320_PRODUCT_ID_LOW_REG      0x02
#define USB3320_PRODUCT_ID_HIGH_REG     0x03
#define USB3320_OTG_CONTROL_REG_READ    0x0A
#define USB3320_OTG_CONTROL_REG_WRITE   0x0A
#define USB3320_OTG_CONTROL_REG_SET     0x0B
#define USB3320_OTG_CONTROL_REG_CLEAR   0x0C

/* USB3320 OTC Control register bits */
#define USB3320_OTG_CONTROL_REG_DRVVBUS_BIT_MASK    0x20


int usb3320_drive_cpen();

#endif /* SRC_USB_H_ */
