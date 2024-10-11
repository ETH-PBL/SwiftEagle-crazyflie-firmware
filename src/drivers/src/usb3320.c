/*
 * Copyright (C) 2024 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 */

#include "xparameters.h"
#include "xusbpsu_hw.h"
#include "sleep.h"
#include "usb3320.h"

/* To drive CPEN port of USB3320 chip, assert DrvVbus (bit 5) in 
   OTG Control register, see also datasheet (page 51): 
   https://ww1.microchip.com/downloads/aemDocuments/documents/OTH/ProductDocuments/DataSheets/00001792E.pdf.
   The USB3320 registers are written through Zynq's GUSB2PHYACC_ULPI register, for details see   
   https://docs.xilinx.com/r/en-US/ug1087-zynq-ultrascale-registers/GUSB2PHYACC_ULPI-USB3_XHCI-Register. */

int usb3320_drive_cpen() {
	uint32_t regVal;

	/* request register write access, and write data */
	usleep(10);
	regVal = GUSB2PHYACC_ULPI_NEWREGREQ_BIT_MASK | GUSB2PHYACC_ULPI_REGWR_BIT_MASK;
	regVal |= USB3320_OTG_CONTROL_REG_SET << GUSB2PHYACC_ULPI_REGWR_BITS_SHIFT;
	regVal |= GUSB2PHYACC_ULPI_REGDATA_BIT_MASK & USB3320_OTG_CONTROL_REG_DRVVBUS_BIT_MASK;
	Xil_Out32(XPAR_XUSBPSU_0_BASEADDR + XUSBPSU_GUSB2PHYACC(0), regVal);
	usleep(10);

	return XST_SUCCESS;
}
