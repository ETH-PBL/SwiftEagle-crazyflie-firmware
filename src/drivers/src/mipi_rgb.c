/*
FILENAME: mipi.c
AUTHOR: Greg Taylor     CREATION DATE: 12 Aug 2019

DESCRIPTION:

CHANGE HISTORY:
12 Aug 2019		Greg Taylor
	Initial version

MIT License

Copyright (c) 2019 Greg Taylor <gtaylor@sonic.net>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
 */
#include "xparameters.h"
#include "FreeRTOS.h"
#include "xcsiss.h"
#include "config.h"
#include "xscugic.h"

static XCsiSs mipi;

static void mipi_rgb_isr();

int mipi_rgb_init(XScuGic *pInterruptController) {
	XCsiSs_Config *mipi_config;
	XScuGic_Config *pxGICConfig;
	int Status;
	
	pxGICConfig = XScuGic_LookupConfig(XPAR_FABRIC_MIPICSISS_1_VEC_ID);
    // no need to call XScuGic_CfgInitialize() since already called in FreeRTOS_tick_config.c
    /* rising edge sensitive */
    XScuGic_SetPriorityTriggerType(pInterruptController, XPAR_FABRIC_MIPICSISS_1_VEC_ID, 151, 0b11);

    /* interrupt handler */ 
    Status = XScuGic_Connect(pInterruptController, XPAR_FABRIC_MIPICSISS_1_VEC_ID, 
        (Xil_ExceptionHandler) mipi_rgb_isr, NULL);
    configASSERT(Status == XST_SUCCESS);
    (void) Status;

	XScuGic_Enable(pInterruptController, XPAR_FABRIC_MIPICSISS_1_VEC_ID);

	if ( (mipi_config = XCsiSs_LookupConfig(XPAR_CAMERA_IN_MIPI_CSI2_RX_SUBSYST_0_RX_DEVICE_ID)) == NULL) {
		xil_printf("XCsiSs_LookupConfig() failed\r\n");
		return XST_FAILURE;
	}
	if (XCsiSs_CfgInitialize(&mipi, mipi_config, mipi_config->BaseAddr) != XST_SUCCESS) {
		xil_printf("XCsiSs_CfgInitialize() failed\r\n");
		return XST_FAILURE;
	}

	/* enable all interrupts */
	if (XCsiSs_Configure(&mipi, 2, 0xffffffff) != XST_SUCCESS) {
		xil_printf("mipi core failed to configure\r\n");
		return XST_FAILURE;
	}

	if (XCsiSs_SelfTest(&mipi) != XST_SUCCESS) {
		xil_printf("mipi core failed self test\r\n");
		return XST_FAILURE;
	}

	if (XCsiSs_Activate(&mipi, 1) != XST_SUCCESS) {
		xil_printf("mipi core failed to activate\r\n");
		return XST_FAILURE;
	}

	xil_printf("MIPI RGB CSI-2 Rx Subsystem initialized\r\n");

	return XST_SUCCESS;
}

static void mipi_rgb_isr() {
	uint32_t status;

	status = XCsiSs_ReadReg(mipi.Config.BaseAddr, XCSI_ISR_OFFSET);

	if (status & XCSI_ISR_SLBF_MASK) {
		xil_printf("error: rgb MIPI stream line buffer occured\r\n");
	}

	/* acknowledge interrupts */
	XCsiSs_WriteReg(mipi.Config.BaseAddr, XCSI_ISR_OFFSET, status);
}
