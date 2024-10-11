/*******************************************************************************
* Copyright (C) 2017 - 2020 Xilinx, Inc.  All rights reserved.
* SPDX-License-Identifier: MIT
*******************************************************************************/

/*****************************************************************************/
/**
*
* @file displayport.c
*
*
* This file contains a initialization routines for the display port driver
*
* @note
*
* None.
*
******************************************************************************/

/***************************** Include Files *********************************/

#include "xil_exception.h"
#include "xil_printf.h"
#include "xil_cache.h"
#include "displayport.h"

/************************** Constant Definitions *****************************/
#define DPPSU_DEVICE_ID		XPAR_PSU_DP_DEVICE_ID
#define AVBUF_DEVICE_ID		XPAR_PSU_DP_DEVICE_ID
#define DPDMA_DEVICE_ID		XPAR_XDPDMA_0_DEVICE_ID
#define DPPSU_INTR_ID		151
#define DPDMA_INTR_ID		154
#define INTC_DEVICE_ID		XPAR_SCUGIC_0_DEVICE_ID

#define DPPSU_BASEADDR		XPAR_PSU_DP_BASEADDR
#define AVBUF_BASEADDR		XPAR_PSU_DP_BASEADDR
#define DPDMA_BASEADDR		XPAR_PSU_DPDMA_BASEADDR

#define BUFFERSIZE			1920 * 1080 * 4		/* HTotal * VTotal * BPP */
#define LINESIZE			1920 * 4			/* HTotal * BPP */
#define STRIDE				LINESIZE			/* The stride value should
													be aligned to 256*/

/************************** Variable Declarations ***************************/
XDpDma DpDma;
XDpPsu DpPsu;
XAVBuf AVBuf;
Run_Config RunCfg;

u8 Frame[BUFFERSIZE] __attribute__ ((__aligned__(256)));
XDpDma_FrameBuffer FrameBuffer;


/************************** Function Prototpyes ***************************/
static void InitRunConfig(Run_Config *RunCfgPtr);
static void SetupInterrupts(Run_Config *RunCfgPtr, XScuGic *pInterruptController);


/**************************** Type Definitions *******************************/

/*****************************************************************************/
/**
*
* Main function to call the DPDMA Video example.
*
* @param	None
*
* @return	XST_SUCCESS if successful, otherwise XST_FAILURE.
*
* @note		None
*
******************************************************************************/
int displayport_init(XScuGic *pInterruptController)
{
	int Status;

	/* Initialize the application configuration */
	InitRunConfig(&RunCfg);
	Status = InitDpDmaSubsystem(&RunCfg);
	if (Status != XST_SUCCESS) {
				return XST_FAILURE;
	}

	SetupInterrupts(&RunCfg, pInterruptController);

	xil_printf("Successfully initialized display port\r\n");

    return XST_SUCCESS;
}

/*****************************************************************************/
/**
*
* The purpose of this function is to initialize the application configuration.
*
* @param	RunCfgPtr is a pointer to the application configuration structure.
*
* @return	None.
*
* @note		None.
*
*****************************************************************************/
static void InitRunConfig(Run_Config *RunCfgPtr)
{
	/* Initial configuration parameters. */
		RunCfgPtr->DpPsuPtr   = &DpPsu;
		RunCfgPtr->AVBufPtr  = &AVBuf;
		RunCfgPtr->DpDmaPtr  = &DpDma;
		RunCfgPtr->VideoMode = XVIDC_VM_1920x1080_60_P;
		RunCfgPtr->Bpc		 = XVIDC_BPC_8;
		RunCfgPtr->ColorEncode			= XDPPSU_CENC_RGB;
		RunCfgPtr->UseMaxCfgCaps		= 1;
		RunCfgPtr->LaneCount			= LANE_COUNT_1;
		RunCfgPtr->LinkRate				= LINK_RATE_540GBPS;
		RunCfgPtr->EnSynchClkMode		= 0;
		RunCfgPtr->UseMaxLaneCount		= 0;
		RunCfgPtr->UseMaxLinkRate		= 1;
}

/*****************************************************************************/
/**
*
* The purpose of this function is to initialize the DP Subsystem (XDpDma,
* XAVBuf, XDpPsu)
*
* @param	RunCfgPtr is a pointer to the application configuration structure.
*
* @return	None.
*
* @note		None.
*
*****************************************************************************/
int InitDpDmaSubsystem(Run_Config *RunCfgPtr)
{
	u32 Status;
	XDpPsu		*DpPsuPtr = RunCfgPtr->DpPsuPtr;
	XDpPsu_Config	*DpPsuCfgPtr;
	XAVBuf		*AVBufPtr = RunCfgPtr->AVBufPtr;
	XDpDma_Config *DpDmaCfgPtr;
	XDpDma		*DpDmaPtr = RunCfgPtr->DpDmaPtr;


	/* Initialize DisplayPort driver. */
	DpPsuCfgPtr = XDpPsu_LookupConfig(DPPSU_DEVICE_ID);
	XDpPsu_CfgInitialize(DpPsuPtr, DpPsuCfgPtr, DpPsuCfgPtr->BaseAddr);
	/* Initialize Video Pipeline driver */
	XAVBuf_CfgInitialize(AVBufPtr, DpPsuPtr->Config.BaseAddr, AVBUF_DEVICE_ID);

	/* Initialize the DPDMA driver */
	DpDmaCfgPtr = XDpDma_LookupConfig(DPDMA_DEVICE_ID);
	XDpDma_CfgInitialize(DpDmaPtr,DpDmaCfgPtr);

	/* Initialize the DisplayPort TX core. */
	Status = XDpPsu_InitializeTx(DpPsuPtr);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}
	/* Set the format graphics frame for DPDMA*/
	Status = XDpDma_SetGraphicsFormat(DpDmaPtr, RGBA8888);
	if (Status != XST_SUCCESS) {
			return XST_FAILURE;
	}
	/* Set the QOS for Video */
	XDpDma_SetQOS(RunCfgPtr->DpDmaPtr, 11);
	/* Set the output Video Format */
	XAVBuf_SetOutputVideoFormat(AVBufPtr, RGB_10BPC);

	/* Select the Input Video Sources. */
	XAVBuf_InputVideoSelect(AVBufPtr, XAVBUF_VIDSTREAM1_LIVE,
							XAVBUF_VIDSTREAM2_NONE);
	/* Configure the output video pipeline */
	XAVBuf_ConfigureOutputVideo(AVBufPtr);
	/* Disable the global alpha, since we are using the pixel based alpha */
	XAVBuf_SetBlenderAlpha(AVBufPtr, 0, 0);
	/* Set the clock mode */
	XDpPsu_CfgMsaEnSynchClkMode(DpPsuPtr, RunCfgPtr->EnSynchClkMode);
	/* Set the clock source */
	XAVBuf_SetAudioVideoClkSrc(AVBufPtr, XAVBUF_PL_CLK, XAVBUF_PS_CLK);
	/* Issue a soft reset after selecting the input clock sources */
	XAVBuf_SoftReset(AVBufPtr);
	/* wait some time after reset */
	usleep(2*1000*1000);

	return XST_SUCCESS;
}

/*****************************************************************************/
/**
*
* The purpose of this function is to setup call back functions for the DP
* controller interrupts.
*
* @param	RunCfgPtr is a pointer to the application configuration structure.
*
* @return	None.
*
* @note		None.
*
*****************************************************************************/
static void SetupInterrupts(Run_Config *RunCfgPtr, XScuGic *pInterruptController)
{
	XDpPsu *DpPsuPtr = RunCfgPtr->DpPsuPtr;
	XScuGic_Config	*IntrCfgPtr;
	u32  IntrMask = XDPPSU_INTR_HPD_IRQ_MASK | XDPPSU_INTR_HPD_EVENT_MASK;

	XDpPsu_WriteReg(DpPsuPtr->Config.BaseAddr, XDPPSU_INTR_DIS, 0xFFFFFFFF);
	XDpPsu_WriteReg(DpPsuPtr->Config.BaseAddr, XDPPSU_INTR_MASK, 0xFFFFFFFF);

	XDpPsu_SetHpdEventHandler(DpPsuPtr, DpPsu_IsrHpdEvent, RunCfgPtr);
	XDpPsu_SetHpdPulseHandler(DpPsuPtr, DpPsu_IsrHpdPulse, RunCfgPtr);

	IntrCfgPtr = XScuGic_LookupConfig(INTC_DEVICE_ID);

	/* Register ISRs. */
	XScuGic_Connect(pInterruptController, DPPSU_INTR_ID,
			(Xil_InterruptHandler)XDpPsu_HpdInterruptHandler, RunCfgPtr->DpPsuPtr);

	/* Trigger DP interrupts on rising edge. */
	XScuGic_SetPriorityTriggerType(pInterruptController, DPPSU_INTR_ID, 146, 0x03);


	/* Connect DPDMA Interrupt */
	XScuGic_Connect(pInterruptController, DPDMA_INTR_ID,
			(Xil_ExceptionHandler)XDpDma_InterruptHandler, RunCfgPtr->DpDmaPtr);

	/* Enable DP interrupts. */
	XScuGic_Enable(pInterruptController, DPPSU_INTR_ID);
	XDpPsu_WriteReg(DpPsuPtr->Config.BaseAddr, XDPPSU_INTR_EN, IntrMask);

	/* Enable DPDMA Interrupts */
	XScuGic_Enable(pInterruptController, DPDMA_INTR_ID);
	XDpDma_InterruptEnable(RunCfgPtr->DpDmaPtr, XDPDMA_IEN_VSYNC_INT_MASK);

}
