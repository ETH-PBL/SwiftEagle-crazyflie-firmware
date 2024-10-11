/*******************************************************************************
* Copyright (C) 2017 - 2020 Xilinx, Inc.  All rights reserved.
* SPDX-License-Identifier: MIT
*******************************************************************************/

/*****************************************************************************/
/**
*
* @file displayport.h
*
*
* This file contains a initialization routines for the display port driver
*
* @note
*
* None.
******************************************************************************/

#ifndef SRC_DPDMA_VIDEO_H_
/* Prevent circular inclusions by using protection macros. */
#define SRC_DPDMA_VIDEO_H_

/******************************* Include Files ********************************/

#include "xparameters.h"	/* SDK generated parameters */
#include "xdpdma.h"			/* DPDMA device driver */
#include "xscugic.h"		/* Interrupt controller device driver */
#include "xdppsu.h"			/* DP controller device driver */
#include "xavbuf.h"    		/* AVBUF is the video pipeline driver */
#include "xavbuf_clk.h"		/* Clock Driver for Video(VPLL) and Audio(RPLL) clocks */

/****************************** Type Definitions ******************************/

typedef enum {
	LANE_COUNT_1 = 1,
	LANE_COUNT_2 = 2,
} LaneCount_t;

typedef enum {
	LINK_RATE_162GBPS = 0x06,
	LINK_RATE_270GBPS = 0x0A,
	LINK_RATE_540GBPS = 0x14,
} LinkRate_t;

typedef struct {
	XDpPsu	*DpPsuPtr;
	XAVBuf	*AVBufPtr;
	XDpDma	*DpDmaPtr;

	XVidC_VideoMode	  VideoMode;
	XVidC_ColorDepth  Bpc;
	XDpPsu_ColorEncoding ColorEncode;

	u8 UseMaxLaneCount;
	u8 UseMaxLinkRate;
	u8 LaneCount;
	u8 LinkRate;
	u8 UseMaxCfgCaps;
	u8 EnSynchClkMode;

	u32 PixClkHz;
} Run_Config;

/************************** Function Prototypes ******************************/
int InitDpDmaSubsystem(Run_Config *RunCfgPtr);

int displayport_init(XScuGic *pInterruptController);

/* DisplayPort interrupt related functions */
void DpPsu_SetupVideoStream(Run_Config *RunCfgPtr);

void DpPsu_Run(Run_Config *RunCfgPtr);
void DpPsu_IsrHpdEvent(void *ref);
void DpPsu_IsrHpdPulse(void *ref);

/************************** Variable Definitions *****************************/


#endif /* SRC_DPDMA_VIDEO_H_ */
