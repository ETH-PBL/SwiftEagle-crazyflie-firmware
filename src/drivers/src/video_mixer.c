/*
 * Copyright (C) 2024 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 */
 
#include "xparameters.h"
#include "xv_mix.h"
#include "vdma.h"
#include "imx219.h"

XV_mix mix;


int mixer_init(void)
{
    int Status;

    Status = XV_mix_Initialize(&mix, XPAR_V_MIX_0_DEVICE_ID);
    if(Status != XST_SUCCESS) {
        xil_printf("ERROR: Mixer device not found\r\n");
        return(XST_FAILURE);
    }

    // general settings
    XV_mix_Set_HwReg_width(&mix, VIDEO_COLUMNS);
    XV_mix_Set_HwReg_height(&mix, VIDEO_ROWS);
    XV_mix_Set_HwReg_layerEnable(&mix, 0x03);
    
    // camera layer
    XV_mix_Set_HwReg_layerAlpha_0(&mix, 256);
    XV_mix_Set_HwReg_layerStartX_0(&mix, 0);
    XV_mix_Set_HwReg_layerStartY_0(&mix, 0);
    XV_mix_Set_HwReg_layerWidth_0(&mix, VIDEO_COLUMNS);
    XV_mix_Set_HwReg_layerHeight_0(&mix, VIDEO_ROWS);
    XV_mix_Set_HwReg_layerScaleFactor_0(&mix, 0);

    // dvs layer
    XV_mix_Set_HwReg_layerAlpha_1(&mix, 256);
    XV_mix_Set_HwReg_layerStartX_1(&mix, 0);
    XV_mix_Set_HwReg_layerStartY_1(&mix, 0);
    XV_mix_Set_HwReg_layerWidth_1(&mix, DVS_IMAGE_COLUMNS);
    XV_mix_Set_HwReg_layerHeight_1(&mix, DVS_IMAGE_ROWS);
    XV_mix_Set_HwReg_layerScaleFactor_1(&mix, 0);

    XV_mix_EnableAutoRestart(&mix);
    XV_mix_Start(&mix);

    xil_printf("Video Mixer configured\r\n");

    return XST_SUCCESS;
}

int mixer_stop_dvs_layer(void) {
    u32 control;
    
    control = XV_mix_Get_HwReg_layerEnable(&mix);
    XV_mix_Set_HwReg_layerEnable(&mix, control & ~0x02);

    return XST_SUCCESS;
}

int mixer_start_dvs_layer(void) {
    u32 control;
    
    control = XV_mix_Get_HwReg_layerEnable(&mix);
    XV_mix_Set_HwReg_layerEnable(&mix, control | 0x02);

    return XST_SUCCESS;
}
