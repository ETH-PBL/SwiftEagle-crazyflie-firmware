/*
 * Copyright (C) 2024 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 */
 
#include "xil_types.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "xil_io.h"
#include "xscugic.h"
#include "sbus.h"


xSemaphoreHandle interruptFromSbus;
StaticSemaphore_t interruptFromSbusBuffer;
static void irq_handler(void);


/* interrupt initialization */
void sbus_irq_init(XScuGic *pInterruptController) {
    BaseType_t xStatus;
    XScuGic_Config *pxGICConfig;
    int Status;

    interruptFromSbus = xSemaphoreCreateBinaryStatic(&interruptFromSbusBuffer);

    pxGICConfig = XScuGic_LookupConfig(XPAR_FABRIC_SBUS_TOP_0_SBUS_IRQ_INTR);
    // no need to call XScuGic_CfgInitialize() since already called in FreeRTOS_tick_config.c
    /* rising edge sensitive */
    XScuGic_SetPriorityTriggerType(pInterruptController, XPAR_FABRIC_SBUS_TOP_0_SBUS_IRQ_INTR, 152, 0b11);

    /* interrupt handler */ 
    xStatus = XScuGic_Connect(pInterruptController, XPAR_FABRIC_SBUS_TOP_0_SBUS_IRQ_INTR, 
        (Xil_ExceptionHandler) irq_handler, NULL);
    configASSERT(xStatus == XST_SUCCESS);
    (void) xStatus;

    XScuGic_Enable(pInterruptController, XPAR_FABRIC_SBUS_TOP_0_SBUS_IRQ_INTR);
}


static void irq_handler(void)
{
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

    xSemaphoreGiveFromISR(interruptFromSbus, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}


void sbus_irq_ack(void) {
    uint32_t status_ctrl;
    status_ctrl = Xil_In32(SBUS_REG_OFFSET + STATUS_CTRL_REG_OFFSET);
    status_ctrl |= STATUS_REG_ACK_INT_BIT;
    Xil_Out32(SBUS_REG_OFFSET + STATUS_CTRL_REG_OFFSET, status_ctrl);
}


/* get */
uint32_t getChannelThresholdLow(void) {

    return Xil_In32(SBUS_REG_OFFSET + CH_THRESHOLD_LOW_REG_OFFSET);
}
uint32_t getChannelThresholdHigh(void) {

    return Xil_In32(SBUS_REG_OFFSET + CH_THRESHOLD_HIGH_REG_OFFSET);
}
uint32_t getChannelInactiveValue(void) {

    return Xil_In32(SBUS_REG_OFFSET + CH_INACTIVE_VALUE_REG_OFFSET);
}
uint32_t getMaxNumberLostFrames(void) {

    return Xil_In32(SBUS_REG_OFFSET + MAX_NBR_LOST_FRAMES_REG_OFFSET);
}
uint32_t getStatusCtrl(void) {

    return Xil_In32(SBUS_REG_OFFSET + STATUS_CTRL_REG_OFFSET);
}
uint32_t getChannelData(uint32_t channel) {

    return Xil_In32(SBUS_REG_OFFSET + CHANNEL_0_REG_OFFSET + channel*4);
}
uint32_t getFlags(void) {

    return Xil_In32(SBUS_REG_OFFSET + FLAGS_REG_OFFSET);
}


/* set */
void setChannelThresholdLow(uint32_t value) {

    Xil_Out32(SBUS_REG_OFFSET + CH_THRESHOLD_LOW_REG_OFFSET, value);
}
void setChannelThresholdHigh(uint32_t value) {

    Xil_Out32(SBUS_REG_OFFSET + CH_THRESHOLD_HIGH_REG_OFFSET, value);
}
void setChannelInactiveValue(uint32_t value) {

    Xil_Out32(SBUS_REG_OFFSET + CH_INACTIVE_VALUE_REG_OFFSET, value);
}
void setMaxNumberLostFrames(uint32_t value) {

    Xil_Out32(SBUS_REG_OFFSET + MAX_NBR_LOST_FRAMES_REG_OFFSET, value);
}
