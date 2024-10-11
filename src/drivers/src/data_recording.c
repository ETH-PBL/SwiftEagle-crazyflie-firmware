/*
 * Copyright (C) 2024 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 */

#include "xparameters.h"
#include "FreeRTOS.h"
#include "task.h"
#include "static_mem.h"
#include "semphr.h"
#include "xil_printf.h"
#include "config.h"
#include "xscugic.h"
#include "xgpio.h"
#include "sleep.h"
#include "xcsiss.h"
#include "genx320.h"
#include "flash.h"
#include "sbus.h"
#include "data_recording.h"
#include "evt_decoder.h"
#include "sensors_lsm6dsm.h"
#include "system.h"
#include "led_bank.h"

static bool isInit;
static XCsiSs mipi;
static XCsiSs_Config *mipi_config;

static XGpio_Config *gpio_cfg_ptr;
static XGpio gpio_recording_active;
#define GPIO_CHANNEL    1

xSemaphoreHandle evtRecordingDone;
StaticSemaphore_t evtRecordingDoneBuffer;
xSemaphoreHandle rgbRecordingDone;
StaticSemaphore_t rgbRecordingDoneBuffer;
xSemaphoreHandle rgbdRecordingDone;
StaticSemaphore_t rgbdRecordingDoneBuffer;
static void evt_irq_handler(void);
static void rgb_irq_handler(void);
static void rgbd_irq_handler(void);

STATIC_MEM_TASK_ALLOC(rawRecordingTask, RAWRECORDING_TASK_STACKSIZE);
static void rawRecordingTask(void* param);

static int start_recording(void);

int data_recording_interrupt_init(XScuGic *pInterruptController) {
    BaseType_t xStatus;
    XScuGic_Config *pxGICConfig;
    int Status;

    /* --- dvs camera --- */
    evtRecordingDone = xSemaphoreCreateBinaryStatic(&evtRecordingDoneBuffer);
    pxGICConfig = XScuGic_LookupConfig(XPAR_FABRIC_EVENT_RECORDER_0_IRQ_INTR);
    // no need to call XScuGic_CfgInitialize() since already called in FreeRTOS_tick_config.c
    /* rising edge sensitive */
    XScuGic_SetPriorityTriggerType(pInterruptController, XPAR_FABRIC_EVENT_RECORDER_0_IRQ_INTR, 152, 0b11);
    /* interrupt handler */ 
    xStatus = XScuGic_Connect(pInterruptController, XPAR_FABRIC_EVENT_RECORDER_0_IRQ_INTR, 
        (Xil_ExceptionHandler) evt_irq_handler, NULL);
    configASSERT(xStatus == XST_SUCCESS);
    (void) xStatus;
    XScuGic_Enable(pInterruptController, XPAR_FABRIC_EVENT_RECORDER_0_IRQ_INTR);

    // configure recording parameters
    Xil_Out32(EVT_RECORDING_BASEADDR + EVT_RECORDING_DEST_ADDR_REG_OFFSET, 
        EVT_RECORDING_DESTINATION_ADDR);
    Xil_Out32(EVT_RECORDING_BASEADDR + EVT_RECORDING_TIME_REG_OFFSET, 
        EVT_RECORDING_TICKS);

    /* --- frame based rgb camera 1--- */
    rgbRecordingDone = xSemaphoreCreateBinaryStatic(&rgbRecordingDoneBuffer);
    pxGICConfig = XScuGic_LookupConfig(XPAR_FABRIC_FRAME_RECORDER_0_IRQ_INTR);
    // no need to call XScuGic_CfgInitialize() since already called in FreeRTOS_tick_config.c
    /* rising edge sensitive */
    XScuGic_SetPriorityTriggerType(pInterruptController, XPAR_FABRIC_FRAME_RECORDER_0_IRQ_INTR, 152, 0b11);
    /* interrupt handler */ 
    xStatus = XScuGic_Connect(pInterruptController, XPAR_FABRIC_FRAME_RECORDER_0_IRQ_INTR, 
        (Xil_ExceptionHandler) rgb_irq_handler, NULL);
    configASSERT(xStatus == XST_SUCCESS);
    (void) xStatus;
    XScuGic_Enable(pInterruptController, XPAR_FABRIC_FRAME_RECORDER_0_IRQ_INTR);

    // configure recording parameters
    Xil_Out32(RGB_RECORDING_BASEADDR + RGB_RECORDING_DEST_ADDR_REG_OFFSET, 
        RGB_RECORDING_DESTINATION_ADDR);
    Xil_Out32(RGB_RECORDING_BASEADDR + RGB_RECORDING_TIME_REG_OFFSET, 
        RGB_RECORDING_TICKS);

    /* --- frame based rgb camera 2--- */
    rgbdRecordingDone = xSemaphoreCreateBinaryStatic(&rgbdRecordingDoneBuffer);
    pxGICConfig = XScuGic_LookupConfig(XPAR_FABRIC_FRAME_RECORDER_1_IRQ_INTR);
    // no need to call XScuGic_CfgInitialize() since already called in FreeRTOS_tick_config.c
    /* rising edge sensitive */
    XScuGic_SetPriorityTriggerType(pInterruptController, XPAR_FABRIC_FRAME_RECORDER_1_IRQ_INTR, 152, 0b11);
    /* interrupt handler */ 
    xStatus = XScuGic_Connect(pInterruptController, XPAR_FABRIC_FRAME_RECORDER_1_IRQ_INTR, 
        (Xil_ExceptionHandler) rgbd_irq_handler, NULL);
    configASSERT(xStatus == XST_SUCCESS);
    (void) xStatus;
    XScuGic_Enable(pInterruptController, XPAR_FABRIC_FRAME_RECORDER_1_IRQ_INTR);

    // configure recording parameters
    Xil_Out32(RGBD_RECORDING_BASEADDR + RGB_RECORDING_DEST_ADDR_REG_OFFSET, 
        RGBD_RECORDING_DESTINATION_ADDR);
    Xil_Out32(RGBD_RECORDING_BASEADDR + RGB_RECORDING_TIME_REG_OFFSET, 
        RGB_RECORDING_TICKS);

    //configure the GPIO port (e.g. for the VICON system)
    gpio_cfg_ptr = XGpio_LookupConfig(XPAR_PSU_GPIO_0_DEVICE_ID);
    XGpio_CfgInitialize(&gpio_recording_active, gpio_cfg_ptr, gpio_cfg_ptr->BaseAddress);
    XGpio_SetDataDirection(&gpio_recording_active, GPIO_CHANNEL, 0); //set port to output
    XGpio_DiscreteWrite(&gpio_recording_active, GPIO_CHANNEL, 0); //set output zero

    return XST_SUCCESS;
}


void data_recording_init(void) {
    if(isInit)
        return;

    STATIC_MEM_TASK_CREATE(rawRecordingTask, rawRecordingTask, RAWRECORDING_TASK_NAME, NULL, RAWRECORDING_TASK_PRI);
    isInit = true;
}


static void evt_irq_handler(void)
{
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

    xSemaphoreGiveFromISR(evtRecordingDone, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}


static void rgb_irq_handler(void)
{
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

    xSemaphoreGiveFromISR(rgbRecordingDone, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}


static void rgbd_irq_handler(void)
{
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

    xSemaphoreGiveFromISR(rgbdRecordingDone, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}


static void rawRecordingTask(void* param) {
    //Wait for the system to be fully started to start stabilization loop
    systemWaitStart();

    while(1) {
        // wait for remote control to trigger recording
        xSemaphoreTake(interruptFromSbus, portMAX_DELAY);
        start_recording();
    }
}


static int start_recording(void) {
    int Status;
    uint32_t ctrl;
    uint32_t recv_bytes = 0;
    char dvs_filename[32] = "dvs";
    char rgb_filename[32] = "rgb";
    char rgbd_filename[32] = "drgb";
    char dvsd_filename[32] = "ddvs";

    led_on(3, 0xFF, 0x00, 0x00);
    XGpio_DiscreteWrite(&gpio_recording_active, GPIO_CHANNEL, 1); //set output one

    // enable EXTTRIG
    ctrl = evt_decoder_getControl();
    evt_decoder_setControl(ctrl | EVT_CONTROL_REG_START_EXTTRIG_BIT);

    // start imu recording
    xSemaphoreGive(startImuRecording);

    xil_printf("start raw data recording ... ");
    ctrl = Xil_In32(EVT_RECORDING_BASEADDR + EVT_RECORDING_CONTROL_REG_OFFSET);
    Xil_Out32(EVT_RECORDING_BASEADDR + EVT_RECORDING_CONTROL_REG_OFFSET, 
        ctrl | EVT_RECORDING_START_BIT);
    ctrl = Xil_In32(RGB_RECORDING_BASEADDR + RGB_RECORDING_CONTROL_REG_OFFSET);
    Xil_Out32(RGB_RECORDING_BASEADDR + RGB_RECORDING_CONTROL_REG_OFFSET, 
        ctrl | RGB_RECORDING_START_BIT);
    ctrl = Xil_In32(RGBD_RECORDING_BASEADDR + RGB_RECORDING_CONTROL_REG_OFFSET);
    Xil_Out32(RGBD_RECORDING_BASEADDR + RGB_RECORDING_CONTROL_REG_OFFSET, 
        ctrl | RGB_RECORDING_START_BIT);

    // wait for evt recording to finish
    xSemaphoreTake(evtRecordingDone, portMAX_DELAY);
    xil_printf("EVT done ... ");

    // wait for rgb recording to finish
    xSemaphoreTake(rgbRecordingDone, portMAX_DELAY);
    xil_printf("RGB done ... ");

    // wait for rgbd recording to finish
    xSemaphoreTake(rgbdRecordingDone, portMAX_DELAY);
    xil_printf("RGBD done ... ");

    // stop imu recording
    xSemaphoreGive(stopImuRecording);

    // wait until imu recording has finished
    xSemaphoreTake(imuRecordingFinished, portMAX_DELAY);
    xil_printf("IMU done ... ");

    // disable EXTTRIG
    ctrl = evt_decoder_getControl();
    evt_decoder_setControl(ctrl & ~EVT_CONTROL_REG_START_EXTTRIG_BIT);

    led_on(3, 0x00, 0x00, 0xFF);
    XGpio_DiscreteWrite(&gpio_recording_active, GPIO_CHANNEL, 0); //set output zero
    xil_printf("all done\r\n");

    // write data to sd card
    recv_bytes = Xil_In32(EVT_RECORDING_BASEADDR + EVT_BYTES_TRANSFERRED_REG_OFFSET);
    flash_write((uint8_t *)EVT_RECORDING_DESTINATION_ADDR, recv_bytes, dvs_filename);
    recv_bytes = Xil_In32(RGB_RECORDING_BASEADDR + RGB_BYTES_TRANSFERRED_REG_OFFSET);
    flash_write((uint8_t *)RGB_RECORDING_DESTINATION_ADDR, recv_bytes, rgb_filename);
    recv_bytes = Xil_In32(RGBD_RECORDING_BASEADDR + RGB_BYTES_TRANSFERRED_REG_OFFSET);
    flash_write((uint8_t *)RGBD_RECORDING_DESTINATION_ADDR, recv_bytes, rgbd_filename);
    sensorsWriteFlash();

    // acknowledge interrupt on sbus IP
    sbus_irq_ack();

    led_off(3);

    return XST_SUCCESS;
}
