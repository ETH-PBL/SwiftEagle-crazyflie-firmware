/*
 * Copyright (C) 2024 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 */


#include "xil_printf.h"
#include "FreeRTOS.h"
#include "xil_exception.h"
#include "xiic.h"
#include "xparameters.h"
#include "xinterrupt_wrap.h"
#include "xscugic.h"
#include "sleep.h"
#include "iic_mipi.h"

static XIic iic;

static void RecvHandler(void *CallbackRef, int ByteCount);
static void SendHandler(void *CallbackRef, int ByteCount);
static void StatusHandler(void *CallbackRef, int Status);

// iic callback handler definitions
volatile struct {
    int EventStatus;
    int RemainingRecvBytes;
    int EventStatusUpdated;
    int RecvBytesUpdated;
    int SendBytesUpdated;
    int BytesSent;
} iic_handler_info;


/***************************************************************************
* initialize iic
*
* @param	None.
*
* @return	XST_SUCCESS or XST_FAILURE.
*
****************************************************************************/
int32_t iic_mipi_init(void)
{
    XIic_Config *iic_config;

	if ( (iic_config = XIic_LookupConfig(XPAR_IIC_1_DEVICE_ID)) == NULL) {
		xil_printf("XIic_LookupConfig() failed\r\n");
		return XST_FAILURE;
	}
	if (XIic_CfgInitialize(&iic, iic_config, iic_config->BaseAddress) != XST_SUCCESS) {
		xil_printf("XIic_CfgInitialize() failed\r\n");
		return XST_FAILURE;
	}

	if (XIic_SelfTest(&iic) != XST_SUCCESS) {
		xil_printf("XIic_SelfTest() failed\r\n");
		return XST_FAILURE;
	}

    XIic_SetRecvHandler(&iic, (void *)&iic_handler_info, RecvHandler);
    XIic_SetSendHandler(&iic, (void *)&iic_handler_info, SendHandler);
    XIic_SetStatusHandler(&iic, (void *)&iic_handler_info, StatusHandler);

    // start the iic module
    if (XIic_Start(&iic) != XST_SUCCESS) {
        xil_printf("XIic_Start() failed\r\n");
        return XST_FAILURE;
    }

    return XST_SUCCESS;
}


/***************************************************************************
* initialize iic interrupt
*
* @param	None.
*
* @return	None.
*
****************************************************************************/
void iic_mipi_interrupt_init(XScuGic *pInterruptController)
{
    int Status;
    XScuGic_Config *pxGICConfig;

    pxGICConfig = XScuGic_LookupConfig(XPAR_SCUGIC_SINGLE_DEVICE_ID);
    // no need to call XScuGic_CfgInitialize() since already called in FreeRTOS_tick_config.c
    // rising edge sensitive
    XScuGic_SetPriorityTriggerType(pInterruptController, 
        XPAR_FABRIC_AXI_IIC_1_IIC2INTC_IRPT_INTR, 150, 0b11);

    // interrupt handler
    Status = XScuGic_Connect(pInterruptController, XPAR_FABRIC_AXI_IIC_1_IIC2INTC_IRPT_INTR, 
        (Xil_ExceptionHandler) XIic_InterruptHandler, &iic);
    configASSERT(Status == XST_SUCCESS);
    (void) Status;

    // enable interrupts
    XScuGic_Enable(pInterruptController, XPAR_FABRIC_AXI_IIC_1_IIC2INTC_IRPT_INTR);
}


/***************************************************************************
* write iic
*
* @param  mipi      mipi interface to write to
* @param  reg       register address to write
* @param  buf       reference to buffer
*
* @return None.
*
****************************************************************************/
int32_t iic_mipi_write(mipi_interface *mipi, uint16_t reg, uint8_t *buf)
{
    u8 exp_buf[1];
    u8 WriteBuffer[sizeof(reg) + mipi->buffer_length];

    //set I2C expander
    XIic_SetAddress(&iic, XII_ADDR_TO_SEND_TYPE, IIC_EXPANDER_DEV_ADDR);
    // reset event handlers
    iic_handler_info.EventStatusUpdated = FALSE;
    iic_handler_info.SendBytesUpdated = FALSE;
    // wait until bus is free
    while(XIic_IsIicBusy(&iic)) {
        usleep(1);
    }
    // enable channel(s)
    exp_buf[0] = mipi->expander_channel;
    XIic_MasterSend(&iic, exp_buf, 1);
    // wait for transmission done (i.e. emulate polling)
    while(1) {
        if(iic_handler_info.SendBytesUpdated == TRUE) {
            break;
        }
        if (iic_handler_info.EventStatusUpdated == TRUE) {
            break;
        }
        usleep(1);
    }

    //do actual sending of message...................

    // reset event handlers
    iic_handler_info.EventStatusUpdated = FALSE;
    iic_handler_info.SendBytesUpdated = FALSE;

    // temporary address and data buffer
    WriteBuffer[0] = (reg >>  8) & 0xFF;
    WriteBuffer[1] = (reg      ) & 0xFF;
    for(int i=0; i<mipi->buffer_length; i++) {
        WriteBuffer[sizeof(reg) + i] = buf[i];
    }

    XIic_SetAddress(&iic, XII_ADDR_TO_SEND_TYPE, mipi->address);

    // wait until bus is free
    while(XIic_IsIicBusy(&iic)) {
        usleep(1);
    }
    // write to mipi interface
    XIic_MasterSend(&iic, WriteBuffer, sizeof(reg) + mipi->buffer_length);

    // wait for transmission done (i.e. emulate polling)
    while(1) {
        if(iic_handler_info.SendBytesUpdated == TRUE) {
            break;
        }
        if (iic_handler_info.EventStatusUpdated == TRUE) {
            break;
        }
        usleep(1);
    }

    return XST_SUCCESS;
}

/***************************************************************************
* read iic
*
* @param  mipi  mipi interface to read from
* @param  reg   register address to read
*
* @return read byte from IIC
*
****************************************************************************/
int32_t iic_mipi_read(mipi_interface *mipi, uint16_t reg, uint8_t *buf)
{
    uint8_t exp_buf[1];
    u8 WriteBuffer[sizeof(reg) + mipi->buffer_length];

    //set I2C expander
    XIic_SetAddress(&iic, XII_ADDR_TO_SEND_TYPE, IIC_EXPANDER_DEV_ADDR);
    // reset event handlers
    iic_handler_info.EventStatusUpdated = FALSE;
    iic_handler_info.SendBytesUpdated = FALSE;
    // wait until bus is free
    while(XIic_IsIicBusy(&iic)) {
        usleep(1);
    }
    // enable channel(s)
    exp_buf[0] = mipi->expander_channel;
    XIic_MasterSend(&iic, exp_buf, 1);
    // wait for transmission done (i.e. emulate polling)
    while(1) {
        if(iic_handler_info.SendBytesUpdated == TRUE) {
            break;
        }
        if (iic_handler_info.EventStatusUpdated == TRUE) {
            break;
        }
        usleep(1);
    }

    //do actual receiving of message...................

    // reset event handlers
    iic_handler_info.EventStatusUpdated = FALSE;
    iic_handler_info.SendBytesUpdated = FALSE;

    // temporary address buffer
    WriteBuffer[0] = reg >> 8;
    WriteBuffer[1] = reg & 0xFF;

    XIic_SetAddress(&iic, XII_ADDR_TO_SEND_TYPE, mipi->address);

    // wait until bus is free
    while(XIic_IsIicBusy(&iic)) {
        usleep(1);
    }
    if(XIic_MasterSend(&iic, WriteBuffer, 2)){
        xil_printf("XIic_MasterSend() failed\r\n");
    }

    // wait for transmission done (i.e. emulate polling)
    while(1) {
        if(iic_handler_info.SendBytesUpdated == TRUE) {
            break;
        }
        if (iic_handler_info.EventStatusUpdated == TRUE) {
            break;
        }
        usleep(1);
    }

    // reset event handlers
    iic_handler_info.EventStatusUpdated = FALSE;
    iic_handler_info.RecvBytesUpdated = FALSE;

    // wait until bus is free
    while(XIic_IsIicBusy(&iic)) {
        usleep(1);
    }
    if(XIic_MasterRecv(&iic, buf, mipi->buffer_length)){
        xil_printf("XIic_MasterRecv() failed\r\n");
    }

    // wait for transmission done (i.e. emulate polling)
    while(1) {
        if(iic_handler_info.RecvBytesUpdated == TRUE) {
            break;
        }
        if (iic_handler_info.EventStatusUpdated == TRUE) {
            break;
        }
        usleep(1);
    }

    return XST_SUCCESS;
}

/*****************************************************************************/
/**
* This receive handler is called asynchronously from an interrupt context and
* and indicates that data in the specified buffer was received. The byte count
* should equal the byte count of the buffer if all the buffer was filled.
*
* @param	CallBackRef is a pointer to the IIC device driver instance which
*		                the handler is being called for.
* @param	ByteCount indicates the number of bytes remaining to be received of
*		              the requested byte count. A value of zero indicates all requested
*		              bytes were received.
*
* @return	None.
*
* @notes	None.
*
****************************************************************************/
static void RecvHandler(void *CallbackRef, int ByteCount)
{
    iic_handler_info.RemainingRecvBytes = ByteCount;
    iic_handler_info.RecvBytesUpdated = TRUE;
}

/*****************************************************************************/
/**
* This send handler is called asynchronously from an interrupt context and
* and indicates that data in the specified buffer was sent.
*
* @param	CallBackRef is a pointer to the IIC device driver instance which
*		                the handler is being called for.
*
* @return	None.
*
* @notes	None.
*
****************************************************************************/
static void SendHandler(void *CallbackRef, int ByteCount)
{
    iic_handler_info.BytesSent = ByteCount;
    iic_handler_info.SendBytesUpdated = TRUE;
}

/*****************************************************************************/
/**
* This status handler is called asynchronously from an interrupt context and
* indicates that the conditions of the IIC bus changed. This  handler should
* not be called for the application unless an error occurs.
*
* @param	CallBackRef is a pointer to the IIC device driver instance which the
*		                handler is being called for.
* @param	Status contains the status of the IIC bus which changed.
*
* @return	None.
*
* @notes	None.
*
****************************************************************************/
static void StatusHandler(void *CallbackRef, int Status)
{
    iic_handler_info.EventStatus |= Status;
    iic_handler_info.EventStatusUpdated = TRUE;
}
