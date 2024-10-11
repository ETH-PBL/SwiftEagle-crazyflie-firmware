/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie 2.0 NRF Firmware
 * Copyright (c) 2014, Bitcraze AB, All rights reserved.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 3.0 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library.
 *
 * i2c_drv.c - i2c driver implementation
 *
 * @note
 * For some reason setting CR1 reg in sequence with
 * I2C_AcknowledgeConfig(I2C_SENSORS, ENABLE) and after
 * I2C_GenerateSTART(I2C_SENSORS, ENABLE) sometimes creates an
 * instant start->stop condition (3.9us long) which I found out with an I2C
 * analyzer. This fast start->stop is only possible to generate if both
 * start and stop flag is set in CR1 at the same time. So i tried setting the CR1
 * at once with I2C_SENSORS->CR1 = (I2C_CR1_START | I2C_CR1_ACK | I2C_CR1_PE) and the
 * problem is gone. Go figure...
 */

//adapted for Xilinx runtime by PBL

// Standard includes.
#include <string.h>
// Scheduler include files.
#include "FreeRTOS.h"
#include "xparameters.h"
#include "xiic.h"
#include "xil_exception.h"
#include "xinterrupt_wrap.h"
#include "sleep.h"
#include "xscugic.h"
#include "usec_time.h"

#include "task.h"
#include "queue.h"
#include "semphr.h"

// Application includes.
#include "i2c_drv.h"
#include "config.h"
//#include "nvicconf.h"
//#include "sleepus.h"

#include "autoconf.h"

#define IIC_MAX_WRITE_SIZE 16
#define IIC_BUSY_TIMEOUT_US     500
#define IIC_TRANSFER_TIMEOUT_US 500


//DEBUG
#ifdef I2CDRV_DEBUG_LOG_EVENTS
#include "usec_time.h"
#endif

// Definitions of sensors I2C bus
#define I2C_DEFAULT_SENSORS_CLOCK_SPEED             400000

// Definition of eeprom and deck I2C buss
#define I2C_DEFAULT_DECK_CLOCK_SPEED                400000

// Misc constants.
#define I2C_NO_BLOCK				    0
#define I2C_SLAVE_ADDRESS7      0x30
#define I2C_MAX_RETRIES         2
#define I2C_MESSAGE_TIMEOUT     M2T(1000)

XIic Iic;		  /* The instance of the IIC device */

static SemaphoreHandle_t i2c_dataMutex;
static StaticSemaphore_t i2c_dataMutexBuffer;

/*
 * The following structure contains fields that are used with the callbacks
 * (handlers) of the IIC driver. The driver asynchronously calls handlers
 * when abnormal events occur or when data has been sent or received. This
 * structure must be volatile to work when the code is optimized.
 */
volatile struct {
  int EventStatus;
  int RemainingRecvBytes;
  int EventStatusUpdated;
  int RecvBytesUpdated;
  int SendBytesUpdated;
  int BytesSent;
} iic_sens_HandlerInfo;

// TODO: fix up to xilinx settings...
// Cost definitions of busses
static const I2cDef sensorBusDef =
{
  .i2cDevice            = XPAR_IIC_0_DEVICE_ID,
};

I2cDrv sensorsBus =
{
  .def                = &sensorBusDef,
};

static const I2cDef deckBusDef =
{
  .i2cDevice            = XPAR_IIC_0_DEVICE_ID, //TODO: change once the hardware is on the fpga fabric
};


I2cDrv deckBus =
{
  .def                = &deckBusDef,
};


//private functions
static int32_t iic_write(void *handle, uint8_t reg, const uint8_t *bufp, uint32_t len);
static int32_t iic_read(void *handle, uint8_t reg, uint8_t *bufp, uint32_t len);
void iic_sens_RecvHandler(void *CallbackRef, int ByteCount);
void iic_sens_SendHandler(void *CallbackRef, int ByteCount);
void iic_sens_StatusHandler(void *CallbackRef, int Status);

//-----------------------------------------------------------

int32_t i2cdrvInterruptInit(XScuGic *pInterruptController){
  /* interrupt handler IIC */
  BaseType_t xStatus;
  XScuGic_Config *pxGICConfig;
  pxGICConfig = XScuGic_LookupConfig(XPAR_SCUGIC_SINGLE_DEVICE_ID);
  XScuGic_SetPriorityTriggerType(pInterruptController, XPAR_FABRIC_AXI_IIC_0_IIC2INTC_IRPT_INTR, 144, 0b11);
  xStatus = XScuGic_Connect(pInterruptController, XPAR_FABRIC_AXI_IIC_0_IIC2INTC_IRPT_INTR, (Xil_ExceptionHandler) XIic_InterruptHandler, &Iic);
  configASSERT(xStatus == XST_SUCCESS);
  (void) xStatus;
  XScuGic_Enable(pInterruptController, XPAR_FABRIC_AXI_IIC_0_IIC2INTC_IRPT_INTR);
  return xStatus;
}

int32_t i2cdrvInit(I2cDrv* i2c)
{
   int Status;
  XIic_Config *ConfigPtr;	/* Pointer to configuration data */
  BaseType_t xStatus;
  XScuGic_Config *pxGICConfig;

  //create semaphores for writing
  i2c_dataMutex = xSemaphoreCreateMutexStatic(&i2c_dataMutexBuffer);

  /*
   * Initialize the IIC driver so that it is ready to use.
   */
  ConfigPtr = XIic_LookupConfig(XPAR_IIC_0_DEVICE_ID);
  if (ConfigPtr == NULL) {
    return XST_FAILURE;
  }

  Status = XIic_CfgInitialize(&Iic, ConfigPtr, ConfigPtr->BaseAddress);
  if (Status != XST_SUCCESS) {
    return XST_FAILURE;
  }

  if (XIic_SelfTest(&Iic) != XST_SUCCESS) {
		xil_printf("XIic_SelfTest() failed\r\n");
		return XST_FAILURE;
	}


  XIic_SetRecvHandler(&Iic, (void *)&iic_sens_HandlerInfo, iic_sens_RecvHandler);
  XIic_SetSendHandler(&Iic, (void *)&iic_sens_HandlerInfo, iic_sens_SendHandler);
  XIic_SetStatusHandler(&Iic, (void *)&iic_sens_HandlerInfo, iic_sens_StatusHandler);

  //Status = XIic_SetAddress(&Iic, XII_ADDR_TO_SEND_TYPE, LSM6DSM_I2C_ADD_H>>1);
  //if (Status != XST_SUCCESS) {
  //  return XST_FAILURE;
  //}
  Status = XIic_Start(&Iic);
  if (Status != XST_SUCCESS) {
		xil_printf("XIic_Start() failed\r\n");
    return XST_FAILURE;
  }

  return Status;
}

void i2cdrvCreateMessage(I2cMessage *message,
                      uint8_t  slaveAddress,
                      I2cDirection  direction,
                      uint32_t length,
                      const uint8_t *buffer)
{
  //ASSERT_DMA_SAFE(buffer);

  message->slaveAddress = slaveAddress;
  message->direction = direction;
  message->isInternal16bit = false;
  message->internalAddress = I2C_NO_INTERNAL_ADDRESS;
  message->messageLength = length;
  message->status = i2cAck;
  message->buffer = (uint8_t *)buffer;
  message->nbrOfRetries = I2C_MAX_RETRIES;
}

void i2cdrvCreateMessageIntAddr(I2cMessage *message,
                             uint8_t  slaveAddress,
                             bool IsInternal16,
                             uint16_t intAddress,
                             I2cDirection  direction,
                             uint32_t length,
                             const uint8_t  *buffer)
{
  //ASSERT_DMA_SAFE(buffer);

  message->slaveAddress = slaveAddress; //
  message->direction = direction; //
  message->isInternal16bit = IsInternal16;
  message->internalAddress = intAddress;
  message->messageLength = length;
  message->status = i2cAck;
  message->buffer = (uint8_t *)buffer;
  message->nbrOfRetries = I2C_MAX_RETRIES;
}

bool i2cdrvMessageTransfer(I2cDrv* i2c, I2cMessage* message)
{
  int xStatus;
  bool ret = false;

  xSemaphoreTake(i2c_dataMutex, portMAX_DELAY); // Protect message data

  //set the device address
  xStatus = XIic_SetAddress(&Iic, XII_ADDR_TO_SEND_TYPE, (message->slaveAddress));
  if (xStatus != XST_SUCCESS) {
    return XST_FAILURE;
  }
  if (message->isInternal16bit == TRUE){
    xil_printf("..ha..ha.. missing 16bit I2C addresses..");
  }

  if (message->direction == i2cWrite){
    //TODO: remove internal address typecast and do properly
    xStatus += iic_write(i2c, (uint8_t) message->internalAddress, message->buffer, message->messageLength);
  }

  if (message->direction == i2cRead){
    //TODO: remove internal address typecast and do properly
    xStatus += iic_read(i2c, (uint8_t) message->internalAddress, message->buffer, message->messageLength);
  }
  xSemaphoreGive(i2c_dataMutex);

  if (xStatus == XST_SUCCESS){
    ret = true;
  }

  return ret;
}

/*
 * @brief  Write iic register
 *
 * @param  handle    customizable argument.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
static int32_t iic_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint32_t len)
{
  int Status;
  handle = handle;
  u8 WriteBuffer[sizeof(reg) + IIC_MAX_WRITE_SIZE];
  uint32_t Index;
  int timeout = 0;

  /* 
   * A temporary write buffer must be used which contains both the
   * register address and the data to be written
   */
  WriteBuffer[0] = (u8)(reg);
  for (Index = 0; Index < len; Index++) {
    WriteBuffer[sizeof(reg) + Index] = bufp[Index];
  }

  iic_sens_HandlerInfo.EventStatusUpdated = FALSE;
  iic_sens_HandlerInfo.SendBytesUpdated = FALSE;
  iic_sens_HandlerInfo.EventStatus = 0;
  Status = XST_FAILURE;

  /*
   * Send register address and buffer as soon bus is not busy
   */
  while(XIic_IsIicBusy(&Iic)) {
    usleep(1);
    timeout++;
    if (timeout > IIC_BUSY_TIMEOUT_US) {
      xil_printf("ERROR: I2C bus access timeout\r\n");
      return XST_FAILURE;
    }
  }
  Status = XIic_MasterSend(&Iic, WriteBuffer, 1 + len);
  if (Status != XST_SUCCESS) {
      xil_printf("ERROR: I2C bus send failed\r\n");
    return Status;
  }

  timeout = 0;
  while(1) {
    if(iic_sens_HandlerInfo.SendBytesUpdated == TRUE) {
      Status = XST_SUCCESS;
      break;
    }
    if (iic_sens_HandlerInfo.EventStatusUpdated == TRUE) {
      break;
    }
    usleep(1);
    timeout++;
    if (timeout > IIC_TRANSFER_TIMEOUT_US) {
      xil_printf("ERROR: I2C bus TX timeout\r\n");
      return XST_FAILURE;
    }
  }

  return Status;
}

/*
 * @brief  Read iic register
 *
 * @param  handle    customizable argument.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t iic_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint32_t len)
{
  int Status;
  handle = handle;
  int timeout = 0;

  iic_sens_HandlerInfo.EventStatusUpdated = FALSE;
  iic_sens_HandlerInfo.SendBytesUpdated = FALSE;
  iic_sens_HandlerInfo.EventStatus = 0;
  Status = XST_FAILURE;


  /*
   * Send register address as soon bus is not busy
   */
  while(XIic_IsIicBusy(&Iic)) {
    usleep(1);
    timeout++;
    if (timeout > IIC_BUSY_TIMEOUT_US) {
      xil_printf("ERROR: I2C bus write access timeout\r\n");
      return XST_FAILURE;
    }
  }
  Status = XIic_MasterSend(&Iic, &reg, 1);
  if (Status != XST_SUCCESS) {
    xil_printf("ERROR: I2C bus send failed\r\n");
    return Status;
  }

  timeout = 0;
  while(1) {
    if(iic_sens_HandlerInfo.SendBytesUpdated == TRUE) {
      Status = XST_SUCCESS;
      break;
    }
    if (iic_sens_HandlerInfo.EventStatusUpdated == TRUE) {
      break;
    }
    usleep(1);
    timeout++;
    if (timeout > IIC_TRANSFER_TIMEOUT_US) {
      xil_printf("ERROR: I2C bus TX timeout\r\n");
      return XST_FAILURE;
    }
  }

  iic_sens_HandlerInfo.EventStatusUpdated = FALSE;
  iic_sens_HandlerInfo.RecvBytesUpdated = FALSE;
  iic_sens_HandlerInfo.EventStatus = 0;

  /*
   * Receive buffer as soon bus is not busy
   */
  timeout = 0;
  while(XIic_IsIicBusy(&Iic)) {
    usleep(1);
    timeout++;
    if (timeout > IIC_BUSY_TIMEOUT_US) {
      xil_printf("ERROR: I2C bus read access timeout\r\n");
      return XST_FAILURE;
    }
  }
  Status = XIic_MasterRecv(&Iic, bufp, len);
  if (Status != XST_SUCCESS) {
    xil_printf("ERROR: I2C bus receive failed\r\n");
    return Status;
  }

  timeout = 0;
  while(1) {
    if(iic_sens_HandlerInfo.RecvBytesUpdated == TRUE) {

      if (iic_sens_HandlerInfo.RemainingRecvBytes == 0) {
        Status = XST_SUCCESS;
      }
      break;
    }
    if (iic_sens_HandlerInfo.EventStatusUpdated == TRUE) {
      break;
    }
    usleep(1);
    timeout++;
    if (timeout > IIC_TRANSFER_TIMEOUT_US) {
      return XST_FAILURE;
      xil_printf("ERROR: I2C bus RX timeout\r\n");
    }
  }

  return Status;
}

/*****************************************************************************/
/**
* This receive handler is called asynchronously from an interrupt context and
* and indicates that data in the specified buffer was received. The byte count
* should equal the byte count of the buffer if all the buffer was filled.
*
* @param	CallBackRef is a pointer to the IIC device driver instance which
*		the handler is being called for.
* @param	ByteCount indicates the number of bytes remaining to be received of
*		the requested byte count. A value of zero indicates all requested
*		bytes were received.
*
* @return	None.
*
* @notes	None.
*
****************************************************************************/
void iic_sens_RecvHandler(void *CallbackRef, int ByteCount)
{
  iic_sens_HandlerInfo.RemainingRecvBytes = ByteCount;
  iic_sens_HandlerInfo.RecvBytesUpdated = TRUE;
}

/*****************************************************************************/
/**
* This send handler is called asynchronously from an interrupt context and
* and indicates that data in the specified buffer was sent.
*
* @param	CallBackRef is a pointer to the IIC device driver instance which
*		the handler is being called for.
*
* @return	None.
*
* @notes	None.
*
****************************************************************************/
void iic_sens_SendHandler(void *CallbackRef, int ByteCount)
{
  iic_sens_HandlerInfo.BytesSent = ByteCount;
  iic_sens_HandlerInfo.SendBytesUpdated = TRUE;
}

/*****************************************************************************/
/**
* This status handler is called asynchronously from an interrupt context and
* indicates that the conditions of the IIC bus changed. This  handler should
* not be called for the application unless an error occurs.
*
* @param	CallBackRef is a pointer to the IIC device driver instance which the
*		handler is being called for.
* @param	Status contains the status of the IIC bus which changed.
*
* @return	None.
*
* @notes	None.
*
****************************************************************************/
void iic_sens_StatusHandler(void *CallbackRef, int Status)
{
  iic_sens_HandlerInfo.EventStatus |= Status;
  iic_sens_HandlerInfo.EventStatusUpdated = TRUE;
}