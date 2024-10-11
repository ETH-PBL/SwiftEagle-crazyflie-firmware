/*
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * system.c - Top level module implementation
 */
#define DEBUG_MODULE "SYS"

#include <stdbool.h>
#include <stdio.h>

#include "xil_printf.h"

/* FreeRtos includes */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

//#include "debug.h"
//#include "version.h"
#include "config.h"
//#include "param.h"
//#include "log.h"
//#include "ledseq.h"
//#include "pm.h"

#include "system.h"
#include "platform.h"
//#include "storage.h"
//#include "configblock.h"
#include "worker.h"
//#include "freeRTOSdebug.h"
//#include "uart_syslink.h"
//#include "uart1.h"
//#include "uart2.h"
//#include "comm.h"
#include "stabilizer.h"
//#include "commander.h"
//#include "console.h"
//#include "usblink.h"
//#include "mem.h"
//#include "crtp_mem.h"
//#include "proximity.h"
//#include "watchdog.h"
//#include "queuemonitor.h"
//#include "buzzer.h"
//#include "sound.h"
//#include "sysload.h"
#include "estimator_kalman.h"
#include "estimator_ukf.h"
//#include "deck.h"
//#include "extrx.h"
//#include "app.h"
#include "static_mem.h"
//#include "peer_localization.h"
//#include "cfassert.h"
#include "i2cdev.h"
#include "autoconf.h"
//#include "vcp_esc_passthrough.h"
//#if CONFIG_ENABLE_CPX
//  #include "cpxlink.h"
//#endif
#include "data_recording.h"
#include "genx320.h"
#include "imx219.h"
#include "demosaic.h"
#include "vdma.h"
#include "gamma_lut.h"
#include "vtc.h"
#include "video_mixer.h"
#include "usb3320.h"
#include "ipi.h"


//debugging override
 #define DEBUG_PRINT(fmt, ...) xil_printf(fmt "\r", ##__VA_ARGS__);

/* Private variable */
bool sys_isInit;
static bool selftestPassed;
static uint8_t dumpAssertInfo = 0;

STATIC_MEM_TASK_ALLOC(systemTask, SYSTEM_TASK_STACKSIZE);

/* System wide synchronisation */
xSemaphoreHandle canStartMutex;
static StaticSemaphore_t canStartMutexBuffer;

/* Private functions */
static void systemTask(void *arg);

/* Public functions */
void systemLaunch(void)
{
  STATIC_MEM_TASK_CREATE(systemTask, systemTask, SYSTEM_TASK_NAME, NULL, SYSTEM_TASK_PRI);
}

// This must be the first module to be initialized!
void systemInit(void)
{
  if(sys_isInit){
    DEBUG_PRINT("systemInit() called twice!\n");
    return;
  }

  canStartMutex = xSemaphoreCreateMutexStatic(&canStartMutexBuffer);
  xSemaphoreTake(canStartMutex, portMAX_DELAY);

  DEBUG_PRINT("-----------------------------\n");
  DEBUG_PRINT("FPGA DRONE is up and running!\n");

  workerInit();
  usb3320_drive_cpen();
  vtc_init();
  vdma_init();
  gamma_lut_init();
  demosaic_init();  
  mixer_init();
  ipi_start();

  sys_isInit = true;
}

bool systemTest(void)
{
  bool pass = sys_isInit;

  //pass &= ledseqTest();
  //pass &= pmTest();
  pass &= workerTest();
  //pass &= buzzerTest();
  
  return pass;
}

/* Private functions implementation */

void systemTask(void *arg)
{
  bool pass = true;

  //usecTimerInit();
  i2cdevInit(I2C3_DEV);

  //Init the high-levels modules
  systemInit();
  //init the data recorder
  data_recording_init();
  //initializer mipi interface (i2c expander)
  iic_mipi_init();
  //initialize cameras
  imx219_init(IIC_EXPANDER_CHANNEL_0);
  imx219_init(IIC_EXPANDER_CHANNEL_3);
  //initialize event cameras
  genx320_init(IIC_EXPANDER_CHANNEL_1);
  genx320_init(IIC_EXPANDER_CHANNEL_2);
  //start event cameras
  genx320_open(IIC_EXPANDER_CHANNEL_1);
  genx320_open(IIC_EXPANDER_CHANNEL_2);
  genx320_on(IIC_EXPANDER_CHANNEL_1);
  genx320_on(IIC_EXPANDER_CHANNEL_2);

  StateEstimatorType estimator = StateEstimatorTypeComplementary; //StateEstimatorTypeKalman; //kalman filter drifts a lot without the flow deck don't use for now int he fpga drone...

  #ifdef CONFIG_ESTIMATOR_KALMAN_ENABLE
  estimatorKalmanTaskInit();
  #endif

  #ifdef CONFIG_ESTIMATOR_UKF_ENABLE
  errorEstimatorUkfTaskInit();
  #endif

  stabilizerInit(estimator);

  //Test the modules
  DEBUG_PRINT("About to run tests in system.c.\n");
  if (systemTest() == false) {
    pass = false;
    DEBUG_PRINT("system [FAIL]\n");
  }
  if (stabilizerTest() == false) {
    pass = false;
    DEBUG_PRINT("stabilizer [FAIL]\n");
  }

  #ifdef CONFIG_ESTIMATOR_KALMAN_ENABLE
  if (estimatorKalmanTaskTest() == false) {
    pass = false;
    DEBUG_PRINT("estimatorKalmanTask [FAIL]\n");
  }
  #endif

  #ifdef CONFIG_ESTIMATOR_UKF_ENABLE
  if (errorEstimatorUkfTaskTest() == false) {
    pass = false;
    DEBUG_PRINT("estimatorUKFTask [FAIL]\n");
  }
  #endif

  //Start the firmware
  if(pass)
  {
    DEBUG_PRINT("Self test passed!\n");
    selftestPassed = 1;
    systemStart();
  }
  else
  {
    selftestPassed = 0;
    if (systemTest())
    {
      while(1)
      {
        vTaskDelay(M2T(2000));
        // System can be forced to start by setting the param to 1 from the cfclient
        if (selftestPassed)
        {
	        DEBUG_PRINT("Start forced.\n");
          systemStart();
          break;
        }
      }
    }
    else
    {
      //nothing
    }
  }
  DEBUG_PRINT("Free heap: %d bytes\n", xPortGetFreeHeapSize());

  workerLoop();

  //Should never reach this point!
  while(1){
    vTaskDelay(portMAX_DELAY);
  }
}


/* Global system variables */
void systemStart()
{
  xSemaphoreGive(canStartMutex);
#ifndef DEBUG
  //watchdogInit();
#endif
}

void systemWaitStart(void)
{
  //This permits to guarantee that the system task is initialized before other
  //tasks waits for the start event.
  while(!sys_isInit)
    vTaskDelay(2);

  xSemaphoreTake(canStartMutex, portMAX_DELAY);
  xSemaphoreGive(canStartMutex);
}

/**
 * This parameter group contain read-only parameters pertaining to the CPU
 * in the Crazyflie.
 *
 * These could be used to identify an unique quad.
 */
//PARAM_GROUP_START(cpu)

/**
 * @brief Size in kB of the device flash memory
 */
//PARAM_ADD_CORE(PARAM_UINT16 | PARAM_RONLY, flash, MCU_FLASH_SIZE_ADDRESS)

/**
 * @brief Byte `0 - 3` of device unique id
 */
//PARAM_ADD_CORE(PARAM_UINT32 | PARAM_RONLY, id0, MCU_ID_ADDRESS+0)

/**
 * @brief Byte `4 - 7` of device unique id
 */
//PARAM_ADD_CORE(PARAM_UINT32 | PARAM_RONLY, id1, MCU_ID_ADDRESS+4)

/**
 * @brief Byte `8 - 11` of device unique id
 */
//PARAM_ADD_CORE(PARAM_UINT32 | PARAM_RONLY, id2, MCU_ID_ADDRESS+8)

//PARAM_GROUP_STOP(cpu)

//PARAM_GROUP_START(system)

/**
 * @brief All tests passed when booting
 */
//PARAM_ADD_CORE(PARAM_INT8 | PARAM_RONLY, selftestPassed, &selftestPassed)

/**
 * @brief Set to nonzero to trigger dump of assert information to the log.
 */
//PARAM_ADD(PARAM_UINT8, assertInfo, &dumpAssertInfo)

/**
 * @brief Test util for log and param. This param sets the value of the sys.testLogParam log variable.
 *
 */
//PARAM_ADD(PARAM_UINT8, testLogParam, &testLogParam)

/**
 * @brief Set to non-zero to trigger a failed assert, useful for debugging
 *
 */
//PARAM_ADD_WITH_CALLBACK(PARAM_UINT8, doAssert, &doAssert, doAssertCallback)


//PARAM_GROUP_STOP(system)

/**
 *  System loggable variables to check different system states.
 */
//LOG_GROUP_START(sys)
/**
 * @brief Test util for log and param. The value is set through the system.testLogParam parameter
 */
//LOG_ADD(LOG_INT8, testLogParam, &testLogParam)

//LOG_GROUP_STOP(sys)
