/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
 *
 * Copyright (C) 2011-2022 Bitcraze AB
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
 *
 */
#define DEBUG_MODULE "STAB"
//#define NO_REMOTE_TESTING 1 //if this is defined, we ignore that no remote is connected - DANGEROUS, only with no propellers mounted!

#include <math.h>
#include <stdio.h>

#include "xil_printf.h"

#include "FreeRTOS.h"
#include "task.h"

#include "config.h"
#include "motors.h"
// #include "pm.h"

#include "stabilizer.h"

#include "sensors.h"
#include "controller.h"
#include "power_distribution.h"
// #include "collision_avoidance.h"
// #include "health.h"
#include "supervisor.h"

#include "estimator.h"
// #include "usddeck.h"
#include "quatcompress.h"
#include "static_mem.h"
#include "rateSupervisor.h"
#include "usec_time.h"
#include "physicalConstants.h"
#include "setpoint.h"
#include "stabilizer_types.h"
#include "semphr.h"
#include "sbus.h"
#include "led_bank.h"
#include "system.h"
#include "data_recording.h"

#define DEBUG_HOLD_BACK_TIME_MS 500
#define LOG_HOLD_BACK_TIME_MS 10

static bool isInit;
static bool emergencyStop = false;
static int emergencyStopTimeout = EMERGENCY_STOP_TIMEOUT_DISABLED;

static uint32_t inToOutLatency;

// State variables for the stabilizer
static setpoint_t setpoint;
static sensorData_t sensorData;
static state_t state;
static control_t control;

static motors_thrust_uncapped_t motorThrustUncapped;
static motors_thrust_uncapped_t motorThrustBatCompUncapped;
static motors_thrust_pwm_t motorPwm;

// For scratch storage - never logged or passed to other subsystems.
static setpoint_t tempSetpoint;
// save a series of data for later debugging
#define LOG_SIZE_ENTRIES  2000
static setpoint_t SetpointLog[LOG_SIZE_ENTRIES];
static uint32_t logTime[LOG_SIZE_ENTRIES];
static state_t stateLog[LOG_SIZE_ENTRIES];
static motors_thrust_pwm_t motorPwmLog[LOG_SIZE_ENTRIES];
static sensorData_t sensorDataLog[LOG_SIZE_ENTRIES];

static StateEstimatorType estimatorType;
static ControllerType controllerType;

static rateSupervisor_t rateSupervisorContext;
static bool rateWarningDisplayed = false;

static struct {
  // position - mm
  int16_t x;
  int16_t y;
  int16_t z;
  // velocity - mm / sec
  int16_t vx;
  int16_t vy;
  int16_t vz;
  // acceleration - mm / sec^2
  int16_t ax;
  int16_t ay;
  int16_t az;
  // compressed quaternion, see quatcompress.h
  int32_t quat;
  // angular velocity - milliradians / sec
  int16_t rateRoll;
  int16_t ratePitch;
  int16_t rateYaw;
} stateCompressed;

static struct {
  // position - mm
  int16_t x;
  int16_t y;
  int16_t z;
  // velocity - mm / sec
  int16_t vx;
  int16_t vy;
  int16_t vz;
  // acceleration - mm / sec^2
  int16_t ax;
  int16_t ay;
  int16_t az;
} setpointCompressed;

/* protect estimator state which is accessed both by the stabilizer task and by IPI */
static SemaphoreHandle_t dataMutex;
static StaticSemaphore_t dataMutexBuffer;

STATIC_MEM_TASK_ALLOC(stabilizerTask, STABILIZER_TASK_STACKSIZE);

static void stabilizerTask(void* param);

static void calcSensorToOutputLatency(const sensorData_t *sensorData)
{
  uint64_t outTimestamp = usecTimestamp();
  inToOutLatency = outTimestamp - sensorData->interruptTimestamp;
}

static void compressState()
{
  stateCompressed.x = state.position.x * 1000.0f;
  stateCompressed.y = state.position.y * 1000.0f;
  stateCompressed.z = state.position.z * 1000.0f;

  stateCompressed.vx = state.velocity.x * 1000.0f;
  stateCompressed.vy = state.velocity.y * 1000.0f;
  stateCompressed.vz = state.velocity.z * 1000.0f;

  stateCompressed.ax = state.acc.x * 9.81f * 1000.0f;
  stateCompressed.ay = state.acc.y * 9.81f * 1000.0f;
  stateCompressed.az = (state.acc.z + 1) * 9.81f * 1000.0f;

  float const q[4] = {
    state.attitudeQuaternion.x,
    state.attitudeQuaternion.y,
    state.attitudeQuaternion.z,
    state.attitudeQuaternion.w};
  stateCompressed.quat = quatcompress(q);

  float const deg2millirad = ((float)M_PI * 1000.0f) / 180.0f;
  stateCompressed.rateRoll = sensorData.gyro.x * deg2millirad;
  stateCompressed.ratePitch = -sensorData.gyro.y * deg2millirad;
  stateCompressed.rateYaw = sensorData.gyro.z * deg2millirad;
}

static void compressSetpoint()
{
  setpointCompressed.x = setpoint.position.x * 1000.0f;
  setpointCompressed.y = setpoint.position.y * 1000.0f;
  setpointCompressed.z = setpoint.position.z * 1000.0f;

  setpointCompressed.vx = setpoint.velocity.x * 1000.0f;
  setpointCompressed.vy = setpoint.velocity.y * 1000.0f;
  setpointCompressed.vz = setpoint.velocity.z * 1000.0f;

  setpointCompressed.ax = setpoint.acceleration.x * 1000.0f;
  setpointCompressed.ay = setpoint.acceleration.y * 1000.0f;
  setpointCompressed.az = setpoint.acceleration.z * 1000.0f;
}

void stabilizerInit(StateEstimatorType estimator)
{
  if(isInit)
    return;

  sensorsInit();
  stateEstimatorInit(estimator);
  controllerInit(ControllerTypeAutoSelect);
  powerDistributionInit();
  initSetpoint(&setpoint); //here the controller mode is set (e.g. position hold, attitude hold, velocity...)
  motorsReset();
  // collisionAvoidanceInit();
  estimatorType = stateEstimatorGetType();
  controllerType = controllerGetType();

  dataMutex = xSemaphoreCreateMutexStatic(&dataMutexBuffer);

  STATIC_MEM_TASK_CREATE(stabilizerTask, stabilizerTask, STABILIZER_TASK_NAME, NULL, STABILIZER_TASK_PRI);

  isInit = true;
}

bool stabilizerTest(void)
{
  bool pass = true;

  pass &= sensorsTest();
  pass &= stateEstimatorTest();
  pass &= controllerTest();
  pass &= powerDistributionTest();

  return pass;
}

static void checkEmergencyStopTimeout()
{
  if (emergencyStopTimeout >= 0) {
    emergencyStopTimeout -= 1;

    if (emergencyStopTimeout == 0) {
      emergencyStop = true;
    }
  }
}

static void setMotorRatios(const motors_thrust_pwm_t* motorPwm)
{
  motorsSet(MOTOR_0, motorPwm->motors.m1);
  motorsSet(MOTOR_1, motorPwm->motors.m2);
  motorsSet(MOTOR_2, motorPwm->motors.m3);
  motorsSet(MOTOR_3, motorPwm->motors.m4);
}

/* The stabilizer loop runs at 999Hz. It is the
 * responsibility of the different functions to run slower by skipping call
 * (ie. returning without modifying the output structure).
 */
static void stabilizerTask(void* param)
{
  uint32_t setpointcounter = 0;
  uint32_t tick;
  uint32_t lastWakeTime;
  uint32_t nowMm_debug = T2M(xTaskGetTickCount());
  uint32_t warningBlockTimeMs_debug = T2M(xTaskGetTickCount());
  uint32_t warningBlockTimeMs_debug2 = T2M(xTaskGetTickCount());
  uint32_t LogTimeMs_debug = T2M(xTaskGetTickCount());
  vTaskSetApplicationTaskTag(0, (void*)TASK_STABILIZER_ID_NBR);

  //Wait for the system to be fully started to start stabilization loop
  systemWaitStart();
  led_on(0, 0x00, 0xFF, 0x00);
  xil_printf("stabilizerTask started\r\n");



  xil_printf("Wait for sensor calibration...\r\n");
  // Wait for sensors to be calibrated
  lastWakeTime = xTaskGetTickCount();
  while(!sensorsAreCalibrated()) {
    vTaskDelayUntil(&lastWakeTime, F2T(RATE_MAIN_LOOP));
  }

  // Initialize tick to something else then 0
  tick = 1;

#ifndef NO_REMOTE_TESTING
  // Wait for remote control
  xil_printf("Wait for remote control...\r\n");
  lastWakeTime = xTaskGetTickCount();
  while(!remoteControlIsActive()) {
    vTaskDelayUntil(&lastWakeTime, F2T(RATE_MAIN_LOOP));
  }
#else
  //fix setpoint:
  setpoint.thrust = 0.0;
  setpoint.attitude.roll = 0.0;
  setpoint.attitude.pitch = 0.0;
  setpoint.attitude.yaw = 0.0;
#endif

  motorsArm();

  rateSupervisorInit(&rateSupervisorContext, xTaskGetTickCount(), M2T(1000), 997, 1003, 1);

  xil_printf("\rReady to fly.\r\n");

  while(1) {
    // The sensor should unlock at roughly 866Hz
    sensorsWaitDataReady();

    // update sensorData struct (for logging variables)
    sensorsAcquire(&sensorData);

    // if (healthShallWeRunTest()) {
    //   healthRunTests(&sensorData);
    // } else {
      // allow to update estimator dynamically
      if (stateEstimatorGetType() != estimatorType) {
        stateEstimatorSwitchTo(estimatorType);
        estimatorType = stateEstimatorGetType();
      }
      // allow to update controller dynamically
      if (controllerGetType() != controllerType) {
        controllerInit(controllerType);
        controllerType = controllerGetType();
      }

      xSemaphoreTake(dataMutex, portMAX_DELAY);
      stateEstimator(&state, tick);
      compressState();
      xSemaphoreGive(dataMutex);   

      // if (crtpCommanderHighLevelGetSetpoint(&tempSetpoint, &state, tick)) {
      //   commanderSetSetpoint(&tempSetpoint, COMMANDER_PRIORITY_HIGHLEVEL);
      // }

#ifndef NO_REMOTE_TESTING
      getSetpoint(&setpoint);
#endif
      compressSetpoint();

      // collisionAvoidanceUpdateSetpoint(&setpoint, &sensorData, &state, tick);

      xSemaphoreTake(dataMutex, portMAX_DELAY);
      controller(&control, &setpoint, &sensorData, &state, tick);
      xSemaphoreGive(dataMutex);

      checkEmergencyStopTimeout();

      //
      // The supervisor module keeps track of Crazyflie state such as if
      // we are ok to fly, or if the Crazyflie is in flight.
      //
      supervisorUpdate(&sensorData);
      powerDistribution(&control, &motorThrustUncapped);
      powerDistributionCap(&motorThrustUncapped, &motorPwm);

      setMotorRatios(&motorPwm);

      calcSensorToOutputLatency(&sensorData);
      tick++;

      if (!rateSupervisorValidate(&rateSupervisorContext, xTaskGetTickCount())) {
        if (!rateWarningDisplayed) {
          xil_printf("WARNING: stabilizer loop rate is off (%lu)\r\n", rateSupervisorLatestCount(&rateSupervisorContext));
          rateWarningDisplayed = true;
        }
      }
    // }
#ifdef CONFIG_MOTORS_ESC_PROTOCOL_DSHOT
    motorsBurstDshot();
#endif
  }
}

void stabilizerSetEmergencyStop()
{
  emergencyStop = true;
}

void stabilizerResetEmergencyStop()
{
  emergencyStop = false;
}

void stabilizerSetEmergencyStopTimeout(int timeout)
{
  emergencyStop = false;
  emergencyStopTimeout = timeout;
}

void getEstimatorState(state_t *st)
{
  xSemaphoreTake(dataMutex, portMAX_DELAY);
  memcpy(st, &state, sizeof(state));
  xSemaphoreGive(dataMutex);
}
