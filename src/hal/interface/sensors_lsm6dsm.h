/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2018 Bitcraze AB
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
 */

#ifndef __SENSORS_LSM6DSM_H__
#define __SENSORS_LSM6DSM_H__

#include "sensors.h"
#include "xscugic.h"
#include "FreeRTOS.h"
#include "semphr.h"

#define IMU_LOGGING_DESTINATION_ADDR 0x50000000

extern xSemaphoreHandle startImuRecording;
extern xSemaphoreHandle stopImuRecording;
extern xSemaphoreHandle imuRecordingFinished;

void sensorsLSM6DSMInit_I2C(void);
bool sensorsLSM6DSMTest(void);
bool sensorsLSM6DSMAreCalibrated(void);
void sensorsLSM6DSMAcquire(sensorData_t *sensors, const uint32_t tick);
void sensorsLSM6DSMWaitDataReady(void);
bool sensorsLSM6DSMReadGyro(Axis3f *gyro);
bool sensorsLSM6DSMReadAcc(Axis3f *acc);
void sensorsLSM6DSMDataAvailableCallback(void);

void sensorsDeviceInit(void);
void sensorsInterruptInit(XScuGic *pInterruptController);
void sensorsWriteFlash(void);

#endif // __SENSORS_LSM6DSM_H__
