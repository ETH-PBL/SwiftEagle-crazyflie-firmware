/**
 * Authored by Michael Hamer (http://www.mikehamer.info), June 2016
 * Thank you to Mark Mueller (www.mwm.im) for advice during implementation,
 * and for derivation of the original filter in the below-cited paper.
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
 * ============================================================================
 *
 * The Kalman filter implemented in this file is based on the papers:
 *
 * "Fusing ultra-wideband range measurements with accelerometers and rate gyroscopes for quadrocopter state estimation"
 * http://ieeexplore.ieee.org/xpl/articleDetails.jsp?arnumber=7139421
 *
 * and
 *
 * "Covariance Correction Step for Kalman Filtering with an Attitude"
 * http://arc.aiaa.org/doi/abs/10.2514/1.G000848
 *
 * Academic citation would be appreciated.
 *
 * BIBTEX ENTRIES:
      @INPROCEEDINGS{MuellerHamerUWB2015,
      author  = {Mueller, Mark W and Hamer, Michael and D’Andrea, Raffaello},
      title   = {Fusing ultra-wideband range measurements with accelerometers and rate gyroscopes for quadrocopter state estimation},
      booktitle = {2015 IEEE International Conference on Robotics and Automation (ICRA)},
      year    = {2015},
      month   = {May},
      pages   = {1730-1736},
      doi     = {10.1109/ICRA.2015.7139421},
      ISSN    = {1050-4729}}

      @ARTICLE{MuellerCovariance2016,
      author={Mueller, Mark W and Hehn, Markus and D’Andrea, Raffaello},
      title={Covariance Correction Step for Kalman Filtering with an Attitude},
      journal={Journal of Guidance, Control, and Dynamics},
      pages={1--7},
      year={2016},
      publisher={American Institute of Aeronautics and Astronautics}}
 *
 * ============================================================================
 *
 * MAJOR CHANGELOG:
 * 2016.06.28, Mike Hamer: Initial version
 * 2019.04.12, Kristoffer Richardsson: Refactored, separated kalman implementation from OS related functionality
 * 2021.03.15, Wolfgang Hoenig: Refactored queue handling
 */
#include <stdio.h>

#include "kalman_core.h"
#include "kalman_supervisor.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "semphr.h"
// #include "sensors.h"
#include "static_mem.h"
#include "config.h"
#include "xil_printf.h"

#include "estimator.h"
#include "estimator_kalman.h"
#include "physicalConstants.h"
#include "supervisor.h"
#include "axis3fSubSampler.h"
#include "system.h"
#include "rateSupervisor.h"

// Measurement models
#include "mm_distance.h"
#include "mm_absolute_height.h"
#include "mm_position.h"
#include "mm_pose.h"
#include "mm_tdoa.h"
#include "mm_flow.h"
#include "mm_tof.h"
#include "mm_yaw_error.h"
#include "mm_sweep_angles.h"

#include "mm_tdoa_robust.h"
#include "mm_distance_robust.h"

#define DEBUG_MODULE "ESTKALMAN"
#include "cfassert.h"


// #define KALMAN_USE_BARO_UPDATE


// Semaphore to signal that we got data from the stabilizer loop to process
static SemaphoreHandle_t runTaskSemaphore;

// Mutex to protect data that is shared between the task and
// functions called by the stabilizer loop
static SemaphoreHandle_t dataMutex;
static StaticSemaphore_t dataMutexBuffer;


/**
 * Tuning parameters
 */
#define PREDICT_RATE RATE_100_HZ // this is slower than the IMU update rate of 500Hz
// The bounds on the covariance, these shouldn't be hit, but sometimes are... why?
#define MAX_COVARIANCE (100)
#define MIN_COVARIANCE (1e-6f)

// Use the robust implementations of TWR and TDoA, off by default but can be turned on through a parameter.
// The robust implementations use around 10% more CPU VS the standard flavours
static bool robustTwr = false;
static bool robustTdoa = false;

/**
 * Quadrocopter State
 *
 * The internally-estimated state is:
 * - X, Y, Z: the quad's position in the global frame
 * - PX, PY, PZ: the quad's velocity in its body frame
 * - D0, D1, D2: attitude error
 *
 * For more information, refer to the paper
 */

NO_DMA_CCM_SAFE_ZERO_INIT static kalmanCoreData_t coreData;

/**
 * Internal variables. Note that static declaration results in default initialization (to 0)
 */

static bool isInit = false;

static Axis3fSubSampler_t accSubSampler;
static Axis3fSubSampler_t gyroSubSampler;
static Axis3f accLatest;
static Axis3f gyroLatest;

static OutlierFilterTdoaState_t outlierFilterTdoaState;
static OutlierFilterLhState_t sweepOutlierFilterState;


// Indicates that the internal state is corrupt and should be reset
bool resetEstimation = false;

static kalmanCoreParams_t coreParams;

// Data used to enable the task and stabilizer loop to run with minimal locking
static state_t taskEstimatorState; // The estimator state produced by the task, copied to the stabilizer when needed.

#define ONE_SECOND 1000
static rateSupervisor_t rateSupervisorContext;

#define WARNING_HOLD_BACK_TIME_MS 2000
static uint32_t warningBlockTimeMs = 0;

#ifdef KALMAN_USE_BARO_UPDATE
static const bool useBaroUpdate = true;
#else
static const bool useBaroUpdate = false;
#endif

static void kalmanTask(void* parameters);
static void updateQueuedMeasurements(const uint32_t nowMs, const bool quadIsFlying);

STATIC_MEM_TASK_ALLOC_STACK_NO_DMA_CCM_SAFE(kalmanTask, KALMAN_TASK_STACKSIZE);

// --------------------------------------------------

// Called one time during system startup
void estimatorKalmanTaskInit() {
  kalmanCoreDefaultParams(&coreParams);

  // Created in the 'empty' state, meaning the semaphore must first be given, that is it will block in the task
  // until released by the stabilizer loop
  runTaskSemaphore = xSemaphoreCreateBinary();
  ASSERT(runTaskSemaphore);

  dataMutex = xSemaphoreCreateMutexStatic(&dataMutexBuffer);

  STATIC_MEM_TASK_CREATE(kalmanTask, kalmanTask, KALMAN_TASK_NAME, NULL, KALMAN_TASK_PRI);

  isInit = true;
}

bool estimatorKalmanTaskTest() {
  return isInit;
}

static void kalmanTask(void* parameters) {
  systemWaitStart();
  
  uint32_t nowMs = T2M(xTaskGetTickCount());
  uint32_t nextPredictionMs = nowMs;

  rateSupervisorInit(&rateSupervisorContext, nowMs, ONE_SECOND, PREDICT_RATE - 1, PREDICT_RATE + 1, 1);

  while (true) {
    xSemaphoreTake(runTaskSemaphore, portMAX_DELAY);
    nowMs = T2M(xTaskGetTickCount()); // would be nice if this had a precision higher than 1ms...

    if (resetEstimation) {
      estimatorKalmanInit();
      resetEstimation = false;
    }

    bool quadIsFlying = supervisorIsFlying();

  #ifdef KALMAN_DECOUPLE_XY
    kalmanCoreDecoupleXY(&coreData);
  #endif

    // Run the system dynamics to predict the state forward.
    if (nowMs >= nextPredictionMs) {
      axis3fSubSamplerFinalize(&accSubSampler);
      axis3fSubSamplerFinalize(&gyroSubSampler);

      kalmanCorePredict(&coreData, &accSubSampler.subSample, &gyroSubSampler.subSample, nowMs, quadIsFlying);
      nextPredictionMs = nowMs + (1000.0f / PREDICT_RATE);

      if (!rateSupervisorValidate(&rateSupervisorContext, nowMs)) {
        xil_printf("WARNING: Kalman prediction rate low (%lu)\r\n", rateSupervisorLatestCount(&rateSupervisorContext));
      }
    }

    // Add process noise every loop, rather than every prediction
    kalmanCoreAddProcessNoise(&coreData, &coreParams, nowMs);

    updateQueuedMeasurements(nowMs, quadIsFlying);

    if (! kalmanSupervisorIsStateWithinBounds(&coreData)) {
      resetEstimation = true;

      if (nowMs > warningBlockTimeMs) {
        warningBlockTimeMs = nowMs + WARNING_HOLD_BACK_TIME_MS;
        xil_printf("State out of bounds, resetting\r\n");
      }
    }

    /**
     * Finally, the internal state is externalized.
     * This is done every round, since the external state includes some sensor data
     */
    xSemaphoreTake(dataMutex, portMAX_DELAY);
    kalmanCoreExternalizeState(&coreData, &taskEstimatorState, &accLatest);
    xSemaphoreGive(dataMutex);
  }
}

void estimatorKalman(state_t *state, const uint32_t tick) {
  // This function is called from the stabilizer loop. It is important that this call returns
  // as quickly as possible. The dataMutex must only be locked short periods by the task.
  xSemaphoreTake(dataMutex, portMAX_DELAY);

  // Copy the latest state, calculated by the task
  memcpy(state, &taskEstimatorState, sizeof(state_t));
  xSemaphoreGive(dataMutex);

  xSemaphoreGive(runTaskSemaphore);
}

static void updateQueuedMeasurements(const uint32_t nowMs, const bool quadIsFlying) {
  /**
   * Sensor measurements can come in sporadically and faster than the stabilizer loop frequency,
   * we therefore consume all measurements since the last loop, rather than accumulating
   */

  // Pull the latest sensors values of interest; discard the rest
  measurement_t m;
  while (estimatorDequeue(&m)) {
    switch (m.type) {
      case MeasurementTypeTDOA:
        if(robustTdoa){
          // robust KF update with TDOA measurements
          kalmanCoreRobustUpdateWithTdoa(&coreData, &m.data.tdoa, &outlierFilterTdoaState);
        }else{
          // standard KF update
          kalmanCoreUpdateWithTdoa(&coreData, &m.data.tdoa, nowMs, &outlierFilterTdoaState);
        }
        break;
      case MeasurementTypePosition:
        kalmanCoreUpdateWithPosition(&coreData, &m.data.position);
        break;
      case MeasurementTypePose:
        kalmanCoreUpdateWithPose(&coreData, &m.data.pose);
        break;
      case MeasurementTypeDistance:
        if(robustTwr){
            // robust KF update with UWB TWR measurements
            kalmanCoreRobustUpdateWithDistance(&coreData, &m.data.distance);
        }else{
            // standard KF update
            kalmanCoreUpdateWithDistance(&coreData, &m.data.distance);
        }
        break;
      case MeasurementTypeTOF:
        kalmanCoreUpdateWithTof(&coreData, &m.data.tof);
        break;
      case MeasurementTypeAbsoluteHeight:
        kalmanCoreUpdateWithAbsoluteHeight(&coreData, &m.data.height);
        break;
      case MeasurementTypeFlow:
        kalmanCoreUpdateWithFlow(&coreData, &m.data.flow, &gyroLatest);
        break;
      case MeasurementTypeYawError:
        kalmanCoreUpdateWithYawError(&coreData, &m.data.yawError);
        break;
      case MeasurementTypeSweepAngle:
        kalmanCoreUpdateWithSweepAngles(&coreData, &m.data.sweepAngle, nowMs, &sweepOutlierFilterState);
        break;
      case MeasurementTypeGyroscope:
        axis3fSubSamplerAccumulate(&gyroSubSampler, &m.data.gyroscope.gyro);
        gyroLatest = m.data.gyroscope.gyro;
        break;
      case MeasurementTypeAcceleration:
        axis3fSubSamplerAccumulate(&accSubSampler, &m.data.acceleration.acc);
        accLatest = m.data.acceleration.acc;
        break;
      case MeasurementTypeBarometer:
        if (useBaroUpdate) {
          kalmanCoreUpdateWithBaro(&coreData, &coreParams, m.data.barometer.baro.asl, quadIsFlying);
        }
        break;
      default:
        break;
    }
  }
}

// Called when this estimator is activated
void estimatorKalmanInit(void)
{
  axis3fSubSamplerInit(&accSubSampler, GRAVITY_MAGNITUDE);
  axis3fSubSamplerInit(&gyroSubSampler, DEG_TO_RAD);

  outlierFilterTdoaReset(&outlierFilterTdoaState);
  outlierFilterLighthouseReset(&sweepOutlierFilterState, 0);

  uint32_t nowMs = T2M(xTaskGetTickCount());
  kalmanCoreInit(&coreData, &coreParams, nowMs);
}

bool estimatorKalmanTest(void)
{
  return isInit;
}

void estimatorKalmanGetEstimatedPos(point_t* pos) {
  pos->x = coreData.S[KC_STATE_X];
  pos->y = coreData.S[KC_STATE_Y];
  pos->z = coreData.S[KC_STATE_Z];
}

void estimatorKalmanGetEstimatedRot(float * rotationMatrix) {
  memcpy(rotationMatrix, coreData.R, 9*sizeof(float));
}
