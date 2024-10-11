#include "xpseudo_asm_gcc.h"
#include "xreg_cortexr5.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "static_mem.h"
#include "xil_printf.h"

#define DEBUG_MODULE "ESTIMATOR"

#include "cfassert.h"
#include "estimator.h"
#include "estimator_complementary.h"
#include "estimator_kalman.h"
#include "estimator_ukf.h"
#include "quatcompress.h"

#define DEFAULT_ESTIMATOR StateEstimatorTypeComplementary
static StateEstimatorType currentEstimator = StateEstimatorTypeAutoSelect;


#define MEASUREMENTS_QUEUE_SIZE (20)
static xQueueHandle measurementsQueue;
STATIC_MEM_QUEUE_ALLOC(measurementsQueue, MEASUREMENTS_QUEUE_SIZE, sizeof(measurement_t));

static void initEstimator(const StateEstimatorType estimator);
static void deinitEstimator(const StateEstimatorType estimator);

typedef struct {
  void (*init)(void);
  void (*deinit)(void);
  bool (*test)(void);
  void (*update)(state_t *state, const uint32_t tick);
  const char* name;
} EstimatorFcns;

#define NOT_IMPLEMENTED ((void*)0)

static EstimatorFcns estimatorFunctions[] = {
    {
        .init = NOT_IMPLEMENTED,
        .deinit = NOT_IMPLEMENTED,
        .test = NOT_IMPLEMENTED,
        .update = NOT_IMPLEMENTED,
        .name = "None",
    }, // Any estimator
    {
        .init = estimatorComplementaryInit,
        .deinit = NOT_IMPLEMENTED,
        .test = estimatorComplementaryTest,
        .update = estimatorComplementary,
        .name = "Complementary",
    },
#ifdef CONFIG_ESTIMATOR_KALMAN_ENABLE
    {
        .init = estimatorKalmanInit,
        .deinit = NOT_IMPLEMENTED,
        .test = estimatorKalmanTest,
        .update = estimatorKalman,
        .name = "Kalman",
    },
#endif
#ifdef CONFIG_ESTIMATOR_UKF_ENABLE
    {
	    .init = errorEstimatorUkfInit,
	    .deinit = NOT_IMPLEMENTED,
	    .test = errorEstimatorUkfTest,
	    .update = errorEstimatorUkf,
	    .name = "Error State UKF",
	},
#endif
#ifdef CONFIG_ESTIMATOR_OOT
    {
        .init = estimatorOutOfTreeInit,
        .deinit = NOT_IMPLEMENTED,
        .test = estimatorOutOfTreeTest,
        .update = estimatorOutOfTree,
        .name = "OutOfTree",
    },
#endif
};

void stateEstimatorInit(StateEstimatorType estimator) {
  measurementsQueue = STATIC_MEM_QUEUE_CREATE(measurementsQueue);
  stateEstimatorSwitchTo(estimator);
}

void stateEstimatorSwitchTo(StateEstimatorType estimator) {
  if (estimator < 0 || estimator >= StateEstimatorType_COUNT) {
    return;
  }

  StateEstimatorType newEstimator = estimator;

  if (StateEstimatorTypeAutoSelect == newEstimator) {
    newEstimator = DEFAULT_ESTIMATOR;
  }

  #if defined(CONFIG_ESTIMATOR_KALMAN)
    #define ESTIMATOR StateEstimatorTypeKalman
  #elif defined(CONFIG_UKF_KALMAN)
    #define ESTIMATOR StateEstimatorTypeUkf
  #elif defined(CONFIG_ESTIMATOR_COMPLEMENTARY)
    #define ESTIMATOR StateEstimatorTypeComplementary
  #else
    #define ESTIMATOR StateEstimatorTypeAutoSelect
  #endif

  StateEstimatorType forcedEstimator = ESTIMATOR;
  if (forcedEstimator != StateEstimatorTypeAutoSelect) {
    xil_printf("Estimator type forced\n");
    newEstimator = forcedEstimator;
  }

  initEstimator(newEstimator);
  StateEstimatorType previousEstimator = currentEstimator;
  currentEstimator = newEstimator;
  deinitEstimator(previousEstimator);

  xil_printf("\r\nUsing %s (%d) estimator\r\n", stateEstimatorGetName(), currentEstimator);
}

StateEstimatorType stateEstimatorGetType(void) {
  return currentEstimator;
}

static void initEstimator(const StateEstimatorType estimator) {
  if (estimatorFunctions[estimator].init) {
    estimatorFunctions[estimator].init();
  }
}

static void deinitEstimator(const StateEstimatorType estimator) {
  if (estimatorFunctions[estimator].deinit) {
    estimatorFunctions[estimator].deinit();
  }
}

bool stateEstimatorTest(void) {
  return estimatorFunctions[currentEstimator].test();
}

void stateEstimator(state_t *state, const uint32_t tick) {
  estimatorFunctions[currentEstimator].update(state, tick);
}

const char* stateEstimatorGetName() {
  return estimatorFunctions[currentEstimator].name;
}


void estimatorEnqueue(const measurement_t *measurement) {
  if (!measurementsQueue) {
    return;
  }

  portBASE_TYPE result;

  u32 cpsr_mode = mfcpsr() & XREG_CPSR_MODE_BITS;
  bool isInInterrupt = (cpsr_mode == XREG_CPSR_IRQ_MODE) || (cpsr_mode == XREG_CPSR_FIQ_MODE);
  if (isInInterrupt) {
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
    result = xQueueSendFromISR(measurementsQueue, measurement, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken == pdTRUE) {
      portYIELD();
    }
  } else {
    result = xQueueSend(measurementsQueue, measurement, 0);
  }
}

bool estimatorDequeue(measurement_t *measurement) {
  return pdTRUE == xQueueReceive(measurementsQueue, measurement, 0);
}
