 /*
 * Copyright (c) 2017, Xilinx Inc. and Contributors. All rights reserved.
 * Copyright (C) 2022, Advanced Micro Devices, Inc.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __IPI_H__
#define __IPI_H__

#include <unistd.h>
#include <stdio.h>
#include <string.h>

#include "stabilizer_types.h"
#include <metal/atomic.h>
#include <metal/alloc.h>
#include <metal/irq.h>
#include <metal/errno.h>
#include <metal/sys.h>
#include <metal/cpu.h>
#include <metal/io.h>
#include <metal/device.h>
#include <sys/types.h>
#include <xil_printf.h>
#include "ipi_init.h"

/* Devices names */
#define BUS_NAME        "generic"
#define IPI_DEV_NAME    "ff310000.ipi"
#define SHM_DEV_NAME    "29000000.shm"

/* IPI registers offset */
#define IPI_TRIG_OFFSET 0x0  /* IPI trigger reg offset */
#define IPI_OBS_OFFSET  0x4  /* IPI observation reg offset */
#define IPI_ISR_OFFSET  0x10 /* IPI interrupt status reg offset */
#define IPI_IMR_OFFSET  0x14 /* IPI interrupt mask reg offset */
#define IPI_IER_OFFSET  0x18 /* IPI interrupt enable reg offset */
#define IPI_IDR_OFFSET  0x1C /* IPI interrupt disable reg offset */

#define IPI_MASK        0x1000000 /* IPI mask for kick to APU. */

/* shared memory offsets */
#define SHM_CTRL_REG_OFFSET         0
#define SHM_ESTIMATOR_STATE_OFFSET  4
#define SHM_SETPOINT_OFFSET         84

/* shared memory control register bits */
#define SHM_CONTROL_REG_GET_STATE       0x01
#define SHM_CONTROL_REG_SET_SETPOINT    0x02

extern struct metal_device *ipi_dev; /* IPI metal device */
extern struct metal_device *shm_dev; /* SHM metal device */

typedef struct setpoint_ipi_s {
  float thrust;
  float roll;
  float pitch;
  float yaw;
} setpoint_ipi_t;

static inline void wait_for_interrupt()
{
    metal_asm volatile("wfi");
}

/**
 * @brief wait_for_notified() - Loop until notified bit
 *        in channel is set.
 *
 * @param[in] notified - pointer to the notified variable
 */
static inline void  wait_for_notified(atomic_flag *notified)
{
    unsigned int flags;

    do {
        flags = metal_irq_save_disable();
        if (!atomic_flag_test_and_set(notified)) {
            metal_irq_restore_enable(flags);
            break;
        }
        wait_for_interrupt();
        metal_irq_restore_enable(flags);
    } while(1);
}

void ipi_getSetpoint(setpoint_t *sp);
int ipi_start(void);
int ipi_stop(void);

#endif /* __IPI_H__ */

