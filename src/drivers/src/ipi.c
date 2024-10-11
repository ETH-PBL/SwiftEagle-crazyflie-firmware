/*
 * Copyright (c) 2017, Xilinx Inc. and Contributors. All rights reserved.
 * Copyright (C) 2022, Advanced Micro Devices, Inc.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*****************************************************************************
 * ipi.c
 * 
 * 
 */

#include <unistd.h>
#include <metal/atomic.h>
#include <metal/io.h>
#include <metal/device.h>
#include <metal/irq.h>
#include "FreeRTOS.h"
#include "task.h"
#include "static_mem.h"
#include "config.h"
#include "stabilizer_types.h"
#include "stabilizer.h"
#include "ipi.h"


struct channel_s {
    struct metal_io_region *ipi_io; /* IPI metal i/o region */
    struct metal_io_region *shm_io; /* Shared memory metal i/o region */
    uint32_t ipi_mask; /* RPU IPI mask */
    atomic_flag remote_nkicked; /* 0 - kicked from remote */
};

struct channel_s ch;
setpoint_ipi_t setpoint;

STATIC_MEM_TASK_ALLOC(ipiTask, IPI_TASK_STACKSIZE);


/**
 * @brief ipi_irq_handler() - IPI interrupt handler
 *        It will clear the notified flag to mark it's got an IPI interrupt.
 *
 * @param[in] vect_id - IPI interrupt vector ID
 * @param[in/out] priv - communication channel data for this application.
 *
 * @return - If the IPI interrupt is triggered by its remote, it returns
 *           METAL_IRQ_HANDLED. It returns METAL_IRQ_NOT_HANDLED, if it is
 *           not the interrupt it expected.
 *
 */
static int ipi_irq_handler (int vect_id, void *priv)
{
    struct channel_s *ch = (struct channel_s *)priv;
    uint32_t val;

    (void)vect_id;

    if (ch) {
        val = metal_io_read32(ch->ipi_io, IPI_ISR_OFFSET);
        if (val & ch->ipi_mask) {
            metal_io_write32(ch->ipi_io, IPI_ISR_OFFSET,
                ch->ipi_mask);
            atomic_flag_clear(&ch->remote_nkicked);
            return METAL_IRQ_HANDLED;
        }
    }
    return METAL_IRQ_NOT_HANDLED;
}


static void ipiTask(void *param)
{
    state_t estState;
    uint32_t ctrl;

    while(1) {
        /* wait for IPI */
        wait_for_notified(&ch.remote_nkicked);

        ctrl = metal_io_read32(ch.shm_io, SHM_CTRL_REG_OFFSET);
        if (ctrl & SHM_CONTROL_REG_GET_STATE) {
            getEstimatorState(&estState);
            /* write to shared memory */
            metal_io_block_write(ch.shm_io, SHM_ESTIMATOR_STATE_OFFSET, 
                &estState, sizeof(estState));
        }
        if (ctrl & SHM_CONTROL_REG_SET_SETPOINT) {
            /* read from shared memory */
            metal_io_block_read(ch.shm_io, SHM_SETPOINT_OFFSET,
                &setpoint, sizeof(setpoint));
            /* clear control bit */
            metal_io_write32(ch.shm_io, SHM_CTRL_REG_OFFSET, ctrl & ~SHM_CONTROL_REG_GET_STATE);
        }
        /* Kick IPI to notify the remote */
        metal_io_write32(ch.ipi_io, IPI_TRIG_OFFSET, ch.ipi_mask);
    }
}


void ipi_getSetpoint(setpoint_t *sp)
{
    sp->thrust = setpoint.thrust;
    sp->attitude.roll = setpoint.roll;
    sp->attitude.pitch = setpoint.pitch;
    sp->attitude.yaw = setpoint.yaw;
}


int ipi_start(void)
{
    int ipi_irq;
    int ret = 0;

    ret = ipi_init();
    if (ret) {
        xil_printf("error: failed to initialize ipi.\r\n");
        ret = -ENODEV;
        goto out;
    }

    memset(&ch, 0, sizeof(ch));

    /* Get shared memory device IO region */
    if (!shm_dev) {
        ret = -ENODEV;
        goto out;
    }
    ch.shm_io = metal_device_io_region(shm_dev, 0);
    if (!ch.shm_io) {
        xil_printf("error: failed to map io region for %s.\r\n", shm_dev->name);
        ret = -ENODEV;
        goto out;
    }

    /* Get IPI device IO region */
    ch.ipi_io = metal_device_io_region(ipi_dev, 0);
    if (!ch.ipi_io) {
        xil_printf("error: failed to map io region for %s.\r\n", ipi_dev->name);
        ret = -ENODEV;
        goto out;
    }

    /* disable IPI interrupt */
    metal_io_write32(ch.ipi_io, IPI_IDR_OFFSET, IPI_MASK);
    /* clear old IPI interrupt */
    metal_io_write32(ch.ipi_io, IPI_ISR_OFFSET, IPI_MASK);

    ch.ipi_mask = IPI_MASK;

    /* Get the IPI IRQ from the opened IPI device */
    ipi_irq = (intptr_t)ipi_dev->irq_info;

    /* Register IPI irq handler */
    ret = metal_irq_register(ipi_irq, ipi_irq_handler, &ch);
    if (ret) {
        xil_printf("error: failed to register ipi irq handler.\r\n");
        ret = -ENODEV;
        goto out;
    }
    metal_irq_enable(ipi_irq);
    /* initialize remote_nkicked */
    ch.remote_nkicked = (atomic_flag)ATOMIC_FLAG_INIT;
    atomic_flag_test_and_set(&ch.remote_nkicked);
    /* Enable IPI interrupt */
    metal_io_write32(ch.ipi_io, IPI_IER_OFFSET, IPI_MASK);

    STATIC_MEM_TASK_CREATE(ipiTask, ipiTask, IPI_TASK_NAME, NULL, IPI_TASK_PRI);

    return 0;
out:
    ipi_cleanup();
    return ret;
}


int ipi_stop(void)
{
    int ipi_irq;

    /* Get the IPI IRQ from the opened IPI device */
    ipi_irq = (intptr_t)ipi_dev->irq_info;

    /* disable IPI interrupt */
    metal_io_write32(ch.ipi_io, IPI_IDR_OFFSET, IPI_MASK);
    /* unregister IPI irq handler */
    metal_irq_disable(ipi_irq);
    metal_irq_unregister(ipi_irq);
    ipi_cleanup();

    return 0;
}

