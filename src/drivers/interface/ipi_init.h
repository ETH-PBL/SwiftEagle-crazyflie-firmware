/******************************************************************************
 *
 * Copyright (C) 2017 Xilinx, Inc.  All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ******************************************************************************/

#ifndef __IPI_INIT_H__
#define __IPI_INIT_H__

#include "xscugic.h"

int ipi_init();
void ipi_cleanup();
int ipi_init_irq(XScuGic *pInterruptController);

#endif /* __IPI_INIT_H__ */
