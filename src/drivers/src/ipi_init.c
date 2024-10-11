/******************************************************************************
 *
 * Copyright (C) 2017 Xilinx, Inc.  All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ******************************************************************************/

#include <xparameters.h>
#include <xil_cache.h>
#include <xil_exception.h>
#include <xstatus.h>
#include <xscugic.h>
#include <xreg_cortexr5.h>

#include <metal/io.h>
#include <metal/device.h>
#include <metal/sys.h>
#include <metal/irq.h>

#include "platform_config.h"
#include "platform.h"
#include "ipi.h"


#define INTC_DEVICE_ID		XPAR_SCUGIC_0_DEVICE_ID

#define IPI_IRQ_VECT_ID         65

#define SHM_BASE_ADDR   0x29000000
#define IPI_BASE_ADDR   0xFF310000

/* Default generic I/O region page shift */
/* Each I/O region can contain multiple pages.
 * In baremetal system, the memory mapping is flat, there is no
 * virtual memory.
 * We can assume there is only one page in the whole baremetal system.
 */
#define DEFAULT_PAGE_SHIFT (-1UL)
#define DEFAULT_PAGE_MASK  (-1UL)

static XScuGic xInterruptController;

const metal_phys_addr_t metal_phys[] = {
	IPI_BASE_ADDR, /**< base IPI address */
	SHM_BASE_ADDR, /**< shared memory base address */
};

/* Define metal devices table for IPI and shared memory.
 * Linux system uses device tree to describe devices. Unlike Linux,
 * there is no standard device abstraction for baremetal system, we
 * uses libmetal devices structure to describe the devices we used in
 * the example.
 * The IPI and shared memory devices are memory mapped
 * devices. For this type of devices, it is required to provide
 * accessible memory mapped regions, and interrupt information.
 * In baremetal system, the memory mapping is flat. As you can see
 * in the table before, we set the virtual address "virt" the same
 * as the physical address.
 */
static struct metal_device metal_dev_table[] = {
	{
		/* IPI device */
		.name = IPI_DEV_NAME,
		.bus = NULL,
		.num_regions = 1,
		.regions = {
			{
				.virt = (void *)IPI_BASE_ADDR,
				.physmap = &metal_phys[0],
				.size = 0x1000,
				.page_shift = DEFAULT_PAGE_SHIFT,
				.page_mask = DEFAULT_PAGE_MASK,
				.mem_flags = DEVICE_NONSHARED | PRIV_RW_USER_RW,
				.ops = {NULL},
			}
		},
		.node = {NULL},
		.irq_num = 1,
		.irq_info = (void *)IPI_IRQ_VECT_ID,
	},
	{
		/* Shared memory management device */
		.name = SHM_DEV_NAME,
		.bus = NULL,
		.num_regions = 1,
		.regions = {
			{
				.virt = (void *)SHM_BASE_ADDR,
				.physmap = &metal_phys[1],
				.size = 0x1000000,
				.page_shift = DEFAULT_PAGE_SHIFT,
				.page_mask = DEFAULT_PAGE_MASK,
				.mem_flags = NORM_SHARED_NCACHE |
						PRIV_RW_USER_RW,
				.ops = {NULL},
			}
		},
		.node = {NULL},
		.irq_num = 0,
		.irq_info = NULL,
	},
};

/**
 * Extern global variables
 */
struct metal_device *ipi_dev = NULL;
struct metal_device *shm_dev = NULL;


/**
 * @brief init_irq() - Initialize and connect IPI interrupt
 *
 * @return 0 - succeeded, non-0 for failures
 */
int ipi_init_irq(XScuGic *pInterruptController)
{
	int ret = 0;

	/* Connect IPI Interrupt ID with libmetal ISR */
	XScuGic_Connect(pInterruptController, IPI_IRQ_VECT_ID,
			   (Xil_ExceptionHandler)metal_xlnx_irq_isr,
			   (void *)IPI_IRQ_VECT_ID);

	XScuGic_Enable(pInterruptController, IPI_IRQ_VECT_ID);

	return 0;
}

/**
 * @brief platform_register_metal_device() - Statically Register libmetal
 *        devices.
 *        This function registers the IPI and shared memory to the libmetal generic bus.
 *        Libmetal uses bus structure to group the devices. Before you can
 *        access the device with libmetal device operation, you will need to
 *        register the device to a libmetal supported bus.
 *        For non-Linux system, libmetal only supports "generic" bus, which is
 *        used to manage the memory mapped devices.
 *
 * @return 0 - succeeded, non-zero for failures.
 */
static int platform_register_metal_device(void)
{
	unsigned int i;
	int ret;
	struct metal_device *dev;

	for (i = 0; i < sizeof(metal_dev_table)/sizeof(struct metal_device);
	     i++) {
		dev = &metal_dev_table[i];
		xil_printf("registering: %d, name=%s\r\n", i, dev->name);
		ret = metal_register_generic_device(dev);
		if (ret)
			return ret;
	}
	return 0;
}

/**
 * @brief open_metal_devices() - Open registered libmetal devices.
 *        This function opens all the registered libmetal devices.
 *
 * @return 0 - succeeded, non-zero for failures.
 */
static int open_metal_devices(void)
{
	int ret;

	/* Open shared memory device */
	ret = metal_device_open(BUS_NAME, SHM_DEV_NAME, &shm_dev);
	if (ret) {
		xil_printf("Failed to open device %s.\r\n", SHM_DEV_NAME);
		goto out;
	}

	/* Open IPI device */
	ret = metal_device_open(BUS_NAME, IPI_DEV_NAME, &ipi_dev);
	if (ret) {
		xil_printf("Failed to open device %s.\r\n", IPI_DEV_NAME);
		goto out;
	}

out:
	return ret;
}

/**
 * @brief close_metal_devices() - close libmetal devices
 *        This function closes all the libmetal devices which have
 *        been opened.
 *
 */
static void close_metal_devices(void)
{
	/* Close shared memory device */
	if (shm_dev)
		metal_device_close(shm_dev);

	/* Close IPI device */
	if (ipi_dev)
		metal_device_close(ipi_dev);
}

/**
 * @brief sys_init() - Register libmetal devices.
 *        This function register the libmetal generic bus, and then
 *        register the IPI, shared memory descriptor and shared memory
 *        devices to the libmetal generic bus.
 *
 * @return 0 - succeeded, non-zero for failures.
 */
int ipi_init()
{
	struct metal_init_params metal_param = METAL_INIT_DEFAULTS;
	int ret;

	enable_caches();

	/* Initialize libmetal environment */
	metal_init(&metal_param);
	/* Initialize metal Xilinx IRQ controller */
	ret = metal_xlnx_irq_init();
	if (ret) {
		xil_printf("%s: Xilinx metal IRQ controller init failed.\r\n",
			__func__);
		return ret;
	}
	/* Register libmetal devices */
	ret = platform_register_metal_device();
	if (ret) {
		xil_printf("%s: failed to register devices: %d\r\n", __func__, ret);
		return ret;
	}

	/* Open libmetal devices which have been registered */
	ret = open_metal_devices();
	if (ret) {
		xil_printf("%s: failed to open devices: %d\r\n", __func__, ret);
		return ret;
	}
	return 0;
}

/**
 * @brief sys_cleanup() - system cleanup
 *        This function finish the libmetal environment
 *        and disable caches.
 *
 * @return 0 - succeeded, non-zero for failures.
 */
void ipi_cleanup()
{
	/* Close libmetal devices which have been opened */
	close_metal_devices();
	/* Finish libmetal environment */
	metal_finish();
	disable_caches();
}

