/*
 * FreeRTOS V202212.00
 * Copyright (C) 2020 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * https://www.FreeRTOS.org
 * https://github.com/FreeRTOS
 *
 */


/* Standard includes. */
#include <stdio.h>

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"
#include "config.h"

/* Xilinx includes. */
#include "platform.h"
#include "xparameters.h"
#include "xscugic.h"
#include "xil_printf.h"

#include "system.h"

/*-----------------------------------------------------------*/

/*
 * Configure the hardware as necessary to run this demo.
 */
static void prvSetupHardware( void );


/* Prototypes for the standard FreeRTOS callback/hook functions implemented
within this file. */
void vApplicationMallocFailedHook( void );
void vApplicationIdleHook( void );
void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName );
void vApplicationTickHook( void );

/*-----------------------------------------------------------*/

int main( void )
{
	/* Configure the hardware ready to run the demo. */
	prvSetupHardware();

	//Launch the system task that will initialize and start everything
	xil_printf("booting CF2 system...\r\n");
	systemLaunch();

	xil_printf("starting scheduler...\r\n");
	//Start the FreeRTOS scheduler
	vTaskStartScheduler();

	xil_printf("ERROR: platform launch failed...\r\n");

	//Should never reach this point!
	while(1);

	/* Don't expect to reach here. */
	return 0;
}
/*-----------------------------------------------------------*/

static void prvSetupHardware( void )
{
BaseType_t xStatus;
XScuGic_Config *pxGICConfig;

	/* Ensure no interrupts execute while the scheduler is in an inconsistent
	state.  Interrupts are automatically enabled when the scheduler is
	started. */
	portDISABLE_INTERRUPTS();

	init_platform();

	/* Ensure the FPU is accessible by enabling access to CP 10 and 11. */
	__asm volatile( "MRC	p15, 0, r0, c1, c0, 2	\n"	\
					"ORR	r0, r0, #(0xF << 20)	\n" \
					"MCR	p15, 0, r0, c1, c0, 2	\n" \
					"ISB							  " );


	/* Ensure the FPU is enabled. */
	__asm volatile( "VMRS	r0, FPEXC 			\n"		\
					"ORR	r1, r0, #(1<<30)	\n"		\
					"VMSR	FPEXC, r1			\n"		\
					::: "r0", "r1" );
}
/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook( void )
{
	/* Called if a call to pvPortMalloc() fails because there is insufficient
	free memory available in the FreeRTOS heap.  pvPortMalloc() is called
	internally by FreeRTOS API functions that create tasks, queues, software
	timers, and semaphores.  The size of the FreeRTOS heap is set by the
	configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */
	taskDISABLE_INTERRUPTS();
	xil_printf("ERROR: vApplicationMallocFailedHook()\r\n");
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{
	( void ) pcTaskName;
	( void ) pxTask;

	/* Run time stack overflow checking is performed if
	configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	function is called if a stack overflow is detected. */
	taskDISABLE_INTERRUPTS();
	xil_printf("ERROR: vApplicationStackOverflowHook()\r\n");
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
volatile size_t xFreeHeapSpace;

	/* This is just a trivial example of an idle hook.  It is called on each
	cycle of the idle task.  It must *NOT* attempt to block.  In this case the
	idle task just queries the amount of FreeRTOS heap that remains.  See the
	memory management section on the http://www.FreeRTOS.org web site for memory
	management options.  If there is a lot of heap memory free then the
	configTOTAL_HEAP_SIZE value in FreeRTOSConfig.h can be reduced to free up
	RAM. */
	xFreeHeapSpace = xPortGetFreeHeapSize();

	/* Remove compiler warning about xFreeHeapSpace being set but never used. */
	( void ) xFreeHeapSpace;
}
/*-----------------------------------------------------------*/

void vMainAssertCalled( const char *pcFileName, uint32_t ulLineNumber )
{
	xil_printf( "ASSERT!  Line %lu of file %s\r\n", ulLineNumber, pcFileName );
	taskENTER_CRITICAL();
	for( ;; );
}


