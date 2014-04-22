/*
    FreeRTOS V8.0.0 - Copyright (C) 2014 Real Time Engineers Ltd.
    All rights reserved

    VISIT http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.

    ***************************************************************************
     *                                                                       *
     *    FreeRTOS provides completely free yet professionally developed,    *
     *    robust, strictly quality controlled, supported, and cross          *
     *    platform software that has become a de facto standard.             *
     *                                                                       *
     *    Help yourself get started quickly and support the FreeRTOS         *
     *    project by purchasing a FreeRTOS tutorial book, reference          *
     *    manual, or both from: http://www.FreeRTOS.org/Documentation        *
     *                                                                       *
     *    Thank you!                                                         *
     *                                                                       *
    ***************************************************************************

    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation >>!AND MODIFIED BY!<< the FreeRTOS exception.

    >>! NOTE: The modification to the GPL is included to allow you to distribute
    >>! a combined work that includes FreeRTOS without being obliged to provide
    >>! the source code for proprietary components outside of the FreeRTOS
    >>! kernel.

    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE.  Full license text is available from the following
    link: http://www.freertos.org/a00114.html

    1 tab == 4 spaces!

    ***************************************************************************
     *                                                                       *
     *    Having a problem?  Start by reading the FAQ "My application does   *
     *    not run, what could be wrong?"                                     *
     *                                                                       *
     *    http://www.FreeRTOS.org/FAQHelp.html                               *
     *                                                                       *
    ***************************************************************************

    http://www.FreeRTOS.org - Documentation, books, training, latest versions,
    license and Real Time Engineers Ltd. contact details.

    http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
    including FreeRTOS+Trace - an indispensable productivity tool, a DOS
    compatible FAT file system, and our tiny thread aware UDP/IP stack.

    http://www.OpenRTOS.com - Real Time Engineers ltd license FreeRTOS to High
    Integrity Systems to sell under the OpenRTOS brand.  Low cost OpenRTOS
    licenses offer ticketed support, indemnification and middleware.

    http://www.SafeRTOS.com - High Integrity Systems also provide a safety
    engineered and independently SIL3 certified version for use in safety and
    mission critical applications that require provable dependability.

    1 tab == 4 spaces!
*/


/*
 * Creates all the demo application tasks, then starts the scheduler.  The WEB
 * documentation provides more details of the standard demo application tasks
 * (which just exist to test the kernel port and provide an example of how to use
 * each FreeRTOS API function).
 *
 * In addition to the standard demo tasks, the following tasks and tests are
 * defined and/or created within this file:
 *
 * "LCD" task - the LCD task is a 'gatekeeper' task.  It is the only task that
 * is permitted to access the display directly.  Other tasks wishing to write a
 * message to the LCD send the message on a queue to the LCD task instead of
 * accessing the LCD themselves.  The LCD task just blocks on the queue waiting
 * for messages - waking and displaying the messages as they arrive.  The use
 * of a gatekeeper in this manner permits both tasks and interrupts to write to
 * the LCD without worrying about mutual exclusion.  This is demonstrated by the
 * check hook (see below) which sends messages to the display even though it
 * executes from an interrupt context.
 *
 * "Check" hook -  This only executes fully every five seconds from the tick
 * hook.  Its main function is to check that all the standard demo tasks are
 * still operational.  Should any unexpected behaviour be discovered within a
 * demo task then the tick hook will write an error to the LCD (via the LCD task).
 * If all the demo tasks are executing with their expected behaviour then the
 * check task writes PASS to the LCD (again via the LCD task), as described above.
 *
 * LED tasks - These just demonstrate how multiple instances of a single task
 * definition can be created.  Each LED task simply toggles an LED.  The task
 * parameter is used to pass the number of the LED to be toggled into the task.
 *
 * "Fast Interrupt Test" - A high frequency periodic interrupt is generated
 * using a free running timer to demonstrate the use of the
 * configKERNEL_INTERRUPT_PRIORITY configuration constant.  The interrupt
 * service routine measures the number of processor clocks that occur between
 * each interrupt - and in so doing measures the jitter in the interrupt timing.
 * The maximum measured jitter time is latched in the ulMaxJitter variable, and
 * displayed on the OLED display by the 'OLED' task as described below.  The
 * fast interrupt is configured and handled in the timertest.c source file.
 *
 */

/* Standard includes. */
#include <stdio.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* Library includes. */
#include "stm32f4xx_it.h"
#include "stm32f4xx_tim.h"
#include "stm32f429i_discovery_lcd.h"
#include "stm32_eval_legacy.h"

/* Demo app includes. */
#include "BlockQ.h"
#include "integer.h"
#include "flash.h"
#include "partest.h"
#include "semtest.h"
#include "PollQ.h"
#include "GenQTest.h"
#include "QPeek.h"
#include "recmutex.h"


/* The time between cycles of the 'check' functionality (defined within the
tick hook. */
#define mainCHECK_DELAY						( ( TickType_t ) 5000 / portTICK_PERIOD_MS )

/* Task priorities. */
#define mainQUEUE_POLL_PRIORITY				( tskIDLE_PRIORITY + 2 )
#define mainSEM_TEST_PRIORITY				( tskIDLE_PRIORITY + 1 )
#define mainBLOCK_Q_PRIORITY				( tskIDLE_PRIORITY + 2 )
#define mainUIP_TASK_PRIORITY				( tskIDLE_PRIORITY + 3 )
#define mainFLASH_TASK_PRIORITY				( tskIDLE_PRIORITY + 2 )
#define mainLCD_TASK_PRIORITY				( tskIDLE_PRIORITY + 3 )
#define mainINTEGER_TASK_PRIORITY           ( tskIDLE_PRIORITY )
#define mainGEN_QUEUE_TASK_PRIORITY			( tskIDLE_PRIORITY )

/* The WEB server has a larger stack as it utilises stack hungry string
handling library calls. */
#define mainBASIC_WEB_STACK_SIZE            ( configMINIMAL_STACK_SIZE * 4 )

/* The length of the queue used to send messages to the LCD task. */
#define mainQUEUE_SIZE						( 3 )

/* The period of the system clock in nano seconds.  This is used to calculate
the jitter time in nano seconds. */
#define mainNS_PER_CLOCK					( ( unsigned long ) ( ( 1.0 / ( double ) configCPU_CLOCK_HZ ) * 1000000000.0 ) )

/*-----------------------------------------------------------*/

/*
 * Configure the hardware for the demo.
 */
static void prvSetupHardware( void );

/*
 * The LCD gatekeeper task as described in the comments at the top of this file.
 * */
static void prvLCDTask( void *pvParameters );

/*
 * Configures the high frequency timers - those used to measure the timing
 * jitter while the real time kernel is executing.
 */
extern void vSetupHighFrequencyTimer( void );

/*-----------------------------------------------------------*/

/* The queue used to send messages to the LCD task. */
QueueHandle_t xLCDQueue;

/*-----------------------------------------------------------*/

int main( void )
{
	prvSetupHardware();

	/* Start the standard demo tasks.  These are just here to exercise the
	kernel port and provide examples of how the FreeRTOS API can be used. */
	vStartBlockingQueueTasks( mainBLOCK_Q_PRIORITY );
	vStartSemaphoreTasks( mainSEM_TEST_PRIORITY );
	vStartPolledQueueTasks( mainQUEUE_POLL_PRIORITY );
	vStartIntegerMathTasks( mainINTEGER_TASK_PRIORITY );
	vStartGenericQueueTasks( mainGEN_QUEUE_TASK_PRIORITY );
	vStartLEDFlashTasks( mainFLASH_TASK_PRIORITY );
	vStartQueuePeekTasks();
	vStartRecursiveMutexTasks();

	/* Create the queue used by the LCD task.  Messages for display on the LCD
	are received via this queue. */
	xLCDQueue = xQueueCreate( mainQUEUE_SIZE, sizeof( char * ) );

	/* Start the LCD gatekeeper task - as described in the comments at the top
	of this file. */
	xTaskCreate( prvLCDTask, "LCD", configMINIMAL_STACK_SIZE * 2, NULL, mainLCD_TASK_PRIORITY, NULL );

	/* Configure the high frequency interrupt used to measure the interrupt
	jitter time.  When debugging it can be helpful to comment this line out
	to prevent the debugger repeatedly going into the interrupt service
	routine. */
	vSetupHighFrequencyTimer();

	/* Start the scheduler. */
	vTaskStartScheduler();

	/* Will only get here if there was insufficient memory to create the idle
	   task.  The idle task is created within vTaskStartScheduler(). */
	for( ;; );
}
/*-----------------------------------------------------------*/

static void prvLCDTask( void *pvParameters )
{
unsigned char *pucMessage;
unsigned long ulLine = Line3;
const unsigned long ulLineHeight = 12;
static char cMsgBuf[ 30 ];
extern unsigned short usMaxJitter;

	( void ) pvParameters;

	/* The LCD gatekeeper task as described in the comments at the top of this
	file. */

	/* Init LCD and LTCD. Enable layer1 only. */
	LCD_Init();
	LCD_LayerInit();
	LTDC_LayerCmd(LTDC_Layer1, ENABLE);
	LTDC_LayerCmd(LTDC_Layer2, DISABLE);
	LTDC_ReloadConfig(LTDC_IMReload);
	LTDC_Cmd(ENABLE);
	LCD_SetLayer(LCD_BACKGROUND_LAYER);

	/* Display startup messages. */
	LCD_SetFont(&Font8x12);
	LCD_Clear(White);
	LCD_SetTextColor(Green);
	LCD_DisplayStringLine( Line0, (uint8_t *)"       www.freertos.org" );
	LCD_SetTextColor(Blue);
	LCD_DisplayStringLine( Line1, (uint8_t *)"     STM32F429i Discovery" );
	LCD_SetTextColor(Black);

	for( ;; )
	{
		/* Wait for a message to arrive to be displayed. */
		xQueueReceive( xLCDQueue, &pucMessage, portMAX_DELAY );

		/* Clear the current line of text. */
		LCD_ClearLine( ulLine );

		/* Move on to the next line. */
		ulLine += ulLineHeight;
		if( ulLine > LCD_LINE_18 )
		{
			ulLine = Line3;
		}

		/* Display the received text, and the max jitter value. */
		sprintf( cMsgBuf, "%s [%luns]", pucMessage, usMaxJitter * mainNS_PER_CLOCK );
		LCD_DisplayStringLine( ulLine, ( unsigned char * ) cMsgBuf );
	}
}
/*-----------------------------------------------------------*/

static void prvSetupHardware( void )
{
	/* Setup STM32 system (clock, PLL and Flash configuration) */
	SystemInit();

	/* Ensure all priority bits are assigned as preemption priority bits. */
	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );

	/* Initialise the IO used for the LED outputs. */
	vParTestInitialise();
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{
	/* This function will get called if a task overflows its stack.   If the
	parameters are corrupt then inspect pxCurrentTCB to find which was the
	offending task. */

	( void ) pxTask;
	( void ) pcTaskName;

	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationTickHook( void )
{
char *pcMessage = "Status: PASS";
static unsigned long ulTicksSinceLastDisplay = 0;
portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

	/* Called from every tick interrupt as described in the comments at the top
	of this file.

	Have enough ticks passed to make it	time to perform our health status
	check again? */
	ulTicksSinceLastDisplay++;
	if( ulTicksSinceLastDisplay >= mainCHECK_DELAY )
	{
		/* Reset the counter so these checks run again in mainCHECK_DELAY
		ticks time. */
		ulTicksSinceLastDisplay = 0;

		/* Has an error been found in any task? */
		if( xAreGenericQueueTasksStillRunning() != pdTRUE )
		{
			pcMessage = "ERROR: GEN Q";
		}
		else if( xAreQueuePeekTasksStillRunning() != pdTRUE )
		{
			pcMessage = "ERROR: PEEK Q";
		}
		else if( xAreBlockingQueuesStillRunning() != pdTRUE )
		{
			pcMessage = "ERROR: BLOCK Q";
		}
		else if( xAreSemaphoreTasksStillRunning() != pdTRUE )
		{
			pcMessage = "ERROR: SEMAPHR";
		}
		else if( xArePollingQueuesStillRunning() != pdTRUE )
		{
			pcMessage = "ERROR: POLL Q";
		}
		else if( xAreIntegerMathsTaskStillRunning() != pdTRUE )
		{
			pcMessage = "ERROR: INT MATH";
		}
		else if( xAreRecursiveMutexTasksStillRunning() != pdTRUE )
		{
			pcMessage = "ERROR: REC MUTEX";
		}

		/* Send the message to the OLED gatekeeper for display.  The
		xHigherPriorityTaskWoken parameter is not actually used here
		as this function is running in the tick interrupt anyway - but
		it must still be supplied. */
		xHigherPriorityTaskWoken = pdFALSE;
		xQueueSendFromISR( xLCDQueue, &pcMessage, &xHigherPriorityTaskWoken );
	}
}
/*-----------------------------------------------------------*/
