/*      Simple AVR32/FreeRTOS demo file      
        Copyright (C) 2006 Ronan BARZIC  

	This file is part of the AVR32 port of FreeRTOS - It is
	available with the same license as FreeRTOS :   

	FreeRTOS is free software; you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation; either version 2 of the License, or
	(at your option) any later version.

	FreeRTOS is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with FreeRTOS; if not, write to the Free Software
	Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

	A special exception to the GPL can be applied should you wish to distribute
	a combined work that includes FreeRTOS, without being obliged to provide
	the source code for any proprietary components.  See the licensing section 
	of http://www.FreeRTOS.org for full details of how and when the exception
	can be applied.

	***************************************************************************
	See http://www.FreeRTOS.org for documentation, latest information, license 
	and contact details.  Please ensure to read the configuration and relevant 
	port sections of the online documentation.
	***************************************************************************
*/
/*  include files */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"


/* Demo application includes. */
#include "partest.h"
#include "flash.h"
#include "integer.h"
#include "PollQ.h"
#include "comtest2.h"
#include "semtest.h"
#include "flop.h"
#include "dynamic.h"
#include "BlockQ.h"
#include "serial.h"



#include "avr32_circuit.h"
#include "avr32_int.h"
#include "avr32_sm.h"

#include "stk1000.h"
#include "appnotes/usart.h"
#include "appnotes/pio.h"
#include "appnotes/sdram.h"
#include "appnotes/mt481c2m32b2tg.h"


// 1 : test task is created and runs
// 0 : task is not created
#define MIN_INTEGER_TASK          1
#define MIN_COMTEST_TASK          1
#define MIN_LEDFLASH_TASK         1
#define MIN_POLLEDQUEUE_TASK      1
#define MIN_MATH_TASK             1          
#define MIN_SEMAPHORE_TASK        1
#define MIN_DYNAMICPRIORITY       1
#define MIN_BLOCKINGQUEUE_TASK    1

/* Constants for the ComTest tasks. */
#define mainCOM_TEST_BAUD_RATE	( ( unsigned portLONG ) 115200 )
#define mainCOM_TEST_LED		( 5 )

/* Priorities for the demo application tasks. */
#define mainLED_TASK_PRIORITY		( tskIDLE_PRIORITY + 3 )
#define mainCOM_TEST_PRIORITY		( tskIDLE_PRIORITY + 2 )
#define mainQUEUE_POLL_PRIORITY		( tskIDLE_PRIORITY + 2 )
#define mainCHECK_TASK_PRIORITY		( tskIDLE_PRIORITY + 4 )
#define mainSEM_TEST_PRIORITY		( tskIDLE_PRIORITY + 1 )
#define mainBLOCK_Q_PRIORITY		( tskIDLE_PRIORITY + 2 )

/* The rate at which the on board LED will toggle when there is/is not an 
error. */
#define mainNO_ERROR_FLASH_PERIOD	( ( portTickType ) 3000 / portTICK_RATE_MS  )
#define mainERROR_FLASH_PERIOD		( ( portTickType ) 100 / portTICK_RATE_MS  )
#define mainON_BOARD_LED_BIT		( ( unsigned portLONG ) 7 )

/* Constants used by the vMemCheckTask() task. */
#define mainCOUNT_INITIAL_VALUE		( ( unsigned portLONG ) 0 )
#define mainNO_TASK					( 0 )

/* The size of the memory blocks allocated by the vMemCheckTask() task. */
#define mainMEM_CHECK_SIZE_1		( ( size_t ) 51 )
#define mainMEM_CHECK_SIZE_2		( ( size_t ) 52 )
#define mainMEM_CHECK_SIZE_3		( ( size_t ) 151 )








int printk(const char *format, ...);



/*!
 * This function will setup the the SDRAM controller, intialize the SDRAM found on the STK1000 and test it.
 * \return The number of errors
 */
void  init_sdram_startup( void )
{
  extern int pio_setup_pin(int pin, int function);
  sdram_info *info;
  sdram_info info_s;
  unsigned long sdram_size, tmp, i;
  int noErrors=0;
  volatile unsigned long *sdram = (void *) STK1000_SDRAM_BASE;
  info = &info_s;

  info->physical_address = STK1000_SDRAM_BASE | 0xA0000000;
  info->cols = mt481c2m32b2tg_cols;
  info->rows = mt481c2m32b2tg_rows;
  info->banks = mt481c2m32b2tg_banks;
  info->cas = mt481c2m32b2tg_cas;
  info->twr = mt481c2m32b2tg_twr;
  info->trc = mt481c2m32b2tg_trc;
  info->trp = mt481c2m32b2tg_trp;
  info->trcd = mt481c2m32b2tg_trcd;
  info->tras = mt481c2m32b2tg_tras;
  info->txsr = mt481c2m32b2tg_txsr;

  /* Calculate sdram size */
  sdram_size = 1 << (info->rows + info->cols + info->banks + 2);

   // Setup the data bit 16-32

 
pio_setup_pin(AVR32_HEBI_DATA_16_PIN , AVR32_HEBI_DATA_16_FUNCTION);
pio_setup_pin(AVR32_HEBI_DATA_17_PIN , AVR32_HEBI_DATA_17_FUNCTION);
pio_setup_pin(AVR32_HEBI_DATA_18_PIN , AVR32_HEBI_DATA_18_FUNCTION);
pio_setup_pin(AVR32_HEBI_DATA_19_PIN , AVR32_HEBI_DATA_19_FUNCTION);
pio_setup_pin(AVR32_HEBI_DATA_20_PIN , AVR32_HEBI_DATA_20_FUNCTION);
pio_setup_pin(AVR32_HEBI_DATA_21_PIN , AVR32_HEBI_DATA_21_FUNCTION);
pio_setup_pin(AVR32_HEBI_DATA_22_PIN , AVR32_HEBI_DATA_22_FUNCTION);
pio_setup_pin(AVR32_HEBI_DATA_23_PIN , AVR32_HEBI_DATA_23_FUNCTION);
pio_setup_pin(AVR32_HEBI_DATA_24_PIN , AVR32_HEBI_DATA_24_FUNCTION);
pio_setup_pin(AVR32_HEBI_DATA_25_PIN , AVR32_HEBI_DATA_25_FUNCTION);
pio_setup_pin(AVR32_HEBI_DATA_26_PIN , AVR32_HEBI_DATA_26_FUNCTION);
pio_setup_pin(AVR32_HEBI_DATA_27_PIN , AVR32_HEBI_DATA_27_FUNCTION);
pio_setup_pin(AVR32_HEBI_DATA_28_PIN , AVR32_HEBI_DATA_28_FUNCTION);
pio_setup_pin(AVR32_HEBI_DATA_29_PIN , AVR32_HEBI_DATA_29_FUNCTION);
pio_setup_pin(AVR32_HEBI_DATA_30_PIN , AVR32_HEBI_DATA_30_FUNCTION);
pio_setup_pin(AVR32_HEBI_DATA_31_PIN , AVR32_HEBI_DATA_31_FUNCTION);

  /* initialize the avr32 sdram controller */
  sdramc_init(info);

  /* initialize the external sdram chip */
  mt481c2m32b2tg_init(info);


 


}


void delay(unsigned long delayl)
{
  volatile unsigned long i;

  for (i=0; i<delayl; i++) {
    asm volatile ("nop");
  };
}




int setup_debug_usart( void )
{
  //int cpu_hz = 20000000;
#ifdef USE_PLL0
  int usart_clk_hz = configCPU_CLOCK_HZ/4; // Should be 4, bug in revA
#else
 int usart_clk_hz = configCPU_CLOCK_HZ; // Should be 4, bug in revA
#endif
  //int usart_clk_hz = 20000000; // Should be 4, bug in revA
  struct usart_options_t opt;

  volatile struct avr32_usart_t *usart = &AVR32_USART1;

  avr32_piomap_t usart_piomap = {				   \
    {AVR32_USART1_RXD_0_PIN, AVR32_USART1_RXD_0_FUNCTION}, \
    {AVR32_USART1_TXD_0_PIN, AVR32_USART1_TXD_0_FUNCTION}   \
  };

  // Set options for the USART
  opt.baudrate = 115200;
  opt.charlength = 8;
  opt.paritytype = USART_NO_PARITY;
  opt.stopbits = USART_1_STOPBIT;
  opt.channelmode = USART_NORMAL_CHMODE;

  // Initialize it in RS232 mode
   {
     int error =  usart_init_rs232(usart, &opt, usart_clk_hz);
    if(error) {
      STK1000_all_led(0x40 | error);
      while(1) ;
    }
  }
 

  // Setup pio for USART
  pio_enable_module(usart_piomap, 2);

  // Print a string to the screen
  printk("This STK1000 board is running FreeRTOS Minimal demo (Using Flash and SDRAM !)!\n");



  return 0;
}

int putchar(int c) {
  return usart_putchar(&AVR32_USART1,c);
}


/*
 * Checks that all the demo application tasks are still executing without error
 * - as described at the top of the file.
 */
static portLONG prvCheckOtherTasksAreStillRunning( unsigned portLONG ulMemCheckTaskCount );

/*
 * The task that executes at the highest priority and calls 
 * prvCheckOtherTasksAreStillRunning().  See the description at the top
 * of the file.
 */
static void vErrorChecks( void *pvParameters );

/*
 * Dynamically created and deleted during each cycle of the vErrorChecks()
 * task.  This is done to check the operation of the memory allocator.
 * See the top of vErrorChecks for more details.
 */
static void vMemCheckTask( void *pvParameters );

/*
 * Configure the processor for use with the Olimex demo board.  This includes
 * setup for the I/O, system clock, and access timings.
 */
static void prvSetupHardware( void );


static volatile check_startup_wait_debugger = 1;
static check_startup(void) {
  volatile avr32_sm_t* sm = &AVR32_SM;
  if(sm->RC_RCAUSE.por) {
    // OK - do nothing 
  } else {
    STK1000_reset_all_led();
    STK1000_all_led(sm->rc_rcause);
    while(check_startup_wait_debugger);
  }


}

/*-----------------------------------------------------------*/

/*
 * Starts all the other tasks, then starts the scheduler. 
 */
volatile unsigned long wait_debugger =0;

int main( void )
{
  // Detect bad reset
  // check_startup();

	/* Setup the hardware for use with the STK1000 board. */
	prvSetupHardware();


         while(wait_debugger == 1) {
           STK1000_reset_all_led();
           STK1000_all_led(0x55);
           delay(500000);
           STK1000_reset_all_led();
           STK1000_all_led(0xAA);
           delay(500000);
         }
         

	/* Start the demo/test application tasks. */
        if(MIN_INTEGER_TASK)      vStartIntegerMathTasks( tskIDLE_PRIORITY );
	if(MIN_COMTEST_TASK)       vAltStartComTestTasks( mainCOM_TEST_PRIORITY, mainCOM_TEST_BAUD_RATE, mainCOM_TEST_LED );
	if(MIN_LEDFLASH_TASK)      vStartLEDFlashTasks( mainLED_TASK_PRIORITY );
	if(MIN_POLLEDQUEUE_TASK)   vStartPolledQueueTasks( mainQUEUE_POLL_PRIORITY );
	if(MIN_MATH_TASK)          vStartMathTasks( tskIDLE_PRIORITY );
	if(MIN_SEMAPHORE_TASK)     vStartSemaphoreTasks( mainSEM_TEST_PRIORITY );
	if(MIN_DYNAMICPRIORITY)    vStartDynamicPriorityTasks();	
	if(MIN_BLOCKINGQUEUE_TASK) vStartBlockingQueueTasks( mainBLOCK_Q_PRIORITY );

	/* Start the check task - which is defined in this file. */
	xTaskCreate( vErrorChecks, ( signed portCHAR * ) "Check", configMINIMAL_STACK_SIZE, NULL, mainCHECK_TASK_PRIORITY, NULL );

	/* Now all the tasks have been started - start the scheduler.

	NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
	The processor MUST be in supervisor mode when vTaskStartScheduler is 
	called.  The demo applications included in the FreeRTOS.org download switch
	to supervisor mode prior to main being called.  If you are not using one of
	these demo application projects then ensure Supervisor mode is used here. */
	vTaskStartScheduler();

	/* Should never reach here! */
	return 0;
}
/*-----------------------------------------------------------*/

static void vErrorChecks( void *pvParameters )
{
portTickType xDelayPeriod = mainNO_ERROR_FLASH_PERIOD;
unsigned portLONG ulMemCheckTaskRunningCount;
xTaskHandle xCreatedTask;

	/* Cycle for ever, delaying then checking all the other tasks are still
	operating without error.  If an error is detected then the delay period
	is decreased from mainNO_ERROR_FLASH_PERIOD to mainERROR_FLASH_PERIOD so
	the on board LED flash rate will increase. 
	
	In addition to the standard tests the memory allocator is tested through
	the dynamic creation and deletion of a task each cycle.  Each time the 
	task is created memory must be allocated for its stack.  When the task is
	deleted this memory is returned to the heap.  If the task cannot be created 
	then it is likely that the memory allocation failed. */

	for( ;; )
	{
		/* Reset xCreatedTask.  This is modified by the task about to be 
		created so we can tell if it is executing correctly or not. */
		xCreatedTask = mainNO_TASK;

		/* Dynamically create a task - passing ulMemCheckTaskRunningCount as a 
		parameter. */
		ulMemCheckTaskRunningCount = mainCOUNT_INITIAL_VALUE;		
		if( xTaskCreate( vMemCheckTask, ( signed portCHAR * ) "MEM_CHECK", configMINIMAL_STACK_SIZE, ( void * ) &ulMemCheckTaskRunningCount, tskIDLE_PRIORITY, &xCreatedTask ) != pdPASS )
		{
			/* Could not create the task - we have probably run out of heap. */
			xDelayPeriod = mainERROR_FLASH_PERIOD;
		}

		/* Delay until it is time to execute again. */
		vTaskDelay( xDelayPeriod );

		/* Delete the dynamically created task. */
		if( xCreatedTask != mainNO_TASK )
		{
			vTaskDelete( xCreatedTask );
		}

		/* Check all the standard demo application tasks are executing without 
		error.  ulMemCheckTaskRunningCount is checked to ensure it was
		modified by the task just deleted. */
		if( prvCheckOtherTasksAreStillRunning( ulMemCheckTaskRunningCount ) != pdPASS )
		{
			/* An error has been detected in one of the tasks - flash faster. */
			xDelayPeriod = mainERROR_FLASH_PERIOD;
		}

		/* The toggle rate of the LED depends on how long this task delays for.
		An error reduces the delay period and so increases the toggle rate. */
		vParTestToggleLED( mainON_BOARD_LED_BIT );
	}
}
/*-----------------------------------------------------------*/



static void prvSetupHardware( void )
{
  unsigned i;
  STK1000_reset_all_led();
  for(i=0;i<10;i++) {
    STK1000_all_led(0x55);
    delay(100000);
    STK1000_all_led(0xAA);
    delay(100000);
  }
  STK1000_reset_all_led();
  setup_debug_usart();
  vParTestInitialise();
}


/*-----------------------------------------------------------*/
volatile unsigned error_code;


static portLONG prvCheckOtherTasksAreStillRunning( unsigned portLONG ulMemCheckTaskCount )
{
portLONG lReturn = ( portLONG ) pdPASS;

	/* Check all the demo tasks (other than the flash tasks) to ensure
	that they are all still running, and that none of them have detected
	an error. */

	if( MIN_INTEGER_TASK && (xAreIntegerMathsTaskStillRunning() != pdTRUE) )
	{
 error_code = 1; 
                printk("-E- Error in IntegerMathsTask\n"); 
		lReturn = ( portLONG ) pdFAIL;
	}

	if( MIN_COMTEST_TASK && (xAreComTestTasksStillRunning() != pdTRUE ))
	{
error_code = 2; 
                printk("-E- Error in ComTestTasks\n");  
		lReturn = ( portLONG ) pdFAIL;
	}

	if( MIN_POLLEDQUEUE_TASK &&  (xArePollingQueuesStillRunning() != pdTRUE ))
	{
error_code = 3; 
                printk("-E- Error in PollingQueues\n");  
		lReturn = ( portLONG ) pdFAIL;
	}

	if( MIN_MATH_TASK && (xAreMathsTaskStillRunning() != pdTRUE ))
	{
error_code = 4; 
                printk("-E- Error in MathTasks\n");  
		lReturn = ( portLONG ) pdFAIL;
	}

	if( MIN_SEMAPHORE_TASK && (xAreSemaphoreTasksStillRunning() != pdTRUE ))
	{
error_code = 5; 
                printk("-E- Error in SemaphoreTasks\n");  
		lReturn = ( portLONG ) pdFAIL;
	}

	if( MIN_DYNAMICPRIORITY && (xAreDynamicPriorityTasksStillRunning() != pdTRUE ))
	{
error_code = 6;
                printk("-E- Error in PriorityTasks\n"); 
		lReturn = ( portLONG ) pdFAIL;
	}

	if( MIN_BLOCKINGQUEUE_TASK && (xAreBlockingQueuesStillRunning() != pdTRUE ))
	{
error_code = 7;
                printk("-E- Error in BlockingQueues\n"); 
		lReturn = ( portLONG ) pdFAIL;
	}

	if( ulMemCheckTaskCount == mainCOUNT_INITIAL_VALUE )
	{
 error_code = 8;
		/* The vMemCheckTask did not increment the counter - it must
		have failed. */
                printk("-E- Error in MemCheck\n");  
		lReturn = ( portLONG ) pdFAIL;
	}
                                                              
	return lReturn;
}
/*-----------------------------------------------------------*/

static void vMemCheckTask( void *pvParameters )
{
unsigned portLONG *pulMemCheckTaskRunningCounter;
void *pvMem1, *pvMem2, *pvMem3;
static portLONG lErrorOccurred = pdFALSE;

	/* This task is dynamically created then deleted during each cycle of the
	vErrorChecks task to check the operation of the memory allocator.  Each time
	the task is created memory is allocated for the stack and TCB.  Each time
	the task is deleted this memory is returned to the heap.  This task itself
	exercises the allocator by allocating and freeing blocks. 
	
	The task executes at the idle priority so does not require a delay. 
	
	pulMemCheckTaskRunningCounter is incremented each cycle to indicate to the
	vErrorChecks() task that this task is still executing without error. */

	pulMemCheckTaskRunningCounter = ( unsigned portLONG * ) pvParameters;

	for( ;; )
	{
		if( lErrorOccurred == pdFALSE )
		{
			/* We have never seen an error so increment the counter. */
			( *pulMemCheckTaskRunningCounter )++;
		}
		else
		{
			/* There has been an error so reset the counter so the check task 
			can tell that an error occurred. */
			*pulMemCheckTaskRunningCounter = mainCOUNT_INITIAL_VALUE;
		}

		/* Allocate some memory - just to give the allocator some extra 
		exercise.  This has to be in a critical section to ensure the
		task does not get deleted while it has memory allocated. */
		vTaskSuspendAll();
		{
			pvMem1 = pvPortMalloc( mainMEM_CHECK_SIZE_1 );
			if( pvMem1 == NULL )
			{
				lErrorOccurred = pdTRUE;
			}
			else
			{
				memset( pvMem1, 0xaa, mainMEM_CHECK_SIZE_1 );
				vPortFree( pvMem1 );
			}
		}
		xTaskResumeAll();

		/* Again - with a different size block. */
		vTaskSuspendAll();
		{
			pvMem2 = pvPortMalloc( mainMEM_CHECK_SIZE_2 );
			if( pvMem2 == NULL )
			{
				lErrorOccurred = pdTRUE;
			}
			else
			{
				memset( pvMem2, 0xaa, mainMEM_CHECK_SIZE_2 );
				vPortFree( pvMem2 );
			}
		}
		xTaskResumeAll();

		/* Again - with a different size block. */
		vTaskSuspendAll();
		{
			pvMem3 = pvPortMalloc( mainMEM_CHECK_SIZE_3 );
			if( pvMem3 == NULL )
			{
				lErrorOccurred = pdTRUE;
			}
			else
			{
				memset( pvMem3, 0xaa, mainMEM_CHECK_SIZE_3 );
				vPortFree( pvMem3 );
			}
		}
		xTaskResumeAll();
	}
}


