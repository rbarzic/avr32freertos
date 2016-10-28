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





#include "avr32_circuit.h"
#include "avr32_int.h"
#include "avr32_sm.h"

#include "stk1000.h"
#include "appnotes/usart.h"
#include "appnotes/pio.h"
#include "appnotes/sdram.h"
#include "appnotes/mt481c2m32b2tg.h"

#include "BasicWEB.h"

#define mainWEBSERVER_PRIORITY      ( tskIDLE_PRIORITY + 2 )


typedef struct {
  unsigned led;
  unsigned delay;
  const char * name;
}task_led_parameters_t;


task_led_parameters_t task_led_params[6] = {
  {
    .led   = 7,
    .delay = 100000,
    .name = "Task1",
  },
  {
    .led   = 6,
    .delay = 200000,
    .name = "Task2",
  },
  {
    .led   = 5,
    .delay = 300000,
    .name = "Task3",
  },
   {
    .led   = 4,
    .delay = 400000,
    .name = "Task4",
   },
  {
    .led   = 3,
    .delay = 500000,
    .name = "Task5",
  },
 {
    .led   = 2,
    .delay = 600000,
    .name = "Task6",
 },
};


void vTaskTestLed(void *pvParameters)
{
  task_led_parameters_t * param = (task_led_parameters_t*)pvParameters;

  while(1) {
    STK1000_set_led(param->led);
    vTaskDelay(param->delay/1000);
    STK1000_reset_led(param->led);
    vTaskDelay(param->delay/1000);

  }
}




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
  printk("Whao !\n");



  return 0;
}

int putchar(int c) {
  return usart_putchar(&AVR32_USART1,c);
}









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

}





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

volatile avr32_sm_t* sm = &AVR32_SM;  
#ifdef USE_PLL0
sm_enable_pll0(sm,0,5,0); // Source = Osc0, mul = 6, div = 1 - 120MHz
sm_wait_for_lockbit0(sm);

sm_set_main_clocks(sm,
		     0, // cpusel
		     0, // cpu div - nodivision
		     0, // ahbsel,
		     1, // ahbdiv, Clk AHB = Clk CPU/2
		     1, // apbasel,
		     1, // apbadiv,  Clk APBA = Clk CPU/4  2**(apbasel + 1)
		     0, // apbbsel,
		     1  // apbbdiv	Clk APBB = Clk AHB = Clk CPU/2		
		     );


sm_switch_to_clock(sm,1); // Switch to PLL0
#endif



  // Detect bad reset
  // check_startup();
  xTaskHandle TaskTestLed[6];

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
#if 0

         { unsigned i = 0; 
           for(i=0;i<100; i++) {
             STK1000_reset_all_led();
             STK1000_all_led(0x55);
             delay(500000);
             STK1000_reset_all_led();
             STK1000_all_led(0xAA);
             delay(500000);
           }
         }
#endif

#if 0
        {
          for(int i=3;i<5;i++) {
            portBASE_TYPE Status = xTaskCreate(vTaskTestLed,
                                               (const signed char*)task_led_params[i].name,
                                               512,
                                               (void*)&task_led_params[i],
                                               tskIDLE_PRIORITY+1,
                                               &TaskTestLed[i]);
            if(Status != pdPASS) {
              STK1000_all_led(0xA0 | i);
              while(1);
            }
          }
        }

#endif
         
         vlwIPInit();

         {
           unsigned int res;
           res = sys_thread_new( vBasicWEBServer, ( void * ) NULL, mainWEBSERVER_PRIORITY );
           if(res == NULL) {
             STK1000_reset_all_led();
             STK1000_all_led(0xAA);
             while(1);
             
           } 
         }

	vTaskStartScheduler();

	/* Should never reach here! */
	return 0;
}


