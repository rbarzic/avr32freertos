/*      Support for STK1000 specific hardware     
        Copyright (C) 2003-2005 Ronan BARZIC  

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

#include "avr32_circuit.h"


#include "stk1000.h"

#if LED_ON_PORTB
#define PIO_LED AVR32_PIOB
#define pioled  piob
#else
#define PIO_LED AVR32_PIOC
#define pioled  pioc
#endif

void STK1000_reset_all_led(void) {
  volatile avr32_pio_t *pioled = &PIO_LED;
  pioled->per = 0x0ff;
  pioled->oer = 0x0ff;
  pioled->idr = 0x0ff;
  pioled->codr = 0x0ff;
}

void STK1000_set_led(unsigned int led) {
  volatile avr32_pio_t *pioled = &PIO_LED;
  pioled->sodr = 1<<led;
}

void STK1000_reset_led(unsigned int led) {
  volatile avr32_pio_t *pioled = &PIO_LED;
  pioled->codr = 1<<led;
}

void STK1000_led(unsigned int led,stk1000_onoff_t onoff) {
  volatile avr32_pio_t *pioled = &PIO_LED;
  if(onoff==on) {
    pioled->sodr = 1<<led;
  } else {
    pioled->codr = 1<<led;
  }
}

void STK1000_all_led(unsigned int value) {
  
  volatile avr32_pio_t *pioled = &PIO_LED;
  value &= 0x0FF; /* Use only 8 leds */
  pioled->sodr = value;
  pioled->codr = (~value) & 0xFF;
  
}

