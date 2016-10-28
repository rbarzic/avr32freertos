/*
	FreeRTOS.org V4.0.2 - Copyright (C) 2003-2006 Richard Barry.

	This file is part of the FreeRTOS.org distribution.

	FreeRTOS.org is free software; you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation; either version 2 of the License, or
	(at your option) any later version.

	FreeRTOS.org is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with FreeRTOS.org; if not, write to the Free Software
	Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

	A special exception to the GPL can be applied should you wish to distribute
	a combined work that includes FreeRTOS.org, without being obliged to provide
	the source code for any proprietary components.  See the licensing section 
	of http://www.FreeRTOS.org for full details of how and when the exception
	can be applied.

	***************************************************************************
	See http://www.FreeRTOS.org for documentation, latest information, license 
	and contact details.  Please ensure to read the configuration and relevant 
	port sections of the online documentation.
	***************************************************************************
*/

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "portable.h"

/* Demo app includes. */
#include "partest.h"

/* Hardware specific definitions. */
#include "stk1000.h"

#define partstNUM_LEDS			( 8 )
#define partstALL_OUTPUTS_OFF	( ( unsigned portLONG ) ~(0xFFFFFFFF << partstNUM_LEDS) )

static unsigned portLONG ulLEDReg;

/*-----------------------------------------------------------
 * Simple parallel port IO routines.
 *-----------------------------------------------------------*/

static void SetLeds (unsigned int leds)
{
  unsigned portLONG ulPIOSetReg, ulPIOClearReg;
  STK1000_all_led(leds);
}
/*-----------------------------------------------------------*/

void vParTestInitialise( void )
{
  // FIXME - Should set the PIO in the right mode, even if value at
  // startup is correct
	/* Set initial state of LEDs. */
	SetLeds( ulLEDReg );
}
/*-----------------------------------------------------------*/

void vParTestSetLED( unsigned portBASE_TYPE uxLED, signed portBASE_TYPE xValue )
{
	/* Switch an LED on or off as requested. */
	if (uxLED < partstNUM_LEDS)
	{
		if( xValue )
		{
			ulLEDReg &= ~(1 << uxLED);
		}
		else
		{
			ulLEDReg |= (1 << uxLED);
		}

		SetLeds( ulLEDReg );
	}
}
/*-----------------------------------------------------------*/

void vParTestToggleLED( unsigned portBASE_TYPE uxLED )
{
	/* Toggle the state of the requested LED. */
	if (uxLED < partstNUM_LEDS)
	{
		ulLEDReg ^= ( 1 << uxLED );
		SetLeds( ulLEDReg );
	}
}

