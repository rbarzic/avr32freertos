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

/* 
Changes from V3.2.4

	+ Also read the EMAC_RSR register in the EMAC ISR as a work around the 
	  the EMAC bug that can reset the RX bit in EMAC_ISR register before the
	  bit has been read.

Changes from V4.0.1

	+ Only check the interrupt status register to see if an EMAC Tx interrupt
	  has occurred.  Previously the TSR register was also inspected.
*/

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "AVR32_EMAC.h"
#include "portmacro.h"
/* Hardware specific includes. */
#include "avr32_circuit.h"
#include "stk1000.h"
#include "appnotes/pio.h"
#include "lwipopts.h"

#include "Emac.h"


extern int printk(const char *format, ...);

/*-----------------------------------------------------------*/

/* The semaphore used to signal the arrival of new data to the interface
task. */
static xSemaphoreHandle xSemaphore = NULL;

/*-----------------------------------------------------------*/
/*
 * The EMAC ISR.  Handles both Tx and Rx complete interrupts.
 */
volatile unsigned num_of_it = 0;
volatile unsigned debug_stop = 0;
extern volatile unsigned portLONG ulNextRxBuffer;
__attribute__((__noinline__)) unsigned int  vRealEMAC_ISR( void )
{
	/* This ISR can cause a context switch, so the first statement must be a
	call to the portENTER_SWITCHING_ISR() macro.  This must be BEFORE any
	variable declarations. */
      

        num_of_it++;
        //printk("=%d=\n",num_of_it);
	/* Variable definitions can be made now. */
	volatile unsigned portLONG ulIntStatus, ulEventStatus,ulEventTransmitStatus;
	portBASE_TYPE xSwitchRequired = pdFALSE;
    extern void vClearEMACTxBuffer( void );

	/* Find the cause of the interrupt. */
	//ulIntStatus = AT91C_BASE_EMAC->EMAC_ISR;
	//ulEventStatus = AT91C_BASE_EMAC->EMAC_RSR;
	ulIntStatus = AVR32_MACB0.isr;
	ulEventStatus = AVR32_MACB0.rsr;
        ulEventTransmitStatus = AVR32_MACB0.tsr;
        //printk("RSR : %x\n",ulEventStatus);
        //printk("ISR : %x\n",ulIntStatus);
        //printk("TSR : %x\n",ulEventTransmitStatus);

	if( ( ulIntStatus & AVR32_MACB_IDR_RCOMP_MASK ) || ( ulEventStatus & AVR32_MACB_REC_MASK ) )
	{
		/* A frame has been received, signal the lwIP task so it can process
		the Rx descriptors. */
		xSwitchRequired = xSemaphoreGiveFromISR( xSemaphore, pdFALSE );
		// AT91C_BASE_EMAC->EMAC_RSR = AT91C_EMAC_REC;
                AVR32_MACB0.rsr =  AVR32_MACB_REC_MASK;
                if((num_of_it >20)  && (ulNextRxBuffer==0) ) debug_stop = 0;

	}

	if( ulIntStatus & AVR32_MACB_TCOMP_MASK )
	{
		/* A frame has been transmitted.  Mark all the buffers used by the
		frame just transmitted as free again. */
		vClearEMACTxBuffer();
		// AT91C_BASE_EMAC->EMAC_TSR = AT91C_EMAC_COMP;
                AVR32_MACB0.tsr =  AVR32_MACB_TSR_COMP_MASK;

	}
        if(ulIntStatus & AVR32_MACB_IER_ROVR_MASK ) {
          // Receive overrun error
          printk("-I- vEMACISR : RBQP = %x\n",AVR32_MACB0.rbqp);
          printk("-I- vEMACISR : *RBQP = %x\n",*((unsigned long *)(AVR32_MACB0.rbqp+0xA0000000)));
          printk("-I- vEMACISR : *RBQP+4 = %x\n",*((unsigned long *)(AVR32_MACB0.rbqp+4+0xA0000000)));
       }

        //if(ulEventTransmitStatus & AVR32_MACB_TSR_UBR_MASK) printk("-E- vEMACISR : UBR bit set\n");
        if(ulEventTransmitStatus & AVR32_MACB_TSR_BEX_MASK) printk("-E- vEMACISR : BEX bit set\n");
        if(ulEventTransmitStatus & AVR32_MACB_TSR_UND_MASK) printk("-E- vEMACISR : UND bit set\n");
        AVR32_MACB0.tsr = 0xFFFFFFFF;
	/* Clear the interrupt. */
	// AT91C_BASE_AIC->AIC_EOICR = 0;

	/* If a task was woken by either a frame being received then we may need to 
	switch to another task. */
        // printk("S : %d\n",xSwitchRequired);
        return xSwitchRequired;

}
/*-----------------------------------------------------------*/



/*-----------------------------------------------------------*/
/*
 * EMAC interrupt service routine.
 */
__attribute__((naked,section (".handlers"))) void vEMAC_ISR( void )
{

 /* This ISR can cause a context switch, so the first statement must be a
     call to the portENTER_SWITCHING_ISR() macro.  This must be BEFORE any
     variable declarations. */
  portENTER_SWITCHING_ISR(EMAC_INT);
  vRealEMAC_ISR();
  portEXIT_SWITCHING_ISR(EMAC_INT);
}
/*----------------*/


void vPassEMACSemaphore( xSemaphoreHandle xCreatedSemaphore )
{
	/* Simply store the semaphore that should be used by the ISR. */
	xSemaphore = xCreatedSemaphore;
}

