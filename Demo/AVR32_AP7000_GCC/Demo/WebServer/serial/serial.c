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
  BASIC INTERRUPT DRIVEN SERIAL PORT DRIVER FOR USART3.
*/

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

/* Demo application includes. */
#include "serial.h"
#include "avr32_circuit.h"
#include "avr32_int.h"
#include "avr32_sm.h"

#include "appnotes/usart.h"
#include "appnotes/pio.h"

/*-----------------------------------------------------------*/



/* Constants to setup and access the USART. */
#define serINVALID_COMPORT_HANDLER    ( ( xComPortHandle ) 0 )
#define serINVALID_QUEUE    ( ( xQueueHandle ) 0 )
#define serHANDLE       ( ( xComPortHandle ) 1 )
#define serNO_BLOCK       ( ( portTickType ) 0 )

/*-----------------------------------------------------------*/

/* Queues used to hold received characters, and characters waiting to be
transmitted. */
static xQueueHandle xRxedChars; 
static xQueueHandle xCharsForTx; 

/*-----------------------------------------------------------*/

/* Forward declaration. */
static void vprvSerialCreateQueues( unsigned portBASE_TYPE uxQueueLength, 
        xQueueHandle *pxRxedChars,
        xQueueHandle *pxCharsForTx );

static portBASE_TYPE prvUSART3_ISR_NonNakedBehaviour( void );



volatile unsigned long num_it = 0;

#define OLD_VERSION
#ifdef OLD_VERSION
/*-----------------------------------------------------------*/

/*
 * USART3 interrupt service routine.
 */
__attribute__((__noinline__)) void  vRealUSART3_ISR( void )
{


    /* Now we can declare the local variables. */
  signed portCHAR     cChar;
  portBASE_TYPE     xTaskWokenByTx = pdFALSE, xTaskWokenByRx = pdFALSE;
  unsigned portLONG     ulStatus;
  volatile avr32_usart_t  *usart3 = &AVR32_USART3;
  num_it++;

  /* What caused the interrupt? */
  ulStatus = usart3->csr & usart3->imr;

  if (ulStatus & AVR32_USART_CSR_TXRDY_MASK)
  {
    /* The interrupt was caused by the THR becoming empty.  Are there any
       more characters to transmit? */
    if( xQueueReceiveFromISR( xCharsForTx, &cChar, &xTaskWokenByTx ) == pdTRUE )
    {
      /* A character was retrieved from the queue so can be sent to the
         THR now. */
      usart3->thr = cChar;
    }
    else
    {
      /* Queue empty, nothing to send so turn off the Tx interrupt. */
      usart3->idr = AVR32_USART_IDR_TXRDY_MASK;
    }
  }


  if (ulStatus & AVR32_USART_CSR_RXRDY_MASK)
  {
    /* The interrupt was caused by the receiver getting data. */
    cChar = usart3->rhr; //TODO

    if (xQueueSendFromISR(xRxedChars, &cChar, pdFALSE))
    {
      xTaskWokenByRx = pdTRUE;
    }
  }


 /* Exit the ISR.  If a task was woken by either a character being received
     or transmitted then a context switch will occur. */

  return ( xTaskWokenByTx || xTaskWokenByRx );
}
/*-----------------------------------------------------------*/
/*
 * USART3 interrupt service routine.
 */
__attribute__((naked,section (".handlers"))) void vUSART3_ISR( void )
{

 /* This ISR can cause a context switch, so the first statement must be a
     call to the portENTER_SWITCHING_ISR() macro.  This must be BEFORE any
     variable declarations. */
  portENTER_SWITCHING_ISR(INT1);
  vRealUSART3_ISR();
  portEXIT_SWITCHING_ISR(INT1);
}
/*-----------------------------------------------------------*/

#else

__attribute__((__noinline__)) unsigned int  MovToR12( unsigned long  value) {
  return value;
}
__attribute__((naked,section (".handlers"))) void vUSART3_ISR( void )
{

 /* This ISR can cause a context switch, so the first statement must be a
     call to the portENTER_SWITCHING_ISR() macro.  This must be BEFORE any
     variable declarations. */
  portENTER_SWITCHING_ISR(INT1);
  
  signed portCHAR     cChar;
  portBASE_TYPE     xTaskWokenByTx = pdFALSE, xTaskWokenByRx = pdFALSE;
  unsigned portLONG     ulStatus;
  volatile avr32_usart_t  *usart3 = &AVR32_USART3;
  num_it++;

  /* What caused the interrupt? */
  ulStatus = usart3->csr & usart3->imr;

  if (ulStatus & AVR32_USART_CSR_TXRDY_MASK)
  {
    /* The interrupt was caused by the THR becoming empty.  Are there any
       more characters to transmit? */
    if( xQueueReceiveFromISR( xCharsForTx, &cChar, &xTaskWokenByTx ) == pdTRUE )
    {
      /* A character was retrieved from the queue so can be sent to the
         THR now. */
      usart3->thr = cChar;
    }
    else
    {
      /* Queue empty, nothing to send so turn off the Tx interrupt. */
      usart3->idr = AVR32_USART_IDR_TXRDY_MASK;
    }
  }


  if (ulStatus & AVR32_USART_CSR_RXRDY_MASK)
  {
    /* The interrupt was caused by the receiver getting data. */
    cChar = usart3->rhr; //TODO

    if (xQueueSendFromISR(xRxedChars, &cChar, pdFALSE))
    {
      xTaskWokenByRx = pdTRUE;
    }
  }
  MovToR12(( xTaskWokenByTx || xTaskWokenByRx ));
  
  portEXIT_SWITCHING_ISR(INT1);
}
#endif


/*
 * Init the serial port for the Minimal implementation.
 */
xComPortHandle xSerialPortInitMinimal( unsigned portLONG ulWantedBaud, unsigned portBASE_TYPE uxQueueLength )
{
  xComPortHandle    xReturn = serHANDLE;
  volatile avr32_usart_t  *usart3 = &AVR32_USART3;
  extern void _evba ;
  unsigned long     usart3group = AVR32_USART3_IRQ/32;
  int                           cd; /* USART3 Clock Divider. */
  unsigned long     offset = ((unsigned long)((void *)&vUSART3_ISR - &_evba) & 0x00003FFF);
  volatile avr32_intc_t   *intc = &AVR32_INTC;


  avr32_piomap_t usart_piomap = {				   \
    {AVR32_USART3_RXD_0_PIN, AVR32_USART3_RXD_0_FUNCTION}, \
    {AVR32_USART3_TXD_0_PIN, AVR32_USART3_TXD_0_FUNCTION}   \
  };

  /* Create the rx and tx queues. */
  vprvSerialCreateQueues( uxQueueLength, &xRxedChars, &xCharsForTx );

  /* Configure USART3. */
  if( ( xRxedChars != serINVALID_QUEUE ) && 
      ( xCharsForTx != serINVALID_QUEUE ) && 
      ( ulWantedBaud != ( unsigned portLONG ) 0 ) )
  {
    portENTER_CRITICAL();
    {
      /**
       ** Reset USART3.
       **/
      /* Disable all USART3 interrupt sources to begin... */
      usart3->idr = 0xFFFFFFFF;

      /* Reset mode and other registers that could cause unpredictable
         behaviour after reset */
      usart3->mr = 0; /* Reset Mode register. */
      usart3->rtor = 0; /* Reset Receiver Time-out register. */
      usart3->ttgr = 0; /* Reset Transmitter Timeguard register. */ 

      /* Shutdown RX and TX, reset status bits, reset iterations in CSR, reset NACK
         and turn off DTR and RTS */
      usart3->cr = (1 << AVR32_USART_CR_RSTRX_OFFSET) |
                  (1 << AVR32_USART_CR_RSTTX_OFFSET) |
                  (1 << AVR32_USART_CR_RXDIS_OFFSET) |
                  (1 << AVR32_USART_CR_TXDIS_OFFSET) |
                  (1 << AVR32_USART_CR_RSTSTA_OFFSET) |
                  (1 << AVR32_USART_CR_RSTIT_OFFSET) |
                  (1 << AVR32_USART_CR_RSTNACK_OFFSET) |
                  (1 << AVR32_USART_CR_DTRDIS_OFFSET) |
                  (1 << AVR32_USART_CR_RTSDIS_OFFSET);

      /**
       ** Configure USART3.
       **/
      /* Enable USART3 RXD & TXD pins. */
        // Setup pio for USART
      pio_enable_module(usart_piomap, 2);


      /* Set the USART3 baudrate to be as close as possible to the wanted baudrate. */
      /*
       *             ** BAUDRATE CALCULATION **
       *
       *                 Selected Clock                       Selected Clock
       *     baudrate = ----------------   or     baudrate = ----------------
       *                    16 x CD                              8 x CD
       *
       *       (with 16x oversampling)              (with 8x oversampling)
       */
      if ( ulWantedBaud < (configCPU_CLOCK_HZ/16)  ){
        /* Use 8x oversampling */
  usart3->mr |= (1<<AVR32_USART_MR_OVER_OFFSET);
        cd = configCPU_CLOCK_HZ / (8*ulWantedBaud);

        if (cd < 2) {
          return serINVALID_COMPORT_HANDLER;
        }
        usart3->brgr = (cd << AVR32_USART_BRGR_CD_OFFSET);
      } else {
        /* Use 16x oversampling */
        usart3->mr &= ~(1<<AVR32_USART_MR_OVER_OFFSET);
        cd =  configCPU_CLOCK_HZ / (16*ulWantedBaud);

        if (cd > 65535) {
          /* Baudrate is too low */
          return serINVALID_COMPORT_HANDLER;
        }
      }
      usart3->brgr = (cd << AVR32_USART_BRGR_CD_OFFSET);

      /* Set the USART3 Mode register: Mode=Normal(0), Clk selection=MCK(0), 
   CHRL=8,  SYNC=0(asynchronous), PAR=None, NBSTOP=1, CHMODE=0, MSBF=0,
         MODE9=0, CKLO=0, OVER(previously done when setting the baudrate), 
         other fields not used in this mode. */
      usart3->mr |= ((8-5) << AVR32_USART_MR_CHRL_OFFSET) |
        (4 << AVR32_USART_MR_PAR_OFFSET) |
        (1 << AVR32_USART_MR_NBSTOP_OFFSET);

      /* Write the Transmit Timeguard Register */
      usart3->ttgr = 0;

      /* Setup the interrupt for USART3.
         Register the USART3 interrupt handler to the interrupt controller 
         at interrupt level 1. */
      intc->intpr[usart3group] = (INT1 << INTLEV) | offset;
      //intc_register_interrupt(vUSART3_ISR,AVR32_USART1_IRQ/32,INT1);
      /* Enable USART3 interrupt sources (but not Tx for now)... */
      usart3->ier = AVR32_USART_IER_RXRDY_MASK;

      /* Enable receiver and transmitter... */
      usart3->cr |= (1<<AVR32_USART_CR_TXEN_OFFSET) |
                   (1<<AVR32_USART_CR_RXEN_OFFSET);

      //Enable level INT1 interrupts.
      __builtin_mtsr(AVR32_SR, __builtin_mfsr(AVR32_SR) & ~((AVR32_SR_I0M_MASK << INT1)));
    }
    portEXIT_CRITICAL();
  }
  else
  {
    xReturn = serINVALID_COMPORT_HANDLER;
  }

  return xReturn;
}
/*-----------------------------------------------------------*/

signed portBASE_TYPE xSerialGetChar( xComPortHandle pxPort, signed portCHAR *pcRxedChar, portTickType xBlockTime )
{
  /* The port handle is not required as this driver only supports UART0. */
  ( void ) pxPort;

  /* Get the next character from the buffer.  Return false if no characters
  are available, or arrive before xBlockTime expires. */
  if( xQueueReceive( xRxedChars, pcRxedChar, xBlockTime ) )
  {
    return pdTRUE;
  }
  else
  {
    return pdFALSE;
  }
}
/*-----------------------------------------------------------*/

void vSerialPutString( xComPortHandle pxPort, const signed portCHAR * const pcString, unsigned portSHORT usStringLength )
{
signed portCHAR *pxNext;

  /* NOTE: This implementation does not handle the queue being full as no
  block time is used! */

  /* The port handle is not required as this driver only supports UART0. */
  ( void ) pxPort;

  /* Send each character in the string, one at a time. */
  pxNext = ( signed portCHAR * ) pcString;
  while( *pxNext )
  {
    xSerialPutChar( pxPort, *pxNext, serNO_BLOCK );
    pxNext++;
  }
}
/*-----------------------------------------------------------*/

signed portBASE_TYPE xSerialPutChar( xComPortHandle pxPort, signed portCHAR cOutChar, portTickType xBlockTime )
{
volatile avr32_usart_t  *usart3 = &AVR32_USART3;

  /* Place the character in the queue of characters to be transmitted. */
  if( xQueueSend( xCharsForTx, &cOutChar, xBlockTime ) != pdPASS )
  {
    return pdFAIL;
  }

  /* Turn on the Tx interrupt so the ISR will remove the character from the
  queue and send it.   This does not need to be in a critical section as
  if the interrupt has already removed the character the next interrupt
  will simply turn off the Tx interrupt again. */
        usart3->ier |= (1 << AVR32_USART_IER_TXRDY_OFFSET);

  return pdPASS;
}
/*-----------------------------------------------------------*/

void vSerialClose( xComPortHandle xPort )
{
  /* Not supported as not required by the demo application. */
}
/*-----------------------------------------------------------*/

/*###########################################################*/

/* 
 * Create the rx and tx queues. 
 */
static void vprvSerialCreateQueues(  unsigned portBASE_TYPE uxQueueLength, xQueueHandle *pxRxedChars, xQueueHandle *pxCharsForTx )
{
  /* Create the queues used to hold Rx and Tx characters. */
  xRxedChars = xQueueCreate( uxQueueLength, ( unsigned portBASE_TYPE ) sizeof( signed portCHAR ) );
  xCharsForTx = xQueueCreate( uxQueueLength + 1, ( unsigned portBASE_TYPE ) sizeof( signed portCHAR ) );

  /* Pass back a reference to the queues so the serial API file can
  post/receive characters. */
  *pxRxedChars = xRxedChars;
  *pxCharsForTx = xCharsForTx;
}
/*-----------------------------------------------------------*/

