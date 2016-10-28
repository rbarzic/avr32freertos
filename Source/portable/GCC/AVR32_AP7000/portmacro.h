/*      AVR32 specific FreeRTOS portmacro.h  file      
        Adaptated from ARM7 port.c/portISR.c files Copyright (C) 2003-2005 Richard Barry.
        AVR32 adaptation  Copyright (C) 2006 Ronan BARZIC  

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

#ifndef PORTMACRO_H
#define PORTMACRO_H

/*-----------------------------------------------------------
 * Port specific definitions.  
 *
 * The settings in this file configure FreeRTOS correctly for the
 * given hardware and compiler.
 *
 * These settings should not be altered.
 *-----------------------------------------------------------
 */

#include "avr32_circuit.h"


/* Type definitions. */
#define portCHAR		char
#define portFLOAT		float
#define portDOUBLE		double
#define portLONG		long
#define portSHORT		short
#define portSTACK_TYPE	unsigned portLONG
#define portBASE_TYPE	portLONG

#if( configUSE_16_BIT_TICKS == 1 )
	typedef unsigned portSHORT portTickType;
	#define portMAX_DELAY ( portTickType ) 0xffff
#else
	typedef unsigned portLONG portTickType;
	#define portMAX_DELAY ( portTickType ) 0xffffffff
#endif
/*-----------------------------------------------------------*/	

/* Architecture specifics. */
#define portSTACK_GROWTH			( -1 )
#define portTICK_RATE_MS			( ( portTickType ) 1000 / configTICK_RATE_HZ )		
#define portBYTE_ALIGNMENT			4
#define portNOP()                               asm volatile ( "NOP" );
/*-----------------------------------------------------------*/	

#define EX_INT0  0
#define EX_INT1  1
#define EX_INT2  2
#define EX_INT3  3
#define EX_SCALL 4

#define GetRAR(exception) \
if(exception==EX_INT0)   asm volatile ("mfsr r0,%0" :: "g" (AVR32_RAR_INT0)); \
if(exception==EX_INT1)   asm volatile ("mfsr r0,%0" :: "g" (AVR32_RAR_INT1)); \
if(exception==EX_INT2)   asm volatile ("mfsr r0,%0" :: "g" (AVR32_RAR_INT2)); \
if(exception==EX_INT3)   asm volatile ("mfsr r0,%0" :: "g" (AVR32_RAR_INT3)); \
if(exception==EX_SCALL)  asm volatile ("mfsr r0,%0" :: "g" (AVR32_RAR_SUP)); \

#define GetRSR(exception) \
if(exception==EX_INT0)   asm volatile ("mfsr r0,%0" :: "g" (AVR32_RSR_INT0)); \
if(exception==EX_INT1)   asm volatile ("mfsr r0,%0" :: "g" (AVR32_RSR_INT1)); \
if(exception==EX_INT2)   asm volatile ("mfsr r0,%0" :: "g" (AVR32_RSR_INT2)); \
if(exception==EX_INT3)   asm volatile ("mfsr r0,%0" :: "g" (AVR32_RSR_INT3)); \
if(exception==EX_SCALL)  asm volatile ("mfsr r0,%0" :: "g" (AVR32_RSR_SUP)); \

#define SetRAR(exception) \
if(exception==EX_INT0)   asm volatile ("mtsr %0,r0" :: "g" (AVR32_RAR_INT0)); \
if(exception==EX_INT1)   asm volatile ("mtsr %0,r0" :: "g" (AVR32_RAR_INT1)); \
if(exception==EX_INT2)   asm volatile ("mtsr %0,r0" :: "g" (AVR32_RAR_INT2)); \
if(exception==EX_INT3)   asm volatile ("mtsr %0,r0" :: "g" (AVR32_RAR_INT3)); \
if(exception==EX_SCALL)  asm volatile ("mtsr %0,r0" :: "g" (AVR32_RAR_SUP)); \

#define SetRSR(exception) \
if(exception==EX_INT0)   asm volatile ("mtsr %0,r0" :: "g" (AVR32_RSR_INT0)); \
if(exception==EX_INT1)   asm volatile ("mtsr %0,r0" :: "g" (AVR32_RSR_INT1)); \
if(exception==EX_INT2)   asm volatile ("mtsr %0,r0" :: "g" (AVR32_RSR_INT2)); \
if(exception==EX_INT3)   asm volatile ("mtsr %0,r0" :: "g" (AVR32_RSR_INT3)); \
if(exception==EX_SCALL)  asm volatile ("mtsr %0,r0" :: "g" (AVR32_RSR_SUP)); \

#define ReturnFromException(exception) \
if(exception==EX_INT0)   asm volatile ("rete"); \
if(exception==EX_INT1)   asm volatile ("rete"); \
if(exception==EX_INT2)   asm volatile ("rete"); \
if(exception==EX_INT3)   asm volatile ("rete"); \
if(exception==EX_SCALL)  asm volatile ("rets"); \

// #define FRS  asm volatile ("frs")
#define FRS  asm volatile ("nop")





//  Stack layout
//  R14/LR -4
//  R0     -8 
//  R1     -12
//  R2    -16
//  R3    -20
//  R4    -24
//  R5    -28
//  R6    -32
//  R7    -36
//  R8    -40
//  R9    -44
//  R10   -48
//  R11   -52
//  R12   -56
//  R13/SP -60
//  R5/PC  -64
//  SR
//  ulCriticalNesting



/*==========================================================*/
/*               !!! NOTE !!!                               */
/* The following assembly codes can certainly be optimized  */
/* I didn't try to go further one it worked (no time ....)  */
/* LR is saved by TaskYield call                            */
/*==========================================================*/
/* FIX : R1 will contain SR before scall and disable interrupt */

#define portSAVE_CONTEXT_SCALL    \
{\
extern volatile void * volatile pxCurrentTCB;        \
extern volatile unsigned portLONG ulCriticalNesting; \
\
 /* Then all other registers */\
 \
   asm volatile ("st.w --sp,lr"); \
   asm volatile ("stm --sp,r0-r12"); /* LR is already pushed */\
 /* Get the status register in r10 (we do it before arithmetic operation)  */ \
   asm volatile ("mfsr r10,0");  \
   asm volatile ("sub sp,sp,4"); /* Skip SP */\
\
/* Test current CPU mode */ \
   asm volatile ("bfextu r11,r10,22,3"); /* Extract M2,M1,M0 */\ 
   asm volatile ("cp.w r11,1"); /* Check if {M2,M1,M0} = 001b */ \
   /* If equal, we came from SUP  or APP mode - Only SUP supported, other configuration is a error*/ \
   asm volatile ("__SAVE_SCALL_NOT_SUP: brne __SAVE_SCALL_NOT_SUP"); \
   GetRAR(EX_SCALL); /* RAR_SUP -> r0 */ \
   asm volatile ("st.w  --sp,r0");\
   /* GetRSR(EX_SCALL);*/  \  
   asm volatile ("st.w  --sp,r1");\
 /* Save ulCriticalNesting variable */ \
 asm volatile ("mov r0,%0" : : "r" (&ulCriticalNesting)); \
 asm volatile ("ld.w r0,r0[0]"); /* r0 = [ulCriticalNesting] */ \
 asm volatile ("st.w --sp,r0"); \
 /* Store SP in pxCurrentTCB */ \
 asm volatile ("ld.w  R0,%0" : : "m" (pxCurrentTCB));  \
 asm volatile ("st.w r0[0],sp"); \
\
}







  // TCB layout
  // R0
  // R1
  // ...
  // SP
  // LR
  // PC
  // SR
  // ulCriticalNesting

/*==========================================================*/
/*               !!! NOTE !!!                               */
/* The following assembly codes can certainly be optimized  */
/* I didn't try to go further one it worked (no time ....)  */
/*==========================================================*/



#define portSAVE_CONTEXT(exception)											\
{\
extern volatile void * volatile pxCurrentTCB;							\
extern volatile unsigned portLONG ulCriticalNesting;					\
\
  /* Save All registers                                         */                  \
  /* We store PC but it is not the good value, but this way R0 always */\ 
  /* points to start of buffer */ \
 \
    asm volatile ("st.w --sp,lr");\
    asm volatile ("st.w --sp,r0");\
    asm volatile ("sub r0,sp,-4");\
    asm volatile ("stm --sp,r1-r12");\
    asm volatile ("st.w --sp,r0");\
      /* PC of interrupted code is stored in RAR_INTx register */ \
  /* SR of interrupted code is stored in RSR_INTx register */ \
  GetRAR(exception); \
  asm volatile ("st.w --sp,r0"); /* PC (the last reg pushed by stm) */ \
  GetRSR(exception); \
  asm volatile ("st.w --sp,r0"); /* push SR */ \
\
  /* Save ulCriticalNesting variable */ \
  asm volatile ("mov r0,%0" : : "r" (&ulCriticalNesting)); \
  asm volatile ("ld.w r0,r0[0]"); /* r0 = [ulCriticalNesting] */ \
  asm volatile ("st.w --sp,r0"); \
  /* Store SP in pxCurrentTCB */ \
  asm volatile ("ld.w  R0,%0" : : "m" (pxCurrentTCB));  \
  asm volatile ("st.w r0[0],sp"); \
\
\
\
}



#define portSAVE_CONTEXT_NOIRQ											\
{\
extern volatile void * volatile pxCurrentTCB;							\
extern volatile unsigned portLONG ulCriticalNesting;					\
\
  /* Save All registers                                         */                  \
  /* We store PC but it is not the good value, but this way R0 always */\ 
  /* points to start of buffer */ \
 \
    asm volatile ("st.w --sp,lr");\
    asm volatile ("st.w --sp,r0");\
    asm volatile ("sub r0,sp,-4");\
    asm volatile ("stm --sp,r1-r12");\
    asm volatile ("st.w --sp,r0");\
      /* PC of interrupted code is stored in RAR_INTx register */ \
  /* SR of interrupted code is stored in RSR_INTx register */ \
  GetRAR(exception); \
  asm volatile ("st.w --sp,r0"); /* PC (the last reg pushed by stm) */ \
  GetRSR(exception); \
  asm volatile ("st.w --sp,r0"); /* push SR */ \
\
  /* Save ulCriticalNesting variable */ \
  asm volatile ("mov r0,%0" : : "r" (&ulCriticalNesting)); \
  asm volatile ("ld.w r0,r0[0]"); /* r0 = [ulCriticalNesting] */ \
  asm volatile ("st.w --sp,r0"); \
  /* Store SP in pxCurrentTCB */ \
  asm volatile ("ld.w  R0,%0" : : "m" (pxCurrentTCB));  \
  asm volatile ("st.w r0[0],sp"); \
\
}








#define portRESTORE_CONTEXT(exception)											\
{\
extern volatile void * volatile pxCurrentTCB;							\
extern volatile unsigned portLONG ulCriticalNesting;					\
\
  /* Restore  All registers                                         */\
 \
  /* Set sp to point to new stack */ \
  asm volatile ("ld.w r0,%0" : : "m" (pxCurrentTCB)); \
  asm volatile ("ld.w sp,r0"); \
  \
    /* Restore ulCriticalNesting variable */\
  asm volatile ("mov r0,%0" : : "r" (&ulCriticalNesting));\
  asm volatile ("ld.w r1,sp++"); \
  asm volatile ("st.w r0[0],r1"); /* [ulCriticalNesting] = r0*/\
  \
  /* Restore status register by writing RSR_INTx */\
  /* SR will be updated by the reti instruction  */\
  asm volatile ("ld.w r0,sp++"); /* r0 = SR from stack */\ 
  SetRSR(exception); /* r0 -> RSR_INTx */\
\
  /* restore PC (restore PC when we'll exit the irq context) */\
  /* 'real' PC will be updated by the reti instruction  */\
  asm volatile ("ld.w r0,sp++"); /* r0 = PC from stack */\
  SetRAR(exception); /* r0 -> RAR_INTx */\
\
\
  /* now we can restore R12.....R0, LR*/\
\
asm volatile ("subal sp,-4");  \
asm volatile ("ldm sp++,r0-r12"); \
asm volatile ("ld.w lr,sp++"); \
FRS;\
ReturnFromException(exception)\
 \
}





// Required only by vPortISRStartFirstTask
#define portRESTORE_CONTEXT_NOIRQ()											\
{\
extern volatile void * volatile pxCurrentTCB;							\
extern volatile unsigned portLONG ulCriticalNesting;					\
 \
  /* Set sp to point to new stack */ \
  /* TopOfStack fiel is the first in TCB struct */ \
  asm volatile ("ld.w r0,%0" : : "m" (pxCurrentTCB)); \
  asm volatile ("ld.w sp,r0"); \
  \
    /* Restore ulCriticalNesting variable */\
  asm volatile ("mov r0,%0" : : "r" (&ulCriticalNesting));\
  asm volatile ("ld.w r1,sp++"); \
  asm volatile ("st.w r0[0],r1"); /* [ulCriticalNesting] = r1*/\
  \
  /* Restore status register */\
\
  asm volatile ("ld.w r1,sp++"); /* r1 = SR from stack */\
  asm volatile ("mtsr 0,r1");    /* r1 = SR from stack */\
 \
\
  asm volatile ("ld.w lr,sp++"); /* PC will go in lr */ \
  asm volatile ("subal sp,-4"); /* Skip SP */\
      \
  /* now we can restore R12.....R0, SP will be updated automatically */\
\
asm volatile ("ldm sp++,r0-r12"); \
asm volatile ("subal sp,-4"); /* Skip LR */ \
FRS;\
asm volatile ("mov pc,lr"); /* Jump to task start address */\
 \
}



#define portRESTORE_CONTEXT_SCALL  \
{\
extern volatile void * volatile pxCurrentTCB;  \
extern volatile unsigned portLONG ulCriticalNesting;   \
\
 /* Restore  All registers                                         */\
 \
 /* Set sp to point to new stack */ \
 asm volatile ("ld.w r0,%0" : : "m" (pxCurrentTCB)); \
 asm volatile ("ld.w sp,r0"); \
 \
   /* Restore ulCriticalNesting variable */\
 asm volatile ("mov r0,%0" : : "r" (&ulCriticalNesting));\
 asm volatile ("ld.w r1,sp++"); \
 asm volatile ("st.w r0[0],r1"); /* [ulCriticalNesting] = r0*/\
 \
 /* Get the **CURRENT** status register in r10  */ \
 asm volatile ("mfsr r10,0");  \
 asm volatile ("bfextu r11,r10,22,3"); /* Extract M2,M1,M0 */\ 
 asm volatile ("cp.w r11,1"); /* Check if {M2,M1,M0} = 001b */ \
 asm volatile ("brne __RESTORE_SCALL_NOT_SUP"); /* Check if {M2,M1,M0} = 001b */ \
 \
 /* Restore status register by writing RSR_SUP */\
 /* SR will be updated by the reti instruction  */\
 asm volatile ("ld.w r0,sp++"); /* r0 = SR from stack */\
 SetRSR(EX_SCALL); /* r0 -> RSR_SUP */\
\
 /* restore PC (restore PC when we'll exit with rets) */\
 /* 'real' PC will be updated by the rets instruction  */\
 asm volatile ("ld.w r0,sp++"); /* r0 = PC from stack */\
 SetRAR(EX_SCALL); /* r0 -> RAR_SUP */\
\
\
 /* now we can restore R12.....R0 */\
\
asm volatile ("subal sp,-4");  /*skip SP */ \
asm volatile ("ldm sp++,r0-r12"); \
 asm volatile ("ld.w lr,sp++"); \
FRS;\
ReturnFromException(EX_SCALL)\
asm volatile ("__RESTORE_SCALL_NOT_SUP: rjmp __RESTORE_SCALL_NOT_SUP");  /* Error : scall should never be called from a ISR */ \
}







/*-----------------------------------------------------------
 * ISR entry and exit macros.  These are only required if a task switch
 * is required from the ISR.
 *----------------------------------------------------------*/


#define portENTER_SWITCHING_ISR(exception) \
extern volatile void * volatile pxCurrentTCB;							\
extern volatile unsigned portLONG ulCriticalNesting;					\
\
  /* Save All registers                                         */                  \
  /* We store PC but it is not the good value, but this way R0 always */\ 
  /* points to start of buffer */ \
 \
    asm volatile ("st.w --sp,lr");\
    asm volatile ("st.w --sp,r0");\
    asm volatile ("sub r0,sp,-4");\
    asm volatile ("stm --sp,r1-r12");\
    asm volatile ("st.w --sp,r0");\
      /* PC of interrupted code is stored in RAR_INTx register */ \
  /* SR of interrupted code is stored in RSR_INTx register */ \
  GetRAR(exception); \
  asm volatile ("st.w --sp,r0"); /* PC (the last reg pushed by stm) */ \
  GetRSR(exception); \
  asm volatile ("st.w --sp,r0"); /* push SR */ \
\
  /* Save ulCriticalNesting variable */ \
  asm volatile ("mov r0,%0" : : "r" (&ulCriticalNesting)); \
  asm volatile ("ld.w r0,r0[0]"); /* r0 = [ulCriticalNesting] */ \
  asm volatile ("st.w --sp,r0"); \
  GetRSR(exception);  /* Read SR in RSR */\
   __asm__ __volatile__ ("bfextu  r0, r0, 22, 3"); /* Extract the mode bits to R0. */\
 __asm__ __volatile__ ("cp.w    r0, 1"); /* Compare the mode bits with supervisor mode(b'001) */\
 __asm__ __volatile__ ("brhi    LABEL_ISR_SKIP_SAVE_CONTEXT%[LINE]" :  : [LINE] "i" (__LINE__));\
  /* Store SP in pxCurrentTCB */ \
  asm volatile ("ld.w  R0,%0" : : "m" (pxCurrentTCB));  \
  asm volatile ("st.w r0[0],sp"); \
  __asm__ __volatile__ ("LABEL_ISR_SKIP_SAVE_CONTEXT%[LINE]:" :  : [LINE] "i" (__LINE__));\



#define portEXIT_SWITCHING_ISR(exception)						\
extern volatile void * volatile pxCurrentTCB;							\
extern volatile unsigned portLONG ulCriticalNesting;	\
/* If a switch is required then we just need to call */			\
  /* vTaskSwitchContext() as the context has already been */		\
  /* saved. */		 \
  GetRSR(exception);  /* Read SR in RSR */\
   __asm__ __volatile__ ("bfextu  r0, r0, 22, 3"); /* Extract the mode bits to R0. */\
 __asm__ __volatile__ ("cp.w    r0, 1"); /* Compare the mode bits with supervisor mode(b'001) */\
 __asm__ __volatile__ ("brhi    LABEL_ISR_SKIP_RESTORE_CONTEXT%[LINE]" :  : [LINE] "i" (__LINE__));\
\
__asm__ __volatile__ ("cp.w    r12, 1"); /* Check if Switch context is required. */\
 __asm__ __volatile__ ("brne    LABEL_ISR_RESTORE_CONTEXT%[LINE]" :  : [LINE] "i" (__LINE__));\
\
/* A critical section has to be used here because vTaskSwitchContext handles FreeRTOS linked lists. */\
 portENTER_CRITICAL();\
 vTaskSwitchContext();\
 portEXIT_CRITICAL();\
\
 __asm__ __volatile__ ("LABEL_ISR_RESTORE_CONTEXT%[LINE]:" :  : [LINE] "i" (__LINE__));\
 /* Restore the context of which ever task is now the highest */\
  /* Set sp to point to new stack */ \
  asm volatile ("ld.w r0,%0" : : "m" (pxCurrentTCB)); \
  asm volatile ("ld.w sp,r0"); \
  \
__asm__ __volatile__ ("LABEL_ISR_SKIP_RESTORE_CONTEXT%[LINE]:" :  : [LINE] "i" (__LINE__));\
    /* Restore ulCriticalNesting variable */\
  asm volatile ("mov r0,%0" : : "r" (&ulCriticalNesting));\
  asm volatile ("ld.w r1,sp++"); \
  asm volatile ("st.w r0[0],r1"); /* [ulCriticalNesting] = r0*/\
  \
  /* Restore status register by writing RSR_INTx */\
  /* SR will be updated by the reti instruction  */\
  asm volatile ("ld.w r0,sp++"); /* r0 = SR from stack */\ 
  SetRSR(exception); /* r0 -> RSR_INTx */\
\
  /* restore PC (restore PC when we'll exit the irq context) */\
  /* 'real' PC will be updated by the reti instruction  */\
  asm volatile ("ld.w r0,sp++"); /* r0 = PC from stack */\
  SetRAR(exception); /* r0 -> RAR_INTx */\
\
  /* now we can restore R12.....R0, LR*/\
\
asm volatile ("subal sp,-4");  \
asm volatile ("ldm sp++,r0-r12"); \
asm volatile ("ld.w lr,sp++"); \
FRS;\
ReturnFromException(exception);\







#define portYIELD()     { \
 asm volatile ("st.w --sp,r1"); \
 asm volatile ("mfsr r1,0"); /* SR -> R1 */ \
 asm volatile ("st.w --sp,r1"); \
 DISABLE_ALL_INTERRUPTS;  \
 asm volatile ("scall");  \
 asm volatile ("ld.w r1,sp++"); \
 asm volatile ("mtsr 0,r1"); \
 asm volatile ("ld.w r1,sp++"); \
 }

//#define portYIELD()					asm volatile("scall ");
/*-----------------------------------------------------------*/


/* Critical section management. */


#define portDISABLE_INTERRUPTS()  DISABLE_ALL_INTERRUPTS
#define portENABLE_INTERRUPTS()   ENABLE_ALL_INTERRUPTS




extern void vPortEnterCritical( void );
extern void vPortExitCritical( void );

#define portENTER_CRITICAL()   vPortEnterCritical();
#define portEXIT_CRITICAL()    vPortExitCritical();
/*-----------------------------------------------------------*/

/* Task function macros as described on the FreeRTOS.org WEB site. */
#define portTASK_FUNCTION_PROTO( vFunction, pvParameters ) void vFunction( void *pvParameters )
#define portTASK_FUNCTION( vFunction, pvParameters ) void vFunction( void *pvParameters )

#endif /* PORTMACRO_H */

