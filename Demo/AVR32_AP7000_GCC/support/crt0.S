/*      C startup file for the AVR32 AP7000 processor
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
	
#define STACK_SIZE  1000
#undef __AVR32_ABI_COMPILER__
#include "avr32_circuit.h"

	.section .reset,"ax"
	.global _start
_start:

        mov	r0, 0
	cache	r0[4], 8
	cache	r0[0], 0
	nop
	nop
	nop
	nop
	sync	0
	nop
        lddpc   r0,exit_start
	mov     lr,r0

// Only useful when debugging FreeRTOS to be sure the bug is not related to
// the return stack		
#ifdef  NO_RETURN_STACK
	mfsr r0, AVR32_CPUCR
	cbr  r0, AVR32_CPUCR_RE 	
	mtsr AVR32_CPUCR,r0
#endif	

	
_set_sr:			                
	lddpc r0,init_sr
	mtsr AVR32_SR,r0

_set_stack1:	
	/* Set initial stack pointer */
	lddpc   sp, sram_start_addr
	sub	sp, -STACK_SIZE        

mcall	.LAddressSDRAMInitInC             
        
_copy_data:	
	lddpc	r12, text_end_addr    /* LMA start - see linker file*/
	lddpc	r11, data_start_addr  /* VMA start */
	lddpc	r10, data_end_addr    /* VMA END */
	
3:      ld.w    r9,r12++
	st.w    r11++,r9
	cp      r11, r10
	brlo    3b	
	
		
_clear_bss:	
		/* Clear BSS */
	lddpc	r2, bss_start_addr
	lddpc   r3, bss_end_addr
	mov	r0, 0
	mov	r1, 0
0:      st.d    r2++, r0
	cp      r2, r3
	brlo    0b
	
_set_stack2:	
	/* Set initial stack pointer */
	lddpc   sp, stack_addr
	sub	sp, -STACK_SIZE

_set_evba:		
	/* Set up EVBA so Interrupts can be enabled. */
	mov R0,lo(__evba_start__)		
	orh R0,hi(__evba_start__)
	mtsr AVR32_EVBA,R0

_set_acba:		
	mov R0,lo(__acba_start__)		
	orh R0,hi(__acba_start__)
	mtsr AVR32_ACBA,R0

        lddpc   r0,exit_start
	mov     lr,r0
        
	/* Start the show */
_go:	lddpc   pc, main_start
	
	.align  4
init_sr:
	/* Supervisor mode,  */
	.long   0x005f0000

text_end_addr:	
	.long __text_end__
	
data_start_addr:
	.long   __data_start__
data_end_addr:
	.long   __data_end__	
	        
bss_start_addr:
	.long   __bss_start__
bss_end_addr:
	.long   __bss_end__
stack_addr:
	.long    __stack_start__

sram_end_addr:  
        .long __sram_end__
sram_start_addr:  
        .long __sram_start__        
		//.long   init_thread_union
main_start:
	.long	main
	
exit_start:
	.long	exit

	.align 4
.LAddressSDRAMInitInC:	
	.word 	init_sdram_startup
        

	
	.section .exit, "ax"
	.public exit

exit:   rjmp exit


	.section	.exception,"ax"
        .align 4   
        // Make symbol accessible from outside
	.global _evba
        
//  Start of Exception Vectors

// Note :  Exceptions don't used banked registers
        
 _evba:
        
        .org  0x0
        // Unrecoverable exception 
_handle_unrecoverable_exception:      
	rjmp	_handle_exception	  
        
        
        .org	0x4
        // TLB Multiple Hit
_handle_TLB_multiple_hit:
	rjmp _handle_exception

        
        .org	0x8
        // Bus error data fetch
_handle_bus_error_data_fetch:
	rjmp _handle_exception

        	.org	0xc
         // Bus error instruction fetch
_handle_bus_error_instruction_fetch:
        rjmp _handle_exception
        

        .org	0x10
        // NMI 
_handle_nmi:
        rjmp _handle_nmi
        
        .org	0x14
        // Instruction Address
_handle_instruction_address:
        rjmp _handle_exception
                
        .org	0x18
        // ITLB Protection
_handle_ITLB_protection:
        rjmp _handle_exception
        
        	.org	0x1c
        // Breakpoint
_handle_Breakpoint:
        rjmp _handle_Breakpoint

        	.org	0x20
        // Illegal OpCode
_handle_Ilegal_Opcode:
        rjmp _handle_exception
         

        	.org	0x24
        // Unimplemented instruction
_handle_Unimplemented_instruction:
        rjmp  _handle_exception
        
                
        	.org	0x28
        // Privilege violation
_handle_Privilege_violation:
        rjmp _handle_exception
        

        	.org	0x2c
        // Floating point
_handle_Floatting_point:
        rjmp _handle_exception

        	.org	0x30
        // Coprocessor absent
_handle_Coprocessor_absent:
        rjmp _handle_exception

        .org	0x34
        // Data Address Read
_handle_data_address_read:
	rjmp _handle_exception

        .org	0x38
        // Data Address Write
_handle_data_address_write:
        rjmp  _handle_exception

        .org	0x3c
        // DTLB Protection Read
_handle_DTLB_protection_read:
        rjmp _handle_exception

        .org	0x40
        // DTLB Protection Write
_handle_DTLB_protection_write:
        rjmp _handle_exception
        	
        .org	0x44
        // DTLB Modified
_handle_DTLB_modified:
        rjmp _handle_exception

        .org	0x50
        // ITLB Miss
_handle_ITLB_miss:
        rjmp  _handle_exception
        
        .org	0x60
        // DTLB Miss Read
_handle_DTLB_miss_read:
        rjmp _handle_exception

        .org	0x70
        // DTLB miss Write
_handle_DTLB_miss_write:
        rjmp _handle_exception
 
        .org	0x100
        // Supervisor call
_handle_Supervisor_Call:        
	mcall	.Lapplication__supervisor_call 



.Lapplication__supervisor_call:	
	.word	SCALLYield	

_handle_exception:
	mfsr r12,AVR32_ECR
	nop
	nop
	nop
	nop
	nop
	mcall .LAddressHandleExceptionInC
	nop
	rjmp _handle_exception

	.align 4
.LAddressHandleExceptionInC:	
	.word 	HandleExceptionInC

	
