/*      Linker file for the AVR32 port of FreeRTOS  
        Copyright (C) 2006  Ronan BARZIC  

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

OUTPUT_FORMAT("elf32-avr32", "elf32-avr32",
	      "elf32-avr32")
OUTPUT_ARCH(avr32:at32ap7000)
ENTRY(_start)
SEARCH_DIR("/usr/avr32/lib");

__gcc_stack_size      = 8K;

MEMORY
  {
    FLASH (rxai) : ORIGIN = 0x00000000, LENGTH = 0x4000000
    SRAM (rwxa) : ORIGIN = 0x24000000, LENGTH = 32K
    SDRAM (rwxa) : ORIGIN = 0x10000000, LENGTH = 8M     
  }
SECTIONS
{
  /* Read-only sections, merged into text segment: */
  PROVIDE (__executable_start = 0xa0000000); . = 0xa0000000;

  .reset : {  *(.reset) } >FLASH AT>FLASH
    . = . & 0x1fffffff;
  PROVIDE ( _exit =  ALIGN(0x100)); . = ALIGN(0x100);
    .exit ALIGN(0x100) : { *(.exit) }  >FLASH AT>FLASH

  PROVIDE (__evba_start__ = ALIGN(0x200)); .  = ALIGN(0x200);
     .exception ALIGN(0x200) : { *(.exception) } >FLASH AT>FLASH

  PROVIDE (__handlers_ = ALIGN(0x600)); . =  ALIGN(0x600);
     .handlers ALIGN(0x600) : {*(.handlers) } >FLASH AT>FLASH     

  PROVIDE (__acba_start__ = ALIGN(0x800)); .  = ALIGN(0x800);
     .acba  ALIGN(0x800) : { *(.acba)  } >FLASH AT>FLASH   
  
PROVIDE (__text_start__ = .);
     .text : 
       { 
                *(.text); 
                *(.rodata); 
                *(.rodata.*); 
		 *(.text.*)
		*(.gnu.linkonce.t*) 
                *(.gnu.linkonce.r*) 
       }   >FLASH AT>FLASH         

  PROVIDE (__text_end__ = .);

  PROVIDE (__data_start__  = ORIGIN(SDRAM));
  . = ORIGIN(SDRAM);
  .data  :  { 
        *(.data) 
	*(.data.*)
        *(.gnu.linkonce.d*) 
        }   >SDRAM AT>FLASH
  PROVIDE (__data_end__ = ALIGN(4)); . = ALIGN(4);


  PROVIDE (__bss_start__ = ALIGN(4));  . = ALIGN(4);
      .bss   :   { 
                *(.bss) 
        }  >SDRAM AT>FLASH         
  PROVIDE (__bss_end__ = ALIGN(4));

        . = ALIGN(4);
  PROVIDE (__stack_start__ = ALIGN(4));        
  PROVIDE ( __stack_end__  = __stack_start__ + __gcc_stack_size);

  PROVIDE (__sram_start__  = ORIGIN(SRAM));
  . = ORIGIN(SRAM);
  .sram  :  { 
        *(.sram) 
	*(.sram.*)
        }   >SRAM 
   PROVIDE (__sram_end__ = ORIGIN(SRAM) + LENGTH(SRAM));     
  
  

}
