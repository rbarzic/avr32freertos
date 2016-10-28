

OUTPUT_FORMAT("elf32-avr32", "elf32-avr32",
	      "elf32-avr32")
OUTPUT_ARCH(avr32)
ENTRY(_start)


/* Misc definitions */
__exit_start_address  = 0xF00;
__gcc_stack_size      = 2K;

__cached_area          = 0x80000000;
__load_mask            = 0x1FFFFFFF; /* Set the 3 MSB to zeros */

MEMORY
{
  intram (rwxa) : ORIGIN = 0x94000000, LENGTH = 0x20000
}




SECTIONS
{
        
        __reset = .;
        .reset 0x94000000 : AT(0x14000000)   { 
                *(.reset);
        } 
        __reset_end_address = .;
        
/* Put exit code, exception and following sections in cached areas  */
/* But be careful of load area (i.e addresses in the hex file       */
        
        
        _exit = .;
        .exit 0x94000100 : AT(0x14000100)  { 
                *(.exit); 
        } 
       /*  __exit_end_address = .; */


     /*    . = __exit_end_address + __cached_area; */
        . = ALIGN(0x200);
        __evba_start__ = .;
        .exception 0x94000200 : AT(0x14000200)
         { 
                *(.exception); 
        } 
        . = ALIGN(0x600);
        __handlers__ = .;
        .handlers 0x94000600 : AT(0x14000600)
         { 
                *(.handlers); 
        } 
        . = 0x94000800;
        __acba_start__ = .;
        .acba 0x94000800 : AT(0x14000800)
         { 
                *(.acba); 
        } 

        

        . = ALIGN(0x100); /* just to ease visual debugging ...*/
        __text_start__ = .;
        .text  0x94000A00 :  AT(0x14000A00)       
          { 
                *(.text); 
                *(.rodata); 
                *(.rodata.*); 
		 *(.text.*)
		*(.gnu.linkonce.t*) 
                *(.gnu.linkonce.r*) 
        }


        . = ALIGN(4);
        __data_start__ = .;
        .data  . :    { 
                *(.data) 
	         *(.data.*)
                 *(.gnu.linkonce.d*) 
        }  = 0x0
        __data_end__ = ALIGN(4);

        __data_init_start__ = .;
        __data_init_end__ = .;


        . = ALIGN(4);
        __bss_start__ = .;
        .bss  . :   { 
                *(.bss) 
        }  = 0x0        
        __bss_end__ = ALIGN(4);




        . = ALIGN(4);
        __stack_start__ = .;        
        . = __stack_start__ + __gcc_stack_size;
        _stack          = .;
        __stack_end__   = .;

}
