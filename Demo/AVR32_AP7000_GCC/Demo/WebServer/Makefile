#       Makefile for the  AVR32/FreeRTOS demo file
#        Adapted from ARM7 Makefile  Copyright (C) 2003-2005 Richard Barry
#        Adaptation to AVR32 :  Copyright (C) 2006 Ronan BARZIC
#
#       This file is part of the AVR32 port of FreeRTOS - It is
#       available with the same license as FreeRTOS :
#
#       FreeRTOS is free software; you can redistribute it and/or modify
#       it under the terms of the GNU General Public License as published by
#       the Free Software Foundation; either version 2 of the License, or
#       (at your option) any later version.
#
#       FreeRTOS is distributed in the hope that it will be useful,
#       but WITHOUT ANY WARRANTY; without even the implied warranty of
#       MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#       GNU General Public License for more details.
#
#       You should have received a copy of the GNU General Public License
#       along with FreeRTOS; if not, write to the Free Software
#       Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
#
#       A special exception to the GPL can be applied should you wish to distribute
#       a combined work that includes FreeRTOS, without being obliged to provide
#       the source code for any proprietary components.  See the licensing section
#       of http://www.FreeRTOS.org for full details of how and when the exception
#       can be applied.
#
#       ***************************************************************************
#       See http://www.FreeRTOS.org for documentation, latest information, license
#       and contact details.  Please ensure to read the configuration and relevant
#       port sections of the online documentation.
#       ***************************************************************************
#

CPU=ap7000
FREERTOS_TOP=../../../..
DEMO_AVR32=$(FREERTOS_TOP)/Demo/AVR32_AP7000_GCC
LWIP_LOC = $(DEMO_AVR32)/lwip-1.1.1
LWIP_PORT_LOC =  $(DEMO_AVR32)/lwip-port
EMAC_LOC =  $(DEMO_AVR32)/EMAC
BASIC_WEB_LOC =  $(DEMO_AVR32)/BasicWEB
AVR32_PORTABLE=$(FREERTOS_TOP)/Source/portable/GCC/AVR32_AP7000/

CC=avr32-gcc
OBJCOPY=avr32-objcopy
ARCH=avr32-ar
CRT0=crt0.o
LD=avr32-ld

LDSCRIPT=$(DEMO_AVR32)/Demo/Minimal/freertos_flash_at32ap7000.lds
OBJDUMP = avr32-objdump
OPTIM= -O2

DEBUG= -g -DNEWCODE -DNO_RETURN_STACK

#AVR32CONFIG= -DUSE_PLL0
AVR32CONFIG= -DUSE_PLL0 -DLED_ON_PORTB

EMAC_CONFIG= -DEMAC_INT=EX_INT0  


CFLAGS=   -Wall -D RUN_FROM_RAM -D TASK_SWITCH_USE_INT0  -DGCC_AVR32_AP7000  $(EMAC_CONFIG) \
	       -I. -I../.. -I$(DEMO_AVR32)/support -I$(AVR32_PORTABLE)/include \
               -I$(FREERTOS_TOP)/Demo/Common/include $(DEBUG) -mcpu=$(CPU)  -T$(LDSCRIPT) \
               -Wcast-align $(OPTIM) -std=gnu99 $(AVR32CONFIG) \
               -I$(LWIP_LOC)/src/include -I$(LWIP_LOC)/src/include/ipv4 -I$(LWIP_PORT_LOC)/AVR32AP700x/ -I$(LWIP_PORT_LOC)/AVR32AP700x/arch \
		-I$(EMAC_LOC) \
               -I$(BASIC_WEB_LOC) \
	       -I$(FREERTOS_TOP)/Source/include \
               -I$(FREERTOS_TOP)/Source/portable/GCC/AVR32_AP7000
 

ASM_FLAGS =  -mcpu=$(CPU)   $(AVR32CONFIG) -I. -I.. -DNO_RETURN_STACK


LINKER_FLAGS= -mcpu=$(CPU) -T$(LDSCRIPT) -nostartfiles

#
# Source files
#




AVR32_SRC = \
mylib.c \
main.c \
$(FREERTOS_TOP)/Source/tasks.c \
$(FREERTOS_TOP)/Source/queue.c \
$(FREERTOS_TOP)/Source/list.c \
$(FREERTOS_TOP)/Source/croutine.c \
$(FREERTOS_TOP)/Source/portable/MemMang/heap_2.c \
$(FREERTOS_TOP)/Source/portable/GCC/AVR32_AP7000/port.c \
$(DEMO_AVR32)/support/stk1000.c \
$(DEMO_AVR32)/support/printf-stdarg.c \
$(DEMO_AVR32)/appnotes/usart.c \
$(DEMO_AVR32)/appnotes/pio.c \
$(DEMO_AVR32)/appnotes/sdram.c \
$(DEMO_AVR32)/appnotes/mt481c2m32b2tg.c \
$(DEMO_AVR32)/support/avr32_int.c \
$(DEMO_AVR32)/support/sm.c \
$(DEMO_AVR32)/support/unhandled_exceptions.c 




AVR32_ASM_SRC = $(DEMO_AVR32)//support/crt0.S

LWIP_SRC= \
 $(LWIP_LOC)/src/core/tcp_out.c \
 $(LWIP_LOC)/src/core/inet.c \
 $(LWIP_LOC)/src/core/mem.c \
 $(LWIP_LOC)/src/core/memp.c \
 $(LWIP_LOC)/src/core/netif.c \
 $(LWIP_LOC)/src/core/pbuf.c \
 $(LWIP_LOC)/src/core/raw.c \
 $(LWIP_LOC)/src/core/stats.c \
 $(LWIP_LOC)/src/core/sys.c \
 $(LWIP_LOC)/src/core/tcp.c \
 $(LWIP_LOC)/src/core/tcp_in.c \
 $(LWIP_LOC)/src/core/ipv4/ip.c \
 $(LWIP_LOC)/src/core/ipv4/ip_addr.c \
 $(LWIP_LOC)/src/core/ipv4/icmp.c \
 $(LWIP_LOC)/src/api/tcpip.c \
 $(LWIP_LOC)/src/api/api_msg.c \
 $(LWIP_LOC)/src/api/err.c \
 $(LWIP_LOC)/src/api/api_lib.c \
 $(LWIP_LOC)/src/netif/etharp.c \
 $(LWIP_LOC)/src/netif/ethernetif.c \
 $(LWIP_LOC)/src/core/udp.c \
 $(LWIP_LOC)/src/core/ipv4/ip_frag.c

LWIP_PORT_SRC= \
	$(LWIP_PORT_LOC)/AVR32AP700x/sys_arch.c

EMAC_SRC= \
$(EMAC_LOC)/AVR32_EMAC.c \
$(EMAC_LOC)/AVR32_EMAC_ISR.c

BASIC_WEB_SRC=$(BASIC_WEB_LOC)/BasicWEB.c



#
# Define all object files.
#
AVR32_OBJ = $(AVR32_SRC:.c=.o)
AVR32_ASM_OBJ = $(AVR32_ASM_SRC:.S=.o)
LWIP_OBJ = $(LWIP_SRC:.c=.o)
LWIP_PORT_OBJ = $(LWIP_PORT_SRC:.c=.o)
EMAC_OBJ = $(EMAC_SRC:.c=.o)
BASIC_WEB_OBJ = $(BASIC_WEB_SRC:.c=.o)

%.lss: %.elf
	@echo
	$(OBJDUMP) -h -S $< > $@

rtosdemo.bin : rtosdemo.elf
	$(OBJCOPY) rtosdemo.elf -O binary  -R.sdram rtosdemo.bin

rtosdemo.hex : rtosdemo.elf
	$(OBJCOPY) rtosdemo.elf -O ihex rtosdemo.hex

rtosdemo.elf : $(AVR32_OBJ)  $(AVR32_ASM_OBJ)    $(LWIP_OBJ) $(LWIP_PORT_OBJ)  $(EMAC_OBJ) $(BASIC_WEB_OBJ) Makefile
	$(CC) -nostartfiles  $(LINKER_FLAGS) $(AVR32_ASM_OBJ)   $(AVR32_OBJ)   $(LWIP_PORT_OBJ) $(LWIP_OBJ) $(EMAC_OBJ) $(BASIC_WEB_OBJ)  -ortosdemo.elf

#rtosdemo.elf : $(AVR32_OBJ)  $(AVR32_ASM_OBJ)    Makefile
#	$(CC) -nostartfiles  $(LINKER_FLAGS) $(AVR32_ASM_OBJ)   $(AVR32_OBJ)    -ortosdemo.elf

$(AVR32_OBJ) : %.o : %.c $(LDSCRIPT) Makefile $(FREERTOS_TOP)/Source/include/portable.h $(FREERTOS_TOP)/Source/portable/GCC/AVR32_AP7000/portmacro.h
	$(CC) -c $(CFLAGS) $< -o $@


$(LWIP_OBJ) : %.o : %.c $(LDSCRIPT) Makefile $(FREERTOS_TOP)/Source/include/portable.h $(FREERTOS_TOP)/Source/portable/GCC/AVR32_AP7000/portmacro.h
	$(CC) -c $(CFLAGS) $< -o $@

$(LWIP_PORT_OBJ) : %.o : %.c $(LDSCRIPT) Makefile $(FREERTOS_TOP)/Source/include/portable.h $(FREERTOS_TOP)/Source/portable/GCC/AVR32_AP7000/portmacro.h
	$(CC) -c $(CFLAGS) $< -o $@

$(EMAC_OBJ) : %.o : %.c $(LDSCRIPT) Makefile $(FREERTOS_TOP)/Source/include/portable.h $(FREERTOS_TOP)/Source/portable/GCC/AVR32_AP7000/portmacro.h
	$(CC) -c $(CFLAGS) $< -o $@

$(BASIC_WEB_OBJ) : %.o : %.c $(LDSCRIPT) Makefile $(FREERTOS_TOP)/Source/include/portable.h $(FREERTOS_TOP)/Source/portable/GCC/AVR32_AP7000/portmacro.h
	$(CC) -c $(CFLAGS) $< -o $@


$(AVR32_ASM_OBJ) : %.o : %.S $(LDSCRIPT) Makefile
	$(CC) -c $(ASM_FLAGS) $< -o $@

clean :
	rm -f *.o inc/*.o $(FREERTOS_TOP)/Source/*.o ../Common/*.o support/*.o \
	$(FREERTOS_TOP)/Source/portable/GCC/AVR32_AP7000/*.o \
	$(FREERTOS_TOP)/Source/portable/MemMang/*.o *.i *.elf *.lss  *.hex *.bin \
	$(LWIP_LOC)/src/core/*.o \
	$(LWIP_LOC)/src/api/*.o \
	$(LWIP_LOC)/src/netif/*.o \
	$(LWIP_PORT_LOC)/AVR32AP700x/*.o \
	$(EMAC_LOC)/*.o \
	$(BASIC_WEB_LOC)/*.o 
