/*      extra AVR32 definitions, not present in std AVR32 include file
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

#ifndef         EXTRA_AVR32_H_
# define        EXTRA_AVR32_H_

// Not in std file ???
#define INT0          0
#define INT1          1
#define INT2          2
#define INT3          3

//Register offsets
#define INTLEV        30
#define OFFSET        0
#define OFFSET_BITS   24


/* Because there are no builtin for these instructions in GCC :( */
#define DISABLE_ALL_INTERRUPTS { asm volatile ("ssrf 16"); asm volatile ("nop"); asm volatile ("nop"); asm volatile ("nop"); asm volatile ("nop"); asm volatile ("nop"); asm volatile ("nop"); asm volatile ("nop"); asm volatile ("nop");}
#define ENABLE_ALL_INTERRUPTS  asm volatile ("csrf 16")


#define DISABLE_INTO asm volatile ("ssrf 17")
#define ENABLE_INTO  asm volatile ("csrf 17")
#define DISABLE_INT1 asm volatile ("ssrf 18")
#define ENABLE_INT1  asm volatile ("csrf 18")
#define DISABLE_INT2 asm volatile ("ssrf 19")
#define ENABLE_INT2  asm volatile ("csrf 19")
#define DISABLE_INT3 asm volatile ("ssrf 20")
#define ENABLE_INT3  asm volatile ("csrf 21")

/* To ease 'automated' debugging */

#define __LABEL(a) asm volatile (a)



#define AVR32_MACB0_PIOMAP  { \
 { AVR32_MACB0_COL_0_PIN, AVR32_MACB0_COL_0_FUNCTION},\
 { AVR32_MACB0_CRS_0_PIN, AVR32_MACB0_CRS_0_FUNCTION},\
 { AVR32_MACB0_MDC_0_PIN, AVR32_MACB0_MDC_0_FUNCTION},\
 { AVR32_MACB0_MDIO_0_PIN, AVR32_MACB0_MDIO_0_FUNCTION},\
 { AVR32_MACB0_RXD_0_PIN, AVR32_MACB0_RXD_0_FUNCTION},\
 { AVR32_MACB0_RXD_1_PIN, AVR32_MACB0_RXD_1_FUNCTION},\
 { AVR32_MACB0_RXD_2_PIN, AVR32_MACB0_RXD_2_FUNCTION},\
 { AVR32_MACB0_RXD_3_PIN, AVR32_MACB0_RXD_3_FUNCTION},\
 { AVR32_MACB0_RX_CLK_0_PIN, AVR32_MACB0_RX_CLK_0_FUNCTION},\
 { AVR32_MACB0_RX_DV_0_PIN, AVR32_MACB0_RX_DV_0_FUNCTION},\
 { AVR32_MACB0_RX_ER_0_PIN, AVR32_MACB0_RX_ER_0_FUNCTION},\
 { AVR32_MACB0_SPEED_0_PIN, AVR32_MACB0_SPEED_0_FUNCTION},\
 { AVR32_MACB0_TXD_0_PIN, AVR32_MACB0_TXD_0_FUNCTION},\
 { AVR32_MACB0_TXD_1_PIN, AVR32_MACB0_TXD_1_FUNCTION},\
 { AVR32_MACB0_TXD_2_PIN, AVR32_MACB0_TXD_2_FUNCTION},\
 { AVR32_MACB0_TXD_3_PIN, AVR32_MACB0_TXD_3_FUNCTION},\
 { AVR32_MACB0_TX_CLK_0_PIN, AVR32_MACB0_TX_CLK_0_FUNCTION},\
 { AVR32_MACB0_TX_EN_0_PIN, AVR32_MACB0_TX_EN_0_FUNCTION},\
 { AVR32_MACB0_TX_ER_0_PIN, AVR32_MACB0_TX_ER_0_FUNCTION}\
 }


#define SDRAM32BITS_PIOMAP {				   \
    {AVR32_EBI_DATA_16_PIN , AVR32_EBI_DATA_16_FUNCTION}, \
    {AVR32_EBI_DATA_17_PIN , AVR32_EBI_DATA_17_FUNCTION}, \
    {AVR32_EBI_DATA_18_PIN , AVR32_EBI_DATA_18_FUNCTION}, \
    {AVR32_EBI_DATA_19_PIN , AVR32_EBI_DATA_19_FUNCTION}, \
    {AVR32_EBI_DATA_20_PIN , AVR32_EBI_DATA_20_FUNCTION}, \
    {AVR32_EBI_DATA_21_PIN , AVR32_EBI_DATA_21_FUNCTION}, \
    {AVR32_EBI_DATA_22_PIN , AVR32_EBI_DATA_22_FUNCTION}, \
    {AVR32_EBI_DATA_23_PIN , AVR32_EBI_DATA_23_FUNCTION}, \
    {AVR32_EBI_DATA_24_PIN , AVR32_EBI_DATA_24_FUNCTION}, \
    {AVR32_EBI_DATA_25_PIN , AVR32_EBI_DATA_25_FUNCTION}, \
    {AVR32_EBI_DATA_26_PIN , AVR32_EBI_DATA_26_FUNCTION}, \
    {AVR32_EBI_DATA_27_PIN , AVR32_EBI_DATA_27_FUNCTION}, \
    {AVR32_EBI_DATA_28_PIN , AVR32_EBI_DATA_28_FUNCTION}, \
    {AVR32_EBI_DATA_29_PIN , AVR32_EBI_DATA_29_FUNCTION}, \
    {AVR32_EBI_DATA_30_PIN , AVR32_EBI_DATA_30_FUNCTION}, \
    {AVR32_EBI_DATA_31_PIN , AVR32_EBI_DATA_31_FUNCTION}, \
}


#endif      /* !EXTRA_AVR32_H_ */

