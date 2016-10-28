#include "avr32_circuit.h"
#include "avr32_int.h"
#include "avr32_sm.h"

#include "stk1000.h"
#include "appnotes/usart.h"
#include "appnotes/pio.h"

extern int printk(const char *format, ...);

static unsigned char _string_buff[128];

unsigned char hex2ascii(unsigned hexvalue) {
  if((hexvalue >=0) && (hexvalue<=9)) return '0'+ hexvalue;
  if((hexvalue >=10) && (hexvalue<=15)) return 'A' + hexvalue-10;
  
  return 'X';			     
}

void  print32bit_word(unsigned char *str,unsigned value) {

  *str++   = hex2ascii((value>>28) & 0x0F);
  *str++   = hex2ascii((value>>24) & 0x0F);
  *str++   = hex2ascii((value>>20) & 0x0F);
  *str++   = hex2ascii((value>>16) & 0x0F);
  *str++   = hex2ascii((value>>12) & 0x0F);
  *str++   = hex2ascii((value>>8)  & 0x0F);
  *str++   = hex2ascii((value>>4)  & 0x0F);
  *str++   = hex2ascii((value>>0)  & 0x0F);

}

void str_to_buff(unsigned char **buff,char *str) {
  unsigned char c;
  while((c=(*str++))) *(*buff)++ = c;
}

void HandleExceptionInC(unsigned ecr) {
  unsigned char *str = _string_buff;
  unsigned rar_ex = __builtin_mfsr(AVR32_RAR_EX);
  unsigned rsr_ex = __builtin_mfsr(AVR32_RSR_EX);
  unsigned ecr2    = __builtin_mfsr(AVR32_ECR);

  printk("rar_ex : %x\n",rar_ex);
  printk("rsr_ex : %x\n",rsr_ex);
  printk("ecr   : %x\n",ecr);
  printk("ecr2   : %x\n",ecr2);

  /*  
  str_to_buff(&str,"ECR    = : <");
  print32bit_word(str,ecr);
  str_to_buff(&str,"> ");
  
  str_to_buff(&str,"RAR_EX = : <");
  print32bit_word(str,rar_ex);
  str_to_buff(&str,"> ");

  str_to_buff(&str,"RSR_EX = : <");
  print32bit_word(str,rsr_ex);
  str_to_buff(&str,"> ");
  *str ='\0';
  
  while(*str) {
    usart_putchar(&AVR32_USART1,*str++);
  }
  */

  while(1) 
    asm volatile ("nop");

}
