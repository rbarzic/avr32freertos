/*
** avr32_int.h
** 
** Made by Ronan BARZIC
** Login   <ronan@>
** 
** Started on  Sat Jun  3 21:58:49 2006 Ronan BARZIC
** Last update Sat Jun  3 21:58:49 2006 Ronan BARZIC
*/

#ifndef   	AVR32_INT_H_
#define   	AVR32_INT_H_
#include "avr32_circuit.h"


#ifndef INTC_MAX_GROUPS
#define INTC_MAX_GROUPS 64 // 64 groups max 
#endif

// copied from intc.h but intcause is now an array
typedef struct freertos_avr32_intc_t {
  /*0x000*/
        unsigned long intpr[64];
  /*0x100*/
  const unsigned long intreq[64];
  /*0x200*/
  //const unsigned long intcause3;
  //const unsigned long intcause2;
  //const unsigned long intcause1;
  //const unsigned long intcause0;
  // Be aware of the reverse order
  unsigned long intcause[4];

} freertos_avr32_intc_t;


typedef void (*int_handler_t)(void);
int  intc_init(int_handler_t user_default_handler);
void intc_register_interrupt(int_handler_t handler, 
			     unsigned group,
			     unsigned level);

#endif 	    /* !AVR32_INT_H_ */
