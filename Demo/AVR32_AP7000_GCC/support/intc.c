#include <stdint.h>
#include "avr32_int.h"

#ifndef INTC_MAX_GROUPS
#define INTC_MAX_GROUPS 64 // 64 groups max 
#endif

uint32_t* int_group[INTC_MAX_GROUPS]; // 64 groups max 





void intc_init(uint32_t* user_default_handler) {
  uint32_t* handler;
  if(user_default_handler == (uint32_t*)0) {
    handler = ;
  } else {
    handler = user_default_handler;
  }
  for(int i=0;i<INTC_MAX_GROUPS;i++) int_group[i] = user_default_handler;
}


void intc_register_interrupt
