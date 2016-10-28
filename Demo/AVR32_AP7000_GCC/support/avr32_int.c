#include <stdint.h>
#include "avr32_int.h"


// "Dynamically computed offset"
static uint32_t __offset_int0;
static uint32_t __offset_int1;
static uint32_t __offset_int2;
static uint32_t __offset_int3;

// Those addresses are provided by crt0.S and the linker file
extern void* __evba_start__;
extern void* __int0_handler__;
extern void* __int1_handler__;
extern void* __int2_handler__;
extern void* __int3_handler__;

extern void* __int_default_handler__;


int_handler_t int_group[INTC_MAX_GROUPS]; // 64 groups max 

unsigned int intc_compute_offset(uint32_t * poffset,uint32_t evba,uint32_t target) {
  volatile unsigned bit_evba;
  volatile unsigned bit_target;
  uint32_t tmp_offset =0;
  for(int i=31;i>=0;i--) {
    bit_evba   = (evba>>i)   & 1;
    bit_target = (target>>i) & 1;
    // if(!bit_evba && !bit_target)  nothing to do 

    if(bit_evba && !bit_target) {
      // Bad, target will be impossible to reach
      *poffset =0;
      return 1;
    }
    if(!bit_evba && bit_target) {
      // Ok, set the bit in offset
      tmp_offset |= 1<<i;
    }
    //if(bit_evba && bit_target)  nothing to do 
  }
  *poffset = tmp_offset;
  return 0;
}

int  intc_init(int_handler_t user_default_handler) {
  freertos_avr32_intc_t *intc = (freertos_avr32_intc_t *)AVR32_INTC_ADDRESS;
  int_handler_t handler;
  
  if(intc_compute_offset(&__offset_int0,(uint32_t)&__evba_start__,(uint32_t)&__int0_handler__)) return 1;
  if(intc_compute_offset(&__offset_int1,(uint32_t)&__evba_start__,(uint32_t)&__int1_handler__)) return 2;
  if(intc_compute_offset(&__offset_int2,(uint32_t)&__evba_start__,(uint32_t)&__int2_handler__)) return 3;
  if(intc_compute_offset(&__offset_int3,(uint32_t)&__evba_start__,(uint32_t)&__int3_handler__)) return 4;


  if(user_default_handler == (int_handler_t)0) {
    handler = (int_handler_t)__int_default_handler__ ;
  } else {
    handler = user_default_handler;
  }
  for(int i=0;i<INTC_MAX_GROUPS;i++) {
    int_group[i] = user_default_handler;
    // All interrupts default to __int3_handler__
    intc->intpr[i] = (INT3 << INTLEV) | (__offset_int3);
  }
  
  return 0;
}


void intc_register_interrupt(int_handler_t handler, 
			     unsigned group,
			     unsigned level) {
  freertos_avr32_intc_t *intc = (freertos_avr32_intc_t *)AVR32_INTC_ADDRESS;
  // FIXME : Add a mask for the offset
  int_group[group] = handler;
  switch(level) {
    case INT0:
      intc->intpr[group] = (INT0 << INTLEV) | (__offset_int0); 
      break;
    case INT1:
      intc->intpr[group] = (INT1 << INTLEV) | (__offset_int1); 
      break;
    case INT2:
      intc->intpr[group] = (INT2 << INTLEV) | (__offset_int2); 
      break;
    case INT3:
      intc->intpr[group] = (INT3 << INTLEV) | (__offset_int3); 
      break;
  }

}


void  _intc_call_user_handler(uint32_t intlevel) {
  
  freertos_avr32_intc_t *intc = (freertos_avr32_intc_t *)AVR32_INTC_ADDRESS;
  uint32_t offset = 3-intlevel; // Registers are stored in the reverse order
  // Get the group
  uint32_t group = intc->intcause[offset];
  // Get the pointer the user function
  void (*user_func)(void) = int_group[group];
  // Call it 
  (*user_func)();
}



void _dummy_intc(void) __attribute__((naked))   __attribute__ ((section (".handlers")));
void _dummy_intc(void)  {
  asm volatile (".align  4");
  asm volatile ("__int0_handler__:");
  asm volatile ("pushm r0-r12,lr");
  _intc_call_user_handler(0);
  asm volatile ("popm  r0-r12,lr");
  asm volatile ("rete");

  asm volatile (".align  4");
  asm volatile ("__int1_handler__:");
  asm volatile ("pushm r0-r12,lr");
  _intc_call_user_handler(1);
  asm volatile ("popm  r0-r12,lr");
  asm volatile ("rete");

  asm volatile (".align  4");
  asm volatile ("__int2_handler__:");
  asm volatile ("pushm r0-r12,lr");
  _intc_call_user_handler(2);
  asm volatile ("popm  r0-r12,lr");
  asm volatile ("rete");

  asm volatile (".align  4");
  asm volatile ("__int3_handler__:");
  asm volatile ("pushm r0-r12,lr");
  _intc_call_user_handler(3);
  asm volatile ("popm  r0-r12,lr");
  asm volatile ("rete");
  asm volatile (".align  4");
  asm volatile ("__int_default_handler__: rjmp __int_default_handler__");
 
}


