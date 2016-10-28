
#ifndef   	STK1000_H_
# define   	STK1000_H_


typedef enum {off,on} stk1000_onoff_t;

void STK1000_reset_all_led(void);
void STK1000_set_led(unsigned int led);
void STK1000_reset_led(unsigned int led);
void STK1000_led(unsigned int led, stk1000_onoff_t onoff);
void STK1000_all_led(unsigned int value);


#define STK1000_SDRAM_BASE 0x10000000

#endif 	    /* !STK1000_H_ */
