#include "avr32_circuit.h"
/* {{{ Pll 0 */

/* {{{    sm_enable_pll0 */
void sm_enable_pll0(volatile avr32_sm_t* sm,
		    unsigned osc,
		    unsigned mul,
		    unsigned div) {
  union {
          unsigned long                  pm_pll0   ;//0x0024
          avr32_sm_pm_pll0_t             PM_PLL0   ;
  } u_pll;

  u_pll.pm_pll0 = 0;
  u_pll.PM_PLL0.pllen  = 1;
  u_pll.PM_PLL0.pllosc = osc;
  u_pll.PM_PLL0.pllmul = mul;
  u_pll.PM_PLL0.plldiv = div;
  
  sm->pm_pll0 = u_pll.pm_pll0;
}
/* }}} */

/* {{{    sm_disable_pll0  */

void sm_disable_pll0(volatile avr32_sm_t* sm) {
  union {
          unsigned long                  pm_pll0   ;//0x0024
          avr32_sm_pm_pll0_t             PM_PLL0   ;
  } u_pll;
  u_pll.pm_pll0 = sm->pm_pll0;
  u_pll.PM_PLL0.pllen  = 0;
  sm->pm_pll0 = u_pll.pm_pll0;
}

/* }}} */

/* {{{    sm_wait_for_lockbit0 */

void sm_wait_for_lockbit0(volatile avr32_sm_t* sm) {
  while(!sm->PM_ISR.lock0);
}

/* }}} */

/* }}} */

/* {{{ Pll 1 */

/* {{{    sm_enable_pll1 */
void sm_enable_pll1(volatile avr32_sm_t* sm,
		    unsigned osc,
		    unsigned mul,
		    unsigned div) {
  union {
          unsigned long                  pm_pll1   ;//1x1124
          avr32_sm_pm_pll1_t             PM_PLL1   ;
  } u_pll;

  u_pll.pm_pll1 = 1;
  u_pll.PM_PLL1.pllen  = 1;
  u_pll.PM_PLL1.pllosc = osc;
  u_pll.PM_PLL1.pllmul = mul;
  u_pll.PM_PLL1.plldiv = div;
  
  sm->pm_pll1 = u_pll.pm_pll1;
}
/* }}} */

/* {{{    sm_disable_pll1  */

void sm_disable_pll1(volatile avr32_sm_t* sm) {
  union {
          unsigned long                  pm_pll1   ;//1x1124
          avr32_sm_pm_pll1_t             PM_PLL1   ;
  } u_pll;
  u_pll.pm_pll1 = sm->pm_pll1;
  u_pll.PM_PLL1.pllen  = 1;
  sm->pm_pll1 = u_pll.pm_pll1;
}

/* }}} */

/* {{{    sm_wait_for_lockbit1 */

void sm_wait_for_lockbit1(volatile avr32_sm_t* sm) {
  while(!sm->PM_ISR.lock1);
}

/* }}} */

/* }}} */

/* {{{ Clocks */

void sm_set_main_clocks(volatile avr32_sm_t* sm,
			unsigned cpusel,
			unsigned cpudiv,
			unsigned ahbsel,
			unsigned ahbdiv,
			unsigned apbasel,
			unsigned apbadiv,
			unsigned apbbsel,
			unsigned apbbdiv			
			) {

  union {
    unsigned long                  pm_cksel  ;//0x0004
    avr32_sm_pm_cksel_t            PM_CKSEL  ;
  } u_cksel;

  union {
          unsigned long                  pm_icr    ;//0x0050
          avr32_sm_pm_icr_t              PM_ICR    ;
  } u_icr;

  // Clear ckrdy flag
  u_icr.pm_icr=0;
  u_icr.PM_ICR.ckrdy = 1;
  sm->pm_icr = u_icr.pm_icr;
  
  u_cksel.PM_CKSEL.cpusel = cpusel;
  u_cksel.PM_CKSEL.cpudiv = cpudiv ;
  u_cksel.PM_CKSEL.ahbsel = ahbsel ;
  u_cksel.PM_CKSEL.ahbdiv  =ahbdiv   ;
  u_cksel.PM_CKSEL.apbasel =apbasel  ;
  u_cksel.PM_CKSEL.apbadiv =apbadiv  ;
  u_cksel.PM_CKSEL.apbbsel =apbbsel  ;
  u_cksel.PM_CKSEL.apbbdiv =apbbdiv  ;

  sm->pm_cksel = u_cksel.pm_cksel;

  // Wait for ckrdy flag
  while(!sm->PM_ISR.ckrdy);
}



void sm_switch_to_clock(volatile avr32_sm_t* sm,
			unsigned clock) {
  union {
          unsigned long                  pm_mcctrl ;//0x0000
          avr32_sm_pm_mcctrl_t           PM_MCCTRL ;
  } u_mcctrl;
  u_mcctrl.pm_mcctrl = sm->pm_mcctrl;
  u_mcctrl.PM_MCCTRL.pllsel = clock;
  sm->pm_mcctrl = u_mcctrl.pm_mcctrl;

}

/* }}} */
