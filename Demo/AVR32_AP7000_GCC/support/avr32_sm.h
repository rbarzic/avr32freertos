void sm_enable_pll0(volatile avr32_sm_t* sm,
		    unsigned osc,
		    unsigned mul,
		    unsigned div);

void sm_disable_pll0(volatile avr32_sm_t* sm);
void sm_wait_for_lockbit0(volatile avr32_sm_t* sm);
void sm_enable_pll1(volatile avr32_sm_t* sm,
		    unsigned osc,
		    unsigned mul,
		    unsigned div);
void sm_disable_pll1(volatile avr32_sm_t* sm);
void sm_wait_for_lockbit1(volatile avr32_sm_t* sm);
void sm_set_main_clocks(volatile avr32_sm_t* sm,
			unsigned cpusel,
			unsigned cpudiv,
			unsigned ahbsel,
			unsigned ahbdiv,
			unsigned apbasel,
			unsigned apbadiv,
			unsigned apbbsel,
			unsigned apbbdiv			
			);
void sm_switch_to_clock(volatile avr32_sm_t* sm,
			unsigned clock);
