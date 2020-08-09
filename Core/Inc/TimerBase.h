#ifndef TIMERBASE_H
#define TIMERBASE_H
#include"TimerMacro.h"

#define BASE_CR (ARR_ENABLE | TIMER_UP_COUNT | TIMER_ONE_PULSE_DISABLE |TIMER_COUNTER_ENABLE    \
			| T1_CH1_SELECT| MASTER_MODE_RESET)
//ARR reg is buffered
//Up count
//one pulse mode disabled
//counter enabled
//CH1 is connected to T1
// Master Mode Reset
#define SMCR_BASECR (SLAVE_MODE| SMS_GATED_M | TRIGGER_FIL_T1)
// set ETP to inverted
//Trigger was selected TI1FP1
//Slave mode selected as gated mode
//slave mode was triggered on t1
#define CCMR_BASECR (CC1_INPUT_IC1_MAP_TI1 | IC1_NO_PRESCALE |IC1_NO_FILER| CC3_OUTPUT |OC3_MODE_PWM_M1 )
// CC1 channel is configured as input, IC1 is mapped on TI1
// IC1 no prescaler
// IC1 no filter
// CC3 channel is configured as PWM mode 1 output
#define CCER_BASECR (CC1_CAPTURE_ENABLED|CC1_CAP_FALLING_EDGE|OC3_ENABLE|OC3_ACTIVELOW)
//capture enabled for CH1
// trigger on falling edge
//output for CH3

#endif // TIMERBASE_H
