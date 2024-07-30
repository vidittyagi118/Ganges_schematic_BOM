#ifndef __LP_TIMER_H_
#define __LP_TIMER_H_

#include <stdbool.h>
#include <stdint.h>


void LPTMR_Initialisation(void);
uint64_t GetLPTimerMsTicks (void);

#endif