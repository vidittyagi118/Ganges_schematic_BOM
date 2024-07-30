#ifndef __WDTIMER_H_
#define __WDTIMER_H_

#include <stdint.h>
#include <stdbool.h>

#define ENABLE_WDT                                                /* Enable Watch Dog Timer */

#ifdef ENABLE_WDT
  #define WDT_RESET_TIME_START        25000 
  #define WDT_RESET_TIME              5000
  #define WDT_OTA_RESET_TIME          25000                     /* We have a NAK time of 10 seconds */
  #define WDOG_REFRESH_INTERVAL       10 //ms  Refresh interval should not be kept less than 3 ms
#endif

bool WDTimerInit (uint32_t timeoutValue);
void WDTimerDeInit (void);
void UpdateWDTimerValue (uint32_t timeOutValue);
void WDTimerRefresh (void);

#endif