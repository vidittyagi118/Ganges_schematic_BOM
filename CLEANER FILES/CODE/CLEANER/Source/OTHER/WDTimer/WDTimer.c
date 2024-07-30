#include "WDTimer.h"
#include "fsl_wdog.h"
#include "fsl_rcm.h"
#include "LPTimer.h"

#define WDOG_WCT_INSTRUCITON_COUNT (256U)

#ifdef ENABLE_WDT  
static WDOG_Type *wdog_base = WDOG;
static RCM_Type *rcm_base = RCM;
static void WaitWctClose(WDOG_Type *base);
#endif

bool WDTimerInit (uint32_t timeoutValue)
{
#ifdef ENABLE_WDT  
    /*If not wdog reset*/
  if (!(RCM_GetPreviousResetSources(rcm_base) & kRCM_SourceWdog))
  {
      WDOG_ClearResetCount(wdog_base);
  }
 /*
   * config.enableWdog = true;
   * config.clockSource = kWDOG_LpoClockSource;
   * config.prescaler = kWDOG_ClockPrescalerDivide1;
   * config.enableUpdate = true;
   * config.enableInterrupt = false;
   * config.enableWindowMode = false;
   * config.windowValue = 0U;
   * config.timeoutValue = 0xFFFFU;
   */
   wdog_config_t config;
   WDOG_GetDefaultConfig(&config);
   config.timeoutValue = timeoutValue; 
   config.enableUpdate = true;
   WDOG_Init(wdog_base, &config);
   WaitWctClose(wdog_base);
#endif   
   return true;
}

void WDTimerDeInit (void)
{
#ifdef ENABLE_WDT  
    WDOG_Deinit(wdog_base);
#endif
}
  
void UpdateWDTimerValue (uint32_t timeOutValue)
{
#ifdef ENABLE_WDT 
  uint32_t primaskValue = DisableGlobalIRQ();
  WDOG_Unlock(wdog_base);
  WDOG_SetTimeoutValue(wdog_base, timeOutValue);
  EnableGlobalIRQ(primaskValue);
  WaitWctClose(wdog_base);
#endif
}

void WDTimerRefresh (void)
{
#ifdef ENABLE_WDT 
  static uint32_t wdog_refresh_time = 0;
  if((GetLPTimerMsTicks()-wdog_refresh_time)>=WDOG_REFRESH_INTERVAL)
  {
    WDOG_Refresh(wdog_base); 
    wdog_refresh_time = GetLPTimerMsTicks(); 
  }
#endif
}

static void WaitWctClose(WDOG_Type *base)
{
#ifdef ENABLE_WDT 
    /* Accessing register by bus clock */
    for (uint32_t i = 0; i < WDOG_WCT_INSTRUCITON_COUNT; i++)
    {
        (void)base->RSTCNT;
    }
#endif
}