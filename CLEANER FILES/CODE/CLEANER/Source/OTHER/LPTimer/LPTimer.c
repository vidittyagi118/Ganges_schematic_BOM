#include "LPTimer.h"
#include "fsl_lptmr.h"
#include "Delay.h"
#include "RobotOperation.h"
#include "RobotControl.h"
#include "RotateSenseCommon.h"
#include "EdgeSenseCommon.h"
#include "Relay.h"
#include "LTC4015_Main.h"
#include "ZigbeeApiMode.h"
#include "LedCommon.h"
#include "i2c_init.h"
#include "CommCommon.h"
#include "SDFAT/SDcardoperation.h"
#include "FaultProcess.h"


#define LPTMR_HANDLER LPTMR0_IRQHandler
#define LPTMR_SOURCE_CLOCK CLOCK_GetFreq(kCLOCK_LpoClk)
#define LPTMR_TIME                      1000                       //Time in microseconds


static volatile uint64_t msTimeCount = 0;

static void MiiliSecondTimerHandler (void) ;

void LPTMR_Initialisation(void)
{
  lptmr_config_t lptmrConfig;
  /* Configure LPTMR */
  /*
  * lptmrConfig.timerMode = kLPTMR_TimerModeTimeCounter;
  * lptmrConfig.pinSelect = kLPTMR_PinSelectInput_0;
  * lptmrConfig.pinPolarity = kLPTMR_PinPolarityActiveHigh;
  * lptmrConfig.enableFreeRunning = false;
  * lptmrConfig.bypassPrescaler = true;
  * lptmrConfig.prescalerClockSource = kLPTMR_PrescalerClock_1;
  * lptmrConfig.value = kLPTMR_Prescale_Glitch_0;
  */
  LPTMR_GetDefaultConfig(&lptmrConfig);
  
  /* Initialize the LPTMR */
  LPTMR_Init(LPTMR0, &lptmrConfig);
  
  /* Set timer period */
  LPTMR_SetTimerPeriod(LPTMR0, USEC_TO_COUNT(LPTMR_TIME, LPTMR_SOURCE_CLOCK));
  
  /* Enable timer interrupt */
  LPTMR_EnableInterrupts(LPTMR0, kLPTMR_TimerInterruptEnable);
  
  /* Enable at the NVIC */
  EnableIRQ(LPTMR0_IRQn);
  
  //Serial_Debug("Low Power Timer Initialisation\r\n");
  
  /* Start counting */
  LPTMR_StartTimer(LPTMR0);
}
    
static void MiiliSecondTimerHandler (void)                     
{
  RobotOperationTimeIncrement_ms();
  RobotTimeIncrement_ms();
  DelayMsTimerhandler();
  RotateSenseTimeIncrement_ms();
  EdgeSenseTimeIncrement_ms();
  RelayTimeIncrement_ms();
  BatChargerTimer();
  ZigbeeAPIModeTimer_ms();
  LedTimeIncrement_ms();
  TempSenseTimer_ms();
  CommTimeIncrement_ms();
  Temperature_SDlogtimer();
  SOC_SDlogtimer();
  MOTORCRNT_SDlogtimer();
  BmstransmitStopTimeIncrement_ms();
  //RpmCount_ms();
  
//  ContinueTimeIncrement_ms();
}   

void LPTMR_HANDLER(void)
{
  
  LPTMR_ClearStatusFlags(LPTMR0, kLPTMR_TimerCompareFlag);
  msTimeCount++;
  MiiliSecondTimerHandler();
}  

uint64_t GetLPTimerMsTicks (void)
{
  return msTimeCount;  
}
