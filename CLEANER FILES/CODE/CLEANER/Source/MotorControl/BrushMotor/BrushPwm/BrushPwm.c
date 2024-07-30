#include "BrushPwm.h"
#include "fsl_ftm.h"
#include "Serial_Debug.h"
#include <math.h>
#include <stdbool.h>

#ifdef SERIAL_DEBUG_BRUSHPWM
  #define Serial_Debug_BrushPwm     Serial_Debug
  #define Serial_Debug_BrushPwm_Num Serial_Debug_Num
#else
  #define Serial_Debug_BrushPwm     (void)
  #define Serial_Debug_BrushPwm_Num (void)
#endif

#define DEFAULT_BRUSHPWM_CH1_DUTY_CYCLE             0
#define DEFAULT_BRUSHPWM_CH2_DUTY_CYCLE             0

static uint8_t pwm1Ch1DutyCycle = DEFAULT_BRUSHPWM_CH1_DUTY_CYCLE;
static uint8_t pwm1Ch2DutyCycle = DEFAULT_BRUSHPWM_CH2_DUTY_CYCLE;
//static bool stopFlag = false;

bool BrushPwmInit(void)
{  
  ftm_config_t ftmInfo;
  
  ftm_chnl_pwm_signal_param_t ftmParam[2];
  
  /* Configure ftm params with frequency 24kHZ */
  ftmParam[0].chnlNumber = (ftm_chnl_t)BRUSHPWM_CHANNEL1;
  ftmParam[0].level = kFTM_HighTrue;
  ftmParam[0].dutyCyclePercent = 0U;
  ftmParam[0].firstEdgeDelayPercent = 0U;
  
  ftmParam[1].chnlNumber = (ftm_chnl_t)BRUSHPWM_CHANNEL2;
  ftmParam[1].level = kFTM_HighTrue;
  ftmParam[1].dutyCyclePercent = 0U;
  ftmParam[1].firstEdgeDelayPercent = 0U;
  
    /*
  * ftmInfo.prescale = kFTM_Prescale_Divide_1;
  * ftmInfo.bdmMode = kFTM_BdmMode_0;
  * ftmInfo.pwmSyncMode = kFTM_SoftwareTrigger;
  * ftmInfo.reloadPoints = 0;
  * ftmInfo.faultMode = kFTM_Fault_Disable;
  * ftmInfo.faultFilterValue = 0;
  * ftmInfo.deadTimePrescale = kFTM_Deadtime_Prescale_1;
  * ftmInfo.deadTimeValue = 0;
  * ftmInfo.extTriggers = 0;
  * ftmInfo.chnlInitState = 0;
  * ftmInfo.chnlPolarity = 0;
  * ftmInfo.useGlobalTimeBase = false;
  */
  FTM_GetDefaultConfig(&ftmInfo);
  /* Initialize FTM module */
  FTM_Init(BRUSHPWM_FTM_BASEADDR, &ftmInfo);
  
  FTM_SetupPwm(BRUSHPWM_FTM_BASEADDR, ftmParam, 2U, kFTM_EdgeAlignedPwm, 24000U, FTM_SOURCE_CLOCK);
  
  FTM_StartTimer(BRUSHPWM_FTM_BASEADDR, kFTM_SystemClock);
  
  if(SetBrushPwmCh1DutyCycle(pwm1Ch1DutyCycle) != SUCCESS)
  {
    Error_Handler();
    return false;
  }
  if(SetBrushPwmCh2DutyCycle(pwm1Ch2DutyCycle) != SUCCESS)
  {
    Error_Handler();
    return false;
  }
  return true;
}

ErrorStatus SetBrushPwmCh1DutyCycle (float dutyCycle)
{
  ErrorStatus status = SUCCESS;
  int16_t duty = (int16_t)(round(dutyCycle));
  if(duty >= 0 && duty <= 100)
  {
    pwm1Ch1DutyCycle = (uint8_t)duty;
    status = SUCCESS;
  }
  else
  {
    status = ERROR;
  }
  return status;
}

ErrorStatus SetBrushPwmCh2DutyCycle (float dutyCycle)
{
  ErrorStatus status = SUCCESS;
  int16_t duty = (int16_t)(round(dutyCycle));
  if(duty >= 0 && duty <= 100)
  {
    pwm1Ch2DutyCycle = (uint8_t)duty;
    status = SUCCESS;
  }
  else
  {
    status = ERROR;
  }
  return status;
}

bool SetUpdateBrushPwmCh1DutyCycle (float dutyCycle)
{
  ErrorStatus status = SUCCESS;
  status = SetBrushPwmCh1DutyCycle(dutyCycle);
  if(status == SUCCESS)
  {
    StartBrushPwmCh1();
  }
  return (status == SUCCESS);
}

bool SetUpdateBrushPwmCh2DutyCycle (float dutyCycle)
{
  ErrorStatus status = SUCCESS;
  status = SetBrushPwmCh2DutyCycle(dutyCycle);
  if(status == SUCCESS)
  {
    StartBrushPwmCh2();
  }
  return (status == SUCCESS);
}

uint8_t GetBrushPwmCh1DutyCycle (void)
{
  return pwm1Ch1DutyCycle;
}

uint8_t GetBrushPwmCh2DutyCycle (void)
{
  return pwm1Ch2DutyCycle;
}

ErrorStatus StartBrushPwmCh1 (void)
{
  ErrorStatus status = SUCCESS;
  
   /* Start PWM mode with updated duty cycle */
  FTM_UpdatePwmDutycycle(BRUSHPWM_FTM_BASEADDR, (ftm_chnl_t)BRUSHPWM_CHANNEL1, kFTM_EdgeAlignedPwm,
                         pwm1Ch1DutyCycle);
  /* Software trigger to update registers */
  FTM_SetSoftwareTrigger(BRUSHPWM_FTM_BASEADDR, true);
 // Serial_Debug_BrushPwm("\n BRUSHPWM CH1 Duty Cycle Output: ");
 // Serial_Debug_BrushPwm_Num(pwm1Ch1DutyCycle);
  return status;  
}

ErrorStatus StartBrushPwmCh2 (void)
{
  ErrorStatus status = SUCCESS;
  
   /* Start PWM mode with updated duty cycle */
  FTM_UpdatePwmDutycycle(BRUSHPWM_FTM_BASEADDR, (ftm_chnl_t)BRUSHPWM_CHANNEL2, kFTM_EdgeAlignedPwm,
                         pwm1Ch2DutyCycle);
  /* Software trigger to update registers */
  FTM_SetSoftwareTrigger(BRUSHPWM_FTM_BASEADDR, true);
 // Serial_Debug_BrushPwm("\n BRUSHPWM CH2 Duty Cycle Output: ");
 // Serial_Debug_BrushPwm_Num(pwm1Ch2DutyCycle);
  return status;  
}

ErrorStatus StopBrushPwmCh1 (void)
{
  ErrorStatus status = SUCCESS;
  
  uint8_t duty_cycle = 0;
   /* Start PWM mode with updated duty cycle */
  FTM_UpdatePwmDutycycle(BRUSHPWM_FTM_BASEADDR, (ftm_chnl_t)BRUSHPWM_CHANNEL1, kFTM_EdgeAlignedPwm,
                         duty_cycle);
  /* Software trigger to update registers */
  FTM_SetSoftwareTrigger(BRUSHPWM_FTM_BASEADDR, true);
 // Serial_Debug_BrushPwm("\n BRUSHPWM CH1 Stop Duty Cycle Output: ");
 // Serial_Debug_BrushPwm_Num(duty_cycle);
  return status;  
}

ErrorStatus StopBrushPwmCh2 (void)
{
  ErrorStatus status = SUCCESS;
  
  uint8_t duty_cycle = 0;
   /* Start PWM mode with updated duty cycle */
  FTM_UpdatePwmDutycycle(BRUSHPWM_FTM_BASEADDR, (ftm_chnl_t)BRUSHPWM_CHANNEL2, kFTM_EdgeAlignedPwm,
                         duty_cycle);
  /* Software trigger to update registers */
  FTM_SetSoftwareTrigger(BRUSHPWM_FTM_BASEADDR, true);
 // Serial_Debug_BrushPwm("\n BRUSHPWM CH2 Stop Duty Cycle Output: ");
 // Serial_Debug_BrushPwm_Num(duty_cycle);
  return status;  
}

//ErrorStatus StopBrushPwm (void)
//{
//  ErrorStatus status;
//  
//  sConfig.Pulse = 0;
//  stopFlag = true;
//  if (HAL_TIM_BRUSHPWM_ConfigChannel(&BrushPwmTimHandle, &sConfig, TIM_CHANNEL_3) != HAL_OK)
//  {
//    Error_Handler();
//    status = ERROR; 
//  }
//  else
//  {
//    if(HAL_TIM_BRUSHPWM_Start(&BrushPwmTimHandle, TIM_CHANNEL_3) != HAL_OK)
//    {
//      Error_Handler();
//      status = ERROR; 
//    }
//    else
//    {
//      status = SUCCESS; 
//    }
//  }
//  return status;  
//}

//uint8_t GetCurrentBrushPwmDuty (void)
//{
//  if(stopFlag == true)
//  {
//    return 0;
//  }
//  else
//  {
//    return(pwm1DutyCycle);
//  }
//}

/**
* @brief  This function is executed in case of error occurrence.
* @param  None
* @retval None
*/
static void Error_Handler(void)
{
 Serial_Debug_BrushPwm("\n BRUSHPWM Error ");
}
