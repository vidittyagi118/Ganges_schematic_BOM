#include "Pwm2.h"
#include "Pwm2_Config.h"
#include "fsl_ftm.h"
#include "Serial_Debug.h"
#include <math.h>
#include <stdbool.h>

#ifdef SERIAL_DEBUG_PWM2
  #define Serial_Debug_Pwm2     Serial_Debug
  #define Serial_Debug_Pwm2_Num Serial_Debug_Num
#else
  #define Serial_Debug_Pwm2     (void)
  #define Serial_Debug_Pwm2_Num (void)
#endif

#define DEFAULT_PWM2_CH1_DUTY_CYCLE             0
#define DEFAULT_PWM2_CH2_DUTY_CYCLE             0

static uint8_t pwm2Ch1DutyCycle = DEFAULT_PWM2_CH1_DUTY_CYCLE;
static uint8_t pwm2Ch2DutyCycle = DEFAULT_PWM2_CH2_DUTY_CYCLE;
//static bool stopFlag = false;

bool Pwm2Init(void)
{  
  ftm_config_t ftmInfo;
  
  ftm_chnl_pwm_signal_param_t ftmParam[2];
  
  /* Configure ftm params with frequency 24kHZ */
  ftmParam[0].chnlNumber = (ftm_chnl_t)PWM2_CHANNEL1;
  ftmParam[0].level = kFTM_HighTrue;
  ftmParam[0].dutyCyclePercent = 0U;
  ftmParam[0].firstEdgeDelayPercent = 0U;
  
  ftmParam[1].chnlNumber = (ftm_chnl_t)PWM2_CHANNEL2;
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
  FTM_Init(PWM2_FTM_BASEADDR, &ftmInfo);
  
  FTM_SetupPwm(PWM2_FTM_BASEADDR, ftmParam, 2U, kFTM_EdgeAlignedPwm, 24000U, FTM_SOURCE_CLOCK);
  
  FTM_StartTimer(PWM2_FTM_BASEADDR, kFTM_SystemClock);
  
  if(SetPwm2Ch1DutyCycle(pwm2Ch1DutyCycle) != SUCCESS)
  {
    Error_Handler();
    return false;
  }
  if(SetPwm2Ch2DutyCycle(pwm2Ch2DutyCycle) != SUCCESS)
  {
    Error_Handler();
    return false;
  }
  return true;
}

ErrorStatus SetPwm2Ch1DutyCycle (float dutyCycle)
{
  ErrorStatus status = SUCCESS;
  int16_t duty = (int16_t)(round(dutyCycle));
  if(duty >= 0 && duty <= 100)
  {
    pwm2Ch1DutyCycle = (uint8_t)duty;
    status = SUCCESS;
  }
  else
  {
    status = ERROR;
  }
  return status;
}

ErrorStatus SetPwm2Ch2DutyCycle (float dutyCycle)
{
  ErrorStatus status = SUCCESS;
  int16_t duty = (int16_t)(round(dutyCycle));
  if(duty >= 0 && duty <= 100)
  {
    pwm2Ch2DutyCycle = (uint8_t)duty;
    status = SUCCESS;
  }
  else
  {
    status = ERROR;
  }
  return status;
}

ErrorStatus SetUpdatePwm2Ch1DutyCycle (float dutyCycle)
{
  ErrorStatus status = SUCCESS;
  status = SetPwm2Ch1DutyCycle(dutyCycle);
  if(status == SUCCESS)
  {
    StartPwm2Ch1();
  }
  return status;
}

ErrorStatus SetUpdatePwm2Ch2DutyCycle (float dutyCycle)
{
  ErrorStatus status = SUCCESS;
  status = SetPwm2Ch2DutyCycle(dutyCycle);
  if(status == SUCCESS)
  {
    StartPwm2Ch2();
  }
  return status;
}

uint8_t GetPwm2Ch1DutyCycle (void)
{
  return pwm2Ch1DutyCycle;
}

uint8_t GetPwm2Ch2DutyCycle (void)
{
  return pwm2Ch2DutyCycle;
}

ErrorStatus StartPwm2Ch1 (void)
{
  ErrorStatus status = SUCCESS;
  
   /* Start PWM mode with updated duty cycle */
  FTM_UpdatePwmDutycycle(PWM2_FTM_BASEADDR, (ftm_chnl_t)PWM2_CHANNEL1, kFTM_EdgeAlignedPwm,
                         pwm2Ch1DutyCycle);
  /* Software trigger to update registers */
  FTM_SetSoftwareTrigger(PWM2_FTM_BASEADDR, true);
 // Serial_Debug_Pwm2("\n PWM2 CH1 Duty Cycle Output: ");
 // Serial_Debug_Pwm2_Num(pwm2Ch1DutyCycle);
  return status;  
}

ErrorStatus StartPwm2Ch2 (void)
{
  ErrorStatus status = SUCCESS;
  
   /* Start PWM mode with updated duty cycle */
  FTM_UpdatePwmDutycycle(PWM2_FTM_BASEADDR, (ftm_chnl_t)PWM2_CHANNEL2, kFTM_EdgeAlignedPwm,
                         pwm2Ch2DutyCycle);
  /* Software trigger to update registers */
  FTM_SetSoftwareTrigger(PWM2_FTM_BASEADDR, true);
//  Serial_Debug_Pwm2("\n PWM2 CH2 Duty Cycle Output: ");
//  Serial_Debug_Pwm2_Num(pwm2Ch2DutyCycle);
  return status;  
}

ErrorStatus StopPwm2Ch1 (void)
{
  ErrorStatus status = SUCCESS;
  
  uint8_t duty_cycle = 0;
   /* Start PWM mode with updated duty cycle */
  FTM_UpdatePwmDutycycle(PWM2_FTM_BASEADDR, (ftm_chnl_t)PWM2_CHANNEL1, kFTM_EdgeAlignedPwm,
                         duty_cycle);
  /* Software trigger to update registers */
  FTM_SetSoftwareTrigger(PWM2_FTM_BASEADDR, true);
 // Serial_Debug_Pwm2("\n PWM2 CH1 Stop Duty Cycle Output: ");
//  Serial_Debug_Pwm2_Num(duty_cycle);
  return status;  
}

ErrorStatus StopPwm2Ch2 (void)
{
  ErrorStatus status = SUCCESS;
  
  uint8_t duty_cycle = 0;
   /* Start PWM mode with updated duty cycle */
  FTM_UpdatePwmDutycycle(PWM2_FTM_BASEADDR, (ftm_chnl_t)PWM2_CHANNEL2, kFTM_EdgeAlignedPwm,
                         duty_cycle);
  /* Software trigger to update registers */
  FTM_SetSoftwareTrigger(PWM2_FTM_BASEADDR, true);
 // Serial_Debug_Pwm2("\n PWM2 CH2 Stop Duty Cycle Output: ");
 // Serial_Debug_Pwm2_Num(duty_cycle);
  return status;  
}

//ErrorStatus StopPwm2 (void)
//{
//  ErrorStatus status;
//  
//  sConfig.Pulse = 0;
//  stopFlag = true;
//  if (HAL_TIM_PWM2_ConfigChannel(&Pwm2TimHandle, &sConfig, TIM_CHANNEL_3) != HAL_OK)
//  {
//    Error_Handler();
//    status = ERROR; 
//  }
//  else
//  {
//    if(HAL_TIM_PWM2_Start(&Pwm2TimHandle, TIM_CHANNEL_3) != HAL_OK)
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

//uint8_t GetCurrentPwm2Duty (void)
//{
//  if(stopFlag == true)
//  {
//    return 0;
//  }
//  else
//  {
//    return(pwm2DutyCycle);
//  }
//}

/**
* @brief  This function is executed in case of error occurrence.
* @param  None
* @retval None
*/
static void Error_Handler(void)
{
 Serial_Debug_Pwm2("\n PWM2 Error ");
}
