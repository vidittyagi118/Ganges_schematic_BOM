#include "BrushMotorControl_hal.h"
#include "BrushPwm.h"
#include "board.h"
#include "Serial_Debug.h"

#ifdef SERIAL_DEBUG_BRUSHMOTORCONTROL
  #define Serial_Debug_BrushMotorControl Serial_Debug
#else
  #define Serial_Debug_BrushMotorControl (void)
#endif

static stBrushMotPolarity brushMotorPolarity;
 
static void BrushMotorInitPins()
{
    gpio_pin_config_t out_config = {
    kGPIO_DigitalOutput, 0,
  };
  
  GPIO_PinInit(BOARD_MOTOR_ENABLE_GPIO, BOARD_BRUSHMOTOR_ENABLE_GPIO_PIN, &out_config);
  BrushMotorDisable();
}  

void BrushMotorInit_hal (void)
{
  brushMotorPolarity.mot1Polarity = 0;
  BrushPwmInit();
  BrushMotorInitPins();
  BrushMotorStop();
  BrushMotorEnable();
}

stBrushMotPolarity * GetSetBrushMotPolarity (void)
{
  return &brushMotorPolarity;
}

void BrushMotorMoveNegative (void)                                                   
{
  const stBrushMotPolarity *motPolarity = GetSetBrushMotPolarity();
  if(motPolarity->mot1Polarity == 0)
  {
    StopBrushPwmCh1();
    StartBrushPwmCh2(); 
  }
  else
  {
    StopBrushPwmCh2();
    StartBrushPwmCh1(); 
  }
  BrushMotorEnable();
}

void BrushMotorMovePositive (void)                                                   
{
  const stBrushMotPolarity *motPolarity = GetSetBrushMotPolarity();
  if(motPolarity->mot1Polarity == 0)
  {
    StopBrushPwmCh2();
    StartBrushPwmCh1(); 
  }
  else
  {
    StopBrushPwmCh1();
    StartBrushPwmCh2();  
  }
  BrushMotorEnable();
}

bool UpdateBrushMotorPositivePwm (float pwmDuty)                                                   
{
  bool status = false;
  const stBrushMotPolarity *motPolarity = GetSetBrushMotPolarity();
  if(motPolarity->mot1Polarity == 0)
  {
    status = SetUpdateBrushPwmCh1DutyCycle(pwmDuty);
  }
  else
  {
    status = SetUpdateBrushPwmCh2DutyCycle(pwmDuty);
  }
  return status;
}

bool UpdateBrushMotorNegativePwm (float pwmDuty)                                                   
{
  bool status = false;
  const stBrushMotPolarity *motPolarity = GetSetBrushMotPolarity();
  if(motPolarity->mot1Polarity == 0)
  {
    status = SetUpdateBrushPwmCh2DutyCycle(pwmDuty);
  }
  else
  {
    status = SetUpdateBrushPwmCh1DutyCycle(pwmDuty);
  }
  return status;
}

bool UpdateBrushMotorStopPwm (float pwmDuty)                                                   
{
  bool status = true;
  status &= SetUpdateBrushPwmCh2DutyCycle(STOP_PWM_VALUE);
  status &= SetUpdateBrushPwmCh1DutyCycle(STOP_PWM_VALUE);
  return status;
}

uint8_t GetBrushMotorPositivePwm (void)                                                   
{
  uint8_t pwmDuty;
  const stBrushMotPolarity *motPolarity = GetSetBrushMotPolarity();
  if(motPolarity->mot1Polarity == 0)
  {
    pwmDuty = GetBrushPwmCh1DutyCycle();
  }
  else
  {
    pwmDuty = GetBrushPwmCh2DutyCycle();
  }
  return pwmDuty;
}

uint8_t GetBrushMotorNegativePwm (void)                                                   
{
  uint8_t pwmDuty;
  const stBrushMotPolarity *motPolarity = GetSetBrushMotPolarity();
  if(motPolarity->mot1Polarity == 0)
  {
    pwmDuty = GetBrushPwmCh2DutyCycle();
  }
  else
  {
    pwmDuty = GetBrushPwmCh1DutyCycle();
  }
  return pwmDuty;
}

void BrushMotorBreak (void)                                                   
{
    BrushMotorDisable();
    StopBrushPwmCh1();
    StopBrushPwmCh2(); 
}

void BrushMotorStop (void)                                            
{
  ErrorStatus status = SUCCESS;
  StopBrushPwmCh1();
  StopBrushPwmCh2(); 
  if(status != SUCCESS)
  {
    Error_Handler();
  }
}

static inline void BrushMotorDisable (void)
{
   GPIO_PortClear(BOARD_MOTOR_ENABLE_GPIO, 1U << BOARD_BRUSHMOTOR_ENABLE_GPIO_PIN); 
}

static inline void BrushMotorEnable (void)
{
   GPIO_PortSet(BOARD_MOTOR_ENABLE_GPIO, 1U << BOARD_BRUSHMOTOR_ENABLE_GPIO_PIN); 
}


/**
* @brief  This function is executed in case of error occurrence.
* @param  None
* @retval None
*/
static void Error_Handler(void)
{
  Serial_Debug_BrushMotorControl("\nBrushMotor Control HAl error ");
}
