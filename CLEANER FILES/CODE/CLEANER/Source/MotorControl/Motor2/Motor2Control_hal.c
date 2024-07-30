#include "Motor2Control_hal.h"
#include "Pwm2.h"
#include "board.h"
#include "Serial_Debug.h"

#ifdef SERIAL_DEBUG_MOTOR2CONTROL
  #define Serial_Debug_Motor2Control Serial_Debug
#else
  #define Serial_Debug_Motor2Control (void)
#endif

static stMot2Polarity motor2Polarity;
 
static void Motor2InitPins()
{
    gpio_pin_config_t out_config = {
    kGPIO_DigitalOutput, 0,
  };
  
  GPIO_PinInit(BOARD_MOTOR_ENABLE_GPIO, BOARD_MOTOR2_ENABLE_GPIO_PIN, &out_config);
  Motor2Disable();
}  

void Motor2Init_hal (void)
{
  motor2Polarity.mot2Polarity = 0;
  Pwm2Init();
  Motor2InitPins();
  Motor2Stop();
  Motor2Enable();
}

stMot2Polarity * GetSetMot2Polarity (void)
{
  return &motor2Polarity;
}

void Motor2MoveNegative (void)                                                   
{
  const stMot2Polarity *motPolarity = GetSetMot2Polarity();
  if(motPolarity->mot2Polarity == 0)
  {
    StopPwm2Ch1();
    StartPwm2Ch2(); 
  }
  else
  {
    StopPwm2Ch2();
    StartPwm2Ch1(); 
  }
  Motor2Enable();
}

void Motor2MovePositive (void)                                                   
{
  const stMot2Polarity *motPolarity = GetSetMot2Polarity();
  if(motPolarity->mot2Polarity == 0)
  {
    StopPwm2Ch2();
    StartPwm2Ch1(); 
  }
  else
  {
    StopPwm2Ch1();
    StartPwm2Ch2();  
  }
  Motor2Enable();
}

bool UpdateMotor2PositivePwm (float pwmDuty)                                                   
{
  bool status = false;
  const stMot2Polarity *motPolarity = GetSetMot2Polarity();
  if(motPolarity->mot2Polarity == 0)
  {
    status = SetUpdatePwm2Ch1DutyCycle(pwmDuty);
  }
  else
  {
    status = SetUpdatePwm2Ch2DutyCycle(pwmDuty);
  }
  return status;
}

bool UpdateMotor2NegativePwm (float pwmDuty)                                                   
{
  bool status = false;
  const stMot2Polarity *motPolarity = GetSetMot2Polarity();
  if(motPolarity->mot2Polarity == 0)
  {
    status = SetUpdatePwm2Ch2DutyCycle(pwmDuty);
  }
  else
  {
    status = SetUpdatePwm2Ch1DutyCycle(pwmDuty);
  }
  return status;
}

bool UpdateMotor2StopPwm (float pwmDuty)                                                   
{
  bool status = true;
  status &= SetUpdatePwm2Ch2DutyCycle(STOP_PWM_VALUE);
  status &= SetUpdatePwm2Ch1DutyCycle(STOP_PWM_VALUE);
  return status;
}

uint8_t GetMotor2PositivePwm (void)                                                   
{
  uint8_t pwmDuty;
  const stMot2Polarity *motPolarity = GetSetMot2Polarity();
  if(motPolarity->mot2Polarity == 0)
  {
    pwmDuty = GetPwm2Ch1DutyCycle();
  }
  else
  {
    pwmDuty = GetPwm2Ch2DutyCycle();
  }
  return pwmDuty;
}

uint8_t GetMotor2NegativePwm (void)                                                   
{
  uint8_t pwmDuty;
  const stMot2Polarity *motPolarity = GetSetMot2Polarity();
  if(motPolarity->mot2Polarity == 0)
  {
    pwmDuty = GetPwm2Ch2DutyCycle();
  }
  else
  {
    pwmDuty = GetPwm2Ch1DutyCycle();
  }
  return pwmDuty;
}

void Motor2Break (void)                                                   
{
    Motor2Disable();
    StopPwm2Ch1();
    StopPwm2Ch2(); 
}

void Motor2Stop (void)                                            
{
  ErrorStatus status = SUCCESS;
  StopPwm2Ch1();
  StopPwm2Ch2(); 
  if(status != SUCCESS)
  {
    Error_Handler();
  }
}

static inline void Motor2Disable (void)
{
   GPIO_PortClear(BOARD_MOTOR_ENABLE_GPIO, 1U << BOARD_MOTOR2_ENABLE_GPIO_PIN); 
}

static inline void Motor2Enable (void)
{
   GPIO_PortSet(BOARD_MOTOR_ENABLE_GPIO, 1U << BOARD_MOTOR2_ENABLE_GPIO_PIN); 
}


/**
* @brief  This function is executed in case of error occurrence.
* @param  None
* @retval None
*/
static void Error_Handler(void)
{
  Serial_Debug_Motor2Control("\nMotor2 Control HAl error ");
}
