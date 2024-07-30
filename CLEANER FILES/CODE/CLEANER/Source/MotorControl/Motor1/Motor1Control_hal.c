#include "Motor1Control_hal.h"
#include "Pwm1.h"
#include "board.h"
#include "Serial_Debug.h"

#ifdef SERIAL_DEBUG_MOTOR1CONTROL
  #define Serial_Debug_Motor1Control Serial_Debug
#else
  #define Serial_Debug_Motor1Control (void)
#endif

static stMot1Polarity motor1Polarity;
 
static void Motor1InitPins()
{
    gpio_pin_config_t out_config = {
    kGPIO_DigitalOutput, 0,
  };
  
  GPIO_PinInit(BOARD_MOTOR_ENABLE_GPIO, BOARD_MOTOR1_ENABLE_GPIO_PIN, &out_config);
  Motor1Disable();
}  

void Motor1Init_hal (void)
{
  motor1Polarity.mot1Polarity = 0;
  Pwm1Init();
  Motor1InitPins();
  Motor1Stop();
  Motor1Enable();
}

stMot1Polarity * GetSetMot1Polarity (void)
{
  return &motor1Polarity;
}

void Motor1MoveNegative (void)                                                   
{
  const stMot1Polarity *motPolarity = GetSetMot1Polarity();
  if(motPolarity->mot1Polarity == 0)
  {
    StopPwm1Ch1();
    StartPwm1Ch2(); 
  }
  else
  {
    StopPwm1Ch2();
    StartPwm1Ch1(); 
  }
  Motor1Enable();
}

void Motor1MovePositive (void)                                                   
{
  const stMot1Polarity *motPolarity = GetSetMot1Polarity();
  if(motPolarity->mot1Polarity == 0)
  {
    StopPwm1Ch2();
    StartPwm1Ch1(); 
  }
  else
  {
    StopPwm1Ch1();
    StartPwm1Ch2();  
  }
  Motor1Enable();
}

bool UpdateMotor1PositivePwm (float pwmDuty)                                                   
{
  bool status = false;
  const stMot1Polarity *motPolarity = GetSetMot1Polarity();
  if(motPolarity->mot1Polarity == 0)
  {
    status = SetUpdatePwm1Ch1DutyCycle(pwmDuty);
  }
  else
  {
    status = SetUpdatePwm1Ch2DutyCycle(pwmDuty);
  }
  return status;
}

bool UpdateMotor1NegativePwm (float pwmDuty)                                                   
{
  bool status = false;
  const stMot1Polarity *motPolarity = GetSetMot1Polarity();
  if(motPolarity->mot1Polarity == 0)
  {
    status = SetUpdatePwm1Ch2DutyCycle(pwmDuty);
  }
  else
  {
    status = SetUpdatePwm1Ch1DutyCycle(pwmDuty);
  }
  return status;
}

bool UpdateMotor1StopPwm (float pwmDuty)                                                   
{
  bool status = true;
  status &= SetUpdatePwm1Ch2DutyCycle(STOP_PWM_VALUE);
  status &= SetUpdatePwm1Ch1DutyCycle(STOP_PWM_VALUE);
  return status;
}

uint8_t GetMotor1PositivePwm (void)                                                   
{
  uint8_t pwmDuty;
  const stMot1Polarity *motPolarity = GetSetMot1Polarity();
  if(motPolarity->mot1Polarity == 0)
  {
    pwmDuty = GetPwm1Ch1DutyCycle();
  }
  else
  {
    pwmDuty = GetPwm1Ch2DutyCycle();
  }
  return pwmDuty;
}

uint8_t GetMotor1NegativePwm (void)                                                   
{
  uint8_t pwmDuty;
  const stMot1Polarity *motPolarity = GetSetMot1Polarity();
  if(motPolarity->mot1Polarity == 0)
  {
    pwmDuty = GetPwm1Ch2DutyCycle();
  }
  else
  {
    pwmDuty = GetPwm1Ch1DutyCycle();
  }
  return pwmDuty;
}

void Motor1Break (void)                                                   
{
    Motor1Disable();
    StopPwm1Ch1();
    StopPwm1Ch2(); 
}

void Motor1Stop (void)                                            
{
  ErrorStatus status = SUCCESS;
  StopPwm1Ch1();
  StopPwm1Ch2(); 
  if(status != SUCCESS)
  {
    Error_Handler();
  }
}

static inline void Motor1Disable (void)
{
   GPIO_PortClear(BOARD_MOTOR_ENABLE_GPIO, 1U << BOARD_MOTOR1_ENABLE_GPIO_PIN); 
}

static inline void Motor1Enable (void)
{
   GPIO_PortSet(BOARD_MOTOR_ENABLE_GPIO, 1U << BOARD_MOTOR1_ENABLE_GPIO_PIN); 
}


/**
* @brief  This function is executed in case of error occurrence.
* @param  None
* @retval None
*/
static void Error_Handler(void)
{
  Serial_Debug_Motor1Control("\nMotor1 Control HAl error ");
}
