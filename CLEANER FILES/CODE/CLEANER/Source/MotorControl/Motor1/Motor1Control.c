#include "Motor1Control.h"
#include "Motor1Control_hal.h"
#include "Pwm1.h"
#include "Serial_Debug.h"
#include "RobotOperation.h"
#include <stdbool.h>

#ifdef SERIAL_DEBUG_MOTOR1CONTROL
  #define Serial_Debug_Motor1Control Serial_Debug
#else
  #define Serial_Debug_Motor1Control (void)
#endif

typedef enum eMotor1DTimeState_def
{
  DTIME_IDLE,
  DTIME_START,
  DTIME_IN_PROGRESS,
  DTIME_COMPLETE  
}eMotor1DTimeState;

static eMotor1DTimeState GetDeadTimeState (void);
static void SetDeadTimeState (eMotor1DTimeState DTimeState);

static eMotor1State motor1State          = MOTOR1_IDLE;
static eMotor1State motor1TargetState    = MOTOR1_IDLE;
static eMotor1DTimeState deadTimeState   = DTIME_IDLE;
static volatile bool motor1TimeOver      = false;
static int32_t motor1_timecount          = 0;
uint32_t motor1DeadTime_ms               = DEFAULT_MOTOR1_DEAD_TIME_MS;

void Motor1Init (void)
{
  Motor1Init_hal ();  
}

void Motor1FSM (void)
{
  eMotor1State motor1TargetState = GetMotor1TargetState();
  eMotor1State motor1PresentState = GetMotor1State();
  
  if(IsLinearEnabled())
  {
    
  }
  else
  {
    SetMotor1TargetState(MOTOR1_STOP);
    SetMotor1State(MOTOR1_STOP);
  }
   
  motor1TargetState = GetMotor1TargetState();
  motor1PresentState = GetMotor1State();
  
  switch (motor1PresentState)
  {
   case MOTOR1_IDLE:
    OperateMotor1_IDLE(motor1TargetState);
    break;
   case MOTOR1_STOP:
    OperateMotor1_STOP(motor1TargetState);
    break;                                                      
   case MOTOR1_POSITIVE:
    OperateMotor1_POSITIVE(motor1TargetState);
    break;
   case MOTOR1_NEGATIVE:
    OperateMotor1_NEGATIVE(motor1TargetState);
    break;
   default:    
    break;
  }
}

void SetMotor1State (eMotor1State motor1StateValue)
{
  motor1State = motor1StateValue;
}

eMotor1State GetMotor1State (void)
{
  return (motor1State);
}

void SetMotor1TargetState (eMotor1State motor1StateValue)
{
  motor1TargetState = motor1StateValue;
}

eMotor1State GetMotor1TargetState (void)
{
  return (motor1TargetState);
}

static void OperateMotor1_IDLE (eMotor1State targetState)
{
  switch(targetState)
  {
   case MOTOR1_IDLE:
    break;
   case MOTOR1_POSITIVE:
   case MOTOR1_NEGATIVE:
   case MOTOR1_STOP:
    Motor1Stop();
    SetMotor1State(MOTOR1_STOP);
    SetDeadTimeState(DTIME_START);
    break;
   default:
    break;
  }
}

bool SetMotor1AllPwm(float duty)
{
  bool status = true;
  status &= SetPwm1Ch1DutyCycle(duty);
  status &= SetPwm1Ch2DutyCycle(duty);  
  return status;
}

bool UpdateMotor1Pwm (float pwmDuty)
{
  bool status = true;
  if (GetMotor1State() == MOTOR1_POSITIVE)
  {
    status = UpdateMotor1PositivePwm(pwmDuty);
  }
  else if (GetMotor1State() == MOTOR1_NEGATIVE)
  {
    status = UpdateMotor1NegativePwm(pwmDuty);   
  }
  else
  {
    status = false;
  }
  return status;
}


uint8_t GetMotor1Pwm (void)
{
  uint8_t pwmDuty;
  if (GetMotor1State() == MOTOR1_POSITIVE)
  {
    pwmDuty = GetMotor1PositivePwm();
  }
  else if (GetMotor1State() == MOTOR1_NEGATIVE)
  {
    pwmDuty = GetMotor1NegativePwm(); 
  }
  else
  {
    pwmDuty = 0;
  }
  return pwmDuty;
}

static void OperateMotor1_STOP (eMotor1State targetState)
{
  DeadTimeFSM();
  switch(targetState)
  {
   case MOTOR1_IDLE:
    Motor1Stop();
    SetMotor1State(MOTOR1_IDLE);
    break;
   case MOTOR1_STOP:
    break;
   case MOTOR1_POSITIVE:
    if(GetDeadTimeState() == DTIME_COMPLETE)
    {
      Motor1MovePositive();
      SetMotor1State(MOTOR1_POSITIVE);
    }
    break;
   case MOTOR1_NEGATIVE:
    if(GetDeadTimeState() == DTIME_COMPLETE)
    {
      Motor1MoveNegative();
      SetMotor1State(MOTOR1_NEGATIVE);
    }
    break;    
   default:
    break;
  }
}

static void OperateMotor1_POSITIVE(eMotor1State targetState)
{
  switch(targetState)
  {
   case MOTOR1_IDLE:
    Motor1Stop();
    SetMotor1State(MOTOR1_IDLE);
    SetDeadTimeState(DTIME_START);
    break;
   case MOTOR1_NEGATIVE:
   case MOTOR1_STOP:
    Motor1Stop();
    SetMotor1State(MOTOR1_STOP);
    SetDeadTimeState(DTIME_START);
    break;
   case MOTOR1_POSITIVE:
    break;
   default:
    break;
  }
}


static void OperateMotor1_NEGATIVE(eMotor1State targetState)
{
  switch(targetState)
  {
   case MOTOR1_IDLE:
    Motor1Stop();
    SetMotor1State(MOTOR1_IDLE);
    SetDeadTimeState(DTIME_START);
    break;
   case MOTOR1_POSITIVE:
   case MOTOR1_STOP:
    Motor1Stop();
    SetMotor1State(MOTOR1_STOP);
    SetDeadTimeState(DTIME_START);
    break;
   case MOTOR1_NEGATIVE:
    break;    
   default:
    break;
  }
}

/* This Function is Only for Testing. It alters the set PWM value to Maximum */
void OperateMotor1ForTest(eMotor1State targetState)
{
  switch(targetState)
  {
   case MOTOR1_IDLE:
    Serial_Debug_Motor1Control("\n TestMode: Motor1 Idle ");
    Motor1Stop();
    break;
   case MOTOR1_POSITIVE:
    Serial_Debug_Motor1Control("\n TestMode: Motor1 Positive");
    Motor1Stop();
    SetPwm1Ch1DutyCycle(99);
    SetPwm1Ch2DutyCycle(99);
    Motor1MovePositive();
    break;
   case MOTOR1_STOP:
    Serial_Debug_Motor1Control("\n TestMode: Motor1 Stop");
    Motor1Stop();
    break;
   case MOTOR1_NEGATIVE:
    Serial_Debug_Motor1Control("\n TestMode: Motor1 Negative");
    Motor1Stop();
    SetPwm1Ch2DutyCycle(99);
    SetPwm1Ch1DutyCycle(99);
    Motor1MoveNegative();
    break;    
   default:
    break;
  } 
}
                         
static void SetDeadTimeState (eMotor1DTimeState DTimeState)
{
  deadTimeState = DTimeState;
}

static eMotor1DTimeState GetDeadTimeState (void)
{
  return(deadTimeState);  
}

static void DeadTimeFSM (void)
{
  eMotor1DTimeState DTimeState = GetDeadTimeState();
  switch (DTimeState)
  {
   case DTIME_IDLE:
    /* No break. If called from idle State it will satrt the dead time routine */
   case DTIME_START:
    Motor1TimerOn(motor1DeadTime_ms);
    SetDeadTimeState(DTIME_IN_PROGRESS);
   case DTIME_IN_PROGRESS:                            /* No break necessary */
    if(IsMotor1TimeOver() == true)
    {
      Motor1TimerStop();
      ClearMotor1TimeOver();
      SetDeadTimeState(DTIME_COMPLETE);                         
    }
    break;
   case DTIME_COMPLETE:
    break;
   default:
    break;
  } 
}

static void Motor1TimerOn (uint32_t setMotor1TimeCount_ms)
{
  if(setMotor1TimeCount_ms == 0)
  {
    motor1TimeOver = true;
    Motor1TimerStop();
  }
  else
  {
    motor1_timecount = setMotor1TimeCount_ms;
    motor1TimeOver = false;
  }
}

static inline void Motor1TimerStop (void)
{
  motor1_timecount = 0;
}

static inline bool IsMotor1TimeOver (void)
{
  return (motor1TimeOver);
}

static inline void ClearMotor1TimeOver (void)
{
  motor1TimeOver = false; 
}

void Motor1TimeIncrement_ms (void)
{
  if(motor1_timecount)
  {
    if(--motor1_timecount <= 0)
    {
      motor1TimeOver = true;
      Motor1TimerStop();
    }
    else
    {
      motor1TimeOver = false;
    }
  }
}
