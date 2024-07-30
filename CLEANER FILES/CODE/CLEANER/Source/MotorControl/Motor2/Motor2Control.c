#include "Motor2Control.h"
#include "Motor2Control_hal.h"
#include "Pwm2.h"
#include "Serial_Debug.h"
#include "RobotOperation.h"

#include <stdbool.h>

#ifdef SERIAL_DEBUG_MOTOR2CONTROL
  #define Serial_Debug_Motor2Control Serial_Debug
#else
  #define Serial_Debug_Motor2Control (void)
#endif

typedef enum eMotor2DTimeState_def
{
  DTIME_IDLE,
  DTIME_START,
  DTIME_IN_PROGRESS,
  DTIME_COMPLETE  
}eMotor2DTimeState;

static eMotor2DTimeState GetDeadTimeState (void);
static void SetDeadTimeState (eMotor2DTimeState DTimeState);

static eMotor2State motor2State          = MOTOR2_IDLE;
static eMotor2State motor2TargetState    = MOTOR2_IDLE;
static eMotor2DTimeState deadTimeState   = DTIME_IDLE;
static volatile bool motor2TimeOver      = false;
static int32_t motor2_timecount          = 0;
uint32_t motor2DeadTime_ms               = DEFAULT_MOTOR2_DEAD_TIME_MS;

void Motor2Init (void)
{
  Motor2Init_hal ();  
}

void Motor2FSM (void)
{
  eMotor2State motor2TargetState = GetMotor2TargetState();
  eMotor2State motor2PresentState = GetMotor2State();
  
  if(IsLinearEnabled())
  {
    
  }
  else
  {
    SetMotor2TargetState(MOTOR2_STOP);
    SetMotor2State(MOTOR2_STOP);
  }
   
  motor2TargetState = GetMotor2TargetState();
  motor2PresentState = GetMotor2State();
  
  switch (motor2PresentState)
  {
   case MOTOR2_IDLE:
    OperateMotor2_IDLE(motor2TargetState);
    break;
   case MOTOR2_STOP:
    OperateMotor2_STOP(motor2TargetState);
    break;                                                      
   case MOTOR2_POSITIVE:
    OperateMotor2_POSITIVE(motor2TargetState);
    break;
   case MOTOR2_NEGATIVE:
    OperateMotor2_NEGATIVE(motor2TargetState);
    break;
   default:    
    break;
  }
}

void SetMotor2State (eMotor2State motor2StateValue)
{
  motor2State = motor2StateValue;
}

eMotor2State GetMotor2State (void)
{
  return (motor2State);
}

void SetMotor2TargetState (eMotor2State motor2StateValue)
{
  motor2TargetState = motor2StateValue;
}

eMotor2State GetMotor2TargetState (void)
{
  return (motor2TargetState);
}

static void OperateMotor2_IDLE (eMotor2State targetState)
{
  switch(targetState)
  {
   case MOTOR2_IDLE:
    break;
   case MOTOR2_POSITIVE:
   case MOTOR2_NEGATIVE:
   case MOTOR2_STOP:
    Motor2Stop();
    SetMotor2State(MOTOR2_STOP);
    SetDeadTimeState(DTIME_START);
    break;
   default:
    break;
  }
}

bool SetMotor2AllPwm(float duty)
{
  bool status = true;
  status &= SetPwm2Ch1DutyCycle(duty);
  status &= SetPwm2Ch2DutyCycle(duty);  
  return status;
}

bool UpdateMotor2Pwm (float pwmDuty)
{
  bool status = true;
  if (GetMotor2State() == MOTOR2_POSITIVE)
  {
    status = UpdateMotor2PositivePwm(pwmDuty);
  }
  else if (GetMotor2State() == MOTOR2_NEGATIVE)
  {
    status = UpdateMotor2NegativePwm(pwmDuty);   
  }
  else
  {
    status = false;
  }
  return status;
}


uint8_t GetMotor2Pwm (void)
{
  uint8_t pwmDuty;
  if (GetMotor2State() == MOTOR2_POSITIVE)
  {
    pwmDuty = GetMotor2PositivePwm();
  }
  else if (GetMotor2State() == MOTOR2_NEGATIVE)
  {
    pwmDuty = GetMotor2NegativePwm(); 
  }
  else
  {
    pwmDuty = 0;
  }
  return pwmDuty;
}

static void OperateMotor2_STOP (eMotor2State targetState)
{
  DeadTimeFSM();
  switch(targetState)
  {
   case MOTOR2_IDLE:
    Motor2Stop();
    SetMotor2State(MOTOR2_IDLE);
    break;
   case MOTOR2_STOP:
    break;
   case MOTOR2_POSITIVE:
    if(GetDeadTimeState() == DTIME_COMPLETE)
    {
      Motor2MovePositive();
      SetMotor2State(MOTOR2_POSITIVE);
    }
    break;
   case MOTOR2_NEGATIVE:
    if(GetDeadTimeState() == DTIME_COMPLETE)
    {
      Motor2MoveNegative();
      SetMotor2State(MOTOR2_NEGATIVE);
    }
    break;    
   default:
    break;
  }
}

static void OperateMotor2_POSITIVE(eMotor2State targetState)
{
  switch(targetState)
  {
   case MOTOR2_IDLE:
    Motor2Stop();
    SetMotor2State(MOTOR2_IDLE);
    SetDeadTimeState(DTIME_START);
    break;
   case MOTOR2_NEGATIVE:
   case MOTOR2_STOP:
    Motor2Stop();
    SetMotor2State(MOTOR2_STOP);
    SetDeadTimeState(DTIME_START);
    break;
   case MOTOR2_POSITIVE:
    break;
   default:
    break;
  }
}


static void OperateMotor2_NEGATIVE(eMotor2State targetState)
{
  switch(targetState)
  {
   case MOTOR2_IDLE:
    Motor2Stop();
    SetMotor2State(MOTOR2_IDLE);
    SetDeadTimeState(DTIME_START);
    break;
   case MOTOR2_POSITIVE:
   case MOTOR2_STOP:
    Motor2Stop();
    SetMotor2State(MOTOR2_STOP);
    SetDeadTimeState(DTIME_START);
    break;
   case MOTOR2_NEGATIVE:
    break;    
   default:
    break;
  }
}

/* This Function is Only for Testing. It alters the set PWM value to Maximum */
void OperateMotor2ForTest(eMotor2State targetState)
{
  switch(targetState)
  {
   case MOTOR2_IDLE:
    Serial_Debug_Motor2Control("\n TestMode: Motor2 Idle ");
    Motor2Stop();
    break;
   case MOTOR2_POSITIVE:
    Serial_Debug_Motor2Control("\n TestMode: Motor2 Positive");
    Motor2Stop();
    SetPwm2Ch1DutyCycle(99);
    SetPwm2Ch2DutyCycle(99);
    Motor2MovePositive();
    break;
   case MOTOR2_STOP:
    Serial_Debug_Motor2Control("\n TestMode: Motor2 Stop");
    Motor2Stop();
    break;
   case MOTOR2_NEGATIVE:
    Serial_Debug_Motor2Control("\n TestMode: Motor2 Negative");
    Motor2Stop();
    SetPwm2Ch2DutyCycle(99);
    SetPwm2Ch1DutyCycle(99);
    Motor2MoveNegative();
    break;    
   default:
    break;
  } 
}
                         
static void SetDeadTimeState (eMotor2DTimeState DTimeState)
{
  deadTimeState = DTimeState;
}

static eMotor2DTimeState GetDeadTimeState (void)
{
  return(deadTimeState);  
}

static void DeadTimeFSM (void)
{
  eMotor2DTimeState DTimeState = GetDeadTimeState();
  switch (DTimeState)
  {
   case DTIME_IDLE:
    /* No break. If called from idle State it will satrt the dead time routine */
   case DTIME_START:
    Motor2TimerOn(motor2DeadTime_ms);
    SetDeadTimeState(DTIME_IN_PROGRESS);
   case DTIME_IN_PROGRESS:                            /* No break necessary */
    if(IsMotor2TimeOver() == true)
    {
      Motor2TimerStop();
      ClearMotor2TimeOver();
      SetDeadTimeState(DTIME_COMPLETE);                         
    }
    break;
   case DTIME_COMPLETE:
    break;
   default:
    break;
  } 
}

static void Motor2TimerOn (uint32_t setMotor2TimeCount_ms)
{
  if(setMotor2TimeCount_ms == 0)
  {
    motor2TimeOver = true;
    Motor2TimerStop();
  }
  else
  {
    motor2_timecount = setMotor2TimeCount_ms;
    motor2TimeOver = false;
  }
}

static inline void Motor2TimerStop (void)
{
  motor2_timecount = 0;
}

static inline bool IsMotor2TimeOver (void)
{
  return (motor2TimeOver);
}

static inline void ClearMotor2TimeOver (void)
{
  motor2TimeOver = false; 
}

void Motor2TimeIncrement_ms (void)
{
  if(motor2_timecount)
  {
    if(--motor2_timecount <= 0)
    {
      motor2TimeOver = true;
      Motor2TimerStop();
    }
    else
    {
      motor2TimeOver = false;
    }
  }
}
