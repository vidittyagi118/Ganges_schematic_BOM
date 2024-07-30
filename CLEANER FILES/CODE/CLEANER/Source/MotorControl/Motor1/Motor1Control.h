#ifndef __MOTOR1CONTROL_H_
#define __MOTOR1CONTROL_H_

#include <stdbool.h>
#include <stdint.h>

typedef enum eMotor1State_def
{
  MOTOR1_IDLE,
  MOTOR1_STOP,
  MOTOR1_POSITIVE,
  MOTOR1_NEGATIVE  
}eMotor1State;

bool SetMotor1AllPwm(float duty);
bool UpdateMotor1Pwm (float pwmDuty);
uint8_t GetMotor1Pwm (void);

void Motor1Init (void);
void Motor1FSM (void);
void Motor1TimeIncrement_ms (void);
void SetMotor1TargetState (eMotor1State motor1StateValue);
void SetMotor1State (eMotor1State motor1StateValue);
eMotor1State GetMotor1State (void);
eMotor1State GetMotor1TargetState (void);

static void OperateMotor1_IDLE (eMotor1State targetState);
static void OperateMotor1_STOP (eMotor1State targetState);
static void OperateMotor1_POSITIVE(eMotor1State targetState);
static void OperateMotor1_NEGATIVE(eMotor1State targetState);

static void DeadTimeFSM (void);
static void Motor1TimerOn (uint32_t setMotor1TimeCount_ms);
static inline void Motor1TimerStop (void);
static inline bool IsMotor1TimeOver (void);
static inline void ClearMotor1TimeOver (void);

void OperateMotor1ForTest(eMotor1State targetState);
#endif