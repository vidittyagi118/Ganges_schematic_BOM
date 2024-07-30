#ifndef __MOTOR2CONTROL_H_
#define __MOTOR2CONTROL_H_

#include <stdbool.h>
#include <stdint.h>

typedef enum eMotor2State_def
{
  MOTOR2_IDLE,
  MOTOR2_STOP,
  MOTOR2_POSITIVE,
  MOTOR2_NEGATIVE  
}eMotor2State;

void Motor2Init (void);
void Motor2FSM (void);
void Motor2TimeIncrement_ms (void);
void SetMotor2TargetState (eMotor2State motor2StateValue);
void SetMotor2State (eMotor2State motor2StateValue);
eMotor2State GetMotor2State (void);
eMotor2State GetMotor2TargetState (void);

bool SetMotor2AllPwm(float duty);
bool UpdateMotor2Pwm (float pwmDuty);
uint8_t GetMotor2Pwm (void);

static void OperateMotor2_IDLE (eMotor2State targetState);
static void OperateMotor2_STOP (eMotor2State targetState);
static void OperateMotor2_POSITIVE(eMotor2State targetState);
static void OperateMotor2_NEGATIVE(eMotor2State targetState);

static void DeadTimeFSM (void);
static void Motor2TimerOn (uint32_t setMotor2TimeCount_ms);
static inline void Motor2TimerStop (void);
static inline bool IsMotor2TimeOver (void);
static inline void ClearMotor2TimeOver (void);

void OperateMotor2ForTest(eMotor2State targetState);
#endif