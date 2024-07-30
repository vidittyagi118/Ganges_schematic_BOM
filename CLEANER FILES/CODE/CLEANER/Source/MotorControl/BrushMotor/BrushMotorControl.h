#ifndef __BRUSHMOTORCONTROL_H_
#define __BRUSHMOTORCONTROL_H_

#include <stdbool.h>
#include <stdint.h>

typedef enum eBrushMotorState_def
{
  BRUSHMOTOR_IDLE,
  BRUSHMOTOR_STOP,
  BRUSHMOTOR_POSITIVE,
  BRUSHMOTOR_NEGATIVE  
}eBrushMotorState;

bool SetBrushMotorAllPwm(float duty);
bool UpdateBrushMotorPwm (float pwmDuty);
uint8_t GetBrushMotorPwm (void);

void BrushMotorInit (void);
void BrushMotorFSM (void);
void BrushMotorTimeIncrement_ms (void);
void SetBrushMotorTargetState (eBrushMotorState brushMotorStateValue);
void SetBrushMotorState (eBrushMotorState brushMotorStateValue);
eBrushMotorState GetBrushMotorState (void);
eBrushMotorState GetBrushMotorTargetState (void);

static void OperateBrushMotor_IDLE (eBrushMotorState targetState);
static void OperateBrushMotor_STOP (eBrushMotorState targetState);
static void OperateBrushMotor_POSITIVE(eBrushMotorState targetState);
static void OperateBrushMotor_NEGATIVE(eBrushMotorState targetState);

static void DeadTimeFSM (void);
static void BrushMotorTimerOn (uint32_t setBrushMotorTimeCount_ms);
static inline void BrushMotorTimerStop (void);
static inline bool IsBrushMotorTimeOver (void);
static inline void ClearBrushMotorTimeOver (void);

void OperateBrushMotorForTest(eBrushMotorState targetState);
#endif