#ifndef __CYCLE_MODE_H_
#define __CYCLE_MODE_H_

#include "RobotControl.h"

#define SERIAL_DEBUG_CYCLEMODE


typedef enum eCycleModeState_def
{
  CYCLE_IDLE,
  CYCLE_START,
  CYCLE_POS_START,
  CYCLE_WAIT_TILL_POS_END,
  CYCLE_NEG_START,
  CYCLE_WAIT_TILL_NEG_END,
  CYCLE_COMPLETED
}eCycleModeState;

stRobotPwmParam * GetSetCycleManualPwmParameters (void);
stRobotPwmParam * GetSetCycleAutoPwmParameters (void);
eCycleModeState GetCycleModeState (void);
void SetCycleModeState (eCycleModeState cycleModeSt);
void CycleModeFSM (void);
bool IsCycleModeContinution (void);
#endif