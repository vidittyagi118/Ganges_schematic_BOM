#ifndef __ROBOT_OPERATION_H_
#define __ROBOT_OPERATION_H_

#include "RobotControl.h"
#include <stdint.h>
#include <stdbool.h>

typedef enum eMotorCommand_def
{
  ROBOT_CMD_IDLE,
  ROBOT_CMD_NONE,
  ROBOT_CMD_POSITIVE,
  ROBOT_CMD_NEGATIVE,
  ROBOT_CMD_CYCLE,
  ROBOT_CMD_NORMAL_STOP,
  ROBOT_CMD_STOP,
  ROBOT_CMD_EMERGENCY_STOW,
  ROBOT_CMD_GOTO_COUNT,
  ROBOT_CMD_MANUAL,
  ROBOT_CMD_AUTO, 
  ROBOT_CMD_CYCLE_TEST,
  ROBOT_CMD_CYCLE_RESUME
}eRobotCommand;

typedef struct {
  uint32_t emergencyStowTargetCount;
} stModeCounts;

typedef enum eMode_def
{
  AUTO,
  MANUAL,  
  IDLE,
  EMERGENCY_STOW,
  COMM_ERROR_EMERGENCY_STOW,
  CYCLE_TEST
}eMode;

typedef enum eManualState_def
{
  MAN_IDLE,
  MAN_POSITIVE,
  MAN_NEGATIVE,
  MAN_CYCLE,
  MAN_NORMAL_STOP,
  MAN_STOP,
  MAN_GOTO_COUNT,
  MAN_RESUME
}eManualState;

typedef enum eMotionPauseState_def
{
  PA_IDLE,
  PA_PAUSE,
  PA_WAIT_FOR_STOP,
  PA_WAIT_FOR_ERROR_CLEAR,
  PA_WAIT_FOR_TIMEOVER_RESUME,
  PA_COMPLETED
}eMotionPauseState;

void SetMotionCommand (eRobotCommand command);
eRobotCommand GetMotionCommand (void);
void RobotOperate (void);
void SetManualPwmParameters (stRobotPwmParam *robotPwmParam);
void SetEmergencyPwmParameters (stRobotPwmParam *robotPwmParam);
void SetModeCounts (stModeCounts *modeSetCounts);
stModeCounts * GetSetModeCounts (void);
stRobotPwmParam * GetSetEmergencyPwmParameters (void);
stRobotPwmParam * GetSetManualPwmParameters (void);
void SetMotionPauseState (eMotionPauseState pauseSt);
eMotionPauseState GetMotionPauseState (void);
eMotionPauseState GetPrevPauseState(void);
void SetPrevPauseState(eMotionPauseState state);
uint32_t GetPrevRotateSense2Count(void);
uint32_t GetPrevRotateSense1Count(void);
bool GetResumeBit(void);
void SetResumeBit(bool value);

void SetOperationMode (eMode setMode, eManualState opManualState);
eMode GetOperationMode (void);
eManualState GetManualState (void);
void LoadRobotParameters(uint32_t count, stRobotPwmParam * modePwmParam);
uint32_t GetGotoCount(void);
bool SetGotoCount(uint32_t);
bool IsCountValid(uint32_t count);
void RobotOperationTimeIncrement_ms (void);
void SetInitialOperationMode (eMode eepromReadMode);
bool * GetSetLinearEnabledState (void);
bool IsLinearEnabled (void);

void SetRowLengthData(stRowLengthData *rowData);
stRowLengthData *GetSetRowLengthData(void);
#endif