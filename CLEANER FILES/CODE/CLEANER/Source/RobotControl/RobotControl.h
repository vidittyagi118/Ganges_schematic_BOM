#ifndef __ROBOTCONTROL_H_
#define __ROBOTCONTROL_H_

#include <stdint.h>
#include <stdbool.h>

#define MAX_NO_OF_TRACKS 20

typedef enum eRobotState_def
{
  ROBOT_IDLE,
  ROBOT_START,
  ROBOT_DIR_CHANGE_STOP,
  ROBOT_WAIT_FOR_VALID_STATE,
  ROBOT_ACCELERATION_SLOW_START,
  ROBOT_ACCELERATION_START,
  ROBOT_ACCELERATION,
  ROBOT_STEADYSTATE,
  ROBOT_NORMAL_STOP,
  ROBOT_DECELERATION_START,
  ROBOT_DECELERATION,
  ROBOT_APPROACH_COUNT,
  ROBOT_IMMEDIATE_STOP,
  ROBOT_HOME_RETURN,
  ROBOT_END,
  ROBOT_WAIT_FOR_BRUSH_STOP,
  ROBOT_COMPLETED,
  ROBOT_RE_START
}eRobotState;

typedef enum eRobotDirection_def
{
  POSITIVE,
  NEGATIVE,
  STOP,
  UNKNOWN  
}eRobotDirection;

typedef enum {
  FORWARD_COUNT, 
  REVERSE_COUNT,
  TOTAL_NO_OF_COUNT
}eRowLengthData;

typedef struct {
 uint32_t accelTime;
  uint32_t decelTime; 
  uint32_t Accel;
  uint32_t acelStartCountDiff;
  uint8_t accelapproachPwm1; 
  uint8_t accelapproachPwm2;
  uint8_t steadyPwm1;
  uint8_t steadyPwm2;
  uint8_t approachPwm1; 
  uint8_t approachPwm2;
  uint32_t Decel;
  uint32_t decelStartCountDiff;
  uint32_t timePwm;
} stRobotPwmParam;

typedef struct {
uint32_t rowLength[MAX_NO_OF_TRACKS][TOTAL_NO_OF_COUNT];
uint32_t rowLength2[MAX_NO_OF_TRACKS][TOTAL_NO_OF_COUNT];
uint32_t rowLength3[MAX_NO_OF_TRACKS][TOTAL_NO_OF_COUNT];
uint32_t rowLength4[MAX_NO_OF_TRACKS][TOTAL_NO_OF_COUNT];
uint32_t rowLength5[MAX_NO_OF_TRACKS][TOTAL_NO_OF_COUNT];
uint32_t rowLength6[MAX_NO_OF_TRACKS][TOTAL_NO_OF_COUNT];
uint32_t rowLength7[MAX_NO_OF_TRACKS][TOTAL_NO_OF_COUNT];
}stRowLengthData;

typedef struct {
  uint32_t count;
  eRobotDirection direction;
  stRobotPwmParam RobotPwmParam;
} stRobotParam;

typedef struct {
  uint32_t maxNegCountForPVsetup;
  uint32_t maxPosCountForPVsetup;
} stCountRange;

typedef struct {
  uint32_t startTime;
  uint32_t endTime;
  float startSOC;
  float endSOC;
  uint32_t totalRotateCount1;
  uint32_t totalRotateCount2;
  uint32_t errorCode;
} stLastOperationStatus;

bool RobotInit (void);
void RobotFSM (void);
bool GetTargetCount(void);
void SetRobotState (eRobotState setRobotState);
eRobotState GetRobotState (void);
eRobotDirection GetRobotActualDirection (void);
eRobotDirection CalculateRobotDirection (float startCount, float tarCount); 
void PwmChangeTimeIncrement_ms (void);
void SetCurrentModeRobotParameter (stRobotParam *motionParam);
void RobotTimeIncrement_ms (void);
stCountRange *GetCountRange(void);
stCountRange *GetSetCountRange(void);
stLastOperationStatus * GetLastOperationData (void);

uint32_t Current_Brush_Count(uint32_t);
uint32_t Get_Current_Brush_Count(void);

#endif