#ifndef __BRUSHCONTROL_H_
#define __BRUSHCONTROL_H_

#include <stdint.h>
#include <stdbool.h>

typedef enum eBrushState_def
{
  BRUSH_IDLE,
  BRUSH_START,
  BRUSH_DIR_CHANGE_STOP,
  BRUSH_WAIT_FOR_VALID_STATE,
  BRUSH_ACCELERATION_START,
  BRUSH_ACCELERATION,
  BRUSH_STEADYSTATE,
  BRUSH_NORMAL_STOP,
  BRUSH_DECELERATION_START,
  BRUSH_DECELERATION,
  BRUSH_APPROACH_COUNT,
  BRUSH_IMMEDIATE_STOP,
  BRUSH_END,
  BRUSH_COMPLETED,
  BRUSH_RE_START,
  BRUSH_DISABLEDSTATE,
}eBrushState;

typedef enum eBrushDirection_def
{
  BRUSH_POSITIVE,
  BRUSH_NEGATIVE,
  BRUSH_STOP,
  BRUSH_UNKNOWN
}eBrushDirection;

typedef struct {
  uint32_t accelTime;
  uint32_t decelTime;
  uint8_t steadyPwm;
  uint32_t brushTime;         //07D
  //uint8_t timePwm; 
} stBrushPwmParam;

typedef struct {
  eBrushDirection direction;
  stBrushPwmParam BrushPwmParam;
} stBrushParam;

bool BrushInit (void);
void BrushFSM (void);
void SetBrushState (eBrushState setBrushState);
eBrushState GetBrushState (void);
void BrushPwmChangeTimeIncrement_ms (void);
stBrushPwmParam * GetSetBrushNormalPwmParameters (void);
stBrushPwmParam * GetSetBrushAutoPwmParameters (void);
void StartBrush (eBrushDirection dir);
void StopBrush ();
void ImmediateStopBrush ();
bool * GetSetBrushEnabledState (void);
#endif