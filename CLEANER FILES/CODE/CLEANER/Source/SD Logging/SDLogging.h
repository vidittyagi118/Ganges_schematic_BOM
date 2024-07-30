#ifndef _SD_LOGGING_H_
#define _SD_LOGGING_H_

#include <stdint.h>
#include <stdbool.h> 
#include <stdio.h>

#include "fsl_rtc.h"
#include "rtc.h"
#include "APIProcessing.h"
#include "switch.h"
#include "ZigbeeApiMode.h"

typedef enum eFileType_def{API,EVENT}eFileType;

typedef enum eEventType_def{
  EV_ROBOT_FORWARD,
  EV_ROBOT_START,
  EV_ROBOT_ACCEL_COMPLETE,
  EV_ROBOT_DECEL_START,
  EV_ROBOT_DECEL_COMPLETE,
  EV_ROBOT_STOP,
  EV_ROBOT_REVERSE,
  EV_MOTOR_1_OVERCURRENT,
  EV_MOTOR_2_OVERCURRENT,
  EV_BRUSH_OVERCURRENT,
  EV_LOW_BATTERY,
  EV_HIGH_TEMPERATURE,
  EV_PERIODIC_LOG
}eEventType;

#define MAX_CHAR_FOR_FILENAME 13
#define PERIODIC_LOG_TIME_MS  5000

void LogEvent(eEventType eventType);

void LogAPI(stMessage apiMessage,eJsonCommand command, eJsonStatus response,eControlModes mode);

void GetFileName(char* fileName,eFileType event);

void PeriodicLogging(void);

void SdLogTimer_ms(void);
#endif