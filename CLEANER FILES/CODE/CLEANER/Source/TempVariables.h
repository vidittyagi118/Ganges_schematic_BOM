#ifndef __TEMP_VARIABLES_H_
#define __TEMP_VARIABLES_H_
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h> 
#include "RobotControl.h"

//typedef struct{
//  
//  uint32_t accelTime;
//  uint32_t decelTime;
//  uint8_t  runPWM;
//  uint8_t  approachPWM;
//  uint16_t decelCountDiff;
//}stPWMParams;

//typedef struct{
//  uint8_t  runPWM;
//  uint32_t accelTime;
//  uint32_t decelTime;
//}stBrushParams;

//typedef struct{
//  uint16_t forCount;
//  uint16_t revCount;
//}stCountValue;

//stPWMParams* GetSetPWMParams(void);
//stBrushParams* GetSetBrushParams(void);
//stCountValue*   GetSetCountValues(void);

//uint16_t        GetDeviceID();
//uint8_t         GetSoC();
bool            GetChargeState(void);
//uint16_t        GetErrorState();
//char*           GetDeviceInfo();
//bool            GetState();

//uint8_t* GetLowBatSoC(void);
//bool* GetMode(void);
//bool* GetEdgeSensorState(void);
//stCountValue*   GetCountValues(void);
//stPWMParams* GetPWMParams(void);
//stBrushParams* GetBrushParams(void);
//stRobotPwmParam* GetSetAutoPwmParameters(void);
#endif