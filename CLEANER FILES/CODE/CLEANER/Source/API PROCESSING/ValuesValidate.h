#ifndef __API_VARIABLES_H_
#define __API_VARIABLES_H_

#include <stdint.h>
#include <stdbool.h> 

static const uint32_t accelTime_Min              = 0;
static const uint32_t accelTime_Max              = 4294967295;
static const uint32_t decelTime_Min              = 0;
static const uint32_t decelTime_Max              = 4294967295;
static const uint16_t decelPulseCountDiff_Min    = 0;
static const uint16_t decelPulseCountDiff_Max    = 65535;
static const uint8_t PWMvalue_Min                = 0;
static const uint8_t PWMvalue_Max                = 100;
static const uint32_t RTCvalue_Min               = 0;
static const uint32_t RTCvalue_Max               = 4294967295;
static const uint8_t lowBatSoC_Min               = 0;
static const uint8_t lowBatSoC_Max               = 100;
static const uint16_t pulseCount_Min             = 0;
static const uint16_t pulseCount_Max             = 65535;
static const float motorCurrentLimit_Min         = 0;
static const float motorCurrentLimit_Max         = 10;
static const uint32_t IloadMaxConfigTime_Min     = 0;
static const uint32_t IloadMaxConfigTime_Max     = 2147483647;

static const uint8_t manualCommand_Min          = 1;
static const uint8_t manualCommand_Max          = 7;

static const uint32_t heartBeatIntervalMs_Min   = 100;
static const uint32_t heartBeatIntervalMs_Max   = 100000;

static const uint32_t NoOfHeartBeatMsg_Min      = 1;
static const uint32_t NoOfHeartBeatMsg_Max      = 500;

static const uint32_t maxReconnectTime_Min      = 1;
static const uint32_t maxReconnectTime_Max      = (15)*(60)*(1000); //10 minutes

//07D change 
static const float wheelDiaLimit_Min         = 0;
static const float wheelDiaLimit_Max         = 500;
static const float pulseLimit_Min           = 1;
static const float pulseLimit_Max           = 4;
static const float cycleFrequencyLimit_Min   = 1;
static const float cycleFrequencyLimit_Max   = 19;
static const uint8_t intervalLimit_Min   = 1;
static const uint8_t intervalLimit_Max   = 60;
static const uint8_t parameterLimit_Min   = 0;
static const uint8_t parameterLimit_Max   = 7;
static const uint8_t Continue1_Min               = 0;
static const uint8_t Continue1_Max               = 5;
static const uint8_t Count1_Min               = 1;
static const uint8_t Count1_Max               = 10;
static const uint8_t Return1_Min               = 1;
static const uint8_t Return1_Max               = 10;

static const uint32_t Brushtime_Min                = 0;
static const uint32_t Brushtime_Max                = 1000;

bool IsPWMvalueValid(uint8_t value);
bool IsRTCvalueValid(uint32_t value);
bool IsMotorCurrentLimitValid(float value);
bool IsAccelTimeValid(uint32_t accelTime);
bool IsDecelTimeValid(uint32_t decelTime);
bool IsDecelPulseCountDiffValid(uint16_t count);
bool IsLowBatSoCValid(uint16_t soc);
bool IsPulseCountValid(uint16_t pulse);
bool IsManualCommandValid(uint8_t value);
bool IsIloadMaxConfigTimeValid(long int configTime);
bool IsHeartBeatIntervalValid(uint32_t value);
bool IsNoOfHeartBeatMsgValid(uint32_t value);
bool IsMaxReconnectTimeValid(uint32_t value);


bool IsAutoModeStateValid(bool value);
bool IsManualModeStateValid(bool value);
bool IsManualControlDirectionValid(bool value);
bool IsBrushMotorStateValid(bool value);
bool IsBrushMotorDirectionValid(bool value);
bool IsEdgeSensorStateValid(bool value);
bool IsHeartBeatStateValid(bool value);

bool IsStateValid(bool value);

bool CheckHexStrValidity(const char * hexStr, uint16_t maxLength);

bool Validate_f (float value, float min,float max);
bool Validate_i (uint32_t value,uint32_t min,uint32_t max);
bool Validate_d (double value, double min, double max);

bool IswheelDiaLimitValid(float value);
bool IspulseLimitValid(float value);
bool IscycleFrequencyLimitValid(float value);
bool IsintervalLimitValid(uint16_t value);
bool IsparameterLimitValid(uint16_t value);
bool IsContinue1Valid(uint16_t value);
bool IsCount1Valid(uint16_t value);
bool IsReturn1Valid(uint16_t value);
bool IsBrushtimeValid(uint32_t value);
#endif