#include "eepromstore.h"
#include "NumStrConversion.h"
#include "Serial_Debug.h"
#include "APIProcessing.h"
#include "TempVariables.h"
#include "RobotOperation.h"
#include "CycleMode.h"
#include "BrushControl.h"
#include "AutoSchedule.h"
#include "MotCurrentFaultCommon.h"
#include "EdgeSenseCommon.h"
#include "RotateSenseCommon.h"
#include "BrushMotorControl_hal.h"
#include "FaultProcess.h"
#include "DefaultValues.h"
#include <string.h>

#define LENGTH_OF_EACH_STORE            1                         /* One 32 bit variables    */
#define LENGTH_OF_EACH_STRING_STORE     1                         /* One 32 bit variables    */
#define EEPROM_VIRTUAL_START_ADDRESS    0x0000
#define EEPROM_DEV_INFO_START_ADDRESS   EEPROM_VIRTUAL_START_ADDRESS 
#define EEPROM_DEV_INFO_END_ADDRESS     (EEPROM_DEV_INFO_START_ADDRESS + ((JSON_NO_OF_SET_DEVICE_INFO-1) * MAX_DEV_INFO_FIELD_LEN * LENGTH_OF_EACH_STRING_STORE)-1)
#define EEPROM_VALIDITY_START_ADDRESS   EEPROM_DEV_INFO_END_ADDRESS +1
#define EEPROM_VALIDITY_END_ADDRESS     (EEPROM_VALIDITY_START_ADDRESS + LENGTH_OF_EACH_STORE -1)
#define EEPROM_VALIDITY_DATA            0x0357                    /*Ensemble Defined data. It can be anything apart from 0xffff */
#define STORE_MIN_START_ADDRESS         (EEPROM_VALIDITY_END_ADDRESS+1)

int Autoschcountval;
float circumference;
//int brushMotorStopDeadTime_ms;
extern int posdistance_int,negdistance_int;
extern int Accel_speed,Decel_speed;
extern int Accel_remaining_distance,Decel_remaining_distance;
extern int forward_remaining_distance,backward_remaining_distance;
extern int Time_seconds,Time_seconds_forward,Time_seconds_backward;
extern int CYCLEROTATIONTIME;
extern int ComDistance_int;

extern int Cumulative_distance;

enum
{
  STORE_MODE_STARTADD                    = STORE_MIN_START_ADDRESS,
  STORE_PWM_MOTOR_MANUAL_STARTADD        = STORE_MODE_STARTADD + ((JSON_NO_OF_MODE_STATE_DATA-1) * LENGTH_OF_EACH_STORE),
  STORE_PWM_BRUSH_MANUAL_STARTADD     = STORE_PWM_MOTOR_MANUAL_STARTADD + ((JSON_NO_OF_PWM_DATA_MOTOR-1) * LENGTH_OF_EACH_STORE),
 // STORE_PWM_BRUSH_MANUAL_STARTADD        = STORE_BRUSH_ENABLED_STATE_STARTADD + ((JSON_NO_OF_MOTOR_MODE_STATE_DATA-1) * LENGTH_OF_EACH_STORE),  
  STORE_BRUSH_MOTOR_POLARITY_STARTADD    = STORE_PWM_BRUSH_MANUAL_STARTADD + ((JSON_NO_OF_PWM_DATA_BRUSH-1) * LENGTH_OF_EACH_STORE),
  STORE_PWM_MOTOR_AUTO_STARTADD          = STORE_BRUSH_MOTOR_POLARITY_STARTADD + ((JSON_NO_OF_MODE_STATE_DATA-1) * LENGTH_OF_EACH_STORE),
  STORE_PWM_BRUSH_AUTO_STARTADD          = STORE_PWM_MOTOR_AUTO_STARTADD + ((JSON_NO_OF_PWM_DATA_MOTOR-1) * LENGTH_OF_EACH_STORE),
  STORE_PULSE_COUNT_STARTADD             = STORE_PWM_BRUSH_AUTO_STARTADD + ((JSON_NO_OF_PWM_DATA_BRUSH-1) * LENGTH_OF_EACH_STORE),
  STORE_EDGE_SENSOR_STATE_STARTADD       = STORE_PULSE_COUNT_STARTADD + ((JSON_NO_OF_PULSE_COUNT_DATA-1) * LENGTH_OF_EACH_STORE),
//  STORE_EDGE_SENSOR_STATE_STARTADD       = STORE_AUTO_SCHEDULE_STARTADD + ((JSON_NO_AUTOSCH-1) * LENGTH_OF_EACH_STORE), 
  STORE_CURRENT_LIMIT_STARTADD           = STORE_EDGE_SENSOR_STATE_STARTADD + ((JSON_NO_OF_SENSOR_STATE_DATA-1) * LENGTH_OF_EACH_STORE),
  STORE_LOW_BAT_SOC_STARTADD             = STORE_CURRENT_LIMIT_STARTADD + ((JSON_NO_OF_CURRENT_LIMIT-1) * LENGTH_OF_EACH_STORE),
  STORE_MOT_FAULT_STARTADD               = STORE_LOW_BAT_SOC_STARTADD + ((JSON_NO_OF_SOC_DATA-1) * LENGTH_OF_EACH_STORE), 
  STORE_HEARTBEAT_CONFIG_STARTADD        = STORE_MOT_FAULT_STARTADD + ((JSON_NO_OF_MOTOR_FAULT_CONDITIONS-1) * LENGTH_OF_EACH_STORE),
  STORE_LOG_INTERVAL_STARTADD            = STORE_HEARTBEAT_CONFIG_STARTADD + ((JSON_NO_OF_HEARTBEAT_CONFIG_PARAMETERS-1) * LENGTH_OF_EACH_STORE),  //07D change 
  STORE_WHEEL_DIA_STARTADD               = STORE_LOG_INTERVAL_STARTADD + ((JSON_NO_OF_INTERVAL_DATA-1) * LENGTH_OF_EACH_STORE),
  STORE_CYCLE_FREQUENCY_STARTADD         = STORE_WHEEL_DIA_STARTADD + ((JSON_NO_OF_WHEELDIA_DATA-1) * LENGTH_OF_EACH_STORE),
  STORE_CONTINUE_STARTADD                = STORE_CYCLE_FREQUENCY_STARTADD + ((JSON_NO_OF_CYCLEFREQUENCY_DATA-1) * LENGTH_OF_EACH_STORE),
  STORE_BRUSH_ENABLED_STATE_STARTADD     = STORE_CONTINUE_STARTADD + ((JSON_NO_OF_CONTINUE-1) * LENGTH_OF_EACH_STORE),
  STORE_RETURN_STARTADD                  =  STORE_BRUSH_ENABLED_STATE_STARTADD + ((JSON_NO_OF_MOTOR_MODE_STATE_DATA-1) * LENGTH_OF_EACH_STORE),
  STORE_AUTO_SCHEDULE_STARTADD           = STORE_RETURN_STARTADD + ((JSON_NO_OF_RETURN-1) * LENGTH_OF_EACH_STORE), 
  STORE_COMDISTANCE_STARTADD             = STORE_AUTO_SCHEDULE_STARTADD + ((JSON_NO_AUTOSCH-1) * LENGTH_OF_EACH_STORE),
  STORE_ROWDAY_STARTADD                =STORE_COMDISTANCE_STARTADD + ((JSON_NO_OF_COMDISTANCE-1) * LENGTH_OF_EACH_STORE), 
  STORE_ROW_LENGTH_STARTADD                 = STORE_ROWDAY_STARTADD + ((JSON_NO_OF_ROWS-1) * LENGTH_OF_EACH_STORE),
  STORE_ROW_LENGTH2_STARTADD                = STORE_ROW_LENGTH_STARTADD  + ((MAX_NO_OF_TRACKS+MAX_NO_OF_TRACKS) * LENGTH_OF_EACH_STORE),
  STORE_ROW_LENGTH3_STARTADD                = STORE_ROW_LENGTH2_STARTADD  + ((MAX_NO_OF_TRACKS+MAX_NO_OF_TRACKS) * LENGTH_OF_EACH_STORE),
  STORE_ROW_LENGTH4_STARTADD                = STORE_ROW_LENGTH3_STARTADD  + ((MAX_NO_OF_TRACKS+MAX_NO_OF_TRACKS) * LENGTH_OF_EACH_STORE),
  STORE_ROW_LENGTH5_STARTADD                = STORE_ROW_LENGTH4_STARTADD  + ((MAX_NO_OF_TRACKS+MAX_NO_OF_TRACKS) * LENGTH_OF_EACH_STORE),
  STORE_ROW_LENGTH6_STARTADD                = STORE_ROW_LENGTH5_STARTADD  + ((MAX_NO_OF_TRACKS+MAX_NO_OF_TRACKS) * LENGTH_OF_EACH_STORE),
  STORE_ROW_LENGTH7_STARTADD                = STORE_ROW_LENGTH6_STARTADD  + ((MAX_NO_OF_TRACKS+MAX_NO_OF_TRACKS) * LENGTH_OF_EACH_STORE),
  STORE_MAX_START_ADDRESS                = STORE_ROW_LENGTH7_STARTADD  + ((MAX_NO_OF_TRACKS+MAX_NO_OF_TRACKS) * LENGTH_OF_EACH_STORE),
  
  STORE_MAX_ADDRESS                      = STORE_MAX_START_ADDRESS - 1                                   
}eEepromEndAddress;

eEepromStatus InitialiseAlltoDefault(stJsonCommand *jsonCmdParams, uint16_t commandParamsTotalCount);

eEepromStatus EepromDataNil(eEpromAccessType eepromOp)
{
  eEepromStatus status = EEPROM_OK; 
  return status;
}

uint16_t GetMaxVirtualAddress (void)
{
  return STORE_MAX_ADDRESS;  
}

eEepromStatus EepromSetMode(eEpromAccessType eepromOp)
{
  eEepromStatus status = EEPROM_OK;
  uint16_t eepromAddress = STORE_MODE_STARTADD;
  eMode setMode = GetOperationMode();
  float eepromValue[JSON_NO_OF_MODE_STATE_DATA-1];
  uint16_t eepromAddIndex = 0;
  if(eepromOp == EEPROM_WRITE)
  {
    eepromValue[eepromAddIndex++] = setMode;
    status = WriteEEPROM (eepromAddress, eepromValue, eepromAddIndex);
  }
  else
  {
    status = ReadEEPROM (eepromAddress, eepromValue, JSON_NO_OF_MODE_STATE_DATA-1);
    if(status == EEPROM_OK)
    {
      eMode getMode = (eMode)eepromValue[eepromAddIndex++];
      SetInitialOperationMode(getMode);
    }
  }
  return status; 
}
eEepromStatus EepromSetMotorPwmManualMode(eEpromAccessType eepromOp)
{
  eEepromStatus status = EEPROM_OK;
  uint16_t eepromAddress = STORE_PWM_MOTOR_MANUAL_STARTADD;
  stRobotPwmParam *setPWMParams = GetSetManualPwmParameters();
  stRobotPwmParam *setPWMCycleParams = GetSetCycleManualPwmParameters();
  float eepromValue[JSON_NO_OF_PWM_DATA_MOTOR-1];
  uint16_t eepromAddIndex = 0;
  if(eepromOp == EEPROM_WRITE)
  {
    eepromValue[eepromAddIndex++] = setPWMParams->accelTime;
    eepromValue[eepromAddIndex++] = setPWMParams->decelTime;
    eepromValue[eepromAddIndex++] = setPWMParams->Accel;
    eepromValue[eepromAddIndex++] = setPWMParams->accelapproachPwm1;
    eepromValue[eepromAddIndex++] = setPWMParams->accelapproachPwm2;
    eepromValue[eepromAddIndex++] = setPWMParams->steadyPwm1;
    eepromValue[eepromAddIndex++] = setPWMParams->steadyPwm2;
    eepromValue[eepromAddIndex++] = setPWMParams->approachPwm1;
    eepromValue[eepromAddIndex++] = setPWMParams->approachPwm2;
    eepromValue[eepromAddIndex++] = setPWMParams->Decel;
    eepromValue[eepromAddIndex++] = setPWMParams->acelStartCountDiff;
    eepromValue[eepromAddIndex++] = setPWMParams->decelStartCountDiff;


   
    status = WriteEEPROM (eepromAddress, eepromValue, eepromAddIndex);
  }
  else
  {
    status = ReadEEPROM (eepromAddress, eepromValue, JSON_NO_OF_PWM_DATA_MOTOR-1);
    if(status == EEPROM_OK)
    {
      setPWMCycleParams->accelTime               =  (uint32_t)eepromValue[eepromAddIndex];
      setPWMParams->accelTime                    =  (uint32_t)eepromValue[eepromAddIndex++];
      setPWMCycleParams->decelTime               =  (uint32_t)eepromValue[eepromAddIndex];
      setPWMParams->decelTime                    =  (uint32_t)eepromValue[eepromAddIndex++];
      
      setPWMParams->Accel           =  (uint32_t)eepromValue[eepromAddIndex++];
 //     Accel_speed           =  (uint16_t)eepromValue[eepromAddIndex++];
      
      setPWMCycleParams->accelapproachPwm1       =  (uint32_t)eepromValue[eepromAddIndex];
      setPWMParams->accelapproachPwm1            =  (uint8_t)eepromValue[eepromAddIndex++];
      setPWMCycleParams->accelapproachPwm2       =  (uint32_t)eepromValue[eepromAddIndex];
      setPWMParams->accelapproachPwm2            =  (uint8_t)eepromValue[eepromAddIndex++]; 
      setPWMCycleParams->steadyPwm1              =  (uint32_t)eepromValue[eepromAddIndex];
      setPWMParams->steadyPwm1                   =  (uint8_t)eepromValue[eepromAddIndex++];
      setPWMCycleParams->steadyPwm2              =  (uint32_t)eepromValue[eepromAddIndex];
      setPWMParams->steadyPwm2                   =  (uint8_t)eepromValue[eepromAddIndex++];
      setPWMCycleParams->approachPwm1            =  (uint32_t)eepromValue[eepromAddIndex];
      setPWMParams->approachPwm1                 =  (uint8_t)eepromValue[eepromAddIndex++];
      setPWMCycleParams->approachPwm2            =  (uint32_t)eepromValue[eepromAddIndex];
      setPWMParams->approachPwm2                 =  (uint8_t)eepromValue[eepromAddIndex++];
      
     setPWMParams->Decel                           =  (uint32_t)eepromValue[eepromAddIndex++];
//      Decel_speed                             =  (uint16_t)eepromValue[eepromAddIndex++];
      
      setPWMCycleParams->acelStartCountDiff      =  (uint32_t)eepromValue[eepromAddIndex];
      setPWMParams->acelStartCountDiff           =  (uint16_t)eepromValue[eepromAddIndex++];
      setPWMCycleParams->decelStartCountDiff     =  (uint32_t)eepromValue[eepromAddIndex];
      setPWMParams->decelStartCountDiff          =  (uint16_t)eepromValue[eepromAddIndex++];
      

    }
//    Serial_Debug("\nRead ");
//    Serial_Debug_Num(setPWMParams->accelTime );Serial_Debug(" ");
//    Serial_Debug_Num(setPWMParams->approachPWM );Serial_Debug(" ");
//    Serial_Debug_Num(setPWMParams->decelCountDiff );Serial_Debug(" ");
//    Serial_Debug_Num(setPWMParams->decelTime );Serial_Debug(" ");
//    Serial_Debug_Num(setPWMParams->runPWM );
  }
  return status;  
}

eEepromStatus EepromSetBrushEnabledState(eEpromAccessType eepromOp)
{
  eEepromStatus status = EEPROM_OK;
  uint16_t eepromAddress = STORE_BRUSH_ENABLED_STATE_STARTADD;
  bool *setBrushEnabledState = GetSetBrushEnabledState();
  bool *setRobotEnabledState = GetSetLinearEnabledState(); 
  float eepromValue[JSON_NO_OF_MOTOR_MODE_STATE_DATA-1];
  uint16_t eepromAddIndex = 0;
  if(eepromOp == EEPROM_WRITE)
  {
    eepromValue[eepromAddIndex++] = *setBrushEnabledState;
    eepromValue[eepromAddIndex++] = *setRobotEnabledState;
    eepromValue[eepromAddIndex++] =  Cumulative_distance;
    status = WriteEEPROM (eepromAddress, eepromValue, eepromAddIndex);
  }
  else
  {
    status = ReadEEPROM (eepromAddress, eepromValue, JSON_NO_OF_MOTOR_MODE_STATE_DATA-1);
    if(status == EEPROM_OK)
    {
      *setBrushEnabledState             =  (bool)eepromValue[eepromAddIndex++];
      *setRobotEnabledState             =  (bool)eepromValue[eepromAddIndex++];
       Cumulative_distance              =  (uint16_t)eepromValue[eepromAddIndex++];
    }
  }
  return status; 
}

eEepromStatus EepromSetBrushPwmManualMode(eEpromAccessType eepromOp)
{
  eEepromStatus status = EEPROM_OK;
  uint16_t eepromAddress = STORE_PWM_BRUSH_MANUAL_STARTADD;
  stBrushPwmParam *setPWMParams = GetSetBrushNormalPwmParameters();
  float eepromValue[JSON_NO_OF_PWM_DATA_BRUSH-1];
  uint16_t eepromAddIndex = 0;
  if(eepromOp == EEPROM_WRITE)
  {
    eepromValue[eepromAddIndex++] = setPWMParams->accelTime;
    eepromValue[eepromAddIndex++] = setPWMParams->decelTime;
    eepromValue[eepromAddIndex++] = setPWMParams->steadyPwm;
    eepromValue[eepromAddIndex++] = setPWMParams->brushTime;
//    eepromValue[eepromAddIndex++] = setPWMParams->timePwm;
//    eepromValue[eepromAddIndex++] = CYCLEROTATIONTIME;
    status = WriteEEPROM (eepromAddress, eepromValue, eepromAddIndex);
  }
  else
  {
    status = ReadEEPROM (eepromAddress, eepromValue, JSON_NO_OF_PWM_DATA_BRUSH-1);
    if(status == EEPROM_OK)
    {
      setPWMParams->accelTime             =  (uint32_t)eepromValue[eepromAddIndex++];
      setPWMParams->decelTime             =  (uint32_t)eepromValue[eepromAddIndex++];
      setPWMParams->steadyPwm             =  (uint32_t)eepromValue[eepromAddIndex++];
      setPWMParams->brushTime             =  (uint32_t)eepromValue[eepromAddIndex++];
//      setPWMParams->timePwm               =  (uint32_t)eepromValue[eepromAddIndex++];
//      CYCLEROTATIONTIME                   =  (uint32_t)eepromValue[eepromAddIndex++];
    }
//    Serial_Debug("\nRead ");
//    Serial_Debug_Num(setPWMParams->accelTime );Serial_Debug(" ");
//    Serial_Debug_Num(setPWMParams->approachPWM );Serial_Debug(" ");
//    Serial_Debug_Num(setPWMParams->decelCountDiff );Serial_Debug(" ");
//    Serial_Debug_Num(setPWMParams->decelTime );Serial_Debug(" ");
//    Serial_Debug_Num(setPWMParams->runPWM );
  }
  return status;  
}

eEepromStatus EepromSetBrushMotorPolarity(eEpromAccessType eepromOp)
{
  eEepromStatus status = EEPROM_OK;
  uint16_t eepromAddress = STORE_BRUSH_MOTOR_POLARITY_STARTADD;
  stBrushMotPolarity *motPolarity = GetSetBrushMotPolarity();
  float eepromValue[JSON_NO_OF_MODE_STATE_DATA-1];
  uint16_t eepromAddIndex = 0;
  if(eepromOp == EEPROM_WRITE)
  {
    eepromValue[eepromAddIndex++] = motPolarity->mot1Polarity;
    status = WriteEEPROM (eepromAddress, eepromValue, eepromAddIndex);
  }
  else
  {
    status = ReadEEPROM (eepromAddress, eepromValue, JSON_NO_OF_MODE_STATE_DATA-1);
    if(status == EEPROM_OK)
    {
      motPolarity->mot1Polarity  =  (bool)eepromValue[eepromAddIndex++];
    }
  }
  return status; 
}

eEepromStatus EepromSetMotorPwmAutoMode(eEpromAccessType eepromOp)
{

  eEepromStatus status = EEPROM_OK;
  uint16_t eepromAddress = STORE_PWM_MOTOR_AUTO_STARTADD;
  stRobotPwmParam *setPWMParams = GetSetCycleAutoPwmParameters();
  float eepromValue[JSON_NO_OF_PWM_DATA_MOTOR-1];
  uint16_t eepromAddIndex = 0;
  if(eepromOp == EEPROM_WRITE)
  {
    eepromValue[eepromAddIndex++] = setPWMParams->accelTime;
    eepromValue[eepromAddIndex++] = setPWMParams->decelTime;
    eepromValue[eepromAddIndex++] = setPWMParams->Accel;
    eepromValue[eepromAddIndex++] = setPWMParams->accelapproachPwm1;
    eepromValue[eepromAddIndex++] = setPWMParams->accelapproachPwm2;
    eepromValue[eepromAddIndex++] = setPWMParams->steadyPwm1;
    eepromValue[eepromAddIndex++] = setPWMParams->steadyPwm2;
    eepromValue[eepromAddIndex++] = setPWMParams->approachPwm1;
    eepromValue[eepromAddIndex++] = setPWMParams->approachPwm2;
    eepromValue[eepromAddIndex++] = setPWMParams->Decel;
    eepromValue[eepromAddIndex++] = setPWMParams->acelStartCountDiff;
    eepromValue[eepromAddIndex++] = setPWMParams->decelStartCountDiff;

    
    status = WriteEEPROM (eepromAddress, eepromValue, eepromAddIndex);
  }
  else
  {
    status = ReadEEPROM (eepromAddress, eepromValue, JSON_NO_OF_PWM_DATA_MOTOR-1);
    if(status == EEPROM_OK)
    {
     setPWMParams->accelTime            =  (uint32_t)eepromValue[eepromAddIndex++];
      setPWMParams->decelTime            =  (uint32_t)eepromValue[eepromAddIndex++];
      setPWMParams->Accel                        =  (uint32_t)eepromValue[eepromAddIndex++];
      setPWMParams->accelapproachPwm1    =  (uint8_t)eepromValue[eepromAddIndex++];
      setPWMParams->accelapproachPwm2    =  (uint8_t)eepromValue[eepromAddIndex++];
      setPWMParams->steadyPwm1           =  (uint8_t)eepromValue[eepromAddIndex++];
      setPWMParams->steadyPwm2           =  (uint8_t)eepromValue[eepromAddIndex++];
      setPWMParams->approachPwm1         =  (uint8_t)eepromValue[eepromAddIndex++];
      setPWMParams->approachPwm2         =  (uint8_t)eepromValue[eepromAddIndex++];
      setPWMParams->Decel                        =  (uint32_t)eepromValue[eepromAddIndex++];    
      setPWMParams->acelStartCountDiff   =  (uint16_t)eepromValue[eepromAddIndex++]; 
      setPWMParams->decelStartCountDiff   =  (uint16_t)eepromValue[eepromAddIndex++];
    }
//    Serial_Debug("\nRead ");
//    Serial_Debug_Num(setPWMParams->accelTime );Serial_Debug(" ");
//    Serial_Debug_Num(setPWMParams->approachPWM );Serial_Debug(" ");
//    Serial_Debug_Num(setPWMParams->decelCountDiff );Serial_Debug(" ");
//    Serial_Debug_Num(setPWMParams->decelTime );Serial_Debug(" ");
//    Serial_Debug_Num(setPWMParams->runPWM );
  }
  return status;  
}

eEepromStatus EepromSetBrushPwmAutoMode(eEpromAccessType eepromOp)
{
  eEepromStatus status = EEPROM_OK;
  uint16_t eepromAddress = STORE_PWM_BRUSH_AUTO_STARTADD;
  stBrushPwmParam *setPWMParams = GetSetBrushAutoPwmParameters();
  float eepromValue[JSON_NO_OF_PWM_DATA_BRUSH-1];
  uint16_t eepromAddIndex = 0;
  if(eepromOp == EEPROM_WRITE)
  {
    eepromValue[eepromAddIndex++] = setPWMParams->accelTime;
    eepromValue[eepromAddIndex++] = setPWMParams->decelTime;
    eepromValue[eepromAddIndex++] = setPWMParams->steadyPwm;
    eepromValue[eepromAddIndex++] = setPWMParams->brushTime;
//    eepromValue[eepromAddIndex++] = setPWMParams->timePwm;
//    eepromValue[eepromAddIndex++] = CYCLEROTATIONTIME;
    status = WriteEEPROM (eepromAddress, eepromValue, eepromAddIndex);
  }
  else
  {
    status = ReadEEPROM (eepromAddress, eepromValue, JSON_NO_OF_PWM_DATA_BRUSH-1);
    if(status == EEPROM_OK)
    {
      setPWMParams->accelTime             =  (uint32_t)eepromValue[eepromAddIndex++];
      setPWMParams->decelTime             =  (uint32_t)eepromValue[eepromAddIndex++];
      setPWMParams->steadyPwm             =  (uint32_t)eepromValue[eepromAddIndex++];
      setPWMParams->brushTime             =  (uint32_t)eepromValue[eepromAddIndex++];
//      setPWMParams->timePwm               =  (uint32_t)eepromValue[eepromAddIndex++];
//      CYCLEROTATIONTIME                   =  (uint32_t)eepromValue[eepromAddIndex];
    }
//    Serial_Debug("\nRead ");
//    Serial_Debug_Num(setPWMParams->accelTime );Serial_Debug(" ");
//    Serial_Debug_Num(setPWMParams->approachPWM );Serial_Debug(" ");
//    Serial_Debug_Num(setPWMParams->decelCountDiff );Serial_Debug(" ");
//    Serial_Debug_Num(setPWMParams->decelTime );Serial_Debug(" ");
//    Serial_Debug_Num(setPWMParams->runPWM );
  }
  return status;  
}
eEepromStatus EepromSetPulseCount(eEpromAccessType eepromOp)
{
  eEepromStatus status = EEPROM_OK;
  uint16_t eepromAddress = STORE_PULSE_COUNT_STARTADD;
  stCountRange *setCountValues = GetSetCountRange();
  float eepromValue[JSON_NO_OF_PULSE_COUNT_DATA-1];
  uint16_t eepromAddIndex = 0;
  if(eepromOp == EEPROM_WRITE)
  {
    eepromValue[eepromAddIndex++] = posdistance_int;
    eepromValue[eepromAddIndex++] = negdistance_int; 
    eepromValue[eepromAddIndex++] = setCountValues->maxPosCountForPVsetup;
    eepromValue[eepromAddIndex++] = setCountValues->maxNegCountForPVsetup;    
    eepromValue[eepromAddIndex++] = Time_seconds_forward;
    eepromValue[eepromAddIndex++] = Time_seconds_backward;
 
    status = WriteEEPROM (eepromAddress, eepromValue, eepromAddIndex);
  }
  else
  {
    status = ReadEEPROM (eepromAddress, eepromValue, JSON_NO_OF_PULSE_COUNT_DATA-1);
    if(status == EEPROM_OK)
    {
      
      posdistance_int            =  (uint16_t)eepromValue[eepromAddIndex++];
      negdistance_int            =  (uint16_t)eepromValue[eepromAddIndex++];
      setCountValues->maxPosCountForPVsetup             =  (uint16_t)eepromValue[eepromAddIndex++];
      setCountValues->maxNegCountForPVsetup             =  (uint16_t)eepromValue[eepromAddIndex++];
      Time_seconds_forward            =  (uint16_t)eepromValue[eepromAddIndex++];
      Time_seconds_backward           =  (uint16_t)eepromValue[eepromAddIndex++];
    }
  }
  return status;  
}

//
//eEepromStatus EepromSetAutoScheduleTime(eEpromAccessType eepromOp)
//{
//  eEepromStatus status = EEPROM_OK;
//  uint16_t eepromAddress = STORE_AUTO_SCHEDULE_STARTADD;
//  stScheduleTime *setScheduleValues = GetSetScheduleTime();
//  float eepromValue[JSON_NO_OF_AUTO_SCHED_DATA-1];
//  uint16_t eepromAddIndex = 0;
//  if(eepromOp == EEPROM_WRITE)
//  {
//    eepromValue[eepromAddIndex++] = setScheduleValues->hour;
//    eepromValue[eepromAddIndex++] = setScheduleValues->minute;
//    status = WriteEEPROM (eepromAddress, eepromValue, eepromAddIndex);
//  }
//  else
//  {
//    status = ReadEEPROM (eepromAddress, eepromValue, JSON_NO_OF_AUTO_SCHED_DATA-1);
//    if(status == EEPROM_OK)
//    {
//      setScheduleValues->hour             =  (uint8_t)eepromValue[eepromAddIndex++];
//      setScheduleValues->minute           =  (uint8_t)eepromValue[eepromAddIndex++];
//    }
//  }
//  return status; 
//}

//07D change
eEepromStatus EepromSetAutoScheduleTime(eEpromAccessType eepromOp)
{
  eEepromStatus status = EEPROM_OK;
  uint16_t eepromAddress = STORE_AUTO_SCHEDULE_STARTADD;
  stScheduleTime *setScheduleValues = GetSetScheduleTime();
  float eepromValue[38];
  uint16_t eepromAddIndex = 0;
  
  if(eepromOp == EEPROM_WRITE)
  {
   for(int i=0;i<19;i++)
   {
      eepromValue[eepromAddIndex++] = setScheduleValues->HOUR[i];eepromValue[eepromAddIndex++] = setScheduleValues->MINUTE[i];
   }  
//     eepromValue[eepromAddIndex++] = setScheduleValues->HOUR[0];eepromValue[eepromAddIndex++] = setScheduleValues->MINUTE[0];
//     eepromValue[eepromAddIndex++] = setScheduleValues->HOUR[1];eepromValue[eepromAddIndex++] = setScheduleValues->MINUTE[1];
//     eepromValue[eepromAddIndex++] = setScheduleValues->HOUR[2];eepromValue[eepromAddIndex++] = setScheduleValues->MINUTE[2];
//     eepromValue[eepromAddIndex++] = setScheduleValues->HOUR[3];eepromValue[eepromAddIndex++] = setScheduleValues->MINUTE[3];
//     eepromValue[eepromAddIndex++] = setScheduleValues->HOUR[4];eepromValue[eepromAddIndex++] = setScheduleValues->MINUTE[4];
//     eepromValue[eepromAddIndex++] = setScheduleValues->HOUR[5];eepromValue[eepromAddIndex++] = setScheduleValues->MINUTE[5];
//     eepromValue[eepromAddIndex++] = setScheduleValues->HOUR[6];eepromValue[eepromAddIndex++] = setScheduleValues->MINUTE[6];
//     eepromValue[eepromAddIndex++] = setScheduleValues->HOUR[7];eepromValue[eepromAddIndex++] = setScheduleValues->MINUTE[7];
//     eepromValue[eepromAddIndex++] = setScheduleValues->HOUR[8];eepromValue[eepromAddIndex++] = setScheduleValues->MINUTE[8];
//     eepromValue[eepromAddIndex++] = setScheduleValues->HOUR[9];eepromValue[eepromAddIndex++] = setScheduleValues->MINUTE[9];
//     eepromValue[eepromAddIndex++] = setScheduleValues->HOUR[10];eepromValue[eepromAddIndex++] = setScheduleValues->MINUTE[10];
//     eepromValue[eepromAddIndex++] = setScheduleValues->HOUR[11];eepromValue[eepromAddIndex++] = setScheduleValues->MINUTE[11];
//     eepromValue[eepromAddIndex++] = setScheduleValues->HOUR[12];eepromValue[eepromAddIndex++] = setScheduleValues->MINUTE[12];
//     eepromValue[eepromAddIndex++] = setScheduleValues->HOUR[13];eepromValue[eepromAddIndex++] = setScheduleValues->MINUTE[13];
//     eepromValue[eepromAddIndex++] = setScheduleValues->HOUR[14];eepromValue[eepromAddIndex++] = setScheduleValues->MINUTE[14];
//     eepromValue[eepromAddIndex++] = setScheduleValues->HOUR[15];eepromValue[eepromAddIndex++] = setScheduleValues->MINUTE[15];
//     eepromValue[eepromAddIndex++] = setScheduleValues->HOUR[16];eepromValue[eepromAddIndex++] = setScheduleValues->MINUTE[16];
//     eepromValue[eepromAddIndex++] = setScheduleValues->HOUR[17];eepromValue[eepromAddIndex++] = setScheduleValues->MINUTE[17];
//     eepromValue[eepromAddIndex++] = setScheduleValues->HOUR[18];eepromValue[eepromAddIndex++] = setScheduleValues->MINUTE[18];   
   
     status = WriteEEPROM (eepromAddress, eepromValue, eepromAddIndex);   
  }
  else
  {
    status = ReadEEPROM (eepromAddress, eepromValue,38);
    if(status == EEPROM_OK)
    {
      for(int i=0;i<19;i++)
      { 
        setScheduleValues->HOUR[i]=(uint8_t)eepromValue[eepromAddIndex++]; setScheduleValues->MINUTE[i]=(uint8_t)eepromValue[eepromAddIndex++];
      }     
      
//      setScheduleValues->HOUR[0]=(uint8_t)eepromValue[eepromAddIndex++]; setScheduleValues->MINUTE[0]=(uint8_t)eepromValue[eepromAddIndex++];
//      setScheduleValues->HOUR[1]=(uint8_t)eepromValue[eepromAddIndex++]; setScheduleValues->MINUTE[1]=(uint8_t)eepromValue[eepromAddIndex++];
//      setScheduleValues->HOUR[2]=(uint8_t)eepromValue[eepromAddIndex++]; setScheduleValues->MINUTE[2]=(uint8_t)eepromValue[eepromAddIndex++];
//      setScheduleValues->HOUR[3]=(uint8_t)eepromValue[eepromAddIndex++]; setScheduleValues->MINUTE[3]=(uint8_t)eepromValue[eepromAddIndex++];
//      setScheduleValues->HOUR[4]=(uint8_t)eepromValue[eepromAddIndex++]; setScheduleValues->MINUTE[4]=(uint8_t)eepromValue[eepromAddIndex++];
//      setScheduleValues->HOUR[5]=(uint8_t)eepromValue[eepromAddIndex++]; setScheduleValues->MINUTE[5]=(uint8_t)eepromValue[eepromAddIndex++];
//      setScheduleValues->HOUR[6]=(uint8_t)eepromValue[eepromAddIndex++]; setScheduleValues->MINUTE[6]=(uint8_t)eepromValue[eepromAddIndex++];
//      setScheduleValues->HOUR[7]=(uint8_t)eepromValue[eepromAddIndex++]; setScheduleValues->MINUTE[7]=(uint8_t)eepromValue[eepromAddIndex++];
//      setScheduleValues->HOUR[8]=(uint8_t)eepromValue[eepromAddIndex++]; setScheduleValues->MINUTE[8]=(uint8_t)eepromValue[eepromAddIndex++];
//      setScheduleValues->HOUR[9]=(uint8_t)eepromValue[eepromAddIndex++]; setScheduleValues->MINUTE[9]=(uint8_t)eepromValue[eepromAddIndex++];
//      setScheduleValues->HOUR[10]=(uint8_t)eepromValue[eepromAddIndex++]; setScheduleValues->MINUTE[10]=(uint8_t)eepromValue[eepromAddIndex++];
//      setScheduleValues->HOUR[11]=(uint8_t)eepromValue[eepromAddIndex++]; setScheduleValues->MINUTE[11]=(uint8_t)eepromValue[eepromAddIndex++];
//      setScheduleValues->HOUR[12]=(uint8_t)eepromValue[eepromAddIndex++]; setScheduleValues->MINUTE[12]=(uint8_t)eepromValue[eepromAddIndex++];
//      setScheduleValues->HOUR[13]=(uint8_t)eepromValue[eepromAddIndex++]; setScheduleValues->MINUTE[13]=(uint8_t)eepromValue[eepromAddIndex++];
//      setScheduleValues->HOUR[14]=(uint8_t)eepromValue[eepromAddIndex++]; setScheduleValues->MINUTE[14]=(uint8_t)eepromValue[eepromAddIndex++];
//      setScheduleValues->HOUR[15]=(uint8_t)eepromValue[eepromAddIndex++]; setScheduleValues->MINUTE[15]=(uint8_t)eepromValue[eepromAddIndex++];
//      setScheduleValues->HOUR[16]=(uint8_t)eepromValue[eepromAddIndex++]; setScheduleValues->MINUTE[16]=(uint8_t)eepromValue[eepromAddIndex++];
//      setScheduleValues->HOUR[17]=(uint8_t)eepromValue[eepromAddIndex++]; setScheduleValues->MINUTE[17]=(uint8_t)eepromValue[eepromAddIndex++];
//      setScheduleValues->HOUR[18]=(uint8_t)eepromValue[eepromAddIndex++]; setScheduleValues->MINUTE[18]=(uint8_t)eepromValue[eepromAddIndex];    
    }
  }
  return status; 
}

eEepromStatus EepromSetEdgeSensorState(eEpromAccessType eepromOp)
{
  eEepromStatus status = EEPROM_OK;
  uint16_t eepromAddress = STORE_EDGE_SENSOR_STATE_STARTADD;
  bool *setEdgeSensorState = GetSetEdgeSensorEnabledState();  
  bool *setRotateSensorState = GetSetRotateSensorEnabledState();
  
  float eepromValue[JSON_NO_OF_SENSOR_STATE_DATA];
  uint16_t eepromAddIndex = 0;
  if(eepromOp == EEPROM_WRITE)
  {
    eepromValue[eepromAddIndex++] = *setEdgeSensorState;
    eepromValue[eepromAddIndex++] = *setRotateSensorState;
    
    
    status = WriteEEPROM (eepromAddress, eepromValue, eepromAddIndex);
  }
  else
  {
    status = ReadEEPROM (eepromAddress, eepromValue, JSON_NO_OF_MODE_STATE_DATA);
    if(status == EEPROM_OK)
    {
      *setEdgeSensorState             =  (bool)eepromValue[eepromAddIndex++];
      *setRotateSensorState             =  (bool)eepromValue[eepromAddIndex++];
    }
  }
  return status; 
}

eEepromStatus EepromSetCurrentLimits(eEpromAccessType eepromOp)
{
  eEepromStatus status = EEPROM_OK;
  uint16_t eepromAddress = STORE_CURRENT_LIMIT_STARTADD;
  stMaxMotorCurrent *setCurrentValues = GetSetMaxMotorCurrentLimits();
  float eepromValue[JSON_NO_OF_CURRENT_LIMIT-1];
  uint16_t eepromAddIndex = 0;
  if(eepromOp == EEPROM_WRITE)
  {
    eepromValue[eepromAddIndex++] = setCurrentValues->Imot1;
    eepromValue[eepromAddIndex++] = setCurrentValues->Imot2;
    eepromValue[eepromAddIndex++] = setCurrentValues->Imot3;
    status = WriteEEPROM (eepromAddress, eepromValue, eepromAddIndex);
  }
  else
  {
    status = ReadEEPROM (eepromAddress, eepromValue, JSON_NO_OF_CURRENT_LIMIT-1);
    if(status == EEPROM_OK)
    {
      setCurrentValues->Imot1             =  eepromValue[eepromAddIndex++];
      setCurrentValues->Imot2             =  eepromValue[eepromAddIndex++];
      setCurrentValues->Imot3             =  eepromValue[eepromAddIndex++];
    }
  }
  return status; 
}

eEepromStatus EepromSetLowBatSoC(eEpromAccessType eepromOp)
{
  eEepromStatus status = EEPROM_OK;
  uint16_t eepromAddress = STORE_LOW_BAT_SOC_STARTADD;
  uint8_t *setLowBatSoC = GetSetMinBattSoc();
  float eepromValue[JSON_NO_OF_SOC_DATA-1];
  uint16_t eepromAddIndex = 0;
  if(eepromOp == EEPROM_WRITE)
  {
    eepromValue[eepromAddIndex++] = *setLowBatSoC;
    status = WriteEEPROM (eepromAddress, eepromValue, eepromAddIndex);
  }
  else
  {
    status = ReadEEPROM (eepromAddress, eepromValue, JSON_NO_OF_SOC_DATA-1);
    if(status == EEPROM_OK)
    {
      *setLowBatSoC  =  (uint8_t)eepromValue[eepromAddIndex++];
    }
  }
  return status; 
}

eEepromStatus EepromZigbeeConfiguration(eEpromAccessType eepromOp)
{
  eEepromStatus status = EEPROM_OK;
  return status;
}

eEepromStatus EepromSetMotorFaultConditions(eEpromAccessType eepromOp)
{
  eEepromStatus status = EEPROM_OK;
  uint16_t eepromAddress = STORE_MOT_FAULT_STARTADD;
  stMaxIloadConfigValues *setMaxIloadConfigValues = GetSetMaxIloadConfigValues();
  float eepromValue[JSON_NO_OF_MOTOR_FAULT_CONDITIONS-1];
  uint16_t eepromAddIndex = 0;
  if(eepromOp == EEPROM_WRITE)
  {
    eepromValue[eepromAddIndex++] = setMaxIloadConfigValues->maxIloadNoiseTimeValue;
    eepromValue[eepromAddIndex++] = setMaxIloadConfigValues->maxIloadFreqTimeValue;
    eepromValue[eepromAddIndex++] = setMaxIloadConfigValues->maxIloadRepeatCountValue;
    eepromValue[eepromAddIndex++] = setMaxIloadConfigValues->motorPauseDelay;
    status = WriteEEPROM (eepromAddress, eepromValue, eepromAddIndex);
  }
  else
  {
    status = ReadEEPROM (eepromAddress, eepromValue, JSON_NO_OF_MOTOR_FAULT_CONDITIONS-1);
    if(status == EEPROM_OK)
    {
      setMaxIloadConfigValues->maxIloadNoiseTimeValue      =  (uint32_t)eepromValue[eepromAddIndex++];
      setMaxIloadConfigValues->maxIloadFreqTimeValue       =  (uint32_t)eepromValue[eepromAddIndex++];
      setMaxIloadConfigValues->maxIloadRepeatCountValue    =  (uint32_t)eepromValue[eepromAddIndex++];
      setMaxIloadConfigValues->motorPauseDelay             =  (uint32_t)eepromValue[eepromAddIndex++];
    }
  }
  return status; 
}

eEepromStatus EepromHeartbeatConfiguration(eEpromAccessType eepromOp)
{
  eEepromStatus status = EEPROM_OK;
  uint16_t eepromAddress = STORE_HEARTBEAT_CONFIG_STARTADD;
  stHeartbeatConfig* storeHeartbeatConfig;
  storeHeartbeatConfig = GetSetHeartbeatConfig();
  float eepromValue[JSON_NO_OF_HEARTBEAT_CONFIG_PARAMETERS-1];
  uint16_t eepromAddIndex = 0;
  if(eepromOp == EEPROM_WRITE)
  {
    eepromValue[eepromAddIndex++] = storeHeartbeatConfig->enable;
    eepromValue[eepromAddIndex++] = storeHeartbeatConfig->interval_ms;
    eepromValue[eepromAddIndex++] = storeHeartbeatConfig->noOfMessages;
    eepromValue[eepromAddIndex++] = storeHeartbeatConfig->maxReconnectTime;
    status = WriteEEPROM (eepromAddress, eepromValue, eepromAddIndex);
  }
  else
  {
    status = ReadEEPROM (eepromAddress, eepromValue, JSON_NO_OF_HEARTBEAT_CONFIG_PARAMETERS-1);
    if(status == EEPROM_OK)
    {
      storeHeartbeatConfig->enable = eepromValue[eepromAddIndex++];
      storeHeartbeatConfig->interval_ms = (long int)eepromValue[eepromAddIndex++];   
      storeHeartbeatConfig->noOfMessages =(long int)eepromValue[eepromAddIndex++];   
      storeHeartbeatConfig->maxReconnectTime =(uint32_t)eepromValue[eepromAddIndex]; 
    }
  }
  return status;
}

eEepromStatus ResetEepromVariables(void)
{
  eEepromStatus status = EEPROM_OK;
  stJsonCommand *jsonCmdParam;
  uint16_t commandParamsTotalCount = GetCommandProcessingParam(&jsonCmdParam);
  status = InitialiseAlltoDefault(jsonCmdParam, commandParamsTotalCount);
  return status;
}

eEepromStatus InitialiseEepromVariables(void)
{
  eEepromStatus status = EEPROM_OK;
  float eepromValue;
  stJsonCommand *jsonCmdParam;
  uint16_t commandParamsTotalCount = GetCommandProcessingParam(&jsonCmdParam);
  status = ReadEEPROM (EEPROM_VALIDITY_START_ADDRESS, &eepromValue, 1);
  if(status == EEPROM_OK)
  {
    if((uint32_t)eepromValue == EEPROM_VALIDITY_DATA)
    {
      for(uint16_t commandParamsCount =0; commandParamsCount < commandParamsTotalCount; commandParamsCount++)
      {
        status |= jsonCmdParam[commandParamsCount].executeStoreProcess(EEPROM_READ);
      }
    }
    else
    {
      status = InitialiseAlltoDefault(jsonCmdParam, commandParamsTotalCount);
    }
  }
  else
  {
    status = InitialiseAlltoDefault(jsonCmdParam, commandParamsTotalCount);     
  }
  status |= ReadEEPROM (EEPROM_VALIDITY_START_ADDRESS, &eepromValue, 1);
  
  return status;
}

eEepromStatus InitialiseAlltoDefault(stJsonCommand *jsonCmdParams, uint16_t commandParamsTotalCount)
{
  Serial_Debug("\n\n Initialising All to Default ");
  eEepromStatus status = EEPROM_OK;
  for(uint16_t commandParamsCount =0; commandParamsCount < commandParamsTotalCount; commandParamsCount++)
  {
    jsonCmdParams[commandParamsCount].executeAssignDefaultProcess();
  }
  for(uint16_t commandParamsCount =0; commandParamsCount < commandParamsTotalCount; commandParamsCount++)
  {
    status |= jsonCmdParams[commandParamsCount].executeStoreProcess(EEPROM_WRITE);
  }
  float eepromValue = EEPROM_VALIDITY_DATA;
  status |= WriteEEPROM (EEPROM_VALIDITY_START_ADDRESS, &eepromValue, 1);
  return status;
}

eEepromStatus EepromDevIDInfo(eEpromAccessType eepromOp)
{
  eEepromStatus status = EEPROM_OK;
  uint16_t eepromAddress = EEPROM_DEV_INFO_START_ADDRESS;
  stDevInfo *devIDInfoValues = GetSetDeviceIDInfo();
  uint16_t totalDataCount = (JSON_NO_OF_SET_DEVICE_INFO-1) * MAX_DEV_INFO_FIELD_LEN;
  char eepromValue[(JSON_NO_OF_SET_DEVICE_INFO-1) * MAX_DEV_INFO_FIELD_LEN];
  uint16_t datacount = 0;
  if(eepromOp == EEPROM_WRITE)
  {
    snprintf(&eepromValue[datacount], MAX_DEV_INFO_FIELD_LEN, devIDInfoValues->devID);
    uint16_t storelen = strlen(&eepromValue[datacount]);
    datacount = datacount+storelen;
    eepromValue[datacount++] = ',';
    snprintf(&eepromValue[datacount], MAX_DEV_INFO_FIELD_LEN, devIDInfoValues->hwVersion);
    storelen = strlen(&eepromValue[datacount]);
    datacount = datacount+storelen;
    eepromValue[datacount++] = ',';
    snprintf(&eepromValue[datacount], MAX_DEV_INFO_FIELD_LEN, devIDInfoValues->serialNo);
    storelen = strlen(&eepromValue[datacount]);
    datacount = datacount+storelen;
    eepromValue[datacount++] = '\0';
    for(uint16_t dummyDataIndex = datacount; dummyDataIndex < totalDataCount; dummyDataIndex++)
    {
      eepromValue[dummyDataIndex] = '\0';
    }
    status = WriteEEPROM_Byte (eepromAddress, (uint8_t*)eepromValue, totalDataCount);
  }
  else
  {
    status = ReadEEPROM_Byte (eepromAddress, (uint8_t*)eepromValue, totalDataCount);
    if(status == EEPROM_OK)
    {
      char *positionInfoStr[JSON_NO_OF_SET_DEVICE_INFO-1];
      uint8_t maxNoStrings=(sizeof positionInfoStr)/4;
      uint8_t totalResulStringCount =  SplitString (eepromValue, positionInfoStr, ',', maxNoStrings );
      if(totalResulStringCount == (JSON_NO_OF_SET_DEVICE_INFO-1))
      {
        snprintf(devIDInfoValues->devID, MAX_DEV_INFO_FIELD_LEN, positionInfoStr[0]);
        snprintf(devIDInfoValues->hwVersion, MAX_DEV_INFO_FIELD_LEN, positionInfoStr[1]);
        snprintf(devIDInfoValues->serialNo, MAX_DEV_INFO_FIELD_LEN, positionInfoStr[2]);
      }
      else
      {
        status = EEPROM_READ_ERROR;
      }
    }
  }
  return status;
}


//07D change
eEepromStatus EepromSetWheelDia(eEpromAccessType eepromOp)
{ 
 // float eepromValue1[1];
  eEepromStatus status = EEPROM_OK;
  uint16_t eepromAddress = STORE_WHEEL_DIA_STARTADD;
//  eMode setMode = GetOperationMode();
  stWheelDia *setWheelDia = GetSetwheelDiaLimits();
  //stMaxMotorCurrent *setCurrentValues = GetSetMaxMotorCurrentLimits();
  float eepromValue[JSON_NO_OF_WHEELDIA_DATA-1];
  
  uint16_t eepromAddIndex = 0;
  if(eepromOp == EEPROM_WRITE)
  {
    eepromValue[eepromAddIndex++] = setWheelDia->Dia;
    eepromValue[eepromAddIndex++] = setWheelDia->Pulse;
    eepromValue[eepromAddIndex++] = setWheelDia->Speed;
    eepromValue[eepromAddIndex++] = CYCLEROTATIONTIME;
    eepromValue[eepromAddIndex++] = circumference;
//    eepromValue[eepromAddIndex++] = forward_remaining_distance;
//    eepromValue[eepromAddIndex++] = backward_remaining_distance;
//    eepromValue[eepromAddIndex++] = Accel_remaining_distance;
//    eepromValue[eepromAddIndex++] = Decel_remaining_distance;
    
    status = WriteEEPROM (eepromAddress, eepromValue, eepromAddIndex);
//    status = ReadEEPROM (eepromAddress, eepromValue1, JSON_NO_OF_WHEELDIA_DATA-1);
  }
  else
  {
    status = ReadEEPROM (eepromAddress, eepromValue, JSON_NO_OF_WHEELDIA_DATA-1);
    if(status == EEPROM_OK)
    {
      setWheelDia->Dia = eepromValue[eepromAddIndex++];
      setWheelDia->Pulse = eepromValue[eepromAddIndex++];
      setWheelDia->Speed = eepromValue[eepromAddIndex++];
      CYCLEROTATIONTIME = eepromValue[eepromAddIndex++];
      circumference = eepromValue[eepromAddIndex++];
//      forward_remaining_distance = eepromValue[eepromAddIndex++];
//      backward_remaining_distance = eepromValue[eepromAddIndex];
//      Accel_remaining_distance = eepromValue[eepromAddIndex++];
//      Decel_remaining_distance = eepromValue[eepromAddIndex];
  //    SetInitialOperationMode(getMode);
    }
  }
  return status; 
}

eEepromStatus EepromSetCycleFrequency(eEpromAccessType eepromOp)
{
 eEepromStatus status = EEPROM_OK;
 uint16_t eepromAddress = STORE_CYCLE_FREQUENCY_STARTADD;
  
  stCycleFrequency *setCycleFrequency = GetSetcycleFrequencyLimits();
  
  float eepromValue[JSON_NO_OF_CYCLEFREQUENCY_DATA-1];
  
  uint16_t eepromAddIndex = 0;
  if(eepromOp == EEPROM_WRITE)
  {
    eepromValue[eepromAddIndex++] = setCycleFrequency->Cycle;
    status = WriteEEPROM (eepromAddress, eepromValue, eepromAddIndex);
  }
  else
  {
    status = ReadEEPROM (eepromAddress, eepromValue, JSON_NO_OF_CYCLEFREQUENCY_DATA-1);
    if(status == EEPROM_OK)
    {
     setCycleFrequency->Cycle = eepromValue[eepromAddIndex++];
     
    }
  }
 
 return status;
}


eEepromStatus EepromSetLogInterval(eEpromAccessType eepromOp)
{
  eEepromStatus status = EEPROM_OK;
  uint16_t eepromAddress = STORE_LOG_INTERVAL_STARTADD;
 
  stInterval *setInterval = GetSetIntervalLimits();
  
  float eepromValue[JSON_NO_OF_INTERVAL_DATA-1];
  uint16_t eepromAddIndex = 0;
  if(eepromOp == EEPROM_WRITE)
  {
    eepromValue[eepromAddIndex++] = setInterval->I1; eepromValue[eepromAddIndex++] = setInterval->P1;
    eepromValue[eepromAddIndex++] = setInterval->I2; eepromValue[eepromAddIndex++] = setInterval->P2;
    eepromValue[eepromAddIndex++] = setInterval->I3; eepromValue[eepromAddIndex++] = setInterval->P3;
    eepromValue[eepromAddIndex++] = setInterval->I4; eepromValue[eepromAddIndex++] = setInterval->P4;
    eepromValue[eepromAddIndex++] = setInterval->I5; eepromValue[eepromAddIndex++] = setInterval->P5;
    eepromValue[eepromAddIndex++] = setInterval->I6; eepromValue[eepromAddIndex++] = setInterval->P6;
    
    status = WriteEEPROM (eepromAddress, eepromValue, eepromAddIndex);
  }
  else
  {
    status = ReadEEPROM (eepromAddress, eepromValue, JSON_NO_OF_INTERVAL_DATA-1);
    if(status == EEPROM_OK)
    {
      setInterval->I1 = eepromValue[eepromAddIndex++]; setInterval->P1 = eepromValue[eepromAddIndex++];
      setInterval->I2 = eepromValue[eepromAddIndex++]; setInterval->P2 = eepromValue[eepromAddIndex++];
      setInterval->I3 = eepromValue[eepromAddIndex++]; setInterval->P3 = eepromValue[eepromAddIndex++];
      setInterval->I4 = eepromValue[eepromAddIndex++]; setInterval->P4 = eepromValue[eepromAddIndex++];
      setInterval->I5 = eepromValue[eepromAddIndex++]; setInterval->P5 = eepromValue[eepromAddIndex++];
      setInterval->I6 = eepromValue[eepromAddIndex++]; setInterval->P6 = eepromValue[eepromAddIndex++];
     
    }
  }
  
  return status;
}


eEepromStatus EepromSetContinue(eEpromAccessType eepromOp)
{
  eEepromStatus status = EEPROM_OK;
  uint16_t eepromAddress = STORE_CONTINUE_STARTADD;
  stContinue *setContinueValue = GetSetContinueLimits();
  float eepromValue[JSON_NO_OF_CONTINUE-1];
  uint16_t eepromAddIndex = 0;
  if(eepromOp == EEPROM_WRITE)
  {
    eepromValue[eepromAddIndex++] = setContinueValue->Continue1;
   // eepromValue[eepromAddIndex++] = setContinueValue->Count1;
    status = WriteEEPROM (eepromAddress, eepromValue, eepromAddIndex);
  }
  else
  {
    status = ReadEEPROM (eepromAddress, eepromValue, JSON_NO_OF_CONTINUE-1);
    if(status == EEPROM_OK)
    {
      setContinueValue->Continue1  =  eepromValue[eepromAddIndex++];
      //setContinueValue->Count1  =  eepromValue[eepromAddIndex++];
    }
  }
  return status; 
}

eEepromStatus EepromSetReturnState(eEpromAccessType eepromOp)
{
  eEepromStatus status = EEPROM_OK;
  uint16_t eepromAddress = STORE_RETURN_STARTADD;
  stReturn *setReturnValue = GetSetReturnLimits();
  float eepromValue[JSON_NO_OF_RETURN-1];
  uint16_t eepromAddIndex = 0;
  if(eepromOp == EEPROM_WRITE)
  {
    eepromValue[eepromAddIndex++] = setReturnValue->Return1;
    status = WriteEEPROM (eepromAddress, eepromValue, eepromAddIndex);
  }
  else
  {
    status = ReadEEPROM (eepromAddress, eepromValue, JSON_NO_OF_RETURN-1);
    if(status == EEPROM_OK)
    {
      setReturnValue->Return1  =  eepromValue[eepromAddIndex++];
      
    }
  }
  return status; 
}

eEepromStatus EepromSetcomdistance(eEpromAccessType eepromOp)
{
  eEepromStatus status = EEPROM_OK;
  uint16_t eepromAddress = STORE_COMDISTANCE_STARTADD;
  
//  stReturn *setReturnValue = GetSetReturnLimits();
  stcomdistance* setcomdistance = GetSetComDistanceLimits();
 // stcomdistance tempSetComDistanceValues = *setcomdistance;
  
  float eepromValue[JSON_NO_OF_COMDISTANCE+1];
  uint16_t eepromAddIndex = 0;
  
  if(eepromOp == EEPROM_WRITE)
  {
    eepromValue[eepromAddIndex++] = ComDistance_int;
    eepromValue[eepromAddIndex++] = setcomdistance->CDistance; 
    status = WriteEEPROM (eepromAddress, eepromValue, eepromAddIndex);
  }
  else
  {
    status = ReadEEPROM (eepromAddress, eepromValue, JSON_NO_OF_COMDISTANCE);
    if(status == EEPROM_OK)
    {
      ComDistance_int  =  eepromValue[eepromAddIndex++];
      setcomdistance->CDistance  =  eepromValue[eepromAddIndex++]; 
    }
  }
  return status; 
}

eEepromStatus EepromStoreRowLength(eEpromAccessType eepromOp)
{
  eEepromStatus status = EEPROM_OK;
  uint16_t eepromAddress = STORE_ROW_LENGTH_STARTADD;
  stRowLengthData *setRowLengthData = GetSetRowLengthData();
  uint8_t eepromRowLengthCount = MAX_NO_OF_TRACKS+MAX_NO_OF_TRACKS;
  float eepromValue[MAX_NO_OF_TRACKS+MAX_NO_OF_TRACKS];
  uint16_t eepromAddIndex = 0;
  if(eepromOp == EEPROM_WRITE)
  {
    //eepromValue[eepromAddIndex++] = setTrackData->trackCount;
    for(int i=0;i<MAX_NO_OF_TRACKS;i++)
    {
      eepromValue[eepromAddIndex++] = setRowLengthData->rowLength[i][FORWARD_COUNT];
      eepromValue[eepromAddIndex++] = setRowLengthData->rowLength[i][REVERSE_COUNT];
    }
    status = WriteEEPROM (eepromAddress, eepromValue, eepromAddIndex);
  }
  else
  {
    status = ReadEEPROM (eepromAddress, eepromValue, eepromRowLengthCount);
    if(status == EEPROM_OK)
    {
      //setTrackData->trackCount             =  (uint32_t)eepromValue[eepromAddIndex++];
      for(int i=0;i<MAX_NO_OF_TRACKS;i++)
      {
       setRowLengthData->rowLength[i][FORWARD_COUNT]       =  (uint32_t)eepromValue[eepromAddIndex++];
       setRowLengthData->rowLength[i][REVERSE_COUNT]       =  (uint32_t)eepromValue[eepromAddIndex++];
      }
      
    }
  }
  return status; 
}

eEepromStatus EepromStoreRowLength2(eEpromAccessType eepromOp)
{
    eEepromStatus status = EEPROM_OK;
  uint16_t eepromAddress = STORE_ROW_LENGTH2_STARTADD;
  stRowLengthData *setRowLengthData = GetSetRowLengthData();
  uint8_t eepromRowLengthCount = MAX_NO_OF_TRACKS+MAX_NO_OF_TRACKS;
  float eepromValue[MAX_NO_OF_TRACKS+MAX_NO_OF_TRACKS];
  uint16_t eepromAddIndex = 0;
  if(eepromOp == EEPROM_WRITE)
  {
    //eepromValue[eepromAddIndex++] = setTrackData->trackCount;
    for(int i=0;i<MAX_NO_OF_TRACKS;i++)
    {
      eepromValue[eepromAddIndex++] = setRowLengthData->rowLength2[i][FORWARD_COUNT];
      eepromValue[eepromAddIndex++] = setRowLengthData->rowLength2[i][REVERSE_COUNT];
    }
    status = WriteEEPROM (eepromAddress, eepromValue, eepromAddIndex);
  }
  else
  {
    status = ReadEEPROM (eepromAddress, eepromValue, eepromRowLengthCount);
    if(status == EEPROM_OK)
    {
      //setTrackData->trackCount             =  (uint32_t)eepromValue[eepromAddIndex++];
      for(int i=0;i<MAX_NO_OF_TRACKS;i++)
      {
       setRowLengthData->rowLength2[i][FORWARD_COUNT]       =  (uint32_t)eepromValue[eepromAddIndex++];
       setRowLengthData->rowLength2[i][REVERSE_COUNT]       =  (uint32_t)eepromValue[eepromAddIndex++];
      }
      
    }
  }
  return status; 
}

eEepromStatus EepromStoreRowLength3(eEpromAccessType eepromOp)
{
    eEepromStatus status = EEPROM_OK;
  uint16_t eepromAddress = STORE_ROW_LENGTH3_STARTADD;
  stRowLengthData *setRowLengthData = GetSetRowLengthData();
  uint8_t eepromRowLengthCount = MAX_NO_OF_TRACKS+MAX_NO_OF_TRACKS;
  float eepromValue[MAX_NO_OF_TRACKS+MAX_NO_OF_TRACKS];
  uint16_t eepromAddIndex = 0;
  if(eepromOp == EEPROM_WRITE)
  {
    //eepromValue[eepromAddIndex++] = setTrackData->trackCount;
    for(int i=0;i<MAX_NO_OF_TRACKS;i++)
    {
      eepromValue[eepromAddIndex++] = setRowLengthData->rowLength3[i][FORWARD_COUNT];
      eepromValue[eepromAddIndex++] = setRowLengthData->rowLength3[i][REVERSE_COUNT];
    }
    status = WriteEEPROM (eepromAddress, eepromValue, eepromAddIndex);
  }
  else
  {
    status = ReadEEPROM (eepromAddress, eepromValue, eepromRowLengthCount);
    if(status == EEPROM_OK)
    {
      //setTrackData->trackCount             =  (uint32_t)eepromValue[eepromAddIndex++];
      for(int i=0;i<MAX_NO_OF_TRACKS;i++)
      {
       setRowLengthData->rowLength3[i][FORWARD_COUNT]       =  (uint32_t)eepromValue[eepromAddIndex++];
       setRowLengthData->rowLength3[i][REVERSE_COUNT]       =  (uint32_t)eepromValue[eepromAddIndex++];
      }
      
    }
  }
  return status; 
}

eEepromStatus EepromStoreRowLength4(eEpromAccessType eepromOp)
{
    eEepromStatus status = EEPROM_OK;
  uint16_t eepromAddress = STORE_ROW_LENGTH4_STARTADD;
  stRowLengthData *setRowLengthData = GetSetRowLengthData();
  uint8_t eepromRowLengthCount = MAX_NO_OF_TRACKS+MAX_NO_OF_TRACKS;
  float eepromValue[MAX_NO_OF_TRACKS+MAX_NO_OF_TRACKS];
  uint16_t eepromAddIndex = 0;
  if(eepromOp == EEPROM_WRITE)
  {
    //eepromValue[eepromAddIndex++] = setTrackData->trackCount;
    for(int i=0;i<MAX_NO_OF_TRACKS;i++)
    {
      eepromValue[eepromAddIndex++] = setRowLengthData->rowLength4[i][FORWARD_COUNT];
      eepromValue[eepromAddIndex++] = setRowLengthData->rowLength4[i][REVERSE_COUNT];
    }
    status = WriteEEPROM (eepromAddress, eepromValue, eepromAddIndex);
  }
  else
  {
    status = ReadEEPROM (eepromAddress, eepromValue, eepromRowLengthCount);
    if(status == EEPROM_OK)
    {
      //setTrackData->trackCount             =  (uint32_t)eepromValue[eepromAddIndex++];
      for(int i=0;i<MAX_NO_OF_TRACKS;i++)
      {
       setRowLengthData->rowLength4[i][FORWARD_COUNT]       =  (uint32_t)eepromValue[eepromAddIndex++];
       setRowLengthData->rowLength4[i][REVERSE_COUNT]       =  (uint32_t)eepromValue[eepromAddIndex++];
      }
      
    }
  }
  return status; 
}

eEepromStatus EepromStoreRowLength5(eEpromAccessType eepromOp)
{
    eEepromStatus status = EEPROM_OK;
  uint16_t eepromAddress = STORE_ROW_LENGTH5_STARTADD;
  stRowLengthData *setRowLengthData = GetSetRowLengthData();
  uint8_t eepromRowLengthCount = MAX_NO_OF_TRACKS+MAX_NO_OF_TRACKS;
  float eepromValue[MAX_NO_OF_TRACKS+MAX_NO_OF_TRACKS];
  uint16_t eepromAddIndex = 0;
  if(eepromOp == EEPROM_WRITE)
  {
    //eepromValue[eepromAddIndex++] = setTrackData->trackCount;
    for(int i=0;i<MAX_NO_OF_TRACKS;i++)
    {
      eepromValue[eepromAddIndex++] = setRowLengthData->rowLength5[i][FORWARD_COUNT];
      eepromValue[eepromAddIndex++] = setRowLengthData->rowLength5[i][REVERSE_COUNT];
    }
    status = WriteEEPROM (eepromAddress, eepromValue, eepromAddIndex);
  }
  else
  {
    status = ReadEEPROM (eepromAddress, eepromValue, eepromRowLengthCount);
    if(status == EEPROM_OK)
    {
      //setTrackData->trackCount             =  (uint32_t)eepromValue[eepromAddIndex++];
      for(int i=0;i<MAX_NO_OF_TRACKS;i++)
      {
       setRowLengthData->rowLength5[i][FORWARD_COUNT]       =  (uint32_t)eepromValue[eepromAddIndex++];
       setRowLengthData->rowLength5[i][REVERSE_COUNT]       =  (uint32_t)eepromValue[eepromAddIndex++];
      }
      
    }
  }
  return status; 
}

eEepromStatus EepromStoreRowLength6(eEpromAccessType eepromOp)
{
    eEepromStatus status = EEPROM_OK;
  uint16_t eepromAddress = STORE_ROW_LENGTH2_STARTADD;
  stRowLengthData *setRowLengthData = GetSetRowLengthData();
  uint8_t eepromRowLengthCount = MAX_NO_OF_TRACKS+MAX_NO_OF_TRACKS;
  float eepromValue[MAX_NO_OF_TRACKS+MAX_NO_OF_TRACKS];
  uint16_t eepromAddIndex = 0;
  if(eepromOp == EEPROM_WRITE)
  {
    //eepromValue[eepromAddIndex++] = setTrackData->trackCount;
    for(int i=0;i<MAX_NO_OF_TRACKS;i++)
    {
      eepromValue[eepromAddIndex++] = setRowLengthData->rowLength6[i][FORWARD_COUNT];
      eepromValue[eepromAddIndex++] = setRowLengthData->rowLength6[i][REVERSE_COUNT];
    }
    status = WriteEEPROM (eepromAddress, eepromValue, eepromAddIndex);
  }
  else
  {
    status = ReadEEPROM (eepromAddress, eepromValue, eepromRowLengthCount);
    if(status == EEPROM_OK)
    {
      //setTrackData->trackCount             =  (uint32_t)eepromValue[eepromAddIndex++];
      for(int i=0;i<MAX_NO_OF_TRACKS;i++)
      {
       setRowLengthData->rowLength6[i][FORWARD_COUNT]       =  (uint32_t)eepromValue[eepromAddIndex++];
       setRowLengthData->rowLength6[i][REVERSE_COUNT]       =  (uint32_t)eepromValue[eepromAddIndex++];
      }
      
    }
  }
  return status; 
}

eEepromStatus EepromStoreRowLength7(eEpromAccessType eepromOp)
{
    eEepromStatus status = EEPROM_OK;
  uint16_t eepromAddress = STORE_ROW_LENGTH7_STARTADD;
  stRowLengthData *setRowLengthData = GetSetRowLengthData();
  uint8_t eepromRowLengthCount = MAX_NO_OF_TRACKS+MAX_NO_OF_TRACKS;
  float eepromValue[MAX_NO_OF_TRACKS+MAX_NO_OF_TRACKS];
  uint16_t eepromAddIndex = 0;
  if(eepromOp == EEPROM_WRITE)
  {
    //eepromValue[eepromAddIndex++] = setTrackData->trackCount;
    for(int i=0;i<MAX_NO_OF_TRACKS;i++)
    {
      eepromValue[eepromAddIndex++] = setRowLengthData->rowLength7[i][FORWARD_COUNT];
      eepromValue[eepromAddIndex++] = setRowLengthData->rowLength7[i][REVERSE_COUNT];
    }
    status = WriteEEPROM (eepromAddress, eepromValue, eepromAddIndex);
  }
  else
  {
    status = ReadEEPROM (eepromAddress, eepromValue, eepromRowLengthCount);
    if(status == EEPROM_OK)
    {
      //setTrackData->trackCount             =  (uint32_t)eepromValue[eepromAddIndex++];
      for(int i=0;i<MAX_NO_OF_TRACKS;i++)
      {
       setRowLengthData->rowLength7[i][FORWARD_COUNT]       =  (uint32_t)eepromValue[eepromAddIndex++];
       setRowLengthData->rowLength7[i][REVERSE_COUNT]       =  (uint32_t)eepromValue[eepromAddIndex++];
      }
      
    }
  }
  return status; 
}

eEepromStatus EepromSetNoofRowday(eEpromAccessType eepromOp)
{
  eEepromStatus status = EEPROM_OK;
  uint16_t eepromAddress = STORE_ROWDAY_STARTADD;
 
//  stInterval *setInterval = GetSetIntervalLimits();
  stnoofrows* setNoofrows = GetSetNoofrowsLimits();
  
  float eepromValue[JSON_NO_OF_ROWS-1];
  uint16_t eepromAddIndex = 0;
  if(eepromOp == EEPROM_WRITE)
  {
    for(int i=0;i<7;i++)
    {
      eepromValue[eepromAddIndex++] = setNoofrows->Row[i];
    }
     eepromValue[eepromAddIndex++] = setNoofrows->TRow;
//    eepromValue[eepromAddIndex++] = setNoofrows->R1; eepromValue[eepromAddIndex++] = setNoofrows->R2;
//    eepromValue[eepromAddIndex++] = setNoofrows->R3; eepromValue[eepromAddIndex++] = setNoofrows->R4;
//    eepromValue[eepromAddIndex++] = setNoofrows->R5; eepromValue[eepromAddIndex++] = setNoofrows->R6;
//    eepromValue[eepromAddIndex++] = setNoofrows->R7; 
    
//    eepromValue[eepromAddIndex++] = setInterval->P4;
//    eepromValue[eepromAddIndex++] = setInterval->I5; eepromValue[eepromAddIndex++] = setInterval->P5;
//    eepromValue[eepromAddIndex++] = setInterval->I6; eepromValue[eepromAddIndex++] = setInterval->P6;
    
    status = WriteEEPROM (eepromAddress, eepromValue, eepromAddIndex);
  }
  else
  {
    status = ReadEEPROM (eepromAddress, eepromValue, JSON_NO_OF_ROWS-1);
    if(status == EEPROM_OK)
    {
      for(int i=0;i<7;i++)
      {
        setNoofrows->Row[i] = eepromValue[eepromAddIndex++];
      }
        setNoofrows->TRow = eepromValue[eepromAddIndex++];
      
//      setNoofrows->R1 = eepromValue[eepromAddIndex++]; setNoofrows->R2 = eepromValue[eepromAddIndex++];
//      setNoofrows->R3 = eepromValue[eepromAddIndex++]; setNoofrows->R4 = eepromValue[eepromAddIndex++];
//      setNoofrows->R5 = eepromValue[eepromAddIndex++]; setNoofrows->R6 = eepromValue[eepromAddIndex++];
//      setNoofrows->R7 = eepromValue[eepromAddIndex++]; 
////      
//      setInterval->P4 = eepromValue[eepromAddIndex++];
//      setInterval->I5 = eepromValue[eepromAddIndex++]; setInterval->P5 = eepromValue[eepromAddIndex++];
//      setInterval->I6 = eepromValue[eepromAddIndex++]; setInterval->P6 = eepromValue[eepromAddIndex++];
//     
    }
  }

}