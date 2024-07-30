#include "FaultProcess.h"
#include "MotCurrentFaultCommon.h"
#include "RobotOperation.h"
#include "Rtc.h"
#include "LTC4015_Main.h"
#include "Temp_Sensors.h"
#include "CommCommon.h"
#include "Serial_Debug.h"
#include "RotateSenseCommon.h"

#include "CycleMode.h"

#define NOT_IMPLEMENTED_CODE

#define BAT_SOC_PERC_HYS        2

static eErrorState errorState = ER_IDLE;
static uint32_t motorFaultData = NO_FAULT;
static stFaultTimeData faultTimeData;

static void SetMotorFault (uint32_t *faultData, eFaultFlags faultFlag);
static bool IsMotorFault (uint32_t *faultData, eFaultFlags faultFlag);
static bool CheckIfMotorToBeStopped (void);
static uint8_t GetErrorArrayData(int8_t *errorArray, uint8_t arraySize);
static void ReportError (uint32_t motorFaultCode);
static void ClearMotorFault (uint32_t *faultData, eFaultFlags faultFlag);
static void CheckClearMotorOverCurrentFaults (void);
static void CheckClearBatteryFault(void);
static void CheckBoardTempFault (void);
static void CheckLowBatteryFault (void);
void CheckBatteryFault(void);

static uint8_t setBatMinSOC;

 char Error_Cleared;
 char only_once;
//static volatile bool ContinueTimeOver          = false;
//static int32_t Continue_timecount              = 0;
//static int32_t brushMotor_timercount           = 0;

extern char Robot_Error_Flag;
char Robot_Error_Count = 0,Robot_Error_Count1;
char SDerrorlog[45];
extern int ERROR_OCCURRED;

uint16_t cleaner_in_comm = 0;

static void CheckADCFault (void);

char Error_Count()
{
  return Robot_Error_Count;
}

char ClearRoboterror()
{
  Robot_Error_Count = 0;
}

eErrorState GetErrorState (void)
{
  return errorState;
}

void SetErrorState (eErrorState errorSt)
{
  errorState = errorSt;
}

uint8_t * GetSetMinBattSoc (void)
{
  return &setBatMinSOC;  
}
  
void CheckError (void)
{
  static uint32_t prevMotorFaultData = NO_FAULT;
  CheckADCFault();
  CheckLowBatteryFault();
  CheckBoardTempFault();
  CheckBatteryFault();
//  CheckMotorOverLoad();
 // CheckMechanicalFault();
  eErrorState errorState = GetErrorState();
  switch (errorState) 
  {
   case ER_IDLE:
      if(Robot_Error_Flag == 0)
        SetErrorState(ER_NO_ERROR);
      
    break;
   case ER_NO_ERROR:
    if(IsMotorFault(&motorFaultData, ALL_FAULT) == true && Robot_Error_Flag != 1)
    {
      (void)CheckIfMotorToBeStopped();
      prevMotorFaultData = motorFaultData;
    //  SetLogEvent(EV_LOG_FAULT, (uint8_t)motorFaultData);
      ReportError(motorFaultData);
      SetErrorState(ER_CLEAR_ERROR);
      
//      if(Robot_Error_Flag != 1)
//      {
//        SetErrorState(ER_ERROR);
//        Robot_Error_Flag = 0;
//      }
//      else
    }
    break;
   case ER_ERROR:
//    {
//      if(prevMotorFaultData != motorFaultData)
//      {
//        (void)CheckIfMotorToBeStopped();
//        prevMotorFaultData = motorFaultData;
////        SetLogEvent(EV_LOG_FAULT, (uint8_t)motorFaultData);
//        ReportError(motorFaultData);
//      }
//      if(IsMotorFault(&motorFaultData, ALL_FAULT) == false)
//      {
//        SetErrorState(ER_CLEAR_ERROR);    
//      }
//    }
    break;
   case ER_CLEAR_ERROR:
//    {
//      eMode opMode =  GetOperationMode();
//      if(opMode != MANUAL)
//      {
//        SetOperationMode(opMode, MAN_IDLE);
//      }
//      SetLogEvent(EV_LOG_FAULT, (uint8_t)motorFaultData);
      SetErrorState(ER_IDLE);
//    }
   default:
    break;
  }
}   

static void CheckLowBatteryFault (void)
{
  float BatterySoc = GetSOCAsPercentage();
  uint8_t *minBatSoc = GetSetMinBattSoc();
  if(BatterySoc < *minBatSoc)
  {
    if(IsMotorFault(&motorFaultData, MIN_BAT_VOLTAGE_FAULT) == false)
    {
      Serial_Debug("\n Low Voltage Detected.");
      SetMotorFault(&motorFaultData, MIN_BAT_VOLTAGE_FAULT);
    }
  }
  else if(BatterySoc > (*minBatSoc + BAT_SOC_PERC_HYS))
  {
    if(IsMotorFault(&motorFaultData, MIN_BAT_VOLTAGE_FAULT) == true)
    {
      Serial_Debug("\n Battery Voltage Normal."); 
      ClearMotorFault(&motorFaultData, MIN_BAT_VOLTAGE_FAULT);
    }
  }
}

static void CheckBoardTempFault (void)
{
  float boardTemperatue = GetTemperatureSensorData(TEMPSENSOR_1);
  stBoardTemperature* boardSetTemperature = GetSetMaxBoardTemperature();
  if(boardTemperatue > boardSetTemperature->maxTempSensorOne)
  {
    if(IsMotorFault(&motorFaultData, OVER_BTEMP_FAULT) == false)
    {
      Serial_Debug("\nCurrent Temperature-->");
      Serial_Debug_Float(boardTemperatue,3);
      Serial_Debug("\nSet Temperature-->");
      Serial_Debug_Float(boardSetTemperature->maxTempSensorOne,3);
      Serial_Debug("\n Over Board Temperature Detected.");
      SetMotorFault(&motorFaultData, OVER_BTEMP_FAULT);
    }
  }
}

void CheckBatteryFault(void)
{
  uint8_t batError = GetBatteryError();
  if(batError != 0)
  {
    if(IsMotorFault(&motorFaultData, BATTERY_FAULT) == false)
    {
      Serial_Debug("\n Battery Fault Detected.");
      SetMotorFault(&motorFaultData, BATTERY_FAULT);
    }    
  }    
}

static void CheckADCFault (void)
{
  stMaxMotorCurrent* getMotorCurrent = GetSetMaxMotorCurrentLimits();
  
  if(IsMotor1OverCurrentFault() && (IsMotorFault(&motorFaultData, OVER_CURRENT_MOT1_FAULT) == false))
  {
    SetMotorFault(&motorFaultData, OVER_CURRENT_MOT1_FAULT);
  } 
  if(IsMotor2OverCurrentFault() && (IsMotorFault(&motorFaultData, OVER_CURRENT_MOT2_FAULT) == false))
  {
    SetMotorFault(&motorFaultData, OVER_CURRENT_MOT2_FAULT);
  } 
  if(IsMotor3OverCurrentFault() && (IsMotorFault(&motorFaultData, OVER_CURRENT_MOT3_FAULT) == false))
  {
    SetMotorFault(&motorFaultData, OVER_CURRENT_MOT3_FAULT);
  } 
}

bool IsOnlyNonMotorError (void)
{
  bool status;
  uint32_t volatile tempMotorFaultData = motorFaultData;
  ClearMotorFault(&motorFaultData, ZIGBEE_FAULT);
//  ClearMotorFault(&motorFaultData, SPI_FLASH_FAULT);
//  ClearMotorFault(&motorFaultData, OTAA_FAILURE);
  if(IsMotorFault (&motorFaultData, ALL_FAULT) == true)
  {
    status =  false; 
  }
  else
  {
    status = true;
  }
  motorFaultData = tempMotorFaultData;
  return status;
}


static bool CheckIfMotorToBeStopped (void)                                   /* Motor Need not be stopped if Only Comm Error or Only RTC Error with Manual Mode */
{
  char restart;
  uint32_t volatile tempMotorFaultData = motorFaultData;
  ClearMotorFault(&motorFaultData, COMMUNICATION_FAULT);
    ClearMotorFault(&motorFaultData, RTC_FAULT);
    ClearMotorFault(&motorFaultData, ZIGBEE_FAULT);
//  ClearMotorFault(&motorFaultData, SPI_FLASH_FAULT);
//  ClearMotorFault(&motorFaultData, OTAA_FAILURE);
  // stContinue *setContinueValue = GetSetContinueLimits();
    
    stReturn *setReturnValue = GetSetReturnLimits();
  
  if(IsMotorFault (&motorFaultData, ALL_FAULT) == true)
  {
    motorFaultData = tempMotorFaultData;
    eRobotState curRobState = GetRobotState();
    
    if((curRobState != ROBOT_COMPLETED) && (curRobState != ROBOT_IDLE))
    {
//      SetMotionPauseState(PA_IDLE);
//      SetRobotState(ROBOT_IMMEDIATE_STOP);  /* Need abrupt Stop of the Motor  */
      if((Robot_Error_Count++ <= setReturnValue->Return1) && Robot_Error_Flag == 0)
      {
        SetMotionPauseState(PA_PAUSE);
        Robot_Error_Flag = 1;
        Robot_Error_Count1 = Robot_Error_Count;
      }
      else
      {
//        SetRobotState(ROBOT_IMMEDIATE_STOP);
        if(Error_Cleared == 0)
        {
          SetMotionPauseState(PA_PAUSE);
          SetErrorState(ER_IDLE);
          Error_Cleared = 1;
        }
        return true;
      }
    }
  }
  else
  {
    motorFaultData = tempMotorFaultData;
//    if(IsMotorFault(&motorFaultData, COMMUNICATION_FAULT) == true)
//    {
//       SetOperationMode(COMM_ERROR_EMERGENCY_STOW, MAN_IDLE); 
//    }
    return false;
  }
}

void CheckCommunicationError (void)
{
  char Track_Changer_Error;
  stReturn *setReturnValue = GetSetReturnLimits();
  stcomdistance* setcomdistance = GetSetComDistanceLimits();
  
  Track_Changer_Error = setReturnValue->Return1;
  if((GetRobotActualDirection() == POSITIVE))
  {  
    if(GetRotateSense1Count() <= setcomdistance->CDistance)
    {
        cleaner_in_comm = 1;
        if(IsCommTimeOver() == true)
        {
          ClearCommTimeOver();
          SetMotorFault(&motorFaultData, COMMUNICATION_FAULT);
          if(Robot_Error_Count <= setReturnValue->Return1 && only_once == 0)
          {
            SetMotionPauseState(PA_PAUSE);
            Robot_Error_Count++;
            Robot_Error_Count1 = Robot_Error_Count;
            only_once = 1;
          } 
          else
          {
            if(Error_Cleared == 0)
            {
              SetMotionPauseState(PA_PAUSE);
              Error_Cleared = 1;
            }
          }      
          Serial_Debug("\r\n~*~*~*~*~*Communication Error~*~*~*~*~\r\n"); 
        }       
      }
      else
      {
        cleaner_in_comm = 0;
      }
    }
  
  else if((GetRobotActualDirection() == NEGATIVE))
  { 
    int CountCheck=0;
    stcomdistance* setcomdistance = GetSetComDistanceLimits();
    stCountRange* setCountValues = GetSetCountRange();
    
    if(setCountValues->maxNegCountForPVsetup > 42)
    {
      CountCheck = ((setCountValues->maxNegCountForPVsetup)-(setcomdistance->CDistance)); 
    }
    else
    {
      CountCheck = 0;
    }
     stReturn *setReturnValue = GetSetReturnLimits(); 
     
    if(GetRotateSense1Count() >= CountCheck)
    {
        cleaner_in_comm = 1;
        if(IsCommTimeOver() == true)
        {
          ClearCommTimeOver();
          SetMotorFault(&motorFaultData, COMMUNICATION_FAULT);
          if(Robot_Error_Count <= setReturnValue->Return1 && only_once == 0)
          {
            SetMotionPauseState(PA_PAUSE);
            Robot_Error_Count = ++Track_Changer_Error;    
            Robot_Error_Count1 = Robot_Error_Count;
            only_once = 1;
          } 
          else
          {
            if(Error_Cleared == 0)
            {
              SetMotionPauseState(PA_PAUSE);
              Error_Cleared = 1;
            }
          }      
          Serial_Debug("\r\n~*~*~*~*~*Communication Error~*~*~*~*~\r\n"); 
        }       
     }
      else
      {
        cleaner_in_comm = 0;
      }
   }
  
  SetPrevPauseState(GetMotionPauseState());
}

static bool IsMotorFault (uint32_t *faultData, eFaultFlags faultFlag)
{
  if(faultFlag != ALL_FAULT)
  {
    if(*faultData & (1 << faultFlag))
    {
      return true;
    }
    else
    {
      return false;
    }
  }
  else
  {
    if(*faultData)
    {
      return true;
    }
    else
    {
      return false;
    }
  }
}

static void SetMotorFault (uint32_t *faultData, eFaultFlags faultFlag)
{
  char SD_Error_Log[10];
  if(faultFlag == ALL_FAULT)
  {
    *faultData = ALL_FAULT;
  }
  else if(faultFlag == NO_FAULT)
  {
    *faultData = NO_FAULT;
  }
  else
  {
    if((faultFlag < MAX_FAULT) && (faultFlag > NO_FAULT))
    {
      faultTimeData.errorCodeTime[faultFlag] = GetRTCDateTimeSeconds();
    }
    *faultData |= (1 << faultFlag);
    
    
    
    eRobotDirection robotState = GetRobotActualDirection();
    if(robotState == 1 || robotState == 0)
    {
        memset(SDerrorlog,'\0',sizeof(SDerrorlog));
    //    GetRTCDateTimeString(&SDerrorlog, sizeof SDerrorlog);
        snprintf(SD_Error_Log,10,"%000X", GetMotorFaultCode());
    //    strcat(SDerrorlog,",");
        strcat(SDerrorlog,SD_Error_Log);
        ERROR_OCCURRED = 1; 
    }
  }
}

void SetFault(eFaultFlags faultFlag)
{
  SetMotorFault (&motorFaultData, faultFlag);
}

void ClearFault(eFaultFlags faultFlag)
{
  ClearMotorFault (&motorFaultData, faultFlag);
}

static void ClearMotorFault (uint32_t *faultData, eFaultFlags faultFlag)
{
  if(faultFlag != ALL_FAULT)
  {
    *faultData &= (~(1 << faultFlag));
  }
  else
  {
    *faultData = NO_FAULT;
  }
}

static void CheckClearMotorOverCurrentFaults (void)
{
  if(IsMotorFault(&motorFaultData, OVER_CURRENT_MOT1_FAULT) == true)
  {
    ClearMotor1OverCurrentFault();
    ClearMotorFault(&motorFaultData, OVER_CURRENT_MOT1_FAULT);
  }
  if(IsMotorFault(&motorFaultData, OVER_CURRENT_MOT2_FAULT) == true)
  {
    ClearMotor2OverCurrentFault();
    ClearMotorFault(&motorFaultData, OVER_CURRENT_MOT2_FAULT);
  }
  if(IsMotorFault(&motorFaultData, OVER_CURRENT_MOT3_FAULT) == true)
  {
    ClearMotor3OverCurrentFault();
    ClearMotorFault(&motorFaultData, OVER_CURRENT_MOT3_FAULT);
  }
}

static void CheckClearBatteryFault(void)
{
  Serial_Debug("\nCheckClearBattery-->");
  Serial_Debug_Num(IsMotorFault(&motorFaultData, BATTERY_FAULT));
  if(IsMotorFault(&motorFaultData, BATTERY_FAULT) == true)
  {
    ClearBatteryError();
    ClearMotorFault(&motorFaultData, BATTERY_FAULT);
  }
}

void ClearAllFaults (void)
{
 if(IsMotorFault(&motorFaultData, ALL_FAULT) == true)
 {
   ClearFaultsOnRobotCommand();
//  CheckClearMotorOverCurrentFaults();
  CheckClearBatteryFault();
  ClearMotorFault(&motorFaultData, ALL_FAULT);
  ClearRoboterror();
 }
 errorState = ER_NO_ERROR; 
}

void ClearFaultsOnRobotCommand (void)
{
 if(IsMotorFault(&motorFaultData, ALL_FAULT) == true)
 {
    CheckClearMotorOverCurrentFaults();
//  ClearMotorFault(&motorFaultData, OVER_BTEMP_FAULT);
    ClearMotorFault(&motorFaultData, MIN_BAT_VOLTAGE_FAULT); 
 }
 errorState = ER_NO_ERROR; 
}

uint32_t GetMotorFaultCode (void)
{
  return motorFaultData;
}

stFaultTimeData * GetFaultTimeData(void)
{
  faultTimeData.noOffaults = GetErrorArrayData(faultTimeData.errorCodes, MAX_FAULT);
  return (&faultTimeData);
}

void ReportErrorOnZigeeStart (void)
{
  if(IsMotorFault(&motorFaultData, ALL_FAULT) == true)
  {
   ReportError(motorFaultData);   
  }  
}

static void ReportError (uint32_t motorFaultCode)
{
  Serial_Debug("\n Error Code :");
  Serial_Debug_Num(motorFaultCode);
  //ProcessReportError();
//  faultTimeData.noOffaults = GetErrorArrayData(faultTimeData.errorCodes, MAX_FAULT);
//  char * alertData;
//  alertData = FormatAlertData(faultTimeData.noOffaults, faultTimeData.errorCodes);
//  UartAllTransmit((uint8_t*)alertData, strlen((const char *)alertData));
}

static uint8_t GetErrorArrayData(int8_t *errorArray, uint8_t arraySize)
{
  uint8_t arrayIndex = 0;
  for(int8_t errorIndex = 1; errorIndex < MAX_FAULT; errorIndex++)
  {
    if(IsMotorFault(&motorFaultData,(eFaultFlags)errorIndex) == true)
    {
      if(arrayIndex < arraySize)
      {
        errorArray[arrayIndex++] = errorIndex;
      }
      else
      {
        break;
      }     
    }
  }
  if(!arrayIndex)
  {
    errorArray[0] = NO_FAULT;
    arrayIndex = 1;
  }
  return arrayIndex;
}

bool IsRTCFault (void)
{
  return(IsMotorFault (&motorFaultData, RTC_FAULT));
}


bool IsAnyFault(void)
{
return(IsMotorFault (&motorFaultData,ALL_FAULT));
}

#ifndef NOT_IMPLEMENTED_CODE
static bool IsOtherFault (uint32_t faultValue, eFaultFlags faultFlag)
{
  faultValue &= (~(1 << faultFlag));
  if(!faultValue)
  {
    return false;
  }
  else
  {
    return true;
  }  
}

static bool IsOnlyOneFault (uint32_t faultValue, eFaultFlags faultFlag)
{
  faultValue &= (~(1 << faultFlag));
  if((!faultValue) && (IsMotorFault(&faultValue, faultFlag)))
  {
    return true;
  }
  else
  {
    return false;
  }  
}



#endif

