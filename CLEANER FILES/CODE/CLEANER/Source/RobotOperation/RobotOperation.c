#include "RobotOperation.h"
#include "RotateSenseCommon.h"
#include "Serial_Debug.h"
#include "NumStrConversion.h"
//#include "ApiProcessing.h"
//#include "CycleTest.h"
#include "CycleMode.h"
#include "AutoSchedule.h"
#include "MotCurrentFaultCommon.h"
#include "FaultProcess.h"
#include "CommCommon.h"
#include "Relay.h"
#include "stdbool.h"
#include "RobotControl.h"
#include "eepromstore.h"

//#include "FaultProcess.h"
//#include "EventLogger.h"
#include <stdio.h>
#include <string.h>

#define DEFAULT_OPERATION_MODE         AUTO

static eMode operationMode              = IDLE;
static eRobotCommand robotCommand       = ROBOT_CMD_IDLE;
static stModeCounts     modeCounts;
static stRobotPwmParam manualModePwmParam;
static stRobotPwmParam emergencyModePwmParam;
static uint32_t gotoCount = 0;
static volatile bool pauseDelayTimeOver = false;
static int32_t pauseDelay_timecount     = 0;
static eManualState curManualState      = MAN_IDLE;
static eMotionPauseState pauseState     = PA_IDLE;

static stRobotParam currentModeRobotParam21;

eRobotCommand prevRobotCommand;
eCycleModeState prevCycleModeState;
stRowLengthData rowLengthData;

static void AssignCommand (eRobotCommand command);

static void PauseDelayTimeIncrement_ms (void);
static void LoadManualMotionParameters (eRobotDirection direction);
static void LoadEmergencyMotionParameters (eRobotDirection direction);
static void PauseDelayTimerOn (uint32_t setPauseDelayTimeCount_ms);
static inline void PauseDelayTimerStop (void);
static inline bool IsPauseDelayTimeOver (void);
static inline void ClearPauseDelayTimeOver (void);
static void OperateMotionModes(void);
static void OperateManualMode(eManualState opManualState);
void OperateMotionPauseFSM (void);

static bool LinearEnabledState                  = true;
unsigned char Schedule_Start;

extern char Error_Cleared;
extern char Robot_Error_Count;
//char Robot_Error_Flag;

extern char Forward_End;
extern char Reverse_End;
extern int Cumulative_distance;

bool * GetSetLinearEnabledState (void)
{
  return(&LinearEnabledState);
}

bool IsLinearEnabled (void)
{
  return LinearEnabledState;
}

static void LoadManualMotionParameters (eRobotDirection direction)
{
  stRobotParam robotParameter;
  robotParameter.RobotPwmParam = manualModePwmParam;
  robotParameter.direction = direction;
  stCountRange * maxCountRange = GetCountRange();
  if(direction == POSITIVE)
  {
    robotParameter.count = maxCountRange->maxPosCountForPVsetup;
  }
  else
  {
    robotParameter.count = maxCountRange->maxNegCountForPVsetup;   
  }
  SetCurrentModeRobotParameter(&robotParameter);
}

static void LoadEmergencyMotionParameters (eRobotDirection direction)
{
  stRobotParam robotParameter;
  robotParameter.RobotPwmParam = emergencyModePwmParam;
  robotParameter.direction = direction;
  stCountRange * maxCountRange = GetCountRange();
  if(direction == POSITIVE)
  {
    robotParameter.count = maxCountRange->maxPosCountForPVsetup;
  }
  else
  {
    robotParameter.count = maxCountRange->maxNegCountForPVsetup;   
  }
  SetCurrentModeRobotParameter(&robotParameter);
}

void LoadRobotParameters(uint32_t count, stRobotPwmParam * modePwmParam) 
{
  stRobotParam robotParameter;
  robotParameter.RobotPwmParam = *modePwmParam;
  robotParameter.count = count;
  stCountRange * maxCountRange = GetCountRange();
  if(robotParameter.count > maxCountRange->maxPosCountForPVsetup)
  {
    robotParameter.count = maxCountRange->maxPosCountForPVsetup;
  }
  else if(robotParameter.count < maxCountRange->maxNegCountForPVsetup)
  {
    robotParameter.count = maxCountRange->maxNegCountForPVsetup;
  }
  SetCurrentModeRobotParameter(&robotParameter);
}

void SetManualPwmParameters (stRobotPwmParam *motionPwmParam)
{
  manualModePwmParam =  *motionPwmParam;
}

void SetEmergencyPwmParameters (stRobotPwmParam *motionPwmParam)
{
  emergencyModePwmParam =  *motionPwmParam;
}

stRobotPwmParam * GetSetManualPwmParameters (void)
{
  return(&manualModePwmParam);
}

stRobotPwmParam * GetSetEmergencyPwmParameters (void)
{
  return(&emergencyModePwmParam);
}

stModeCounts * GetSetModeCounts (void)
{
  return(&modeCounts);
}

void SetRowLengthData(stRowLengthData *rowData)
{
  rowLengthData = *rowData;
}

stRowLengthData *GetSetRowLengthData(void)
{
  return(&rowLengthData);
}

static void AssignCommand (eRobotCommand command)
{
  eMode getMode =  GetOperationMode ();
  switch(command)
  {
   case ROBOT_CMD_IDLE:
//     SetOperationMode(getMode, MAN_IDLE);
     SetOperationMode(DEFAULT_OPERATION_MODE, MAN_IDLE);                                   /* This is the Initial Operation Mode. */
    break;
   case ROBOT_CMD_NONE:
    break;
   case ROBOT_CMD_NORMAL_STOP:
//     SetOperationMode(getMode, MAN_NORMAL_STOP);
    SetOperationMode(MANUAL, MAN_NORMAL_STOP); 
    EepromSetMode(EEPROM_WRITE);
    break;
   case ROBOT_CMD_STOP:
//     SetOperationMode(getMode, MAN_STOP);
    SetOperationMode(MANUAL, MAN_STOP);
    EepromSetMode(EEPROM_WRITE); 
    break;
   case ROBOT_CMD_MANUAL:
     if(GetOperationMode()==AUTO)
     {
//       SetOperationMode(getMode, MAN_NORMAL_STOP);
       SetOperationMode(MANUAL, MAN_NORMAL_STOP);
     }
     else
     {
//      SetOperationMode(getMode, MAN_IDLE);
      SetOperationMode(MANUAL, MAN_IDLE);
     }
    break;
   case ROBOT_CMD_CYCLE:
//     SetOperationMode(getMode, MAN_CYCLE);
    SetOperationMode(MANUAL, MAN_CYCLE); 
    EepromSetMode(EEPROM_WRITE);
    break;
  case ROBOT_CMD_CYCLE_RESUME :
//    SetOperationMode(getMode, MAN_RESUME);
    SetOperationMode(MANUAL, MAN_RESUME);
   break;
   case ROBOT_CMD_AUTO:
    if(GetOperationMode() != AUTO)
    {
//      SetOperationMode(getMode, MAN_IDLE);
      SetOperationMode(AUTO, MAN_IDLE);
    }
    break;
   case ROBOT_CMD_POSITIVE:
     ClearRoboterror();
    SetOperationMode(MANUAL, MAN_POSITIVE );
    EepromSetMode(EEPROM_WRITE);
    break;
  case ROBOT_CMD_NEGATIVE:
     ClearRoboterror();
    SetOperationMode(MANUAL, MAN_NEGATIVE); 
    EepromSetMode(EEPROM_WRITE);
    break;
   case ROBOT_CMD_EMERGENCY_STOW:
    SetOperationMode(EMERGENCY_STOW, MAN_IDLE);
    break;
   case ROBOT_CMD_GOTO_COUNT:
    SetOperationMode(MANUAL, MAN_GOTO_COUNT);
    break;
   case ROBOT_CMD_CYCLE_TEST:
    SetOperationMode(CYCLE_TEST, MAN_IDLE);
    break;
   default:
    break;
  }
  SetMotionCommand(ROBOT_CMD_NONE);
}

void RobotOperate (void)
{
  eRobotCommand command = GetMotionCommand();
  stReturn *setReturnValue = GetSetReturnLimits();
//  eRobotDirection robotState = GetRobotActualDirection();
  
  AssignCommand(command);
  if((command != ROBOT_CMD_NONE) && (command != ROBOT_CMD_IDLE) || (GetOperationMode() ==  AUTO ))
  {
    //SetLogEvent (EV_LOG_MOT_CMD, (uint8_t)command); 
    if((Robot_Error_Count < setReturnValue->Return1) && GetCycleModeState() != CYCLE_COMPLETED)
    {
      ClearFaultsOnRobotCommand();
    }
    
  }
  //CheckCommunicationError();
  CheckError();
  OperateMotionModes();
  OperateMotionPauseFSM();
}

static void OperateMotionModes(void)
{ 
 //static eMotionPauseState prevPauseState = PA_IDLE;
 if(GetMotionPauseState() == PA_IDLE)
 {
   
  bool nonMotorErstatus = IsOnlyNonMotorError();
  if((GetErrorState() == ER_IDLE || GetErrorState() == ER_NO_ERROR) || nonMotorErstatus)
  {
    if(GetOperationMode() == AUTO)
    {
      
       if(IsSchedueledTime())
       {
          eCycleModeState curCycleModeState = GetCycleModeState();
           if((curCycleModeState == CYCLE_IDLE) || (curCycleModeState == CYCLE_COMPLETED))   
          {
            Cumulative_distance = 0;
            SetCycleModeState(CYCLE_START);
          }
          /* 07S */
          if(GetRelayStatus() != RELAY_ON)
          {
            DoInitialRelayOn();
            Schedule_Start = true;
          }
       }
       else 
       {
         if(Schedule_Start != true)
         {
            if(GetRelayStatus() != RELAY_OFF)
            {
               DoInitialRelayOff();
            }
         }
       }
          /* 07S */
       CycleModeFSM();
    }
    else if(GetOperationMode() == MANUAL)
    {
      /* 07S */
      if(GetRelayStatus() != RELAY_ON)
      {
       DoInitialRelayOn();
      }
      /* 07S */
      if((curManualState == MAN_CYCLE)||(curManualState == MAN_RESUME))
      {
        CycleModeFSM();
        //SetPrevPauseState(GetMotionPauseState());
      }
      else
      {
         /* Do Nothing for All other manual Modes */
      }
      Schedule_Start = false;
    }
    else if(GetOperationMode() == CYCLE_TEST)
    {
  //    CycleTestFSM();
    }
    else
    {
        /* Do Nothing in All other Modes */
    }
  }
 }
 else
 {
 //SetPrevPauseState(GetMotionPauseState());
 }
}
eMotionPauseState prevPauseState;

eMotionPauseState GetPrevPauseState(void)
{
 return prevPauseState;
}

void SetPrevPauseState(eMotionPauseState state)
{
prevPauseState = state;
}

static void OperateManualMode(eManualState opManualState)
{
  switch(opManualState)
  {
   case MAN_IDLE:
    break;
   case MAN_NORMAL_STOP:
    SetRobotState(ROBOT_NORMAL_STOP);   
    break;
   case MAN_STOP:
    SetRobotState(ROBOT_IMMEDIATE_STOP);                                                 /* Need abrupt Stop of the Motor    */
    break;
   case MAN_POSITIVE:
    SetRobotState(ROBOT_START);
    LoadManualMotionParameters(POSITIVE);
    break;
   case MAN_NEGATIVE:
    SetRobotState(ROBOT_START);
    LoadManualMotionParameters(NEGATIVE);
    break;
   case MAN_CYCLE:
    {
      eCycleModeState curCycleModeState = GetCycleModeState();
      if((curCycleModeState == CYCLE_IDLE) || (curCycleModeState == CYCLE_COMPLETED))   
      {
        SetCycleModeState(CYCLE_START);
      }
    }
    break;
   case MAN_GOTO_COUNT:
    {
      uint32_t count = GetGotoCount();
      SetRobotState(ROBOT_START);
      LoadRobotParameters(count, &manualModePwmParam);
    }
    break;
  case MAN_RESUME :
    {
    if((prevCycleModeState == CYCLE_WAIT_TILL_POS_END)||(prevCycleModeState == CYCLE_POS_START))
    {
      if(GetTargetCount())
       {
         SetRobotState(ROBOT_IMMEDIATE_STOP);
         SetCycleModeState(CYCLE_NEG_START);
         SetPrevPauseState(PA_IDLE);
          break;
       }
      else {
        SetCycleModeState(CYCLE_POS_START);    
      }
    }
    else if((prevCycleModeState == CYCLE_WAIT_TILL_NEG_END)||(prevCycleModeState == CYCLE_NEG_START))
    {
       if(GetTargetCount())
       {
         SetRobotState(ROBOT_IMMEDIATE_STOP);  
         SetPrevPauseState(PA_IDLE); 
          break;
       }
       else
       {
          SetCycleModeState(CYCLE_NEG_START);
       }    
    }
    }
    break;
   default:
    break;
  }
}

eMode GetOperationMode (void)
{
  return (operationMode);
}

void SetInitialOperationMode (eMode eepromReadMode)
{
  if(eepromReadMode == AUTO)
  {
    SetMotionCommand(ROBOT_CMD_AUTO);
  }
//  else if(eepromReadMode == MANUAL)
//  {
//    SetMotionCommand(ROBOT_CMD_MANUAL);
//  }
  else
  {
    SetMotionCommand(ROBOT_CMD_MANUAL);
  }     
}

eManualState GetManualState (void)
{
  return (curManualState);
}

void SetOperationMode (eMode setMode, eManualState opManualState)
{
 // SetLogEvent (EV_LOG_OP_MODE, (uint8_t)setMode);
  if((setMode == AUTO) && (IsRTCFault() == true))
    {
      setMode = MANUAL;
      opManualState = MAN_STOP;
    }
  operationMode = setMode; 
  curManualState = opManualState;
//  if(GetMotionPauseState() == PA_WAIT_FOR_TIMEOVER)
//  {
//    return;
//  }
  switch(setMode)
  {
   case IDLE:
    break;
   case MANUAL:
    OperateManualMode(opManualState);
    break;
   case EMERGENCY_STOW:
    SetCycleModeState(CYCLE_IDLE);
    SetRobotState(ROBOT_START);
    LoadEmergencyMotionParameters(NEGATIVE);
    break;
   case COMM_ERROR_EMERGENCY_STOW:
    SetCycleModeState(CYCLE_IDLE);
    SetRobotState(ROBOT_START);
    LoadEmergencyMotionParameters(NEGATIVE);
    break;
   case CYCLE_TEST:
    SetCycleModeState(CYCLE_IDLE);
  //  SetCycleTestState(CT_START);
    break;
    case AUTO:
     {
       SetCycleModeState(CYCLE_IDLE);
       SetRobotState(ROBOT_NORMAL_STOP); 
     }
    break;
   default:
    break;
  }
}

void SetMotionPauseState (eMotionPauseState pauseSt)
{
  pauseState = pauseSt;
}

eMotionPauseState GetMotionPauseState (void)
{
  return pauseState;
}

uint32_t prevRotateSense1Count = 0;
uint32_t prevRotateSense2Count = 0;
bool resumeBit = 0;

void SetResumeBit(bool value)
{
resumeBit = value;
}

bool GetResumeBit(void)
{
return resumeBit;
}

uint32_t GetPrevRotateSense1Count(void)
{
  return prevRotateSense1Count;
}

uint32_t GetPrevRotateSense2Count(void)
{
  return prevRotateSense2Count;
}
//#define PAUSE_TIMEOUT 70000//ms
void OperateMotionPauseFSM (void)
{
  eMotionPauseState pauseSt = GetMotionPauseState();
  eMode getMode =  GetOperationMode();    
  stReturn *setReturnValue = GetSetReturnLimits();
  
  switch (pauseSt)
  {
   case PA_IDLE:
    break;
    
    
   case PA_PAUSE:
    {
      prevRobotCommand = GetMotionCommand();
      prevCycleModeState = GetCycleModeState();
      
      if(getMode == MANUAL){
        SetMotionCommand(ROBOT_CMD_STOP);
      }
      else if(getMode == AUTO){
        SetRobotState(ROBOT_IMMEDIATE_STOP); 
      }
//      SetOperationMode(getMode, MAN_IDLE);
      SetMotionPauseState(PA_WAIT_FOR_STOP);
    }
    break;
  case PA_WAIT_FOR_STOP:
    {
      if(GetRobotState()==ROBOT_COMPLETED)
      {
        prevRotateSense1Count = GetRotateSense1Count();
        prevRotateSense2Count = GetRotateSense2Count();
//        stHeartbeatConfig* hbtConfig = GetSetHeartbeatConfig();
//        PauseDelayTimerOn(hbtConfig->maxReconnectTime);
        PauseDelayTimerOn(1000);
        
        if(Error_Count() <= setReturnValue->Return1)
        {
        SetMotionPauseState(PA_WAIT_FOR_TIMEOVER_RESUME);
        }
        
        if(Error_Count() > setReturnValue->Return1)
        {
            SetMotionPauseState(PA_WAIT_FOR_ERROR_CLEAR);
        }
      }
    }
      break;
     
  case PA_WAIT_FOR_ERROR_CLEAR:
       if(Error_Count() == 0)
       {
          SetMotionPauseState(PA_WAIT_FOR_TIMEOVER_RESUME);
          Error_Cleared = 0;
       }
    
        break;
   case PA_WAIT_FOR_TIMEOVER_RESUME:
    if(IsPauseDelayTimeOver() == true)
    {
      
    RestartCommTimer();
//      SetCycleModeState(CYCLE_COMPLETED);
      if(getMode == MANUAL){
        if(GetMotionCommand() == ROBOT_CMD_POSITIVE)
        {
            SetMotionCommand(ROBOT_CMD_POSITIVE);
        }
        else if(GetMotionCommand() == ROBOT_CMD_NEGATIVE)
        {
            SetMotionCommand(ROBOT_CMD_NEGATIVE);
        }
        else
        {
            SetMotionCommand(ROBOT_CMD_CYCLE_RESUME);
        }
      }
      else if(getMode == AUTO){
        if((prevCycleModeState == CYCLE_WAIT_TILL_POS_END)||(prevCycleModeState == CYCLE_POS_START))
        {
          if(GetTargetCount())
           {
             SetRobotState(ROBOT_IMMEDIATE_STOP);  
             SetCycleModeState(CYCLE_NEG_START);
             SetPrevPauseState(PA_IDLE);   
             SetMotionPauseState(PA_IDLE);
              break;
           }
          else {
            SetCycleModeState(CYCLE_POS_START);    
            }
        }
        else if((prevCycleModeState == CYCLE_WAIT_TILL_NEG_END)||(prevCycleModeState == CYCLE_NEG_START))
        {
             if(GetTargetCount())
             {
               SetRobotState(ROBOT_IMMEDIATE_STOP);
               SetPrevPauseState(PA_IDLE);  
               SetMotionPauseState(PA_IDLE);
               break;
             }
             else
             {
                SetCycleModeState(CYCLE_NEG_START);
             }
        }
      }
      SetPrevPauseState(GetMotionPauseState());
      SetMotionPauseState(PA_IDLE);
    }
//    if(GetResumeBit())
//    {
//      SetMotionCommand(ROBOT_CMD_CYCLE_RESUME);
//      ClearFault(COMMUNICATION_FAULT);
//      SetMotionPauseState(PA_IDLE);
//      RestartCommTimer();  
//      SetResumeBit(0);
//    }    
    break;
   default:
    break;
  }
}

void SetMotionCommand (eRobotCommand command)
{
  robotCommand = command;  
}

eRobotCommand GetMotionCommand (void)
{
  return(robotCommand);  
}

uint32_t GetGotoCount(void)
{
  return gotoCount;
}

bool SetGotoCount(uint32_t count)
{
  if(IsCountValid(count) == true)
  {
    gotoCount = count;
    return true;
  }
  else
  {
    return false;
  }
}

bool IsCountValid(uint32_t count)
{
  stCountRange *countRange = GetCountRange();
  if((count >=  countRange->maxNegCountForPVsetup) && (count <= countRange->maxPosCountForPVsetup))
  {
    return true;
  }
  else
  {
    return false;
  }
}

static void PauseDelayTimerOn (uint32_t setPauseDelayTimeCount_ms)
{
  if(setPauseDelayTimeCount_ms == 0)
  {
    pauseDelayTimeOver = true;
    PauseDelayTimerStop();
  }
  else
  {
    pauseDelay_timecount = setPauseDelayTimeCount_ms;
    pauseDelayTimeOver = false;
  }
}

static inline void PauseDelayTimerStop (void)
{
  pauseDelay_timecount = 0;
}

static inline bool IsPauseDelayTimeOver (void)
{
  return (pauseDelayTimeOver);
}

static inline void ClearPauseDelayTimeOver (void)
{
  pauseDelayTimeOver = false; 
}

static void PauseDelayTimeIncrement_ms (void)
{
  if(pauseDelay_timecount)
  {
    if(--pauseDelay_timecount <= 0)
    {
      pauseDelayTimeOver = true;
      PauseDelayTimerStop();
    }
    else
    {
      ClearPauseDelayTimeOver();
    }
  }
}

void RobotOperationTimeIncrement_ms (void)
{
    IloadMotMaxTimeIncrement_ms();
    PauseDelayTimeIncrement_ms();  
}