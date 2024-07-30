#include "Mot1CurrentFault.h"
#include "MotCurrentFaultCommon.h"
#include "RobotOperation.h"
#include "adc.h"
#include "Serial_Debug.h"

static volatile bool iloadTimeOver = false;
static volatile int32_t iloadmax_timecount = 0;
static volatile bool IloadMaxTimeOnFlag = false;
static volatile bool maxIloadFreqTimeOver = false;
static uint32_t ImotOverCurrentCount = 0;
static int32_t maxIloadFreq_timecount = 0;

static volatile bool mot1OverCurrentFault = false;

static void SetMot1OverCurrentFault (void);
static void IloadMaxTimerOn (uint32_t setMotorTimeCount_ms);
static inline void IloadMaxTimerStop (void);
static inline bool IsIloadMaxTimeOver (void);
static inline void ClearIloadMaxTimeOver (void);
static void ClearIloadMaxOverCount(void);
static inline bool IsIloadMaxTimeOn (void);
static void MaxIloadFreqTimerOn (uint32_t setMaxIloadFreqTimeCount_ms);
static inline void MaxIloadFreqTimerStop (void);
static inline bool IsMaxIloadFreqTimeOver (void);
static inline void ClearMaxIloadFreqTimeOver (void);
static void MaxIloadFreqTimeIncrement_ms (void);

bool IsMot1OverCurrentFault (void)
{
 return mot1OverCurrentFault;
}

void ClearMot1OverCurrentFault (void)
{
  mot1OverCurrentFault = false;
}

void SetMot1OverCurrentFault (void)
{
  mot1OverCurrentFault = true;
}

void ClearMot1MaxOverCurrentCount (void)
{
  ClearIloadMaxOverCount();
}

void CheckMot1OverLoad (void)
{
  float mot1Iload = GetImot1Value();
  stMaxIloadConfigValues maxIloadConfigValues;
  GetMaxIloadConfigValues(&maxIloadConfigValues);
  stMaxMotorCurrent motCurLimits;
  GetMaxMotorCurrentLimits(&motCurLimits);
  float mot1IloadLimit = motCurLimits.Imot1;
  if(GetMotionPauseState() == PA_IDLE)
  {
    if(mot1Iload >= mot1IloadLimit)
    {
      if(IsMot1OverCurrentFault() == false)
      {  
        if(IsIloadMaxTimeOn() == false)
        {
          IloadMaxTimerOn (maxIloadConfigValues.maxIloadNoiseTimeValue);
        }
        else
        {
          if(IsIloadMaxTimeOver() == true)
          {
            IloadMaxTimerStop();
            IloadMaxTimeOnFlag = false;
            if(++ImotOverCurrentCount >= maxIloadConfigValues.maxIloadRepeatCountValue)
            {
              Serial_Debug("\n*** Mot1 Over Current Detected. Iload: *****");
              Serial_Debug_Float(mot1Iload,2);
              eMode opMode =  GetOperationMode();
    //      if(opMode != MANUAL)
    //      {
    //        SetOperationMode(opMode, MAN_IDLE);
    //      }
              
//              SetOperationMode(MANUAL, MAN_STOP);                //  COMM
              SetMot1OverCurrentFault();
//              SetMotionPauseState(PA_IDLE);
              ImotOverCurrentCount = 0;
              MaxIloadFreqTimerStop();
            }
            else
            {
              Serial_Debug("\n *** <Mot1 Over Current Detected - Paused.");
//              SetMotionPauseState(PA_PAUSE); 
              MaxIloadFreqTimerOn(maxIloadConfigValues.maxIloadFreqTimeValue);
            }
          }
        }
      }
    }
    else
    {
      IloadMaxTimerStop();
      IloadMaxTimeOnFlag = false;
    }
    if(IsMaxIloadFreqTimeOver() == true)
    {
       ClearIloadMaxOverCount();
    }
  }
}

static void ClearIloadMaxOverCount (void)
{
  IloadMaxTimerStop();
  ImotOverCurrentCount = 0;
  IloadMaxTimeOnFlag = false;
//  SetMotionPauseState(PA_IDLE);
  MaxIloadFreqTimerStop();
  ClearMaxIloadFreqTimeOver();
}

static void IloadMaxTimerOn (uint32_t setIloadMaxTimeCount_ms)
{
  if(setIloadMaxTimeCount_ms == 0)
  {
    iloadTimeOver = true;
    IloadMaxTimerStop();
  }
  else
  {
    iloadmax_timecount = setIloadMaxTimeCount_ms;
    IloadMaxTimeOnFlag = true;
    iloadTimeOver = false;
  }
}

static bool IsIloadMaxTimeOn (void)
{
  return(IloadMaxTimeOnFlag);
}

inline void IloadMaxTimerStop (void)
{
  iloadmax_timecount = 0;
}

static inline bool IsIloadMaxTimeOver (void)
{
  return (iloadTimeOver);
}

static inline void ClearIloadMaxTimeOver (void)
{
  iloadTimeOver = false; 
}

void IloadMot1MaxTimeIncrement_ms (void)
{
  if(iloadmax_timecount)
  {
    if(--iloadmax_timecount <= 0)
    {
      iloadTimeOver = true;
      IloadMaxTimerStop();
    }
    else
    {
      ClearIloadMaxTimeOver();
    }
  }
  MaxIloadFreqTimeIncrement_ms();
}

static void MaxIloadFreqTimerOn (uint32_t setMaxIloadFreqTimeCount_ms)
{
  if(setMaxIloadFreqTimeCount_ms == 0)
  {
    maxIloadFreqTimeOver = true;
    MaxIloadFreqTimerStop();
  }
  else
  {
    maxIloadFreq_timecount = setMaxIloadFreqTimeCount_ms;
    maxIloadFreqTimeOver = false;
  }
}

static inline void MaxIloadFreqTimerStop (void)
{
  maxIloadFreq_timecount = 0;
}

static inline bool IsMaxIloadFreqTimeOver (void)
{
  return (maxIloadFreqTimeOver);
}

static inline void ClearMaxIloadFreqTimeOver (void)
{
  maxIloadFreqTimeOver = false; 
}

static void MaxIloadFreqTimeIncrement_ms (void)
{
  if(maxIloadFreq_timecount)
  {
    if(--maxIloadFreq_timecount <= 0)
    {
      maxIloadFreqTimeOver = true;
      MaxIloadFreqTimerStop();
    }
    else
    {
      ClearMaxIloadFreqTimeOver();
    }
  }
}