#include "MotCurrentFaultCommon.h"
#include "Mot1CurrentFault.h"
#include "Mot2CurrentFault.h"
#include "Mot3CurrentFault.h"
#include "adc.h"
#include "Serial_Debug.h"

static stMaxIloadConfigValues maxIloadConfigValues;
static stMaxMotorCurrent maxMotorCurrentLimits;

static stWheelDia maxWheelDiaLimits;   //RT
static stCycleFrequency maxCycleFrequencyLimits; 
static stInterval maxIntervalLimits;
static stContinue maxContinueLimits;
static stReturn maxReturnLimits;
static stcomdistance maxComDistanceLimits;

static stnoofrows maxNoofrowsLimits;


stMaxIloadConfigValues* GetSetMaxIloadConfigValues(void)
{
  return(&maxIloadConfigValues);  
}

void GetMaxIloadConfigValues(stMaxIloadConfigValues * configValues)
{
  *configValues = maxIloadConfigValues;  
}

stMaxMotorCurrent* GetSetMaxMotorCurrentLimits(void)
{
  return(&maxMotorCurrentLimits);
}

void GetMaxMotorCurrentLimits(stMaxMotorCurrent * motCurLimits)
{
  *motCurLimits = maxMotorCurrentLimits;
}

//RT

stWheelDia* GetSetwheelDiaLimits(void)
{
  return(&maxWheelDiaLimits);
}

void GetWheelDiaLimits(stWheelDia * wheeldiaLimits)
{
  *wheeldiaLimits = maxWheelDiaLimits;
}


stCycleFrequency* GetSetcycleFrequencyLimits(void)
{
  return(&maxCycleFrequencyLimits);
}

void GetCycleFrequencyLimits(stCycleFrequency * CycleFrequencyLimits)
{
  *CycleFrequencyLimits = maxCycleFrequencyLimits;
}


stInterval* GetSetIntervalLimits(void)
{
  return(&maxIntervalLimits);
}

void GetIntervalLimits(stInterval * IntervalLimits)
{
  *IntervalLimits = maxIntervalLimits;
}


stContinue* GetSetContinueLimits(void)
{
  return(&maxContinueLimits);
}

void GetContinueLimits(stContinue * ContinueLimits)
{
  *ContinueLimits = maxContinueLimits;
}

stReturn* GetSetReturnLimits(void)
{
  return(&maxReturnLimits);
}

void GetReturnLimits(stReturn * ReturnLimits)
{
  *ReturnLimits = maxReturnLimits;
}

stcomdistance* GetSetComDistanceLimits(void)
{
  return(&maxComDistanceLimits);
}

void GetComDistanceLimits(stcomdistance * ComDistanceLimits)
{
  *ComDistanceLimits = maxComDistanceLimits;
}


bool IsMotorOverCurrentFault (void)
{
  bool status = true;
  status &= IsMot1OverCurrentFault();
  status &= IsMot2OverCurrentFault();
  status &= IsMot3OverCurrentFault();
  return status;  
}

void ClearMotorOverCurrentFault (void)
{
  ClearMotor1OverCurrentFault();
  ClearMotor2OverCurrentFault();
  ClearMotor3OverCurrentFault();  
}

bool IsMotor1OverCurrentFault (void)
{
  return IsMot1OverCurrentFault();
}

bool IsMotor2OverCurrentFault (void)
{
  return IsMot2OverCurrentFault();
}

bool IsMotor3OverCurrentFault (void)
{
  return IsMot3OverCurrentFault();
}

void ClearMotor1OverCurrentFault (void)
{
  ClearMot1OverCurrentFault();  
  ClearMot1MaxOverCurrentCount();
}

void ClearMotor2OverCurrentFault (void)
{
  ClearMot2OverCurrentFault();  
  ClearMot2MaxOverCurrentCount();
}

void ClearMotor3OverCurrentFault (void)
{
  ClearMot3OverCurrentFault();  
  ClearMot3MaxOverCurrentCount();
}
  
void CheckMotorOverLoad (void)
{
  CheckADCValues();
  CheckMot1OverLoad();
  CheckMot2OverLoad();
  CheckMot3OverLoad();
}

void CheckADCValues (void)
{
  CheckADC();  
}

void IloadMotMaxTimeIncrement_ms (void)
{
  IloadMot1MaxTimeIncrement_ms();
  IloadMot2MaxTimeIncrement_ms();
  IloadMot3MaxTimeIncrement_ms();
}

void FindAndUpdateImotOffsetValue (void)
{
  CheckADCValues();
  CheckADCValues();
  CheckADCValues();  
  UpdateImotOffsetValue();  
}

stnoofrows* GetSetNoofrowsLimits(void)
{
  return(&maxNoofrowsLimits);
}

void GetNoofrowsLimits(stnoofrows * NoofrowsLimits)
{
  *NoofrowsLimits = maxNoofrowsLimits;
}
