#ifndef _MOT_CURRENT_FAULT_COMMON_H_
#define _MOT_CURRENT_FAULT_COMMON_H_

#include <stdint.h>
#include <stdbool.h>

typedef struct
{
  uint32_t maxIloadNoiseTimeValue;
  uint32_t maxIloadFreqTimeValue; 
  uint32_t maxIloadRepeatCountValue; 
  uint32_t motorPauseDelay;
}stMaxIloadConfigValues;

typedef struct{
  float Imot1;
  float Imot2;
  float Imot3;
}stMaxMotorCurrent;
                                       //07D
typedef struct{
  float Dia;
  float Pulse;
  uint32_t Speed;
}stWheelDia;

typedef struct{
  float Cycle;
}stCycleFrequency;

typedef struct{
  uint32_t I1,I2,I3,I4,I5,I6,I7;
  uint32_t P1,P2,P3,P4,P5,P6,P7;
}stInterval;

typedef struct{
  uint32_t Continue1;
  uint32_t Count1;
}stContinue;

typedef struct{
  uint32_t Return1;
}stReturn;

typedef struct {
  uint32_t CDistance;
} stcomdistance;

typedef struct{
  uint32_t Row[7],TRow;
}stnoofrows;

stMaxMotorCurrent* GetSetMaxMotorCurrentLimits (void);
void GetMaxMotorCurrentLimits(stMaxMotorCurrent * motCurLimits);

stMaxIloadConfigValues *GetSetMaxIloadConfigValues (void);
void GetMaxIloadConfigValues(stMaxIloadConfigValues * configValues);

//07D
stWheelDia* GetSetwheelDiaLimits(void);
void GetWheelDiaLimits(stWheelDia * wheeldiaLimits);

stCycleFrequency* GetSetcycleFrequencyLimits(void);
void GetCycleFrequencyLimits(stCycleFrequency * CycleFrequencyLimits);

stInterval* GetSetIntervalLimits(void);
void GetIntervalLimits(stInterval * IntervalLimits);

stContinue* GetSetContinueLimits(void);
void GetContinueLimits(stContinue * ContinueLimits);

stReturn* GetSetReturnLimits(void);
void GetReturnLimits(stReturn * ReturnLimits);

stcomdistance* GetSetComDistanceLimits(void);
void GetComDistanceLimits(stcomdistance * ComDistanceLimits);

stnoofrows* GetSetNoofrowsLimits(void);
void GetNoofrowsLimits(stnoofrows * NoofrowsLimits); 

void CheckMotorOverLoad (void);
void CheckADCValues (void);
void FindAndUpdateImotOffsetValue (void);
bool IsMotorOverCurrentFault (void);
void ClearMotorOverCurrentFault (void);
void IloadMotMaxTimeIncrement_ms (void);

bool IsMotor1OverCurrentFault (void);
bool IsMotor2OverCurrentFault (void);
bool IsMotor3OverCurrentFault (void);

void ClearMotor1OverCurrentFault (void);
void ClearMotor2OverCurrentFault (void);
void ClearMotor3OverCurrentFault (void);
#endif