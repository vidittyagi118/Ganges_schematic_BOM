#include "RotateSense1.h"
#include "RotateSense2.h"
#include "RobotOperation.h"

#define COUNTSENSOR     ROTATESENSOR_1          // ROTATESENSOR_2

static bool RotateSensorEnabledState = true;

bool RotateSenseInit (void)
{
  bool status = true;
  status &= RotateSense1Init();
  status &= RotateSense2Init();
  return status;  
}

bool *GetSetRotateSensorEnabledState(void)
{
  return(&RotateSensorEnabledState);
}

bool IsRotateSensorEnabled (void)
{
  return RotateSensorEnabledState;
}


void SetRotateSenseCount (uint32_t count)
{
  SetRotateSense1Count(count);
  SetRotateSense2Count(count);  
}

void ClearRotateSenseCount (void)
{
  ClearRotateSense1Count();
  ClearRotateSense2Count();
}

void StartRotateSenseCount (void)
{
  StartRotateSense1Count();
  StartRotateSense2Count(); 
  SetPrevPauseState(GetMotionPauseState());
}

void ResumeRotateSenseCount (void)
{
  ResumeRotateSense1Count();
  ResumeRotateSense2Count();
}

void StopRotateSenseCount (void)
{
  StopRotateSense1Count();
  StopRotateSense2Count();
}

uint32_t GetRotateSenseCount (void)
{
#if COUNTSENSOR == ROTATESENSOR_1
  return GetRotateSense1CountValue();  
#else
  return GetRotateSense1CountValue(); 
#endif
}

uint32_t GetRotateSense1Count (void)
{
  return GetRotateSense1CountValue();  
}

uint32_t GetRotateSense2Count (void)
{
  return GetRotateSense2CountValue();    
}

void RotateSenseTimeIncrement_ms (void)
{
  RotateSense1TimeIncrement_ms();
  RotateSense2TimeIncrement_ms();
}