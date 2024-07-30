#include "EdgeSenseCommon.h"
#include "EdgeSense1.h"
#include "EdgeSense2.h"

static bool edgeSensorEnabledState = true;

bool EdgeSenseInit (void)
{
  bool status = true;
  status &= EdgeSense1Init();
  status &= EdgeSense2Init();  
  return status;
}

bool *GetSetEdgeSensorEnabledState(void)
{
  return(&edgeSensorEnabledState);
}

bool IsEdgeSensorEnabled (void)
{
  return edgeSensorEnabledState;
}

bool IsEdgeSensor1Detected (void)
{
  if(IsEdgeSensorEnabled())
  {
    return IsEdgeSense1Detected(); 
  }
  else 
  {
    return false;
  }
}

bool IsEdgeSensor2Detected (void)
{
  if(IsEdgeSensorEnabled())
  {
    return IsEdgeSense2Detected(); 
  }
  else 
  {
    return false;
  }  
}

bool IsPositiveEdgeReached (void)
{
  return IsEdgeSensor1Detected();
}

bool IsNegativeEdgeReached (void)
{
   return IsEdgeSensor2Detected(); 
}

void EdgeSenseTimeIncrement_ms (void)
{
  EdgeSense1TimeIncrement_ms();
  EdgeSense2TimeIncrement_ms();  
}