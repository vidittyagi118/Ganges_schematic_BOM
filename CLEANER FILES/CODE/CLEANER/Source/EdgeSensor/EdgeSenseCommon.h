#ifndef __EDGE_SENSE_COMMON_H_
#define __EDGE_SENSE_COMMON_H_

#include <stdbool.h>

bool EdgeSenseInit (void);

bool IsEdgeSensor1Detected (void);
bool IsEdgeSensor2Detected (void);
bool IsPositiveEdgeReached (void);
bool IsNegativeEdgeReached (void);
void EdgeSenseTimeIncrement_ms (void);
bool *GetSetEdgeSensorEnabledState(void);
bool IsEdgeSensorEnabled (void);

#endif