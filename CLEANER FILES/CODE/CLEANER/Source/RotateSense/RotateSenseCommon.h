#ifndef __ROTATE_SENSE_COMMON_H_
#define __ROTATE_SENSE_COMMON_H_

#include <stdbool.h>
#include <stdint.h>

bool RotateSenseInit (void);

uint32_t GetRotateSenseCount (void);
uint32_t GetRotateSense1Count (void);
uint32_t GetRotateSense2Count (void);
void SetRotateSenseCount (uint32_t count);
void ClearRotateSenseCount (void);
void StartRotateSenseCount (void);
void ResumeRotateSenseCount (void);
void StopRotateSenseCount (void);
bool IsRotateSensorEnabled (void);
bool *GetSetRotateSensorEnabledState(void);
void RotateSenseTimeIncrement_ms (void);


#endif