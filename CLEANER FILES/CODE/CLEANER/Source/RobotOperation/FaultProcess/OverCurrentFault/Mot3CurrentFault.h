#ifndef _MOT3_CURRENT_FAULT_H_
#define _MOT3_CURRENT_FAULT_H_

#include <stdint.h>
#include <stdbool.h>

void IloadMot3MaxTimeIncrement_ms (void);
bool IsMot3OverCurrentFault (void);
void ClearMot3OverCurrentFault (void);
void CheckMot3OverLoad (void);
void ClearMot3MaxOverCurrentCount (void);

#endif