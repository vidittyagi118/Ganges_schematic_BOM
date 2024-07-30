#ifndef _MOT2_CURRENT_FAULT_H_
#define _MOT2_CURRENT_FAULT_H_

#include <stdint.h>
#include <stdbool.h>

void IloadMot2MaxTimeIncrement_ms (void);
bool IsMot2OverCurrentFault (void);
void ClearMot2OverCurrentFault (void);
void CheckMot2OverLoad (void);
void ClearMot2MaxOverCurrentCount (void);

#endif