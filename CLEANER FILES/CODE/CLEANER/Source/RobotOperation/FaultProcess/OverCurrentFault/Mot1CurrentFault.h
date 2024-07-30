#ifndef _MOT1_CURRENT_FAULT_H_
#define _MOT1_CURRENT_FAULT_H_

#include <stdint.h>
#include <stdbool.h>

void IloadMot1MaxTimeIncrement_ms (void);
bool IsMot1OverCurrentFault (void);
void ClearMot1OverCurrentFault (void);
void CheckMot1OverLoad (void);
void ClearMot1MaxOverCurrentCount (void);

#endif