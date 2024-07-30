#ifndef _ADC_H_
#define _ADC_H_

#include <stdint.h>
#include <stdbool.h>

bool ADCInit (void);
void CheckADC (void);
float GetImot1Value (void);
float GetImot2Value (void);
float GetImot3Value (void);
void UpdateImotOffsetValue (void);
#endif