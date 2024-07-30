#ifndef __VBAT_REG_H_
#define __VBAT_REG_H_

#include <stdint.h>
#include <stdbool.h>

volatile uint32_t * GetQCountStoreRegPointer (void);
volatile uint32_t * GetRTCInvalidClockRegPointer (void);

#endif