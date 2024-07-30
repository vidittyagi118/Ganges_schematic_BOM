#ifndef _STORE_SOC_H_
#define _STORE_SOC_H_

#include <stdio.h>
#include <stdint.h>

#define TIME_TO_UPDATE  3 //secs

void StoreQC(uint32_t value);
uint32_t GetStoredQC(void);
void CheckAndStoreQC(void);
uint16_t GetQCountFromVBat(void);

#endif