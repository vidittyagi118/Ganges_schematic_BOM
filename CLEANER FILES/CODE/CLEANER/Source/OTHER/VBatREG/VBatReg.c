#include "VBatReg.h"
#include "board.h"

static volatile uint32_t * QCountRegPoint = &RFVBAT->REG[0];
static volatile uint32_t * RTCInvalidTimeRegPoint = &RFVBAT->REG[1];

volatile uint32_t * GetQCountStoreRegPointer (void)
{
  return QCountRegPoint;  
}

volatile uint32_t * GetRTCInvalidClockRegPointer (void)
{
  return RTCInvalidTimeRegPoint;  
}

