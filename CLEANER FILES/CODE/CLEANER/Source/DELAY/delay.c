
#include "board.h"
#include "delay.h"

static volatile uint64_t delayMillisecondTime = 0;
static uint64_t GetDelayMsTimeValue(void);

void msdelay (uint16_t msdelay_time)
{
    volatile uint64_t delayTime = delayMillisecondTime;
    while((GetDelayMsTimeValue() - delayTime)  < msdelay_time);
}

void DelayMsTimerhandler(void)
{
  delayMillisecondTime++;
}

uint64_t GetDelayMsTimeValue(void)
{
  return delayMillisecondTime;
}