#ifndef __AUTO_SCHEDULE_H_
#define __AUTO_SCHEDULE_H_

#include <stdint.h>
#include <stdbool.h>

typedef struct{
  uint8_t hour;
  uint8_t minute;
  uint8_t HOUR[30];
  uint8_t MINUTE[30];
}stScheduleTime;

//typedef struct{
//  char Cycle;
//}stCycleFrequency;


void ClearAutoDoneSeconds(void);
bool IsSchedueledTime(void);
uint32_t GetScheduledTimeSeconds (void);
void SetScheduleTime(stScheduleTime *schdTime);
stScheduleTime* GetSetScheduleTime(void);

//static stCycleFrequency maxCycleFrequencyLimits;
void Increment_Done_Schedule(void);
char Get_Remaining_Schedule();
char Get_Completed_Schedule();
//
//stCycleFrequency* GetSetcycleFrequencyLimits(void);
//void GetCycleFrequencyLimits(stCycleFrequency * CycleFrequencyLimits);

#endif