#include "EventLogger.h"
#include "EventLoggerFlash.h"
#include "Rtc.h"
#include "Serial_Debug.h"
#include <stdio.h>

#define MAX_NO_OF_EVENT_LOGS_RAM                            80
#define SINGLE_STORE_BYTE_COUNT                8                                   /*  Always have the this as Even Number. As we use continuous read and Write , most operations happens as two bytes(Even First and Odd Next) */

static stEventLog logEventData[MAX_NO_OF_EVENT_LOGS_RAM] = {EV_LOG_NO_EVENT, 0};
static uint8_t logEventIndex = 0;

static bool logMemoryType = RAM;                                          

bool StoreInAllLogFlash(eLogEvents setLogEvent, uint16_t setLogEventValue);
bool StoreAllCurrentLogsInEeprom(void);

void SetLogEvent (eLogEvents setLogEvent, uint16_t setLogEventValue)
{
  if(logMemoryType == RAM)
  {
    if(logEventIndex >= MAX_NO_OF_EVENT_LOGS_RAM)
    {
      logEventIndex = 0;
    }
    logEventData[logEventIndex].logEvent = setLogEvent;
    logEventData[logEventIndex].logEventValue = setLogEventValue;
    logEventIndex++;
  }
  else
  {
    (void)StoreInAllLogFlash(setLogEvent, setLogEventValue);
  }
}

void EventLogInit (eLogMemoryType logMemorySetType)
{
  logMemoryType = logMemorySetType;
  if(logMemoryType == RAM)
  {
    for(uint8_t log_index =0; log_index < MAX_NO_OF_EVENT_LOGS_RAM; log_index++)
    {
      logEventData[log_index].logEvent = EV_LOG_NO_EVENT;
    }
    logEventIndex = 0;
    //SetLogEvent(EV_LOG_STARTED, RAM);
  }
  else
  {
    EventLoggerAllLogFlash_Init();
    if(logEventIndex != 0)
    {
      StoreAllCurrentLogsInEeprom();                                            // Ehen EEPROM Available move all RAM stored data to EEPROM.
    }
    //SetLogEvent(EV_LOG_STARTED, EEPROM);    
  }

}

void GetLogEvent (stEventLog **storedLogEvent, uint8_t *currentLogEventIndex)
{
  *storedLogEvent = logEventData;
  *currentLogEventIndex = logEventIndex;  
}

bool StoreAllCurrentLogsInEeprom(void)
{
  for(uint16_t logIndex = 0;  logIndex<logEventIndex; logIndex++)
  {
    StoreInAllLogFlash(logEventData[logIndex].logEvent, logEventData[logIndex].logEventValue);  
  }  
  return true;
}
                                 

bool StoreInAllLogFlash(eLogEvents setLogEvent, uint16_t setLogEventValue)
{
  
    uint8_t logStoreData[SINGLE_STORE_BYTE_COUNT];
    uint32_t utcTime = GetRTCDateTimeSeconds();
    logStoreData[0] = (utcTime >> 24)& 0xFF;
    logStoreData[1] = (utcTime >> 16)& 0xFF;
    logStoreData[2] = (utcTime >> 8)& 0xFF;
    logStoreData[3] =  utcTime & 0xFF;
    logStoreData[4] = (uint8_t)setLogEvent;
    logStoreData[5] = (setLogEventValue>>8)& 0xFF;
    logStoreData[6] = (setLogEventValue)& 0xFF;
    if(StoreFlashAllLog(logStoreData, sizeof logStoreData) == true)
    {
      Serial_Debug("\r\nTime->");
      Serial_Debug_Num(utcTime);
      Serial_Debug("\r\nData->");
      for(int i=0;i<7;i++)
      {
      Serial_Debug_Num(logStoreData[i]);
      Serial_Debug(" ");
      }
      Serial_Debug("\n Event Log Flash Stored");
      return true;
    }
    else
    {
      Serial_Debug("\n Event Log Flash Store Error ");
      return false;
    }
}

uint32_t ReadFromLogFlash(uint8_t *logStoreData, uint16_t maxNoofReadLogs, uint16_t startIndex, uint16_t endIndex)
{
  uint32_t noOfReadData = ReadFlashAllLog(logStoreData, maxNoofReadLogs, startIndex, endIndex);
  return noOfReadData;
}

void ClearAllLogData(void)
{
  ClearFlashAllLog();  
}
    