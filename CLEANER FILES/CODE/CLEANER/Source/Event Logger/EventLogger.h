#ifndef __EVENT_LOGGER_H_
#define __EVENT_LOGGER_H_

#include <stdint.h>
#include <stdbool.h>

//#include "EventLoggerFlash.h"

//#define MAX_NO_OF_EVENT_LOGS_REQUEST                  50

typedef enum eLogMemoryType_def
{
  RAM,
  EEPROM  
}eLogMemoryType;

typedef enum eLogEvents_def
 {
   EV_LOG_NO_EVENT,
   EV_LOG_STARTED,                                       /* Program Started           */
   EV_LOG_OP_MODE,
   EV_LOG_MOT_CMD,
   EV_LOG_PULSE_COUNT1,
   EV_LOG_PULSE_COUNT2,
   EV_LOG_RTC,
   EV_LOG_INCLINOMETER,
   EV_LOG_ADC,
   EV_LOG_EEPROM,                                        /* EEprom is Flash Memory used as EEPROM.   */
   EV_LOG_UART,
   EV_LOG_USB_UART,
   EV_LOG_ZB_UART,  
   EV_LOG_PAUSE_STATE,
   EV_LOG_MOT_STATE,
   EV_LOG_FAULT,
   EV_LOG_SPIFLASH,
   EV_LOG_RESET,
   EV_LOG_OTA
 }eLogEvents; 

typedef struct {
  eLogEvents logEvent;
  uint16_t logEventValue;
} stEventLog;

typedef enum eMotEvents_def
{
  EV_MOT_START = 1,
  EV_MOT_POSITIVE,
  EV_MOT_NEGATIVE,
  EV_MOT_ACCEL_START,
  EV_MOT_ACCEL_COMPLETE,
  EV_MOT_DECEL_START,
  EV_MOT_DECEL_COMPLETE,
  EV_MOT_STOP
}eMotEvents;

typedef enum eRtcEvents_def
{
  EV_RTC_STARTED,
  EV_RTC_ERROR,
  EV_RTC_TIME_UPDATED,
  EV_RTC_TIME_READ,
  EV_RTC_TIME_ERROR
}eRtcEvents; 

typedef enum eOtaEvents_def
{
  EV_OTA_STARTED,
  EV_OTA_ERROR,
  EV_OTA_SUCCESS
}eOtaEvents; 

typedef enum eIncEvents_def
{
  EV_INC_STARTED,
  EV_INC_ERROR,
  EV_INC_READ,
  EV_INC_READ_ERROR
}eIncEvents; 

typedef enum eAdcEvents_def
{
  EV_ADC_STARTED,
  EV_ADC_ERROR,
  EV_ADC_READ,
  EV_ADC_READ_ERROR
}eAdcEvents; 

typedef enum eMemEvents_def
{
  EV_MEM_STARTED,
  EV_MEM_ERROR,
  EV_MEM_READ_ERROR,
  EV_MEM_WRITE_ERROR,
  EV_MEM_READ,
  EV_MEM_WRITE,
}eMemEvents; 

typedef enum eSpiFlashEvents_def
{
  EV_SFLASH_STARTED,
  EV_SFLASH_ERROR,
  EV_SFLASH_READ_ERROR,
  EV_SFLASH_WRITE_ERROR,
  EV_SFLASH_READ,
  EV_SFLASH_WRITE,
}eSpiFlashEvents; 

typedef enum eUartEvents_def
{
  EV_UART_STARTED,
  EV_UART_ERROR,
  EV_UART_READ,
  EV_UART_READ_ERROR,
  EV_UART_WRITE,
  EV_UART_WRITE_ERROR
}eUartEvents; 


typedef enum eResetEvents_def
{
  EV_RESET_OPTIONS_LOADING,
  EV_RESET_PIN,
  EV_RESET_POR,
  EV_RESET_SOFTWARE,
  EV_RESET_IWDT,
  EV_RESET_WWDT,
  EV_RESET_LOW_POWER,
  EV_RESET_UNKNOWN
}eResetEvents; 

#ifdef __cplusplus
extern "C" {
#endif 
  
void EventLogInit (eLogMemoryType logMemoryType); 
void SetLogEvent (eLogEvents setLogEvent, uint16_t setLogEventValue);
void GetLogEvent (stEventLog **storedLogEvent, uint8_t *currentLogEventIndex);
void ClearAllLogData(void);

bool StoreInAllLogFlash(eLogEvents setLogEvent, uint16_t setLogEventValue);
uint32_t ReadFromLogFlash(uint8_t *logStoreData, uint16_t maxNoofReadLogs, uint16_t startIndex, uint16_t endIndex);
#ifdef __cplusplus
}
#endif

#endif