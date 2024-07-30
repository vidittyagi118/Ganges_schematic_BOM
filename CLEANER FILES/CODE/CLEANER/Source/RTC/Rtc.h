/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __RTC_H_
#define __RTC_H_

#include <stdint.h>
#include <stdbool.h>

#define SECONDS_IN_A_DAY (86400U)
#define SECONDS_IN_A_HOUR (3600U)
#define SECONDS_IN_A_MINUTE (60U)

typedef struct 
{
  uint16_t year;  /*!< Range from 1970 to 2099.*/
  uint8_t month;  /*!< Range from 1 to 12.*/
  uint8_t date;    /*!< Range from 1 to 31 (depending on month).*/
  uint8_t hour;   /*!< Range from 0 to 23.*/
  uint8_t minute; /*!< Range from 0 to 59.*/
  uint8_t second; /*!< Range from 0 to 59.*/
}sDateTime;

typedef struct 
{
  uint16_t year;  /*!< Range from 1970 to 2099.*/
  uint8_t month;  /*!< Range from 1 to 12.*/
  uint8_t date;    /*!< Range from 1 to 31 (depending on month).*/
  uint8_t hour;   /*!< Range from 0 to 23.*/
  uint8_t minute; /*!< Range from 0 to 59.*/
  uint8_t second; /*!< Range from 0 to 59.*/
}sd_DateTime;
  
  
bool RTCInit (void);
void GetRTCDateTime (sDateTime * dateTime);
bool SetRTCDateTime (sDateTime * dateTime);
uint32_t GetRTCDateTimeSeconds (void);
bool CheckRTCDateTimeFormat(const sDateTime *dateTime);
bool SetRTCDefaultDateTime (void);
void GetRTCDateTimeString(char * rtcString, uint8_t rtcStringLength);
void ConvertRTCDateTimeToString(const sDateTime *dateTime, char * rtcString, uint8_t rtcStringLength);
void ConvertRTCSecondsToDatetime(uint32_t seconds, sDateTime *datetime);
uint32_t ConvertRTCDatetimeToSeconds(const sDateTime *datetime);
uint32_t ConvertRTCDateToSeconds(sDateTime * datetime);
uint32_t ConvertRTCTimeToSeconds(sDateTime * datetime);
bool IsDateTimeUpdated (void);
void ClearDateTimeUpdated (void);
bool IsRTCSecondOccured (void);
void ClearRTCSecondOccured (void);
bool IsRTCFreshStart (void);
#endif