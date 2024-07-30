#include "Rtc.h"
#include "Serial_Debug.h"
#include "fsl_common.h"
#include "fsl_rtc.h"
#include "VBatReg.h"

#include <stdio.h>

#define DEFAULT_RTC_YEAR                        2022U
#define DEFAULT_RTC_MONTH                       01U
#define DEFAULT_RTC_DAY                         01U
#define DEFAULT_RTC_HOUR                        01U
#define DEFAULT_RTC_MINUTE                      01U
#define DEFAULT_RTC_SECOND                      01U

#define INVALID_CLOCK_VALUE                     0x1010

static void BOARD_SetRtcClockSource(void);
static void SetDateTimeUpdated(void);
static bool checkRTCTimeValid (void) ;
static bool IsRTCInvalidClock (void);
static void StoreRTCInvalidClock (void);
static void ClearRTCInvalidClock (void);

static bool dateTimeUpdatedFlag = false;
static bool rtcSecondsUpdatedFlag = false;

static bool RTCFreshStartFlag = true;

static void BOARD_SetRtcClockSource(void)
{
    /* Enable the RTC 32KHz oscillator */
    RTC->CR |= RTC_CR_OSCE_MASK;
    RTC->CR &= (uint32_t)~(uint32_t)RTC_CR_CLKO_MASK;                           // Ensemble added....
}

void RTC_Seconds_IRQHandler(void)
{
   rtcSecondsUpdatedFlag = true;
}

bool IsRTCSecondOccured (void)
{
  return rtcSecondsUpdatedFlag;
}

void ClearRTCSecondOccured (void)
{
  rtcSecondsUpdatedFlag = false;  
}

bool IsRTCFreshStart (void)
{
  return (RTCFreshStartFlag == true);  
}

static void StoreRTCInvalidClock (void)
{
  volatile uint32_t * rtcInvalidValue = GetRTCInvalidClockRegPointer();
 *rtcInvalidValue = INVALID_CLOCK_VALUE;
}

static void ClearRTCInvalidClock (void)
{
  volatile uint32_t * rtcInvalidValue = GetRTCInvalidClockRegPointer();
 *rtcInvalidValue = 0xFFFF;
}

static bool IsRTCInvalidClock (void)
{
  volatile uint32_t * rtcInvalidValue = GetRTCInvalidClockRegPointer();
  return (*rtcInvalidValue == INVALID_CLOCK_VALUE);
}

bool RTCInit (void)
{
  bool status = false;
  rtc_config_t rtcConfig;
  /*
  * rtcConfig.wakeupSelect = false;
  * rtcConfig.updateMode = false;
  * rtcConfig.supervisorAccess = false;
  * rtcConfig.compensationInterval = 0;
  * rtcConfig.compensationTime = 0;
  */
  CLOCK_EnableClock(kCLOCK_Rtc0);
      
  RTC_SetOscCapLoad(RTC, 0);                                                  // Ensemble Added...
      
  if(checkRTCTimeValid() != true)                  // Ensemble created.. when RTC is valid .. no need to set time again...
  {
    RTC_GetDefaultConfig(&rtcConfig);
    RTC_Init(RTC, &rtcConfig);
    BOARD_SetRtcClockSource();
    SetRTCDefaultDateTime();
    ClearDateTimeUpdated();
    status = false;
    RTCFreshStartFlag = true;
  }
  else
  {
    
    if(IsRTCInvalidClock())
    {
      SetRTCDefaultDateTime();
      ClearDateTimeUpdated();
      status = false;
    }
    else
    {
      status = true;
    }
    RTC_StartTimer(RTC);
//  EnableIRQ(RTC_IRQn);
    EnableIRQ(RTC_Seconds_IRQn);
    RTCFreshStartFlag = false;
  }
  RTC_EnableInterrupts(RTC, kRTC_SecondsInterruptEnable); 
  if(status == false)
  {
    StoreRTCInvalidClock();
  }
  return status;
}

static bool checkRTCTimeValid (void)    
{ 
  if((RTC_GetStatusFlags(RTC) & kRTC_TimeInvalidFlag))
  {
      return(false);
  }
  else
  {
    return (true);
  }
}

void GetRTCDateTime (sDateTime * dateTime)
{
  rtc_datetime_t date_time;
  RTC_GetDatetime(RTC, &date_time);
  dateTime->date = date_time.day;
  dateTime->year = date_time.year;
  dateTime->month = date_time.month;
  dateTime->hour = date_time.hour;
  dateTime->minute = date_time.minute;
  dateTime->second = date_time.second;
}

bool SetRTCDateTime (sDateTime * dateTime)
{
    bool status = true;
    rtc_datetime_t date;
    date.year = (uint16_t)dateTime->year;
    date.month = (uint8_t)dateTime->month;
    date.day = (uint8_t)dateTime->date;
    date.hour = (uint8_t)dateTime->hour;
    date.minute = (uint8_t)dateTime->minute;
    date.second = (uint8_t)dateTime->second;

    /* RTC time counter has to be stopped before setting the date & time in the TSR register */
    RTC_StopTimer(RTC);
    DisableIRQ(RTC_Seconds_IRQn);
    if (kStatus_Success != RTC_SetDatetime(RTC, &date))
    {
       status = false;
       Serial_Debug("\nDate Time Validity Error");
    }
    else
    {
       SetDateTimeUpdated();
       ClearRTCInvalidClock();
       status = true;
       Serial_Debug("\n RTC Set Success");
    }
    RTC_StartTimer(RTC);
    EnableIRQ(RTC_Seconds_IRQn);
    return(status);
}

bool CheckRTCDateTimeFormat(const sDateTime *dateTime)
{
  rtc_datetime_t date;
  date.year = (uint16_t)dateTime->year;
  date.month = (uint8_t)dateTime->month;
  date.day = (uint8_t)dateTime->date;
  date.hour = (uint8_t)dateTime->hour;
  date.minute = (uint8_t)dateTime->minute;
  date.second = (uint8_t)dateTime->second;
  return RTC_CheckDatetimeFormat(&date);
}

bool SetRTCDefaultDateTime (void)
{
  sDateTime defDateTime;
  
  defDateTime.year = DEFAULT_RTC_YEAR;
  defDateTime.month = DEFAULT_RTC_MONTH;
  defDateTime.date = DEFAULT_RTC_DAY;
  defDateTime.hour = DEFAULT_RTC_HOUR;
  defDateTime.minute = DEFAULT_RTC_MINUTE;
  defDateTime.second = DEFAULT_RTC_SECOND;
  
  if(SetRTCDateTime(&defDateTime) == true)
  {
    Serial_Debug("\nDefault RTCC Set");
    return true;
  }
  else
  {    
    Serial_Debug("\nDefault RTCC Set ERROR");
    return false;
  }   
}

uint32_t GetRTCDateTimeSeconds (void)
{
  return RTC_GetDatetimeSeconds(RTC);
}

void GetRTCDateTimeString(char * rtcString, uint8_t rtcStringLength)
{
   sDateTime currentDateTime;
   GetRTCDateTime(&currentDateTime);
   snprintf((char *)rtcString, rtcStringLength, "%02d-%02d-%04d %02d:%02d:%02d", currentDateTime.date, 
        currentDateTime.month,currentDateTime.year, currentDateTime.hour, 
        currentDateTime.minute, currentDateTime.second);
}

void ConvertRTCDateTimeToString(const sDateTime *dateTime, char * rtcString, uint8_t rtcStringLength)
{
  snprintf((char *)rtcString, rtcStringLength, "%02d-%02d-%04d %02d:%02d:%02d", dateTime->date, 
          dateTime->month,dateTime->year, dateTime->hour, 
          dateTime->minute, dateTime->second);
}

void ConvertRTCSecondsToDatetime(uint32_t seconds, sDateTime *dateTime)
{
  rtc_datetime_t date_time;
  RTC_ConvertSecondsToDatetime(seconds, &date_time);
  dateTime->date = date_time.day;
  dateTime->year = date_time.year;
  dateTime->month = date_time.month;
  dateTime->hour = date_time.hour;
  dateTime->minute = date_time.minute;
  dateTime->second = date_time.second;
}

uint32_t ConvertRTCDatetimeToSeconds(const sDateTime *datetime)
{
  rtc_datetime_t date;
  date.year = (uint16_t)datetime->year;
  date.month = (uint8_t)datetime->month;
  date.day = (uint8_t)datetime->date;
  date.hour = (uint8_t)datetime->hour;
  date.minute = (uint8_t)datetime->minute;
  date.second = (uint8_t)datetime->second;
  return RTC_ConvertDatetimeToSeconds(&date);
}

uint32_t ConvertRTCDateToSeconds(sDateTime * datetime)                  
{
  rtc_datetime_t date;
  date.year = (uint16_t)datetime->year;
  date.month = (uint8_t)datetime->month;
  date.day = (uint8_t)datetime->date;
  date.hour = 0;
  date.minute = 0;
  date.second = 0;
  return RTC_ConvertDatetimeToSeconds(&date);
}

uint32_t ConvertRTCTimeToSeconds(sDateTime * datetime)                  
{
   uint32_t seconds = (datetime->hour * SECONDS_IN_A_HOUR) +
              (datetime->minute * SECONDS_IN_A_MINUTE) + datetime->second;

   return seconds;
}

//bool IsSpaRtcChanged (uint32_t dateSeconds)
//{
//  bool status = false;
//  if(IsDayChanged() == true)
//  {
//    ClearDayChanged();
//    if(IsNewDay(dateSeconds) == true)
//    {
//      status = true;
//    }
//    else
//    {
//      RestartAutoModeIndex ();
//    }
//  }
//  return status;
//}
//
//static bool IsNewDay (uint32_t dateSeconds)
//{
// sDateTime curDateTime;
// RTC_GetDateTime (&curDateTime);
// uint32_t curDateSeconds =  RTC_ConvertDateToSeconds(&curDateTime);
// if(curDateSeconds != dateSeconds)
// {
//    dateSeconds = curDateSeconds;
//    return true;
// }
// else
// {
//    return false;  
// }
//}

static void SetDateTimeUpdated (void)
{
  dateTimeUpdatedFlag = true;  
}

bool IsDateTimeUpdated (void)
{
  return dateTimeUpdatedFlag;
}

void ClearDateTimeUpdated (void)
{
  dateTimeUpdatedFlag = false;  
}


