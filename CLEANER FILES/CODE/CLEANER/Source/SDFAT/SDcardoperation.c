
#include "spi.h"
#include "board.h"
#include "ff.h"			/* Declarations of FatFs API */
#include "diskio.h"		/* Declarations of disk I/O functions */
#include "ffconf.h"
#include "SDFAT/SDcardoperation.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "temp_sensors.h"
#include "ApiProcessing.h"
#include "MotCurrentFaultCommon.h"
#include "LTC4015_Main.h"
#include "RotateSenseCommon.h"
#include "WDTimer.h"
#include "RobotControl.h"
#include "AutoSchedule.h"
#include "RobotOperation.h"
#include "DefaultValuesConfig.h"
#include "Rtc.h"

DWORD RECordMemory_size = 100*1000;       //(Multiply by 10 - 1MB,Multiply by 100 - 10MB,Multiply by 1000 - 100MB)

FATFS FatFs;	/* FatFs work area needed for each volume */
FIL Fil;		/* File object needed for each open file */

extern char Battemp_str[10],Drivetemp_str[10],Dietemp_str[10];
extern char Vin_str[10],Vbat_str[10],Iin_str[10],Ibat_str[10],Dtemp_str[10],Soc_Str[10];
extern char Imot1_Str[10],Imot2_Str[10],Imot3_Str[10];
extern char bat_arr[10],pv_arr[10],mpptcurr_arr[10],mpptout_arr[10],inttemp_arr[10],exttemp_arr[10],batstate_arr[10], loadout_arr[10],loadtrip_arr[10];

extern char schedule_log[100];
extern char schedule_log_21[100];
extern char schedule_BYTE;
extern char SDerrorlog[45];
extern char Robot_Error_Count;

extern BYTE SD_MOUNT_STATUS;
extern BYTE SD_MOUNT_STATUS;
  char Board_ON;

BYTE *le;
int ERROR_OCCURRED;
int CYCLE_OCCURRED;
int MOTOR_STARTED;

/////////////LOG write buffer/////////////
//BYTE  Temperaturedata_write_buffer[TemperatureData_write_Cnt];
//BYTE  SOCdata_write_buffer[SOCData_write_Cnt];
//BYTE  MOTORdata_write_buffer[MOTORData_write_Cnt];
//BYTE  ERRORdata_write_buffer[ERRORData_write_Cnt];

/////////////LOG Read buffer/////////////
BYTE  data_read_buffer[Data_Read_Cnt];
BYTE  data_read_buffer2[Data_Read_Cnt];
//BYTE  data_read_buffer[100];

//BYTE  TEMPdata_read_buffer[Data_Read_Cnt];
//BYTE  SOCdata_read_buffer[Data_Read_Cnt];
//BYTE  MOTORdata_read_buffer[Data_Read_Cnt];
//BYTE  CYCLEdata_read_buffer[Data_Read_Cnt];
//BYTE  ERRORdata_read_buffer[Data_Read_Cnt];


DWORD bw,pxFile;
DWORD file_len;
float Temperature_frequency;
uint32_t dataCount=0,loop=0;
uint16_t databufferLength; 


BYTE Temperature_logtime;
uint32_t Temperature_SDlogTimer = 0;
uint32_t Temperature_SDcurntcount = 0;

BYTE SOC_logtime;
uint32_t SOC_SDlogTimer = 0;
uint32_t SOC_SDcurntcount = 0;


BYTE MOTORCRNT_logtime;
uint32_t MOTORCRNT_SDlogTimer = 0;
uint32_t MOTORCRNT_SDcurntcount = 0;

extern int distance;
extern uint16_t track_no,day_count;
extern int Cumulative_distance;
char sd_continue_read = 1;

BYTE SDcardMount()
{
        static BYTE mountcount=0;
    mount:
        if (f_mount(&FatFs, "", 1) == FR_OK) 
        { 
           mountcount =0;
           f_open(&Fil, "Temperat.txt",  FA_WRITE |FA_OPEN_ALWAYS|  FA_READ );
           SDdelay1(10);
           f_open(&Fil, "SOClog.txt",  FA_WRITE |FA_OPEN_ALWAYS|  FA_READ );
           SDdelay1(10);
           f_open(&Fil, "MOTORCRT.txt",  FA_WRITE |FA_OPEN_ALWAYS|  FA_READ );
           SDdelay1(10);
           f_open(&Fil, "AUTOCYCL.txt",  FA_WRITE |FA_OPEN_ALWAYS|  FA_READ );
           SDdelay1(10);
           f_open(&Fil, "ERRORLOG.txt",  FA_WRITE |FA_OPEN_ALWAYS|  FA_READ );
           SDdelay1(10);
           return 1;
        }
        else{
          if(mountcount++>=10){
             mountcount =0;
             return 0;
          }
          goto mount;
        }
}

////////////////////////////////////////////Temperature Log main Function Start////////////////////////////////
void Temperature_log_Data(char *Log_dataas)
{
    #define Log_Array_count 100
    
    BYTE Log_Array[Log_Array_count];
    memset(Log_Array , '\0' , Log_Array_count );
   
//    uint32_t getTime = GetRTCDateTimeSeconds();
    
    char getTime_str[40];
    GetRTCDateTimeString(&getTime_str, sizeof getTime_str);
    
    strcpy(Log_Array,getTime_str);
//    dataCount = 0; 
////    Log_Array[dataCount] = '#';      
//    uint8_t tempCount1 = sprintf(&Log_Array[dataCount+1],"%s",getTime_str);    
    strcat(Log_Array,"," );
    
    Get_SD_Card_Temps();
    
  //   databufferLength = sizeof(Log_dataas)-dataCount;
    dataCount =   strlen(Log_Array);
    uint8_t tempCount2 = snprintf(&Log_Array[dataCount],24 ,"%s,%s,%s",Battemp_str,Drivetemp_str,Dietemp_str);
     
    strcat(Log_Array,"\r\n" );
     
/*    dataCount =  strlen(Temperaturedata_write_buffer);// data lenght
    
    if(dataCount>= TemperatureData_write_Cnt)
    {
      SDdelay1(1);
      memset(Temperaturedata_write_buffer,NULL,TemperatureData_write_Cnt);
    }
    if(dataCount == NULL)
     strcpy(Temperaturedata_write_buffer,Log_Array);
    else
     strcat(Temperaturedata_write_buffer,Log_Array);
 */   
    if (SD_MOUNT_STATUS == 1 ) 
    {  
       if (f_open(&Fil, "Temperat.txt",  FA_WRITE |FA_OPEN_ALWAYS|  FA_READ ) == FR_OK)// 
       {           

              dataCount =  strlen(Log_Array);// data lenght
              DWORD read_count=0;
              for(read_count=1;read_count<=RECordMemory_size;read_count++) { WDTimerRefresh();  f_read(&Fil ,data_read_buffer,Data_Read_Cnt,&bw);} //modi
              f_write(&Fil,Log_Array,dataCount, &pxFile);      
              f_sync(&Fil);//f_close(&Fil);
              SDdelay1(10);
       
       }
    //   GETLog_Temperature();
    }
  
}

void Temperature_SDlogtimer(void )
{
  Temperature_SDlogTimer++;
}


uint32_t GetTemperature_SDlogTimer(void)
{
  return Temperature_SDlogTimer;
}



void Temperaturelog_main(void)
{
  
   Temperature_SDcurntcount = GetTemperature_SDlogTimer();
   stInterval* getInterval = GetSetIntervalLimits(); 
   
   if(getInterval->I1 == 0.0){
     getInterval->I1 = 1.0; //1.0 - one minutes
   }

  if(Temperature_SDcurntcount >= getInterval->I1 * 59500)//1000- 1sec
  {    
      Temperature_logtime = true;      
  }
  
 // Temperature_logtime= true; //testing only
  if(Temperature_logtime == true)
  {
      Temperature_logtime = false;
      Temperature_SDlogTimer =0;
      Temperature_log_Data("logicinside");
  }
  
}

int sizof;

void GETLog_Temperature()
{
  char *ptr;
  BYTE temp_buffer[Data_Read_Cnt];
  
  BYTE *End_Ptr, *Start_Ptr;
    
  char getTime_str[15];
    GetRTCDateTimeString(&getTime_str, sizeof getTime_str);
    
  uint32_t utcTime = GetRTCDateTimeSeconds();
  uint32_t hr1_utcTime = utcTime - 3600;
  
  char getTime_str21[16];
  sd_DateTime setTime;
  ConvertRTCSecondsToDatetime(hr1_utcTime,&setTime);
  ConvertRTCDateTimeToString(&setTime, getTime_str21, sizeof getTime_str21);
    
  memset(data_read_buffer2,'\0',Data_Read_Cnt);
  
       if (f_open(&Fil, "Temperat.txt",FA_READ ) == FR_OK) 
       {
                SDdelay1(100);
                DWORD read_count=0;
                for(read_count=1;read_count<=RECordMemory_size;read_count++) 
                { 
                  WDTimerRefresh(); 
                  memset(data_read_buffer,'\0',Data_Read_Cnt);
                  f_read(&Fil ,data_read_buffer,Data_Read_Cnt,&bw);
                  
                    
                  Start_Ptr = strstr(data_read_buffer,getTime_str21); 
                  End_Ptr = strstr(data_read_buffer,getTime_str);
                 
//                   if ((Start_Ptr != NULL) && (End_Ptr != NULL)) 
//                  {
//                      memset(data_read_buffer2, NULL, Data_Read_Cnt);
//                      
//                      if(Start_Ptr > End_Ptr)
//                      {   
//                        memcpy(data_read_buffer2, Start_Ptr,strlen(Start_Ptr));
//                      }
//                      else if(End_Ptr > Start_Ptr)
//                      { 
//                        memcpy(data_read_buffer2, End_Ptr,strlen(End_Ptr));
//                      }
//                      
//                      break;
//                  }
//                  else if ((Start_Ptr != NULL)) 
//                  {
//                      memset(data_read_buffer2, NULL, Data_Read_Cnt);
//                      memcpy(data_read_buffer2, Start_Ptr,strlen(Start_Ptr));
//                      break;
//                  }  
                  if ((Start_Ptr != NULL) || sd_continue_read == 2) 
                  {
                      if(sd_continue_read == 1)
                      {
                        memset(data_read_buffer2, NULL, Data_Read_Cnt);
                        memcpy(data_read_buffer2, Start_Ptr,strlen(Start_Ptr));
                      }
                      else
                      {
                        strcat(data_read_buffer2,data_read_buffer);
                      }
                      
                      if(data_read_buffer[10000] == '\0' || sd_continue_read == 2 || data_read_buffer2[10000] != '\0')
                      {
                        if(data_read_buffer2[10000] != '\0')
                        {
                          UartTransmitType(CONFIG_UART,data_read_buffer2,2500); 
                        }
                        sd_continue_read = 1;
                        break;
                      }
                      else
                      {
                        sd_continue_read = 2;
                        continue;
                      }
                   } 
                  else if((End_Ptr != NULL)) 
                  {
                      memset(data_read_buffer2, NULL, Data_Read_Cnt);
                      memcpy(data_read_buffer2, End_Ptr,strlen(End_Ptr));
                      break;
                  }   
                  
                }
                SDdelay1(100);
                f_sync(&Fil);//f_close(&Fil);           
       }       
}

////////////////////////////////////////////Temperature Log main Function END////////////////////////////////

////////////////////////////////////////////SOC Log main Function START////////////////////////////////

void SOC_SDlogtimer(void )
{
  SOC_SDlogTimer++;
}


uint32_t  GetSOC_SDlogTimer(void)
{
  return SOC_SDlogTimer;
}

void SOC_log_Data(char *Log_dataas)
{
    #define Log_Array_count 100
  
    BYTE Log_Array[Log_Array_count];
    memset(Log_Array , '\0' , Log_Array_count );
   
    
    char getTime_str[40];
    GetRTCDateTimeString(&getTime_str, sizeof getTime_str);
    
    strcpy(Log_Array,getTime_str);
//    dataCount = 0; 
////    Log_Array[dataCount] = '#';  
//    
//
//    uint8_t tempCount1 = sprintf(&Log_Array[dataCount+1],"%s",getTime_str);
    
    strcat(Log_Array,"," );
    
    Get_SD_Card_SOC();
    
  //   databufferLength = sizeof(Log_dataas)-dataCount;
    dataCount =   strlen(Log_Array);
 
     //   uint8_t tempCount = snprintf(&Log_Array[dataCount],100,"%s,%s,%s,%s,%s,%s,%d",Vin_str,Iin_str,Vbat_str, Ibat_str,Dtemp_str,Soc_Str,GetChargingOrNot());  //before bms
   
    uint8_t tempCount = snprintf(&Log_Array[dataCount],200,"%s,%s,%s,%s,%s,%s,%d,%s,%s,%s,%s,%s,%s,%s,%s,%s",Vin_str,Iin_str,Vbat_str, Ibat_str,Dtemp_str,Soc_Str,GetChargingOrNot(),bat_arr,pv_arr,mpptcurr_arr,mpptout_arr,inttemp_arr,exttemp_arr,batstate_arr,loadout_arr,loadtrip_arr);
    strcat(Log_Array,"\r\n" );
 /*    
    dataCount =  strlen(SOCdata_write_buffer);// data lenght
    
    if(dataCount>= SOCData_write_Cnt)
    {
      SDdelay1(1);
      memset(SOCdata_write_buffer,NULL,SOCData_write_Cnt);
    }
    if(dataCount == NULL)
     strcpy(SOCdata_write_buffer,Log_Array);
    else
     strcat(SOCdata_write_buffer,Log_Array);
*/
    if (SD_MOUNT_STATUS == 1 ) 
    {  
       if (f_open(&Fil, "SOClog.txt",  FA_WRITE |FA_OPEN_ALWAYS|  FA_READ ) == FR_OK)// 
       {    
              dataCount =  strlen(Log_Array);// data lenght
              DWORD read_count=0;
              for(read_count=1;read_count<=RECordMemory_size;read_count++) { WDTimerRefresh(); f_read(&Fil ,data_read_buffer,Data_Read_Cnt,&bw);} //modi
              f_write(&Fil,Log_Array,dataCount, &pxFile);            
              f_sync(&Fil);//f_close(&Fil);
              SDdelay1(10);
       
       }
    //   GETLog_SOC();
    }
  
}

void SOClog_main(void )
{
   SOC_SDcurntcount = GetSOC_SDlogTimer();
   stInterval* getInterval = GetSetIntervalLimits(); 
   
   if(getInterval->I2 == 0.0){
     getInterval->I2 = 1.0; //1.0 - one minutes
   }
   
  if(SOC_SDcurntcount >= getInterval->I2 * 59500)//1000- 1sec
  {    
      SOC_logtime = true;      
  }
  
 // SOC_logtime= true; //testing only
  if(SOC_logtime == true)
  {
      SOC_logtime = false;
      SOC_SDlogTimer =0;
      SOC_log_Data("logicinside");
  }
  
}

void GETLog_SOC()
{
  BYTE *End_Ptr, *Start_Ptr;
    
  char getTime_str[15];
  GetRTCDateTimeString(&getTime_str, sizeof getTime_str);
    
  uint32_t utcTime = GetRTCDateTimeSeconds();
  uint32_t hr1_utcTime = utcTime - 3600;
  
  char getTime_str21[16];
  sd_DateTime setTime;
  ConvertRTCSecondsToDatetime(hr1_utcTime,&setTime);
  ConvertRTCDateTimeToString(&setTime, getTime_str21, sizeof getTime_str21);
  
   memset(data_read_buffer2,'\0',Data_Read_Cnt);
   
       if (f_open(&Fil, "SOClog.txt",FA_READ ) == FR_OK) 
       {
            SDdelay1(100);
                DWORD read_count=0;
                for(read_count=1;read_count<=RECordMemory_size;read_count++) 
                { 
                  WDTimerRefresh();      
                  memset(data_read_buffer,'\0',Data_Read_Cnt);
                  f_read(&Fil ,data_read_buffer,Data_Read_Cnt,&bw);
                  
                  
                  Start_Ptr = strstr(data_read_buffer,getTime_str21); 
                  End_Ptr = strstr(data_read_buffer,getTime_str);
                 
                  
//                   if ((Start_Ptr != NULL) && (End_Ptr != NULL)) 
//                   {
//                      memset(data_read_buffer2, NULL, sizeof(data_read_buffer2));
//                      
//                      if(Start_Ptr > End_Ptr)
//                      {
//                        memcpy(data_read_buffer2, Start_Ptr,strlen(Start_Ptr));
//                      }
//                      else if(End_Ptr > Start_Ptr)
//                      {
//                        memcpy(data_read_buffer2, End_Ptr,strlen(End_Ptr));
//                      }
//                      
//                      break;
//                    }
//                    else if ((Start_Ptr != NULL)) 
//                    {
//                        memset(data_read_buffer2, NULL, sizeof(data_read_buffer2));
//                        memcpy(data_read_buffer2, Start_Ptr,strlen(Start_Ptr));
//                        break;
//                    }  
                   if ((Start_Ptr != NULL) || sd_continue_read == 2) 
                  {
                      if(sd_continue_read == 1)
                      {
                        memset(data_read_buffer2, NULL, Data_Read_Cnt);
                        memcpy(data_read_buffer2, Start_Ptr,strlen(Start_Ptr));
                      }
                      else
                      {
                        strcat(data_read_buffer2,data_read_buffer);
                      }
                      
                      if(data_read_buffer[10000] == '\0' || sd_continue_read == 2 || data_read_buffer2[10000] != '\0')
                      {
                        if(data_read_buffer2[10000] != '\0')
                        {
                          UartTransmitType(CONFIG_UART,data_read_buffer2,2500); 
                        }
                        sd_continue_read = 1;
                        break;
                      }
                      else
                      {
                        sd_continue_read = 2;
                        continue;
                      }
                   }  
                    else if((End_Ptr != NULL)) 
                    {
                        memset(data_read_buffer2, NULL, sizeof(data_read_buffer2));
                        memcpy(data_read_buffer2, End_Ptr,strlen(End_Ptr));
                        break;
                    }  
                  
                }
                SDdelay1(100);
                f_sync(&Fil);//f_close(&Fil);         
       }       
}

////////////////////////////////////////////SOC Log main Function END////////////////////////////////

////////////////////////////////////////////MOTORCRNT Log main Function START////////////////////////////////

void MOTORCRNT_SDlogtimer(void )
{
  MOTORCRNT_SDlogTimer++;
}


uint32_t  GetMOTORCRNT_SDlogTimer(void)
{
  return MOTORCRNT_SDlogTimer;
}



void MOTORCRNT_log_Data(char *Log_dataas)
{
    #define Log_Array_count 100
  
    BYTE Log_Array[Log_Array_count];
    int mode_now;
    memset(Log_Array , '\0' , Log_Array_count );
   
    DWORD Currentmotordistance;
    char getTime_str[40];
    GetRTCDateTimeString(&getTime_str, sizeof getTime_str);
    
    strcpy(Log_Array,getTime_str);
//    dataCount = 0; 
//    Log_Array[dataCount] = '#';  
//    
//
//    uint8_t tempCount1 = sprintf(&Log_Array[dataCount+1],"%s",getTime_str);
    
    strcat(Log_Array,"," );
    
    Get_SD_Card_MOTORCRNT();
    
  //   databufferLength = sizeof(Log_dataas)-dataCount;
    dataCount =   strlen(Log_Array);
    PulseCalculation();
  
   Currentmotordistance = GetRotateSense1Count() * distance;
   eRobotDirection robotState = GetRobotActualDirection();
   eMode getMode =  GetOperationMode ();
   
//    uint8_t modeValue = 0;
//    switch(getMode)
//    {
//    case AUTO: 
//      modeValue = 0;
//      break;
//    case MANUAL:
//      modeValue = 1;
//      break;
//    default : 
//      modeValue = 0;
//      break;
//    }
//    
    if(getMode == AUTO)
        mode_now =  Get_Completed_Schedule();
    else
         mode_now = 0;
 
   uint8_t tempCount = snprintf(&Log_Array[dataCount],100,"%d,%d,%s,%s,%s,%d,%d",mode_now,robotState,Imot1_Str,Imot2_Str,Imot3_Str,GetRotateSense1Count(),Currentmotordistance);
   
    strcat(Log_Array,"\r\n" );
  
    if (SD_MOUNT_STATUS == 1 ) 
    {  
       if (f_open(&Fil, "MOTORCRT.txt",  FA_WRITE |FA_OPEN_ALWAYS|  FA_READ ) == FR_OK)// 
       {    
              dataCount =  strlen(Log_Array);// data lenght
             DWORD read_count=0;
             for(read_count=1;read_count<=RECordMemory_size;read_count++) { WDTimerRefresh(); f_read(&Fil ,data_read_buffer,Data_Read_Cnt,&bw);}
              f_write(&Fil,Log_Array,dataCount, &pxFile);            
               f_sync(&Fil);//f_close(&Fil);
              SDdelay1(10);
       
       }
      // GETLog_MOTORCRNT();
    }
  
}




void GETLog_MOTORCRNT()
{
    
  BYTE *End_Ptr, *Start_Ptr;
  
   char getTime_str[15];
   GetRTCDateTimeString(&getTime_str, sizeof getTime_str);
    
  uint32_t utcTime = GetRTCDateTimeSeconds();
  uint32_t hr1_utcTime = utcTime - 3600;
  
  char getTime_str21[16];
  sd_DateTime setTime;
  ConvertRTCSecondsToDatetime(hr1_utcTime,&setTime);
  ConvertRTCDateTimeToString(&setTime, getTime_str21, sizeof getTime_str21);
  
  memset(data_read_buffer2,'\0',sizeof(data_read_buffer2));

       if (f_open(&Fil, "MOTORCRT.txt",FA_READ ) == FR_OK) 
       {
            SDdelay1(100);
                DWORD read_count=0;
                for(read_count=1;read_count<=RECordMemory_size;read_count++) 
                { 
                  WDTimerRefresh();
                  memset(data_read_buffer,'\0',Data_Read_Cnt); 
                  f_read(&Fil ,data_read_buffer,Data_Read_Cnt,&bw);
                  
                  
                  Start_Ptr = strstr(data_read_buffer,getTime_str21); 
                  End_Ptr = strstr(data_read_buffer,getTime_str);
                  
               
//                  if ((Start_Ptr != NULL) && (End_Ptr != NULL)) 
//                  {
//                      memset(data_read_buffer2, NULL, sizeof(data_read_buffer2));
//                      
//                      if(Start_Ptr > End_Ptr)
//                      {
//                        memcpy(data_read_buffer2, Start_Ptr,strlen(Start_Ptr));
//                      }
//                      else if(End_Ptr > Start_Ptr)
//                      {
//                        memcpy(data_read_buffer2, End_Ptr,strlen(End_Ptr));
//                      }
//                      
//                      break;
//                  }
//                  else if ((Start_Ptr != NULL)) 
//                  {
//                      memset(data_read_buffer2, NULL, sizeof(data_read_buffer2));
//                      memcpy(data_read_buffer2, Start_Ptr,strlen(Start_Ptr));
//                      break;
//                  }  
                   if ((Start_Ptr != NULL) || sd_continue_read == 2) 
                  {
                      if(sd_continue_read == 1)
                      {
                        memset(data_read_buffer2, NULL, Data_Read_Cnt);
                        memcpy(data_read_buffer2, Start_Ptr,strlen(Start_Ptr));
                      }
                      else
                      {
                        strcat(data_read_buffer2,data_read_buffer);
                      }
                      
                      if(data_read_buffer[10000] == '\0' || sd_continue_read == 2 || data_read_buffer2[10000] != '\0')
                      {
                        if(data_read_buffer2[10000] != '\0')
                        {
                          UartTransmitType(CONFIG_UART,data_read_buffer2,2500); 
                        }
                        sd_continue_read = 1;
                        break;
                      }
                      else
                      {
                        sd_continue_read = 2;
                        continue;
                      }
                   }  
                  else if((End_Ptr != NULL)) 
                  {
                      memset(data_read_buffer2, NULL, sizeof(data_read_buffer2));
                      memcpy(data_read_buffer2, End_Ptr,strlen(End_Ptr));
                      break;
                  } 
                  
                }
                SDdelay1(100);
                f_sync(&Fil);//f_close(&Fil); 
       }
}

void MOTORCRNTlog_main(void )
{
   MOTORCRNT_SDcurntcount = GetMOTORCRNT_SDlogTimer();
   stInterval* getInterval = GetSetIntervalLimits(); 
   
   if(getInterval->I3 == 0.0)
   {
     getInterval->I3 = 1.0; //1.0 - one minutes
   }

  if(MOTORCRNT_SDcurntcount >= getInterval->I3 * 5000)//1000- 1sec
  {    
      MOTORCRNT_logtime = true;      
  }
  
  
  eRobotDirection robotState = GetRobotActualDirection();
  
//  MOTORCRNT_logtime= true; //testing only
  if((MOTORCRNT_logtime == true)&&(robotState == 1 || robotState == 0))
  {
      MOTORCRNT_logtime = false;
      MOTORCRNT_SDlogTimer =0;
      MOTORCRNT_log_Data("logicinside");
      //MOTOR_STARTED = 0;
  }
  
}

////////////////////////////////////////////MOTORCRNT Log main Function END////////////////////////////////


////////////////////////////////////////////Error Log main Function START//////////////////////////////

void  ERRORlog_main(void)
{  
   if(ERROR_OCCURRED == 1) 
   {
       ERROR_log_Data("logicinside");
       ERROR_OCCURRED = 0;
   }  
}

void ERROR_log_Data(char *Log_dataas)
{
   #define Log_Array_count 100
   int Card_reset;
  
    BYTE Log_Array[Log_Array_count];
    memset(Log_Array , '\0' , Log_Array_count );
   
    DWORD Currentmotordistance;
    char getTime_str[40];
    int Currentdistance,mode_now;
    GetRTCDateTimeString(&getTime_str, sizeof getTime_str);
    
    strcpy(Log_Array,getTime_str);
//    dataCount = 0; 
//    Log_Array[dataCount] = '#';  
    

//    uint8_t tempCount1 = sprintf(&Log_Array[dataCount+1],"%s",getTime_str);
    
    strcat(Log_Array,"," );
    
    dataCount =   strlen(Log_Array);
    PulseCalculation();
   Currentdistance = GetRotateSense1Count() * distance;
   eRobotDirection robotState = GetRobotActualDirection();
    
    eMode getMode =  GetOperationMode ();
    uint8_t modeValue = 0;
    switch(getMode)
    {
    case AUTO: 
      modeValue = 0;
      mode_now =  Get_Completed_Schedule();
      break;
    case MANUAL:
      modeValue = 1;
      mode_now = 0;
      break;
    default : 
      modeValue = 0;
      mode_now =  Get_Completed_Schedule();
      break;
    }
    
    uint8_t tempCount;
    
    if(Board_ON == 1)
    {
        //tempCount = snprintf(&Log_Array[dataCount],100,"%s",DEFAULT_DEVICE_WELCOME_MSG);
         Board_ON = 0;
         Card_reset = 1;
         SDerrorlog[0] = '0';
    }
    else 
    {
        //uint8_t tempCount = snprintf(&Log_Array[dataCount],100,"%d,%d,%d,%d,%d,%s",getMode,robotState,mode_now,GetRotateSense1Count(),Currentdistance,SDerrorlog);
        Card_reset = 0;
    }
   
    tempCount = snprintf(&Log_Array[dataCount],100,"%d,%d,%d,%d,%d,%s,%d",modeValue,robotState,mode_now,GetRotateSense1Count(),Currentdistance,SDerrorlog,Card_reset);
   
    strcat(Log_Array,"\r\n" );
   
   /* dataCount =  strlen(ERRORdata_write_buffer);// data lenght
    
    if(dataCount>= ERRORData_write_Cnt)
    {
      SDdelay1(1);
      memset(ERRORdata_write_buffer,NULL,ERRORData_write_Cnt);
    }
    if(dataCount == NULL)
     strcpy(ERRORdata_write_buffer,Log_Array);
    else
     strcat(ERRORdata_write_buffer,Log_Array);
   */ 
    
    if (SD_MOUNT_STATUS == 1 ) 
    {  
         if (f_open(&Fil, "ERRORLOG.txt",  FA_WRITE |FA_OPEN_ALWAYS|  FA_READ ) == FR_OK)// 
         { 
              dataCount =  strlen(Log_Array);// data lenght
              DWORD read_count=0;
              for(read_count=1;read_count<=RECordMemory_size;read_count++){ WDTimerRefresh();  f_read(&Fil ,data_read_buffer,Data_Read_Cnt,&bw);} //modi
              f_write(&Fil,Log_Array,dataCount, &pxFile);            
              f_sync(&Fil); //f_close(&Fil);
              SDdelay1(10);
         }
    } 
 //   GETLog_ERROR();
}

void GETLog_ERROR()
{
  BYTE *End_Ptr, *Start_Ptr;
   BYTE *End_Ptr2, *Start_Ptr2;
    
  char getTime_str[15];
  GetRTCDateTimeString(&getTime_str, sizeof getTime_str);
    
  uint32_t utcTime = GetRTCDateTimeSeconds();
  uint32_t hr1_utcTime = utcTime - 3600;
  
  char getTime_str21[15];
  sd_DateTime setTime;
  ConvertRTCSecondsToDatetime(hr1_utcTime,&setTime);
  ConvertRTCDateTimeToString(&setTime, getTime_str21, sizeof getTime_str21);
  
  memset(data_read_buffer2,'\0',sizeof(data_read_buffer2));

      if (f_open(&Fil, "ERRORLOG.txt",FA_READ ) == FR_OK) 
       {
                DWORD read_count=0;
                for(read_count=1;read_count<=RECordMemory_size;read_count++) 
                { 
                  WDTimerRefresh();      
                  memset(data_read_buffer,'\0',Data_Read_Cnt);
                  f_read(&Fil ,data_read_buffer,Data_Read_Cnt,&bw);
                 
                  Start_Ptr = strstr(data_read_buffer,getTime_str21); 
                  End_Ptr = strstr(data_read_buffer,getTime_str);
                  
//                  if ((Start_Ptr != NULL) && (End_Ptr != NULL)) 
//                   {
//                      memset(data_read_buffer2, NULL, sizeof(data_read_buffer2)); 
//                      
//                      if(Start_Ptr > End_Ptr)
//                      {
//                        memcpy(data_read_buffer2, Start_Ptr,strlen(Start_Ptr));
//                      }
//                      else if(End_Ptr > Start_Ptr)
//                      {
//                        memcpy(data_read_buffer2, End_Ptr,strlen(End_Ptr));
////                          memcpy(data_read_buffer2, End_Ptr,End_Ptr - Start_Ptr);
//
//                      }
//                      
//                      break;
//                    }
//                    else if ((Start_Ptr != NULL)) 
//                    {
//                        memset(data_read_buffer2, NULL, sizeof(data_read_buffer2));
//                        memcpy(data_read_buffer2, Start_Ptr,strlen(Start_Ptr));
//                        break;
//                    }  
                   if ((Start_Ptr != NULL) || sd_continue_read == 2) 
                  {
                      if(sd_continue_read == 1)
                      {
                        memset(data_read_buffer2, NULL, Data_Read_Cnt);
                        memcpy(data_read_buffer2, Start_Ptr,strlen(Start_Ptr));
                      }
                      else
                      {
                        strcat(data_read_buffer2,data_read_buffer);
                      }
                      
                      if(data_read_buffer[10000] == '\0' || sd_continue_read == 2 || data_read_buffer2[10000] != '\0')
                      {
                        if(data_read_buffer2[10000] != '\0')
                        {
                          UartTransmitType(CONFIG_UART,data_read_buffer2,2500); 
                        }
                        sd_continue_read = 1;
                        break;
                      }
                      else
                      {
                        sd_continue_read = 2;
                        continue;
                      }
                   }  
                    else if((End_Ptr != NULL)) 
                    {
                        memset(data_read_buffer2, NULL, sizeof(data_read_buffer2));
                        memcpy(data_read_buffer2, End_Ptr,strlen(End_Ptr));
                        break;
                    } 
                     
                }   
                
                SDdelay1(100);
                f_sync(&Fil);//f_close(&Fil);         
       }
  
}

//void ERROR_LogTypes()
//{
// /*   if(Battery_error == 1){
//       strcat(Log_Array, "" );      
//    }
//    if(Temperature_error == 1){
//      strcat(Log_Array, "" );
//    }
//    if(MotorSOC_error ==1){
//      strcat(Log_Array, "" );
//    }
//   */ 
//}

////////////////////////////////////////////Error Log main Function END////////////////////////////////
////////////////////////////////////////////Autocycle Log main Function END/////////////////////////////

void AutoCycle_main()
{
  if (CYCLE_OCCURRED == 1)
  {
    AutoCycle_log_Data("logicinside");
    CYCLE_OCCURRED = 0;
  }
}

void AutoCycle_log_Data(char *Log_dataas)
{
   #define Log_Array_count 105
  
   int Currentdistance;
  
    BYTE Log_Array[Log_Array_count];
    memset(Log_Array , '\0' , Log_Array_count );
   
    DWORD Currentmotordistance;
    char getTime_str[40];
    int mode_now;
    GetRTCDateTimeString(&getTime_str, sizeof getTime_str);
    
    strcpy(Log_Array,getTime_str);
//    dataCount = 0; 
//    Log_Array[dataCount] = '#';  
//    
//
//    uint8_t tempCount1 = sprintf(&Log_Array[dataCount+1],"%s",getTime_str);
    
    strcat(Log_Array,"," );
    
   // Get_SD_Card_MOTORCRNT();
    
  //   databufferLength = sizeof(Log_dataas)-dataCount;
    dataCount =   strlen(Log_Array);
    //PulseCalculation();
  
  // Currentmotordistance = GetRotateSense1Count() * distance;
 
   //uint8_t tempCount = snprintf(&Log_Array[dataCount],40,"%s,%s,%s,%d",Imot1_Str,Imot2_Str,Imot3_Str,Currentmotordistance);
    
   PulseCalculation();
   Currentdistance = GetRotateSense1Count() * distance;
//   eRobotDirection robotState = GetRobotActualDirection();
    eMode getMode =  GetOperationMode ();
    
//    uint8_t modeValue = 0;
//    switch(getMode)
//    {
//    case AUTO: 
//      modeValue = 0;
//      mode_now =  Get_Completed_Schedule();
//      break;
//    case MANUAL:
//      modeValue = 1;
//      mode_now = 0;
//      break;
//    default : 
//      modeValue = 0;
//      mode_now =  Get_Completed_Schedule();
//      break;
//    }
     
    if(getMode == AUTO)
        mode_now =  Get_Completed_Schedule();
    else
         mode_now = 0;
  
    uint8_t tempCount;
      
    if(schedule_BYTE == 0)
    {
      tempCount = snprintf(&Log_Array[dataCount],105,"%d,%d,%d,%s,%d,%d,%d",mode_now,GetRotateSense1Count(),Currentdistance,schedule_log,track_no,day_count,Cumulative_distance);
    }
    else
    {
      memset(Log_Array , '\0' , Log_Array_count );
    dataCount =   strlen(Log_Array);
      tempCount = snprintf(&Log_Array[dataCount],105,"%s, Total Distance: %d",schedule_log_21,Cumulative_distance);
     schedule_BYTE = 0;
    }
    strcat(Log_Array,"\r\n" );
       
    if (SD_MOUNT_STATUS == 1 ) 
    {  
         if (f_open(&Fil, "AUTOCYCL.txt",  FA_WRITE |FA_OPEN_ALWAYS|  FA_READ ) == FR_OK)// 
         { 
              dataCount =  strlen(Log_Array);// data lenght
              DWORD read_count=0;
              for(read_count=1;read_count<=RECordMemory_size;read_count++){ WDTimerRefresh();  f_read(&Fil ,data_read_buffer,Data_Read_Cnt,&bw);} //modi
              
              f_write(&Fil,Log_Array,dataCount, &pxFile);            
              f_sync(&Fil); //f_close(&Fil);
              SDdelay1(10);
         }
    }  

}


void GETLog_AUTOCYCLE()
{ 
   BYTE *End_Ptr, *Start_Ptr;
    
   char getTime_str[15];
   GetRTCDateTimeString(&getTime_str, sizeof getTime_str);
    
  uint32_t utcTime = GetRTCDateTimeSeconds();
  uint32_t hr1_utcTime = utcTime - 3600;
  
  char getTime_str21[15];
  sd_DateTime setTime;
  ConvertRTCSecondsToDatetime(hr1_utcTime,&setTime);
  ConvertRTCDateTimeToString(&setTime, getTime_str21, sizeof getTime_str21);
  
  memset(data_read_buffer2,'\0',sizeof(data_read_buffer2));

       if (f_open(&Fil, "AUTOCYCL.txt",FA_READ ) == FR_OK) 
       {
            SDdelay1(100);
                DWORD read_count=0;
                for(read_count=1;read_count<=RECordMemory_size;read_count++) 
                { 
                  WDTimerRefresh();
                  memset(data_read_buffer,'\0',Data_Read_Cnt); 
                  f_read(&Fil ,data_read_buffer,Data_Read_Cnt,&bw);
                  
                  
                  Start_Ptr = strstr(data_read_buffer,getTime_str21); 
                  End_Ptr = strstr(data_read_buffer,getTime_str);
                  
//                 if ((Start_Ptr != NULL) && (End_Ptr != NULL)) 
//                  {
//                      memset(data_read_buffer2, NULL, sizeof(data_read_buffer2));
//                      
//                      if(Start_Ptr > End_Ptr)
//                      {
//                        memcpy(data_read_buffer2, Start_Ptr,strlen(Start_Ptr));
//                      }
//                      else if(End_Ptr > Start_Ptr)
//                      {
//                        memcpy(data_read_buffer2, End_Ptr,strlen(End_Ptr));
//                      }
//                      
//                      break;
//                  }
//                  else if ((Start_Ptr != NULL)) 
//                  {
//                      memset(data_read_buffer2, NULL, sizeof(data_read_buffer2));
//                      memcpy(data_read_buffer2, Start_Ptr,strlen(Start_Ptr));
//                      break;
//                  }  
                   if ((Start_Ptr != NULL) || sd_continue_read == 2) 
                  {
                      if(sd_continue_read == 1)
                      {
                        memset(data_read_buffer2, NULL, Data_Read_Cnt);
                        memcpy(data_read_buffer2, Start_Ptr,strlen(Start_Ptr));
                      }
                      else
                      {
                        strcat(data_read_buffer2,data_read_buffer);
                      }
                      
                      if(data_read_buffer[10000] == '\0' || sd_continue_read == 2 || data_read_buffer2[10000] != '\0')
                      {
                        if(data_read_buffer2[10000] != '\0')
                        {
                          UartTransmitType(CONFIG_UART,data_read_buffer2,2500); 
                        }
                        sd_continue_read = 1;
                        break;
                      }
                      else
                      {
                        sd_continue_read = 2;
                        continue;
                      }
                   }  
                  else if((End_Ptr != NULL)) 
                  {
                      memset(data_read_buffer2, NULL, sizeof(data_read_buffer2));
                      memcpy(data_read_buffer2, End_Ptr,strlen(End_Ptr));
                      break;
                  } 
//                  else 
//                  {
//                      memset(data_read_buffer2, NULL, Data_Read_Cnt);
//                      strcpy(data_read_buffer2,"\nNo Cyle log available for last one hour\n");
//                      break;
//                  }
                  
                  
                }
                SDdelay1(100);
                f_sync(&Fil);//f_close(&Fil);            
       }       
}
////////////////////////////////////////////Autocycle Log main Function END////////////////////////////////



void SDcardOperation(void)
{
  if(SD_MOUNT_STATUS == 1)
  {
     Temperaturelog_main();
     SOClog_main();
     MOTORCRNTlog_main();
     ERRORlog_main();
     AutoCycle_main();
  }
}

void Delete_SDdata(void)
{
     f_unlink("Temperat.txt");
     f_unlink("SOClog.txt");
     f_unlink("MOTORCRT.txt");
     f_unlink("AUTOCYCL.txt");
     f_unlink("ERRORLOG.txt");        
     SDcardMount();    
}