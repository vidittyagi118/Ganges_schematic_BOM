#ifndef _SDCARDOPERATION_DEFINED
#define _SDCARDOPERATION_DEFINED

#include "integer.h"	        /* Basic integer types */
#include "ffconf.h"		/* FatFs configuration options */

#define BUFF_COUNT 10000+1

#define Data_Read_Cnt  BUFF_COUNT   
//#define TemperatureData_write_Cnt  BUFF_COUNT   // 3480+1<-120/2hour ,1740+1 <- 60/1hour , 5220<- 180/3hours

//#define SOCData_Read_Cnt  BUFF_COUNT  
////#define SOCData_write_Cnt  BUFF_COUNT   
//
//#define MOTORData_Read_Cnt BUFF_COUNT   
////#define MOTORData_write_Cnt  BUFF_COUNT   
//
//#define ERRORData_Read_Cnt  BUFF_COUNT   
////#define ERRORData_write_Cnt  BUFF_COUNT   

void SDcardOperation(void);
BYTE SDcardMount();
void Delete_SDdata(void);

/////////////TEMPERATURE LOG FUNCTIONS//////////

void Temperature_log_Data(char *Log_dataas);
void Temperature_SDlogtimer(void );
uint32_t GetTemperature_SDlogTimer(void);
void GETLog_Temperature();
void Temperaturelog_main(void);

/////////////SOC LOG FUNCTIONS//////////

void SOC_log_Data(char *Log_dataas);
void SOC_SDlogtimer(void );
uint32_t GetSOC_SDlogTimer(void);
void GETLog_SOC();
void SOClog_main(void);

/////////////MOTOR LOG FUNCTIONS//////////

void MOTORCRNT_log_Data(char *Log_dataas);
void MOTORCRNT_SDlogtimer(void );
uint32_t GetMOTORCRNT_SDlogTimer(void);
void GETLog_MOTORCRNT();
void MOTORCRNTlog_main(void);

/////////////ERROR LOG FUNCTIONS//////////

void ERROR_log_Data(char *Log_dataas);
void GETLog_ERROR();
void ERRORlog_main(void);
void ERROR_LogTypes();

///////////// AutoCycle LOG FUNCTIONS///////////

void AutoCycle_main();
void AutoCycle_log_Data(char *Log_dataas);
void GETLog_AUTOCYCLE();


#endif