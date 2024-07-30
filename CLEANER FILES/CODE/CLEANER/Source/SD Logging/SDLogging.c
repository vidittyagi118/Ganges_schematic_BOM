#include "SDLogging.h"
#include "SDCARD/SDC_Main.h"	
#include "Serial_Debug.h"

#include "FaultProcess.h"
#include "RobotOperation.h"
#include "RobotControl.h"
#include "RotateSenseCommon.h"
#include "EdgeSenseCommon.h"
#include "DefaultValues.h"
#include "DefaultValuesConfig.h"
#include "CycleMode.h"
#include "LTC4015_Main.h"
#include "Rtc.h"
#include "Adc.h"
#include "BrushControl.h"
#include "BrushMotorControl_hal.h"
#include "AutoSchedule.h"
#include "MotCurrentFaultCommon.h"
#include "NumStrConversion.h"
#include "APIProcessing.h"
#include "PwmDriveSpi.h"

uint32_t sdLogTimer_ms = 0;
uint32_t sdPrevLogTime_ms = 0;
bool periodicLogBit = 0;

char eventTypeStr[13][50] ={
  "EV_ROBOT_FORWARD",
  "EV_ROBOT_START",
  "EV_ROBOT_ACCEL_COMPLETE",
  "EV_ROBOT_DECEL_START",
  "EV_ROBOT_DECEL_COMPLETE",
  "EV_ROBOT_STOP",
  "EV_ROBOT_REVERSE",
  "EV_MOTOR_1_OVERCURRENT",
  "EV_MOTOR_2_OVERCURRENT",
  "EV_BRUSH_OVERCURRENT",
  "EV_LOW_BATTERY",
  "EV_HIGH_TEMPERATURE",
  "EV_PERIODIC_LOG"
};

char eJsonCommandStr [52][50] ={
  "CMD_SET_MODE",                                                      
  "CMD_SET_MOTOR_DIR_MANUAL",
  "CMD_SET_MOTOR_PWM_MANUAL",
  "CMD_SET_BRUSH_MOTOR_STATE",
  "CMD_SET_BRUSH_MOTOR_PWM_MANUAL",
  "CMD_SET_BRUSH_MOTOR_DIR",
  "CMD_SET_AUTO_MODE_STATE",
  "CMD_SET_MOTOR_PWM_AUTO",
  "CMD_SET_BRUSH_MOTOR_PWM_AUTO",
  "CMD_SET_AUTO_SCHEDULE_TIME",
  "CMD_SET_EDGE_SENSE_STATE",
  "CMD_SET_RTC_VALUE",
  "CMD_SET_DEVICE_INFO",
  "CMD_SET_OVERCURRENT_LIMIT",
  "CMD_CLEAR_ERRORCODES",
  "CMD_GET_SOC",
  "CMD_GET_CHARGER_STATE",
  "CMD_GET_CONTROLLER_STATUS",
  "CMD_GET_DEVICE_INFO",
  "CMD_GET_CURRENT_MODE",
  "CMD_GET_MOTOR_DIR_MANUAL",
  "CMD_GET_MOTOR_PWM_MANUAL",
  "CMD_GET_BRUSH_MOTOR_STATE",
  "CMD_GET_BRUSH_MOTOR_PWM_MANUAL",
  "CMD_GET_BRUSH_MOTOR_DIR",
  "CMD_GET_MOTOR_PWM_AUTO",
  "CMD_GET_BRUSH_MOTOR_PWM_AUTO",
  "CMD_GET_AUTO_SCHEDULE_TIME",
  "CMD_GET_EDGE_SENSE_STATE",
  "CMD_GET_RTC_VALUE",
  "CMD_GET_CURRENT_STAT",
  "CMD_GET_LAST_CYCLE_STAT",
  "CMD_GET_OVERCURRENT_LIMIT",
  "CMD_GET_MOT_CURRENT_VALUES", 
  "CMD_SET_PULSE_COUNT_VALUES",
  "CMD_SET_LOW_BAT_SOC",
  "CMD_GET_PULSE_COUNT_VALUES",
  "CMD_GET_LOW_BAT_SOC",
  "CMD_SET_ZIGBEE_CONF",
  "CMD_GET_ZIGBEE_CONF",
  "CMD_GET_ZIGBEE_NTW_PARAM",
  "CMD_RESET_ZIGBEE_NTW",
  "CMD_GET_BAT_AND_PV_VAL",
  "CMD_RESET_TO_DEFAULTS",
  "CMD_GET_TEMPERATURES",
  "CMD_SET_MOTOR_FAULT_CONDITION",
  "CMD_GET_MOTOR_FAULT_CONDITION",
  "CMD_REQ_ACTIVITY_LOG",
  "CMD_CLEAR_ACTIVITY_LOG",
  "CMD_SET_PULSE_COUNT_AND_CYCLE",
  "CMD_GET_CURRENT_ROBOT_STATUS",
  "CMD_INVALID"
};
char eJsonStatusStr[9][50]  = {
  "JSON_NO_ERROR",
  "JSON_PARSE_ERROR",
  "JSON_INVALID_CMD_VALUE",
  "JSON_INVALID_DATA",  
  "JSON_DATA_OUT_OF_RANGE",
  "JSON_EEPROM_ERROR",
  "JSON_INVALID_DEV_ID", 
  "JSON_WILDCARD_DEV_ID",
  "JSON_NULL_DATA"
};

char eControlModeStr[2][30] = {"AUTO_CONTROL","MANUAL_CONTROL"};

typedef struct stLogValues_def{
  bool edgeSense[2];
  uint32_t rSenseCount[2];
  char Vin[7];
  char Vbat[7];
  char Iin[7];
  char Ibat[7];
  char Dtemp[7];
  char BatPercent[7];
  char Imt1[7];
  char Imt2[7];
  char Ibrush[7];
  bool limitSwitch[2];
  uint32_t proxCount;
  uint16_t driveData[3][6];
}stLogValues;


void SdLogTimer_ms(void)
{
  sdLogTimer_ms++;
  if(sdLogTimer_ms-sdPrevLogTime_ms>=PERIODIC_LOG_TIME_MS)
  {
    periodicLogBit = 1;
  }
}

uint32_t GetSdLogTimer_ms(void)
{
  return sdLogTimer_ms;
}

void PeriodicLogging(void)
{
  if(periodicLogBit)
  {
    LogEvent(EV_PERIODIC_LOG);
    periodicLogBit = 0;
    sdPrevLogTime_ms = GetSdLogTimer_ms();
  }
}

void GetLogValues(stLogValues* logValue)
{
  logValue->edgeSense[0] = IsEdgeSensor1Detected();
  logValue->edgeSense[1] = IsEdgeSensor2Detected();
  logValue->rSenseCount[0] = GetRotateSense1Count();
  logValue->rSenseCount[1] = GetRotateSense2Count();
  float tempValue = GetSOCAsPercentage();
  ftoa(tempValue,logValue->BatPercent,2);
  tempValue = GetInputVoltage();
  ftoa(tempValue,logValue->Vin,2);
  tempValue = GetBatteryVoltage();
  ftoa(tempValue,logValue->Vbat,2);
  tempValue = GetInputCurrent();
  ftoa(tempValue,logValue->Iin,2);
  stBatCurrent batCurr;
  tempValue = GetBatteryCurrent(&batCurr);
  ftoa(tempValue,logValue->Ibat,2);
  tempValue = GetImot1Value();
  ftoa(tempValue,logValue->Imt1,2);
  tempValue = GetImot2Value();
  ftoa(tempValue,logValue->Imt2,2);
  tempValue = GetImot3Value();
  ftoa(tempValue,logValue->Ibrush,2);
  tempValue = GetDieTemperature();
  ftoa(tempValue,logValue->Dtemp,2);
  logValue->limitSwitch[0] = IsEdgeSensor1Detected();
  logValue->limitSwitch[1] = IsEdgeSensor2Detected();
  logValue->proxCount = GetRotateSenseCount();
  GetDriveAllData (DRIVE_1, logValue->driveData[0], (sizeof logValue->driveData[0] / sizeof logValue->driveData[0][0]));
  GetDriveAllData (DRIVE_2, logValue->driveData[1], (sizeof logValue->driveData[1] / sizeof logValue->driveData[1][0]));
  GetDriveAllData (DRIVE_3, logValue->driveData[2], (sizeof logValue->driveData[2] / sizeof logValue->driveData[2][0]));
}

void LogEvent(eEventType eventType)
{
  if(SdcardPresent())
  {
    sdPrevLogTime_ms = GetSdLogTimer_ms();
    periodicLogBit = 0;
    char fileName[20]; //= {"NewFile.txt\n"};
    GetFileName(fileName,EVENT);  
    char ReadData[100];
    char defData[] = {"****------------------PRIME EVENT LOG--------------------****"};
    char defData2[] = {"\r\nTime,Event,VIN,VBAT,IIN,IBAT,DTEMP,BAT %,IMT1,IMT2,IBRUSH,LSWITCH1,LSWITCH2,PROXIMITY COUNT,ERROR CODES,FAULT_1,VDS_GDF_1,MAIN_1,IDRIVE_ADT_1,VDS_1,CONFIG_1,FAULT_2,VDS_GDF_2,MAIN_2,IDRIVE_ADT_2,VDS_2,CONFIG_2,FAULT_3,VDS_GDF_3,MAIN_3,IDRIVE_ADT_3,VDS_3,CONFIG_3\r\n"};
    ReadDataFromSDC(ReadData,strlen(defData)+1, fileName, strlen(fileName));
    Serial_Debug(ReadData);
    if(strncmp(defData,ReadData,strlen(defData)))
    {
      WriteDataToSDC(defData,strlen(defData)+3, fileName, strlen(fileName));
      WriteDataToSDC(defData2,strlen(defData2)+3, fileName, strlen(fileName));
    }
    char LogBuffer[300];
    char time[50];
    GetRTCDateTimeString(time,50); 
    char eventName[50];
    snprintf(eventName,50,"%s",eventTypeStr[eventType]);
    stLogValues logValues;
    GetLogValues(&logValues);
    uint16_t bufferLength = snprintf(LogBuffer,300,"%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%d,%d,%d,0x%08X",\
      time,eventName,logValues.Vin,logValues.Vbat,logValues.Iin,logValues.Ibat,logValues.Dtemp,\
        logValues.BatPercent,logValues.Imt1,logValues.Imt2,logValues.Ibrush,logValues.limitSwitch[0],\
          logValues.limitSwitch[1],logValues.proxCount,GetMotorFaultCode());
    
    bufferLength+=snprintf(&LogBuffer[bufferLength],300-bufferLength,",0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x",\
      logValues.driveData[0][0],logValues.driveData[0][1],logValues.driveData[0][2],logValues.driveData[0][3],\
        logValues.driveData[0][4],logValues.driveData[0][5]);
    
    bufferLength+=snprintf(&LogBuffer[bufferLength],300-bufferLength,",0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x",\
      logValues.driveData[1][0],logValues.driveData[1][1],logValues.driveData[1][2],logValues.driveData[1][3],\
        logValues.driveData[1][4],logValues.driveData[1][5]);
    
    bufferLength+=snprintf(&LogBuffer[bufferLength],300-bufferLength,",0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x\r\n",\
      logValues.driveData[2][0],logValues.driveData[2][1],logValues.driveData[2][2],logValues.driveData[2][3],\
        logValues.driveData[2][4],logValues.driveData[2][5]);
    
    Serial_Debug("\r\nBufferLength = ");
    Serial_Debug_Num(bufferLength);
    Serial_Debug("\r\nLog Buffer-> ");
    Serial_Debug(LogBuffer);
    WriteDataToSDC(LogBuffer,strlen(LogBuffer)+3,fileName,strlen(fileName));
  }
}



void LogAPI(stMessage apiMessage,eJsonCommand command, eJsonStatus response,eControlModes mode)
{
  if(SdcardPresent())
  {
    char fileName[20]; //= {"NewFile.txt\n"};
    GetFileName(fileName,API);  
    char ReadData[100];
    char defData[] = {"****------------------PRIME API LOG--------------------****"};
    char defData2[] = {"\r\nTime;Device ID;Command;Data;Response;Gateway/Stick\r\n"};
    ReadDataFromSDC(ReadData,strlen(defData)+1, fileName, strlen(fileName));
    Serial_Debug(ReadData);
    if(strncmp(defData,ReadData,strlen(defData)))
    {
      WriteDataToSDC(defData,strlen(defData)+3, fileName, strlen(fileName));
      WriteDataToSDC(defData2,strlen(defData2)+3, fileName, strlen(fileName));
    }
    char LogBuffer[200];
    char time[50];
    GetRTCDateTimeString(time,50); 
    uint64_t macAddr = apiMessage.deviceID;                                   
    uint16_t maxStrLength = MAC_ID_STR_LEN;
    char macStr[MAC_ID_STR_LEN];
    Int64ToHexString (macAddr, macStr, &maxStrLength);
    Serial_Debug("\r\napiMessage.values = ");
    Serial_Debug(apiMessage.values);
    if(strlen(apiMessage.values)>1)
    {
      apiMessage.values[strlen(apiMessage.values)-2] = 0; //remove CRC
    }
    else
    {
      apiMessage.values[0] = 0; 
    }
    snprintf(LogBuffer,200,"%s;%s;%s;%s;%s;%s\r\n",time,macStr,eJsonCommandStr[command],\
      apiMessage.values,eJsonStatusStr[response],eControlModeStr[mode]);
    Serial_Debug("\r\nLog Buffer-> ");
    Serial_Debug(LogBuffer);
    WriteDataToSDC(LogBuffer,strlen(LogBuffer)+3,fileName,strlen(fileName));
  }
}


void GetFileName(char* fileName, eFileType event)
{
  rtc_datetime_t getTime;
  RTC_GetDatetime(RTC, &getTime);  
  char day[3]; char month[3];char year[5];
  if(getTime.day<10)
  {
    day[0]='0';
    day[1]= getTime.day+48;
    day[2] = 0;
  }
  else
  {
    snprintf(day,3,"%d",getTime.day);
  }
  if(getTime.month<10)
  {
    month[0]='0';
    month[1]= getTime.month+48;
    month[2] = 0;
  }
  else
  {
    snprintf(month,3,"%d",getTime.month);
  }
  snprintf(year,5,"%d",getTime.year);
  if(event == API)
  {
    snprintf(fileName,MAX_CHAR_FOR_FILENAME,"%s%s%s_a.txt",day,month,&year[2]);
  }
  else
  {
    snprintf(fileName,MAX_CHAR_FOR_FILENAME,"%s%s%s_e.txt",day,month,&year[2]);
  }
  Serial_Debug(fileName);
}