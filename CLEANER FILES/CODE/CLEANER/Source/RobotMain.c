#include "board.h"
#include "fsl_port.h"
#include "pin_mux.h"
#include "clock_config.h"

#include "delay.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "LTC4015_Main.h"
#include "adc.h"
#include "spi.h"
#include "i2c_init.h"
#include "PwmDriveSpi.h"
#include "WDTimer.h"
#include "RobotControl.h"       
#include "RotateSenseCommon.h"       
#include "EdgeSenseCommon.h"       
#include "ZigbeeUart.h"       
#include "DebugUart.h"  
#include "ConfigUart.h"      
#include "DefaultValues.h"       
#include "RobotOperation.h"       
#include "PortInterrupt.h"      
#include "Relay.h"               
#include "ZigbeeAPIMode.h"       
#include "MotCurrentFaultCommon.h"       
#include "CommCommon.h"  
#include "store_soc.h"     
#include "FaultProcess.h" 
#include "APIProcessing.h"
#include "ConfigAPIProcess.h"
#include "temp_sensors.h"
#include "eeprommain.h"
#include "Rtc.h"
#include "LPTimer.h"
#include "LedCommon.h"
#include "Serial_Debug.h"
#include "NumStrConversion.h"
#include "Led3.h"
#include "EventLogger.h"
#include "eepromStdtoHalDriver.h"
#include "switch.h"

#include "SDFAT/ff.h"
#include "SDFAT/ffconf.h"
#include "SDFAT/diskio.h"
#include "SDFAT/integer.h"
#include "SDFAT/SDcardoperation.h"

/*******************************************************************************
* Definitions
******************************************************************************/
#define Time 1296000  //for 15days
#define Bmstime 10000

uint32_t Initialtime;
uint32_t Timetodelete;
uint32_t currenttime;

BYTE SD_MOUNT_STATUS;
void DispDebugInfo (void);
static void ProcessUartData(void);
 
extern char Board_ON;
extern char ERROR_OCCURRED;

int Bmscount = 0;
static void BmstransmitTimeron (uint32_t setBmstransmitTimeCount_ms);
static inline void bmstransmitStopTimerStop (void);
static volatile bool bmstransmitStopTimeOver          = true;
static inline void bmstransmitStopTimerStop (void);
static int32_t bmstransmitStop_timecount              = 0;
static int32_t brushMotor_timercount                 = 0;
void BmstransmitStopTimeIncrement_ms (void);
void Bmsoperate(void);

unsigned long long hex2dec(char *hex,BYTE len);

uint8_t rxDataBuffer[MAX_UART_RX_BUFFER_LENGTH];


static void BmstransmitTimeron (uint32_t setBmstransmitTimeCount_ms)
{
  //int i = 0;
  if(setBmstransmitTimeCount_ms == 0)
  {
    bmstransmitStopTimeOver = true;
    bmstransmitStopTimerStop();
  }
  else
  {
    bmstransmitStop_timecount = setBmstransmitTimeCount_ms;
    bmstransmitStopTimeOver = false;
  }
}

static inline void bmstransmitStopTimerStop (void)
{
  bmstransmitStop_timecount = 0;
}

void BmstransmitStopTimeIncrement_ms (void)
{
  if(bmstransmitStop_timecount)
  {
    if(--bmstransmitStop_timecount <= 0)
    {
      bmstransmitStopTimeOver = true;
      bmstransmitStopTimerStop();
    }
    else
    {
      bmstransmitStopTimeOver = false;
    }
  }
}


 void main(void)
{  
  BOARD_InitPins();
  BOARD_BootClockRUN();
  WDTimerInit(WDT_RESET_TIME_START);
  LedInit();
  LPTMR_Initialisation();
  DebugUartInit(); 
  ConfigUartInit();        
  TransmitWelcomeMessage(DEBUG_UART);
  TransmitWelcomeMessage(CONFIG_UART); 
  if(RTCInit() == false)
  {
    SetFault(RTC_FAULT);
    Serial_Debug("\n RTC Initialisation ERROR *~*~*~*~*~*");
  }
  else
  {
    Serial_Debug("\n RTC Initialisation OK");  
  } 
  
  //SpiInit();
  //Direct_SPI_Init();//direct pin inits
  ADCInit();
  
  WDTimerRefresh();
  
  if(EEPROM_Init() == false)
  {
    Serial_Debug("\n EEPROM Initialisation Failed");   
  }
  else
  {
    uint8_t status = (uint8_t)InitialiseEepromVariables();   
    //SetLogEvent(EV_LOG_EEPROM, status);  
  }

  EventLogInit(EEPROM);
  DefaultValuesInit();   
  RelayInit();
//  DoInitialRelayOff();  //07S
  RobotInit();          
  RotateSenseInit();    
  EdgeSenseInit();     
  EnablePortInterrupt();
  BatteryChargerInit();
  InitI2C();
  InitSwitch();
  //msdelay(2000);
  
  if(ZigbeeUartInit() == ERROR)
  {
    SetFault(ZIGBEE_FAULT);
    //SetLogEvent(EV_LOG_ZB_UART, (uint8_t)EV_UART_ERROR);
  }
  else
  {
    //SetLogEvent(EV_LOG_ZB_UART, (uint8_t)EV_UART_STARTED); 
  } 
  WDTimerRefresh();
#ifndef ZIGBEE_API_MODE_ENABLE
  TransmitWelcomeMessage(ZIGBEE_UART);
#endif
  FindAndUpdateImotOffsetValue();
  UpdateWDTimerValue(WDT_RESET_TIME);
  RestartCommTimer();
  
    Direct_SPI_Init();
    SD_MOUNT_STATUS = SDcardMount();
  
  ERROR_OCCURRED = 1;
  Board_ON = 1;
  
  Initialtime = GetRTCDateTimeSeconds();
  Timetodelete = Initialtime+Time;
  
  while (1)
  {
    WDTimerRefresh();
    OperateLeds();
    ChargerErrorHandler();
    ProcessUartData();
    RelayFSM();
    RobotOperate();
    RobotFSM();
    CheckMotorOverLoad();
    ZigbeePoll();
    DispDebugInfo();
    CheckAndStoreQC();
    ControlledCharging();
    SDcardOperation();
    Bmsoperate();
    currenttime = GetRTCDateTimeSeconds();
    if(currenttime>=Timetodelete)
    {
      Delete_SDdata();
      Timetodelete=0;
      Initialtime = GetRTCDateTimeSeconds();
      Timetodelete = Initialtime+Time;
    }
  }
}

static void ProcessUartData(void)
{
  //uint8_t rxDataBuffer[MAX_UART_RX_BUFFER_LENGTH];
  uint64_t rxMacAddress = DEFAULT_RX_MAC_ADDR;
  if(DebugUartReceive(rxDataBuffer, (sizeof(rxDataBuffer)/sizeof rxDataBuffer[0])) == SUCCESS)
  {
    SwitchLed3On();
    SetUartType(DEBUG_UART);
    ProcessReceivedJsonData(rxDataBuffer,strlen(rxDataBuffer));
    SwitchLed3Off();
  }
  if(ZigbeeUartReceive(&rxMacAddress, rxDataBuffer, (sizeof(rxDataBuffer)/sizeof rxDataBuffer[0])) == SUCCESS)
  {
    RestartCommTimer();
    ClearCommTimeOver();
    SwitchLed3On();
    SetUartType(ZIGBEE_UART);
    SetReceivedMACAddress(rxMacAddress);
#ifdef ZIGBEE_JSON_MODE 
    ProcessReceivedJsonData_config(rxDataBuffer);
#else
    ProcessReceivedJsonData(rxDataBuffer,strlen(rxDataBuffer));
#endif
    SetReceivedMACAddress(0);
    SwitchLed3Off();
  }
  if(ConfigUartReceive(rxDataBuffer, (sizeof(rxDataBuffer)/sizeof rxDataBuffer[0])) == SUCCESS)
  {
    SwitchLed3On();
    SetUartType(CONFIG_UART);
    if(GetSwitchState()==AUTO_CONTROL)
    {
      Bmscalculation();
    }
    else if(GetSwitchState() == MANUAL_CONTROL)
    {
      ProcessReceivedJsonData_config(rxDataBuffer);
    }
    RestartCommTimer();
  }
}

void DispDebugInfo (void)
{
  char dispstring[200];
  static uint32_t timeValue = 0;
  uint32_t msTimeCount = GetLPTimerMsTicks();
  if((msTimeCount - timeValue) > 2000)
  {
    timeValue = msTimeCount;
    
    snprintf(dispstring, sizeof dispstring, "\nRotateSenseCount : %d, %d, %d, Edge Sensor : %d, %d",
             GetRotateSenseCount(),GetRotateSense1Count(),GetRotateSense2Count(),
             IsEdgeSensor1Detected(), IsEdgeSensor2Detected()); 
    
    Serial_Debug(dispstring);
    float Imot1 = GetImot1Value();
    float Imot2 = GetImot2Value();
    float Imot3 = GetImot3Value();
    char temp1Str[20], temp2Str[30], temp3Str[30];
    ftoa(Imot1, temp1Str, 2);
    ftoa(Imot2, temp2Str, 2);
    ftoa(Imot3, temp3Str, 2);
    snprintf(dispstring, sizeof dispstring, "\nImot Values-> Imot_1 = %s, Imot_2 = %s, Imot_3 = %s",temp1Str,temp2Str, temp3Str); 
    Serial_Debug(dispstring);  
//    Imot1 = GetTemperatureSensorData(TEMPSENSOR_1);
//    Imot2 = GetTemperatureSensorData(TEMPSENSOR_2);
//    ftoa(Imot1, temp1Str, 2);
//    ftoa(Imot2, temp2Str, 2);
//    snprintf(dispstring, sizeof dispstring, "\nTEMPERATURE-> TEMP_SENSE_1 = %s, TEMP_SENSE_2 = %s",temp1Str,temp2Str); 
//    Serial_Debug(dispstring); 
    BatteryInfo();
  }
}

void Bmsoperate(void)
{
  if(GetSwitchState()==AUTO_CONTROL)
  {
    if((bmstransmitStopTimeOver == true )&&(Bmscount == 0))
    {
      ConfigUartTransmit(":110310000010CC", 15);
      Bmscount=1;
      BmstransmitTimeron(Bmstime);
    }
    else if((bmstransmitStopTimeOver == true )&&(Bmscount == 1))
    {
       ConfigUartTransmit(":11031010000EBE", 15);
      Bmscount=2;
      BmstransmitTimeron(Bmstime);
    }  
    else if((bmstransmitStopTimeOver == true )&&(Bmscount == 2))
    {
      ConfigUartTransmit(":1103101F0006B7", 15);
      Bmscount=0;
      BmstransmitTimeron(Bmstime);
    }   
  }
}

unsigned long long hex2dec(char *hex,BYTE len)
{
     
   long long decimal = 0, base = 1;
   int i = 0, length;

   length = strlen(hex);
    for(i = length--; i >= 0; i--)
    {
        if(hex[i] >= '0' && hex[i] <= '9')
        {
            decimal += (hex[i] - 48) * base;
            base *= 16;
        }
        else if(hex[i] >= 'A' && hex[i] <= 'F')
        {
            decimal += (hex[i] - 55) * base;
            base *= 16;
        }
        else if(hex[i] >= 'a' && hex[i] <= 'f')
        {
            decimal += (hex[i] - 87) * base;
            base *= 16;
        }
    }
    return decimal;
 }
