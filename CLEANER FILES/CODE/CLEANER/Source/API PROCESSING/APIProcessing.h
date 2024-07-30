#ifndef _APIPROCESSING_H_
#define _APIPROCESSING_H_

#include <stdint.h>
#include <stdbool.h> 
#include <stdlib.h>

#include "EepromStore.h"
#include "CommCommon.h"

#define DEVICE_ID               255
#define WILDCARD_DEVICE_ID      0xFFFF
#define JAN_2000_START_SECONDS         946684800
#define DEC_2099_END_SECONDS           4102444799

#define MAX_UART_TX_BUFFER_LENGTH            10000//   1500
#define MAX_UART_RX_BUFFER_LENGTH               500
#define MAX_ACK_TX_BUFFER_LENGTH                200

#define MAX_NO_OF_VALUES 30
#define MAX_COMMAND_SIZE 4
#define START_CHAR      0x01
#define STOP_CHAR       0x04
#define DELIMITER_1     0x02
#define DELIMITER_2     0x03

#define MAX_NO_OF_SET_DATA                      20

#define JSON_NO_OF_MODE_STATE_DATA              1+1 //+1 is CRC    //07D change   
#define JSON_NO_OF_MOTOR_MODE_STATE_DATA        3+1             //SH    //2+1
#define JSON_NO_OF_SENSOR_STATE_DATA            2+1             //SH
#define JSON_NO_OF_PWM_DATA_MOTOR               12+1       //changed from 12
#define JSON_NO_OF_PWM_DATA_BRUSH               4+1
#define JSON_NO_OF_RTC_DATA                     6+1
#define JSON_NO_OF_SET_DEVICE_INFO              3+1
#define JSON_NO_OF_GET_DEVICE_INFO              4+1
#define JSON_NO_OF_CURRENT_LIMIT                3+1
#define JSON_NO_OF_ERROR_CODE_DATA              1+1
//#define JSON_NO_OF_PULSE_COUNT_DATA             2+1
#define JSON_NO_OF_SOC_DATA                     1+1
#define JSON_NO_OF_ZIGBEE_CONFIG_DATA           6+1
#define JSON_NO_OF_ZIGBEE_NW_RESET_DATA         1+1
#define JSON_NO_OF_MOTOR_FAULT_CONDITIONS       4+1
#define JSON_NO_OF_REQUEST_LOG_VALUES           2+1
#define JSON_NO_OF_HEARTBEAT_CONFIG_PARAMETERS  4+1

//#define JSON_NO_OF_WHEELDIA_DATA                     2+1    WORKING DEFAULT
#define JSON_NO_OF_INTERVAL_DATA                     12+1    //NEW
#define JSON_NO_OF_CYCLEFREQUENCY_DATA               1+1    //NEW
#define JSON_NO_OF_CONTINUE                          1+1
#define JSON_NO_OF_RETURN                            1+1
#define JSON_NO_OF_REQUEST_LOG                       1+1

#define JSON_NO_OF_COMDISTANCE             1+1 

#define JSON_NO_AUTOSCH                 19+1
//#define JSON_NO_AUTOSCH1                38+1

#define JSON_NO_OF_PULSE_COUNT_DATA             6+1          //change
#define JSON_NO_OF_PULSE_CYCLE_COUNT_DATA             4+1          //change
#define JSON_NO_OF_WHEELDIA_DATA                     5+1    //NEW
//#define JSON_NO_OF_AUTO_SCHED_DATA      48+1          //working fine

#define JSON_NO_OF_AUTO_SCHED_DATA      2+1

#define JSON_NO_OF_SETGET_PWM_DATA_MOTOR (JSON_NO_OF_PWM_DATA_MOTOR -2)        /* We are using single steady and approach pwm instaed of two */

#define JSON_NO_OF_ROW_LENGTH_DATA              ((MAX_NO_OF_SET_DATA*2)+2+1)
#define JSON_NO_OF_ROWS                     8+1    //NEW
#define JSON_NO_OF_GET_TRACK_INFO               2+1
//#define CYCLEROTATIONTIME     15

static char txBuff[MAX_UART_TX_BUFFER_LENGTH];
static char txAckBuff[MAX_ACK_TX_BUFFER_LENGTH];

//float Time_seconds;
extern int Autoschcountval;
extern float circumference;
extern int brushMotorStopDeadTime_ms;

void Bmscalculation(void);

typedef enum eJsonCommand_def                                                             
{
  CMD_NUM_SET_MODE,                                                      
  CMD_NUM_SET_MOTOR_DIR_MANUAL,
  CMD_NUM_SET_MOTOR_PWM_MANUAL,
  CMD_NUM_SET_BRUSH_MOTOR_STATE,
  CMD_NUM_SET_BRUSH_MOTOR_PWM_MANUAL,
  CMD_NUM_SET_BRUSH_MOTOR_DIR,
  CMD_NUM_SET_AUTO_MODE_STATE,
  CMD_NUM_SET_MOTOR_PWM_AUTO,
  CMD_NUM_SET_BRUSH_MOTOR_PWM_AUTO,
  CMD_NUM_SET_AUTO_SCHEDULE_TIME,
  CMD_NUM_SET_EDGE_SENSE_STATE,
  CMD_NUM_SET_RTC_VALUE,
  CMD_NUM_SET_DEVICE_INFO,
  CMD_NUM_SET_OVERCURRENT_LIMIT,
  CMD_NUM_CLEAR_ERRORCODES,
  CMD_NUM_GET_SOC,
  CMD_NUM_GET_CHARGER_STATE,
  CMD_NUM_GET_CONTROLLER_STATUS,
  CMD_NUM_GET_DEVICE_INFO,
  CMD_NUM_GET_CURRENT_MODE,
  CMD_NUM_GET_MOTOR_DIR_MANUAL,
  CMD_NUM_GET_MOTOR_PWM_MANUAL,
  CMD_NUM_GET_BRUSH_MOTOR_STATE,
  CMD_NUM_GET_BRUSH_MOTOR_PWM_MANUAL,
  CMD_NUM_GET_BRUSH_MOTOR_DIR,
  CMD_NUM_GET_MOTOR_PWM_AUTO,
  CMD_NUM_GET_BRUSH_MOTOR_PWM_AUTO,
  CMD_NUM_GET_AUTO_SCHEDULE_TIME,
  CMD_NUM_GET_EDGE_SENSE_STATE,
  CMD_NUM_GET_RTC_VALUE,
  CMD_NUM_GET_CURRENT_STAT,
  CMD_NUM_GET_LAST_CYCLE_STAT,
  CMD_NUM_GET_OVERCURRENT_LIMIT,
  CMD_NUM_GET_MOT_CURRENT_VALUES, 
  CMD_NUM_SET_PULSE_COUNT_VALUES,
  CMD_NUM_SET_LOW_BAT_SOC,
  CMD_NUM_GET_PULSE_COUNT_VALUES,
  CMD_NUM_GET_LOW_BAT_SOC,
  CMD_NUM_SET_ZIGBEE_CONF,
  CMD_NUM_GET_ZIGBEE_CONF,
  CMD_NUM_GET_ZIGBEE_NTW_PARAM,
  CMD_NUM_RESET_ZIGBEE_NTW,
  CMD_NUM_GET_BAT_AND_PV_VAL,
  CMD_NUM_RESET_TO_DEFAULTS,
  CMD_NUM_GET_TEMPERATURES,
  CMD_NUM_SET_MOTOR_FAULT_CONDITION,
  CMD_NUM_GET_MOTOR_FAULT_CONDITION,
  CMD_NUM_REQ_ACTIVITY_LOG,
  CMD_NUM_CLEAR_ACTIVITY_LOG,
  CMD_NUM_SET_PULSE_COUNT_AND_CYCLE,
  CMD_NUM_GET_CURRENT_ROBOT_STATUS,
  CMD_NUM_SET_HEARTBEAT_CONFIG,
  CMD_NUM_GET_HEARTBEAT_CONFIG,
  CMD_NUM_INVALID,                              //07D included comma
  CMD_NUM_SET_LOG_INTERVAL,                     //54 
  CMD_NUM_GET_LOG_INTERVAL,                     //new
  CMD_NUM_SET_DIA_WHEEL,                        //new
  CMD_NUM_GET_DIA_WHEEL,                         //new
  CMD_NUM_SET_CYCLE_FREQUENCY,                    //new
  CMD_NUM_GET_CYCLE_FREQUENCY,                    //new
  CMD_NUM_SET_CONTINUE,                           //new
  CMD_NUM_GET_CONTINUE_COUNT,                     //new
  CMD_NUM_SET_EMERGENCY_RETURN,                   //new
  CMD_NUM_GET_RETURN_STATE,
  CMD_NUM_SET_COM_DISTANCE,
  CMD_NUM_GET_COM_DISTANCE,
  CMD_NUM_SET_N0_ROW_DAY,
  CMD_NUM_GET_N0_ROW_DAY,
  CMD_NUM_SET_ROW_LENGTH_DATA,
  CMD_NUM_GET_ROW_LENGTH_DATA,
  CMD_NUM_SET_ROW_LENGTH2_DATA,
  CMD_NUM_GET_ROW_LENGTH2_DATA,
  CMD_NUM_SET_ROW_LENGTH3_DATA,
  CMD_NUM_GET_ROW_LENGTH3_DATA,
  CMD_NUM_SET_ROW_LENGTH4_DATA,
  CMD_NUM_GET_ROW_LENGTH4_DATA,
  CMD_NUM_SET_ROW_LENGTH5_DATA,
  CMD_NUM_GET_ROW_LENGTH5_DATA,
  CMD_NUM_SET_ROW_LENGTH6_DATA,
  CMD_NUM_GET_ROW_LENGTH6_DATA,
  CMD_NUM_SET_ROW_LENGTH7_DATA,
  CMD_NUM_GET_ROW_LENGTH7_DATA,
  CMD_NUM_REQ_LOG
}eJsonCommand;

typedef enum eJsonStatus_def {
  JSON_NO_ERROR,
  JSON_PARSE_ERROR,
  JSON_INVALID_CMD_VALUE,
  JSON_INVALID_DATA,  
  JSON_DATA_OUT_OF_RANGE,
  JSON_EEPROM_ERROR,
  JSON_INVALID_DEV_ID, 
  JSON_WILDCARD_DEV_ID,
  JSON_NULL_DATA
}eJsonStatus;

typedef eJsonStatus (*RxCommandProcess_Ptr)(const char * jsonString);
typedef void (*AssignDefaultProcess_Ptr)(void);

char Get_SD_Card_Temps();
char Get_SD_Card_SOC();
char Get_SD_Card_MOTORCRNT();


typedef struct stJsonCommand_def {
  eJsonCommand command;
  const uint16_t command_Hex;
  RxCommandProcess_Ptr executeCommand;
  EepromStoreProcess_Ptr executeStoreProcess;
  AssignDefaultProcess_Ptr executeAssignDefaultProcess;
}stJsonCommand;

typedef struct stMessage_def
{
uint64_t deviceID;
uint16_t CMD;
uint8_t values[100];
uint8_t CRC;
}stMessage;

static const stJsonCommand * presentJsonCommandParams = NULL; 

typedef bool (*CheckValidity_Ptr_float)(float value);
typedef bool (*CheckValidity_Ptr_int)(uint32_t value);
typedef bool (*CheckValidity_Ptr_double)(double value);
typedef bool (*CheckValidity_Ptr_bool)(bool value);

void SetReceivedMACAddress(uint64_t macAddr);
uint64_t GetReceivedMACAddress(void);

eJsonStatus ProcessReceivedJsonData (uint8_t * jsonReceivedData,uint16_t size);
static eJsonStatus AssignJsonCommand (uint16_t  presentCommand_Hex); 
static eJsonStatus ProcessJsonCommand (const uint8_t * jsonString, uint16_t presentCommand_Hex);
static eJsonStatus ParseRxdata(stMessage* message);
uint16_t GetCommandProcessingParam (stJsonCommand **jsonCmdParam);

static eJsonStatus ProcessSetMode (const char * jsonString);
static eJsonStatus ProcessSetManualDirection (const char * jsonString);
static eJsonStatus ProcessSetMotorPWMManual (const char * jsonString);
static eJsonStatus ProcessSetBrushMotorState (const char * jsonString);
static eJsonStatus ProcessSetBrushPWMManual (const char * jsonString);
static eJsonStatus ProcessSetBrushDirection (const char * jsonString);
static eJsonStatus ProcessSetMotorPWMAuto (const char * jsonString);
static eJsonStatus ProcessSetBrushPWMAuto (const char * jsonString);
static eJsonStatus ProcessSetAutoScheduleTime (const char * jsonString);
static eJsonStatus ProcessSetEdgeSensorState (const char * jsonString);
static eJsonStatus ProcessSetRTCValue (const char * jsonString);
static eJsonStatus ProcessSetDeviceInfo (const char * jsonString);
static eJsonStatus ProcessSetMotorCurrentLimits (const char * jsonString);
static eJsonStatus ProcessClearErrorCodes (const char * jsonString);
static eJsonStatus ProcessSetLowBatSoC (const char * jsonString);
static eJsonStatus ProcessSetPulseCount (const char * jsonString);
static eJsonStatus ProcessResetToDefaults (const char * jsonString);
static eJsonStatus ProcessSetMotorFaultConditions (const char * jsonString);
static eJsonStatus ClearActivityLog (const char * jsonString);
static eJsonStatus ProcessSetPulseCountAndCycle (const char * jsonString);
static eJsonStatus ProcessSetHeartbeatConfig (const char * jsonString);

static eJsonStatus ProcessGetSoC (const char * jsonString);
static eJsonStatus ProcessGetChargerState (const char * jsonString);
static eJsonStatus ProcessGetErrorStatus (const char * jsonString);
static eJsonStatus ProcessGetDeviceInfo (const char * jsonString);
static eJsonStatus ProcessGetCurrentMode (const char * jsonString);
static eJsonStatus ProcessGetMotorDirectionManual (const char * jsonString);
static eJsonStatus ProcessGetMotorPWMManual (const char * jsonString);
static eJsonStatus ProcessGetBrushMotorEnabledState (const char * jsonString);
static eJsonStatus ProcessGetBrushMotorPWMManual (const char * jsonString);
static eJsonStatus ProcessGetBrushMotorPolarity (const char * jsonString);
static eJsonStatus ProcessGetMotorPWMAuto (const char * jsonString);
static eJsonStatus ProcessGetBrushMotorPWMAuto (const char * jsonString);
static eJsonStatus ProcessGetAutoScheduleTime (const char * jsonString);
static eJsonStatus ProcessGetEdgeSenseEnabledState (const char * jsonString);
static eJsonStatus ProcessGetRTCValue (const char * jsonString);
static eJsonStatus ProcessGetCurrentStat (const char * jsonString);
static eJsonStatus ProcessGetLastOperationStat (const char * jsonString);
static eJsonStatus ProcessGetOverCurrentLimits (const char * jsonString);
static eJsonStatus ProcessGetMotorCurrentValues (const char * jsonString);
static eJsonStatus ProcessGetPulseCountValues (const char * jsonString);
static eJsonStatus ProcessGetLowBatSoC (const char * jsonString);
static eJsonStatus ProcessSetZigbeeConfig (const char * jsonString);
static eJsonStatus ProcessResetZigbeeNetwork (const char * jsonString);
static eJsonStatus ProcessGetZigbeeNetworkParams (const char * jsonString);
static eJsonStatus ProcessGetZigbeeConfig (const char * jsonString);
static eJsonStatus ProcessGetBatteryAndPVvalues (const char * jsonString);
static eJsonStatus ProcessGetTemperaturevalues (const char * jsonString);
static eJsonStatus ProcessGetMotorFaultConditions (const char * jsonString);
static eJsonStatus RequestActivityLog (const char * jsonString);
static eJsonStatus ProcessGetCurrentRobotStat (const char * jsonString);
static eJsonStatus ProcessGetHeartbeatConfig (const char * jsonString);

//07D change

static eJsonStatus ProcessSetWheelDia (const char * jsonString);
static eJsonStatus ProcessGetWheelDia (const char * jsonString);
static eJsonStatus ProcessSetLogInterval (const char * jsonString);
static eJsonStatus ProcessGetLogInterval (const char * jsonString);
static eJsonStatus ProcessSetCycleFrequency (const char * jsonString);
static eJsonStatus ProcessGetCycleFrequency (const char * jsonString);

static eJsonStatus ProcessSetContinue (const char * jsonString);
static eJsonStatus ProcessGetContinueCount (const char * jsonString);
static eJsonStatus ProcessSetReturnState (const char * jsonString);
static eJsonStatus ProcessGetReturnState (const char * jsonString);

static eJsonStatus ProcessSetcomdistance (const char * jsonString);
static eJsonStatus ProcessGetcomdistance (const char * jsonString);

static eJsonStatus ProcessSetNoofRowday (const char * jsonString);
static eJsonStatus ProcessGetNoofRowday (const char * jsonString);
static eJsonStatus ProcessSetRowLengthData (const char * jsonString);
static eJsonStatus ProcessGetRowLengthData (const char * jsonString);
static eJsonStatus ProcessSetRowLength2Data (const char * jsonString);
static eJsonStatus ProcessGetRowLength2Data (const char * jsonString);
static eJsonStatus ProcessSetRowLength3Data (const char * jsonString);
static eJsonStatus ProcessGetRowLength3Data (const char * jsonString);
static eJsonStatus ProcessSetRowLength4Data (const char * jsonString);
static eJsonStatus ProcessGetRowLength4Data (const char * jsonString);
static eJsonStatus ProcessSetRowLength5Data (const char * jsonString);
static eJsonStatus ProcessGetRowLength5Data (const char * jsonString);
static eJsonStatus ProcessSetRowLength6Data (const char * jsonString);
static eJsonStatus ProcessGetRowLength6Data (const char * jsonString);
static eJsonStatus ProcessSetRowLength7Data (const char * jsonString);
static eJsonStatus ProcessGetRowLength7Data (const char * jsonString);

static eJsonStatus RequestLog (const char * jsonString);

void SD_SOC_Start();
void SD_SOC_End();
void SD_SOC_Start_21();
void SD_SOC_End_21();

void PulseCalculation(void);

//float circumferencefunc(float Dia_value);
//void PulseCalculation(float value);                               //07D

uint32_t Brushstop_time(int Btime);

void TransmitWelcomeMessage (eUartType uarttType);

extern eEepromStatus InitialiseAlltoDefault(stJsonCommand *jsonCmdParams, uint16_t commandParamsTotalCount);
void SetDefaultNil(void);

static void Error_Handler(void);
//void ExtractData(uint8_t* sourceString, uint8_t* destString, uint8_t delimiter1, uint8_t delimiter2);
eJsonStatus ExtractCommand(uint8_t* sourceBuffer,uint16_t size, uint16_t* command);
eJsonStatus ExtractData(uint8_t* sourceBuffer,uint8_t size,stMessage* message);
static bool  SetParameterValue_bool(const char * inputString, bool * number, CheckValidity_Ptr_bool ExecuteCheckValidity);
static bool  SetParameterValue_int(const char * inputString, uint32_t * number, CheckValidity_Ptr_int ExecuteCheckValidity);
static bool  SetParameterValue_float(const char * inputString, float * number, CheckValidity_Ptr_float ExecuteCheckValidity);
static bool  SetParameterValue_double(const char * inputString, double * number, CheckValidity_Ptr_double ExecuteCheckValidity);
#endif