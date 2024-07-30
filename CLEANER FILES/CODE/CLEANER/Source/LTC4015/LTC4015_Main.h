#ifndef _LTC4015_MAIN_H_
#define _LTC4015_MAIN_H_
#include "board.h"

typedef struct BatParam{float current;bool direction;}stBatCurrent;

#define RSENSE_IN       4  //in milliOhms
#define RSENSE_BAT      4  //in milliOhms

#define CHARGE_CURRENT 3 //A

#define QCOUNT_MINIMUM 0
#define QCOUNT_MAXIMUM 65535

#define CHARGE 1
#define DISCHARGE 0

#define DEFAULT_BATTERY_CAPACITY 20 //Ah
#define SOC_UPDATE_TIME         3000 //ms

#define CHARGE_CURRENT_HIGH_TEMP 1 //A

#define TEMP_LIMIT_MAX  110
#define TEMP_LIMIT_MIN  80

#define SUSPEND_CHARGER_INTERVAL        15 //MINS

void BatChargerTimer(void); //has to be called every 1 ms
uint32_t GetBatChargerTimer(void);

bool BatteryChargerInit(void);
int SetQcountFromRegister(void);
int SetQcountFromBatVolt(void);

void ControlledCharging(void);

float GetInputVoltage(void);
float GetInputCurrent(void);
float GetBatteryVoltage(void);
float GetBatteryCurrent(stBatCurrent* batCurr);
float GetDieTemperature(void);
float GetSYSVoltage(void);
uint16_t GetChargerState(void);
uint16_t GetQCountValue(void);
float GetSOCAsPercentage(void);
bool GetChargingOrNot(void);

int  SuspendCharger(bool data);
int  ForceToMeasure(bool data);
int  SetPrescaleValue(float data);
int  SetQCountValue(uint16_t data);
int  SetQCountFromPercentage(float Percent);
int  EnableJEITA(bool data);

void ClearSmbAlert(void);
void SMB_Irq(void);
void ChargerErrorHandler(void); //This has to be called continously in while
uint8_t GetBatteryError(void);
void ClearBatteryError(void);


void ReadAndDisplayConfigData(void);
void BatteryInfo(void);
uint16_t GetRegisterValue(uint16_t registerAddress);//refer bit field addresses from LTC4015_reg_defs.h 
int WriteToRegister(uint16_t registerAddress, uint16_t data);

#endif