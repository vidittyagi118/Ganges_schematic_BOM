#include "store_soc.h"
#include "LTC4015_Main.h"
#include "VBatReg.h"

#define BATTERY_VOLT_PERCENT_100         27
#define BATTERY_VOLT_PERCENT_80          26.4
#define BATTERY_VOLT_PERCENT_60          26.1
#define BATTERY_VOLT_PERCENT_40          25.8
#define BATTERY_VOLT_PERCENT_20          25.5
#define BATTERY_VOLT_PERCENT_10          24.9
#define BATTERY_VOLT_PERCENT_0           20


volatile uint32_t storeTime = 0;

bool IsTimeToStore(void)
{
if((GetBatChargerTimer()-storeTime)>=TIME_TO_UPDATE*1000)
{
storeTime = GetBatChargerTimer() ;
return true;
}
return false;
}

void StoreQC(uint32_t value)
{
  volatile uint32_t * QCount = GetQCountStoreRegPointer();
 *QCount = value;
}

uint32_t GetStoredQC(void)
{
  volatile uint32_t * QCount = GetQCountStoreRegPointer();
  return *QCount;
}

void CheckAndStoreQC(void)
{
if(IsTimeToStore())
{
StoreQC((uint32_t)GetQCountValue());
}
}

uint16_t GetQCountFromVBat(void)
{
  float battery = GetBatteryVoltage();
  uint8_t batPercent = 0;
  uint16_t QCount = 0;
  if((battery>=BATTERY_VOLT_PERCENT_100))
  {
    batPercent = 100;
  }
  else if((battery<BATTERY_VOLT_PERCENT_100)&&(battery>=BATTERY_VOLT_PERCENT_80))
  {
    batPercent = 80;
  }
  else if((battery<BATTERY_VOLT_PERCENT_80)&&(battery>=BATTERY_VOLT_PERCENT_60))
  {
    batPercent = 60;
  }
  else if((battery<BATTERY_VOLT_PERCENT_60)&&(battery>=BATTERY_VOLT_PERCENT_40))
  {
    batPercent = 40;
  }
  else if((battery<BATTERY_VOLT_PERCENT_40)&&(battery>=BATTERY_VOLT_PERCENT_20))
  {
    batPercent = 20;
  }
  else if((battery<BATTERY_VOLT_PERCENT_20)&&(battery>=BATTERY_VOLT_PERCENT_10))
  {
    batPercent = 10;
  }
  else if((battery<BATTERY_VOLT_PERCENT_10)&&(battery>=BATTERY_VOLT_PERCENT_0))
  {
    batPercent = 0;
  }
  else
  {
    batPercent = 0;
  }
  QCount = (int)((QCOUNT_MAXIMUM - QCOUNT_MINIMUM)*batPercent/100)+QCOUNT_MINIMUM;
  return QCount;
}