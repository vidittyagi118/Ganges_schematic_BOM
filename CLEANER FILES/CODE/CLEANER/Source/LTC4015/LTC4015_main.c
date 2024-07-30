#include "LTC4015_Main.h"
#include "LTC4015.h"
#include "bat_i2c.h"
#include "Serial_Debug.h"
#include "board.h"
#include "PortInterrupt.h"
#include "fsl_port.h"
#include "delay.h"
#include <stdlib.h>
#include <stdio.h>
#include "NumStrConversion.h"
#include "store_soc.h"
#include "Rtc.h"
#include "temp_sensors.h"



LTC4015 chip;
port_configuration_t I2C_Data_2;

enum {
 success = 0,
 failure = !success
};

volatile bool smbalert = 0;
    
uint8_t batteryError = 0;

static float prevSOCPercent = 0;

uint8_t GetIChargeValue(float current); //Gives out ICHARGE setting based on current

static void InitLTC4015(void);
static void ConfigSmbAlertInterrupt (void);


void SMB_Irq(void)
{
  smbalert=true;
}

static void InitLTC4015(void)
{ 
  InitBatI2C();
  I2C_Data_2.I2C_No = BOARD_BAT_I2C_BASEADDR; 
  LTC4015_chip_cfg_t cfg =
  {
    .addr = LTC4015_ADDR_68,
    .read_register = BatI2C_ReadRegs,
    .write_register = BatI2C_WriteReg,
    //!< Pointer to port configuration struct not used without physical port
    .port_configuration = &I2C_Data_2
  };
  
  chip = LTC4015_init(&cfg);
}

void BatChargerTimer(void)
{
  BatI2CTimer();
}
uint32_t GetBatChargerTimer(void)
{
 return(GetBatI2CTime());
}

uint8_t GetIChargeValue(float current)
{
uint8_t temp = (uint8_t)((float)(current*RSENSE_BAT)-1);
return temp;
}

bool BatteryChargerInit(void)
{  
  InitLTC4015(); 
  
  int status = success;
  status |= SuspendCharger(1);
  msdelay(500);
  status |= SuspendCharger(0);
  status |= ForceToMeasure(1);
  msdelay(100);
  status |= EnableJEITA(1);
  msdelay(100);
  status |= WriteToRegister(LTC4015_MPPT_EN_I2C_BF,0);
  status |= WriteToRegister(LTC4015_ICHARGE_TARGET,GetIChargeValue(CHARGE_CURRENT));
  if(GetRegisterValue(LTC4015_EN_JEITA_BF))
  {  
  status |= WriteToRegister(LTC4015_ICHARGE_JEITA_2_BF,GetIChargeValue(CHARGE_CURRENT));
  status |= WriteToRegister(LTC4015_ICHARGE_JEITA_3_BF,GetIChargeValue(CHARGE_CURRENT));
  status |= WriteToRegister(LTC4015_ICHARGE_JEITA_4_BF,GetIChargeValue(CHARGE_CURRENT));
  status |= WriteToRegister(LTC4015_ICHARGE_JEITA_5_BF,GetIChargeValue(CHARGE_CURRENT));
  status |= WriteToRegister(LTC4015_ICHARGE_JEITA_6_BF,GetIChargeValue(CHARGE_CURRENT));
  }
  status |= SetPrescaleValue(0); // passing 0 sets to default battery capacity or the actual capacity in Ah can be passed 
  status |= WriteToRegister(LTC4015_EN_QCOUNT_BF,1); //Enable coulomb counter
  
  //status |= SetQCountValue((uint16_t)GetStoredQC());
  if(IsRTCFreshStart())
  {
    status|= SetQcountFromBatVolt();
  }
  else
  {
    if(GetStoredQC()>=6553)
    {
      status|=SetQcountFromRegister();
    }
    else
    {
      status|= SetQcountFromBatVolt();
    }
  }
    
  status |= WriteToRegister(LTC4015_EN_BAT_MISSING_FAULT_ALERT_BF,0x1); //Enable battery missing alert
  ClearSmbAlert();
  status |= WriteToRegister(LTC4015_BAT_MISSING_FAULT_ALERT_BF,0); //clear previous battery missing fault
 
  if(status == success)
  {
    ConfigSmbAlertInterrupt();
    ReadAndDisplayConfigData();
  }
  else
  {
    Serial_Debug("\nBattery Charger Initialisation Write Error");
  }
  return (status == success);
}

int SetQcountFromRegister(void)
{
return(SetQCountValue((uint16_t)GetStoredQC()));
}

int SetQcountFromBatVolt(void)
{
return(SetQCountValue(GetQCountFromVBat()));
}

static void ConfigSmbAlertInterrupt (void)
{
    port_pin_config_t pull_up_Config = {
    kPORT_PullUp,
    kPORT_SlowSlewRate,
    kPORT_PassiveFilterDisable,
    kPORT_OpenDrainDisable,
    kPORT_HighDriveStrength,
    kPORT_MuxAsGpio,
    kPORT_UnlockRegister,     
  }; 
  
  PORT_SetPinConfig(LTC4015_PORT, SMBALERT_PIN, &pull_up_Config);
  GPIO_PinInit(LTC4015_GPIO, SMBALERT_PIN, &(gpio_pin_config_t){kGPIO_DigitalInput, 0});
  PORT_SetPinInterruptConfig(LTC4015_PORT, SMBALERT_PIN, kPORT_InterruptFallingEdge);
  EnableSmbPortInterrupt();
}

void ClearSmbAlert(void)
{
  uint8_t tempAlert = 0;
  BatI2C_ReadAlert(0x0C,0,&tempAlert,BOARD_BAT_I2C_BASEADDR);
}

void ReadAndDisplayConfigData(void)
{
  uint16_t cellCount;
  cellCount = GetRegisterValue(LTC4015_VCHARGE_SETTING_BF);
  Serial_Debug("\r\nVCHARGE SETTING-->");
  Serial_Debug_Num(cellCount);
  cellCount = GetRegisterValue(LTC4015_CELL_COUNT_PINS_BF);
  Serial_Debug("\r\nCell Count-->");
  Serial_Debug_Num(cellCount);
  cellCount = GetRegisterValue(LTC4015_CHEM_BF);
  Serial_Debug("\r\nChem-->");
  Serial_Debug_Num(cellCount);
  cellCount = GetRegisterValue(LTC4015_EN_C_OVER_X_TERM_BF);
  Serial_Debug("\r\nC over X-->");
  Serial_Debug_Num(cellCount);
  cellCount = GetRegisterValue(LTC4015_VABSORB_DELTA_BF);
  Serial_Debug("\r\nVABSORB_DELTA-->");
  Serial_Debug_Num(cellCount);
  cellCount = GetRegisterValue(LTC4015_SUSPEND_CHARGER_BF);
  Serial_Debug("\r\nSuspend register-->");
  Serial_Debug_Num(cellCount);
  cellCount = GetRegisterValue(LTC4015_MAX_CHARGE_TIME);
  Serial_Debug("\r\nCHARGE time-->");
  Serial_Debug_Num(cellCount);
  cellCount = GetRegisterValue(LTC4015_MAX_CV_TIME_BF);
  Serial_Debug("\r\nMAX CV time-->");
  Serial_Debug_Num(cellCount);
  cellCount = GetRegisterValue(LTC4015_EN_QCOUNT_BF);
  Serial_Debug("\r\nLTC4015_EN_QCOUNT_BF-->");
  Serial_Debug_Num(cellCount);

  cellCount = GetRegisterValue(LTC4015_MPPT_EN_I2C_BF);
  Serial_Debug("\r\nLTC4015_MPPT_EN_I2C_BF-->");
  Serial_Debug_Num(cellCount);

  cellCount = GetRegisterValue(LTC4015_ICHARGE_JEITA_2_BF);
  Serial_Debug("\r\n LTC4015_ICHARGE_JEITA_2_BF-->");
  Serial_Debug_Num(cellCount);
  cellCount = GetRegisterValue(LTC4015_ICHARGE_JEITA_6_BF);
  Serial_Debug("\r\n LTC4015_ICHARGE_JEITA_6_BF-->");
  Serial_Debug_Num(cellCount);

  cellCount = GetRegisterValue(LTC4015_EN_LIMIT_ALERTS);
  Serial_Debug("\r\n LTC4015_EN_LIMIT_ALERTS-->");
  Serial_Debug_Num(cellCount);
  cellCount = GetRegisterValue(LTC4015_EN_CHARGER_STATE_ALERTS);
  Serial_Debug("\r\n LTC4015_EN_CHARGER_STATE_ALERTS-->");
  Serial_Debug_Num(cellCount);

  cellCount = GetRegisterValue(LTC4015_EN_CHARGE_STATUS_ALERTS);
  Serial_Debug("\r\n LTC4015_EN_CHARGE_STATUS_ALERTS-->");
  Serial_Debug_Num(cellCount);

  cellCount = GetRegisterValue(LTC4015_EN_CHARGER_STATE_ALERTS);
  Serial_Debug("\nIn Main bat mis fault-->");
  Serial_Debug_Num(cellCount);
}

void ChargerErrorHandler(void)
{
    if(smbalert)
    {
      ClearSmbAlert();
      if(GetRegisterValue(LTC4015_BAT_MISSING_FAULT_ALERT_BF))
      {
      batteryError = 1;  
      SetQCountValue(0);
      WriteToRegister(LTC4015_EN_QCOUNT_BF,0);
      }
      smbalert = false;
    }
}

uint8_t GetBatteryError(void)
{
  return batteryError;
}

void ClearBatteryError(void)
{
  //if(GetRegisterValue(LTC4015_BAT_MISSING_FAULT_ALERT_BF))
  //{  
    WriteToRegister(LTC4015_BAT_MISSING_FAULT_ALERT_BF,0)  ;
    batteryError = 0;
    WriteToRegister(LTC4015_EN_QCOUNT_BF,1);
    SetQcountFromBatVolt(); 
    prevSOCPercent = (float)((float)(GetQCountValue()-QCOUNT_MINIMUM)/(QCOUNT_MAXIMUM-QCOUNT_MINIMUM))*100;
  //}
}

void SoftResetCharger(void)
{
  SuspendCharger(1);
  msdelay(500);
  SuspendCharger(0);
}

int  SetPrescaleValue(float data)
{
  float temp;  
  if(data == 0)
  {
    temp = DEFAULT_BATTERY_CAPACITY * 3600;
  }
  else
  {
    temp = data * 3600;
  }
  temp = (float)temp/65535;
  
  temp = (float)temp*8333.33*(0.004);//*2;
  //Serial_Debug_Float(temp,3);
  Serial_Debug("\r\nPrescale Value-->");
  int preScaleVal = (int)temp;
  float diff = temp-preScaleVal;
  if(diff>0.5)
  {
    preScaleVal+=1;
  }
  Serial_Debug_Num(preScaleVal);
  return (LTC4015_write_register(chip,LTC4015_QCOUNT_PRESCALE_FACTOR,preScaleVal));
}


uint16_t GetQCountValue(void)
{
  uint16_t data;
  LTC4015_read_register(chip, LTC4015_QCOUNT, &data);
  return data;  
}

float GetSOCAsPercentage(void)
{
  uint32_t currentTime_soc = GetBatChargerTimer();
  static uint32_t prevTime_soc = 0;
  if(((currentTime_soc-prevTime_soc)>=SOC_UPDATE_TIME)||(prevTime_soc==0))
  {
    uint16_t QCountVal = GetQCountValue();
    float SOCPercent = (float)((float)(QCountVal-QCOUNT_MINIMUM)/(QCOUNT_MAXIMUM-QCOUNT_MINIMUM))*100;
    prevSOCPercent = SOCPercent;
    prevTime_soc = currentTime_soc;
    //Serial_Debug("\r\nBatPercent-->");
    //Serial_Debug_Float(SOCPercent,3);
    return SOCPercent;
  }
  else
  {
    return prevSOCPercent;
  }
}

int  SetQCountValue(uint16_t data)
{
  return (LTC4015_write_register(chip,LTC4015_QCOUNT,data));
}

int SetQCountFromPercentage(float Percent)
{
  uint16_t QCount = (int)((QCOUNT_MAXIMUM - QCOUNT_MINIMUM)*Percent/100)+QCOUNT_MINIMUM;
  Serial_Debug("\r\nQcount-->");
  Serial_Debug_Num(QCount);
  return (LTC4015_write_register(chip,LTC4015_QCOUNT,QCount));
}

uint16_t GetRegisterValue(uint16_t registerAddress)
{
  uint16_t data;
  LTC4015_read_register(chip, registerAddress, &data);
  return data;
}

int WriteToRegister(uint16_t registerAddress, uint16_t data)
{
  return(LTC4015_write_register(chip,registerAddress,data));
}

uint16_t GetChargerState(void)
{
  uint16_t data;
  LTC4015_read_register(chip, LTC4015_CHARGER_STATE, &data);
  return data;
}

int ForceToMeasure(bool data)
{
  return (LTC4015_write_register(chip,LTC4015_FORCE_MEAS_SYS_ON_BF,data));
}

int  SuspendCharger(bool data)
{
  return (LTC4015_write_register(chip,LTC4015_SUSPEND_CHARGER_BF,data));
}

int  EnableJEITA(bool data)
{
  return (LTC4015_write_register(chip,LTC4015_EN_JEITA_BF,data));
}

float GetSYSVoltage(void)
{
  uint16_t data;
  LTC4015_read_register(chip, LTC4015_VSYS, &data);
  float Vin=0;
  Vin = (float)(data*0.001648);
  return Vin;
}

float GetInputVoltage(void)
{
  uint16_t data;
  LTC4015_read_register(chip, LTC4015_VIN, &data);
  float Vin=0;
  Vin = (float)(data*0.001648);
  return Vin;
}

float GetInputCurrent(void)
{
  uint16_t data;
  LTC4015_read_register(chip, LTC4015_IIN, &data);
  float Iin=0;
  Iin = (float)(data*0.00146487/RSENSE_IN);
  return Iin;
}

float GetBatteryVoltage(void)
{
  uint16_t data;
  LTC4015_read_register(chip, LTC4015_VBAT, &data);
  float Vbat=0;
  uint16_t noOfCells = GetRegisterValue(LTC4015_CELL_COUNT_PINS_BF);
  Vbat = (float)(data*0.000192264*noOfCells);
  return Vbat;
}

float GetBatteryCurrent(stBatCurrent* batCurr)
{
  uint16_t data;
  LTC4015_read_register(chip, LTC4015_IBAT, &data);
  float Ibat=0;
   batCurr->direction = CHARGE;
  bool check_neg = data&(0x8000);
  if(check_neg)
  {
    data = (~(data))+1;
    batCurr->direction = DISCHARGE;
  }
  Ibat = (float)(data*0.00146487/RSENSE_BAT);
  batCurr->current = Ibat;
  return Ibat;
}

float GetDieTemperature(void)
{
  uint16_t data;
  LTC4015_read_register(chip, LTC4015_DIE_TEMP, &data);
  float Dtemp=0;
  Dtemp = (float)((data-12010)/45.6);
  return Dtemp;
}

bool GetChargingOrNot(void)
{
  return((bool)GetRegisterValue(LTC4015_CC_CV_CHARGE_BF)); 
}

void ControlledCharging(void)
{
  static uint32_t prevSuspendTime = 0;
  int status = success;
  static bool isAlreadyHighTemp = false;
  static bool isAlreadylowTemp = false;
  if((GetTemperatureSensorData(TEMPSENSOR_1)>=TEMP_LIMIT_MAX)&&(!isAlreadyHighTemp))
  {
    
    status |= SuspendCharger(1);
    status |= WriteToRegister(LTC4015_ICHARGE_TARGET,GetIChargeValue(CHARGE_CURRENT_HIGH_TEMP));
    if(GetRegisterValue(LTC4015_EN_JEITA_BF))
    {  
      status |= WriteToRegister(LTC4015_ICHARGE_JEITA_2_BF,GetIChargeValue(CHARGE_CURRENT_HIGH_TEMP));
      status |= WriteToRegister(LTC4015_ICHARGE_JEITA_3_BF,GetIChargeValue(CHARGE_CURRENT_HIGH_TEMP));
      status |= WriteToRegister(LTC4015_ICHARGE_JEITA_4_BF,GetIChargeValue(CHARGE_CURRENT_HIGH_TEMP));
      status |= WriteToRegister(LTC4015_ICHARGE_JEITA_5_BF,GetIChargeValue(CHARGE_CURRENT_HIGH_TEMP));
      status |= WriteToRegister(LTC4015_ICHARGE_JEITA_6_BF,GetIChargeValue(CHARGE_CURRENT_HIGH_TEMP));
    }
    status |= SuspendCharger(0);
    isAlreadyHighTemp = true;
    isAlreadylowTemp = false;
  }
  else if(((GetTemperatureSensorData(TEMPSENSOR_1)<=TEMP_LIMIT_MIN))&&(!isAlreadylowTemp))
  {
    status |= SuspendCharger(1);
    status |= WriteToRegister(LTC4015_ICHARGE_TARGET,GetIChargeValue(CHARGE_CURRENT));
    if(GetRegisterValue(LTC4015_EN_JEITA_BF))
    {  
      status |= WriteToRegister(LTC4015_ICHARGE_JEITA_2_BF,GetIChargeValue(CHARGE_CURRENT));
      status |= WriteToRegister(LTC4015_ICHARGE_JEITA_3_BF,GetIChargeValue(CHARGE_CURRENT));
      status |= WriteToRegister(LTC4015_ICHARGE_JEITA_4_BF,GetIChargeValue(CHARGE_CURRENT));
      status |= WriteToRegister(LTC4015_ICHARGE_JEITA_5_BF,GetIChargeValue(CHARGE_CURRENT));
      status |= WriteToRegister(LTC4015_ICHARGE_JEITA_6_BF,GetIChargeValue(CHARGE_CURRENT));
    }
    status |= SuspendCharger(0);
    isAlreadyHighTemp = false;
    isAlreadylowTemp = true;
  }
  if((GetBatChargerTimer()-prevSuspendTime)>=(SUSPEND_CHARGER_INTERVAL*60*1000))
  {
  SuspendCharger(1);
  //Serial_Debug("\n****************************\n       Charger Restart       \n****************************");
  msdelay(5);
  SuspendCharger(0);
  prevSuspendTime = GetBatChargerTimer();
  }
}

void BatteryInfo(void)
{
  uint16_t data;
  // uint16_t ValuesStrCount = 0;
  stBatCurrent BatteryCurrent;
  char ValuesStr[300];
  float Vin_f,Vbat_f,Vsys_f,Iin_f,Ibat_f,Dtemp_f,BatPercent_f;
  char Vin_str[10],Vbat_str[10],Vsys_str[10],Iin_str[10],Ibat_str[10],Dtemp_str[10],BatPercent_str[10];
  Vin_f = GetInputVoltage();
  ftoa(Vin_f,Vin_str,3);
  Vbat_f = GetBatteryVoltage();
  ftoa(Vbat_f,Vbat_str,3);
  Vsys_f = GetSYSVoltage();
  ftoa(Vsys_f,Vsys_str,3);
  Iin_f = GetInputCurrent();
  ftoa(Iin_f,Iin_str,3);
  Dtemp_f = GetDieTemperature();
  ftoa(Dtemp_f,Dtemp_str,3);
  GetBatteryCurrent(&BatteryCurrent);
  Ibat_f = BatteryCurrent.current;
  if(!BatteryCurrent.direction)
    Ibat_f = 0-Ibat_f;
  ftoa(Ibat_f,Ibat_str,3);
  BatPercent_f = GetSOCAsPercentage();
  ftoa(BatPercent_f,BatPercent_str,3);
  snprintf(ValuesStr,300,"\r\nVSYS : %s, VIN : %s, IIN : %s, VBAT : %s, IBAT : %s, DTEMP : %s, BAT_PERCENT : %s\r\n",Vsys_str,Vin_str,Iin_str,Vbat_str,Ibat_str,Dtemp_str,BatPercent_str);
  Serial_Debug("\r\nValues-->");Serial_Debug(ValuesStr);
  //Serial_Debug_Float(vin_f,3);
  data = GetChargerState();
  Serial_Debug("\r\nCHARGER_STATE -->");
  Serial_Debug_Num(data);
  data = GetRegisterValue(LTC4015_CHARGE_STATUS);
  Serial_Debug("\r\nCHARGE STATUS->");Serial_Debug_Num(data);
  //LTC4015_read_register(chip, LTC4015_BAT_MISSING_FAULT_ALERT_BF, &data);
  data = GetRegisterValue(LTC4015_SYSTEM_STATUS);
  Serial_Debug("\r\nSYSTEM_STATE-->");Serial_Debug_Num(data);
//  data = GetRegisterValue(LTC4015_EN_JEITA_BF);
//  Serial_Debug("\r\nJEITA_STATE-->");Serial_Debug_Num(data);
//  data = GetRegisterValue(LTC4015_JEITA_REGION);
//  Serial_Debug("\r\nJEITA_REGION-->");Serial_Debug_Num(data);
//  
  data = GetRegisterValue(LTC4015_CHARGER_STATE_ALERTS);
  Serial_Debug("\r\nLTC4015_CHARGER_STATE_ALERTS-->");Serial_Debug_Num(data); 
//  data = GetRegisterValue(LTC4015_CHARGE_STATUS_ALERTS);
//  Serial_Debug("\r\nLTC4015_CHARGE_STATUS_ALERTS-->");Serial_Debug_Num(data); 
//  data = GetRegisterValue(LTC4015_LIMIT_ALERTS);
//  Serial_Debug("\r\nLTC4015_LIMIT_ALERTS-->");Serial_Debug_Num(data);
//  data = GetRegisterValue(LTC4015_QCOUNT_PRESCALE_FACTOR);
//  Serial_Debug("\r\nQCOUNT_PRESCALE_FACTOR-->");Serial_Debug_Num(data);
//  //data = GetRegisterValue(LTC4015_QCOUNT);
//  Serial_Debug("\r\nQCOUNT-->");Serial_Debug_Num(GetQCountValue());
//  //float percent=(float)(((data-16384)/32768)*100);
//  Serial_Debug("\r\nBATTERY PERCENT-->");
//  Serial_Debug_Float(GetSOCAsPercentage(),3);
  Serial_Debug("\r\nBATTERY ERROR-->");
  Serial_Debug_Num(GetBatteryError());
  //ClearBatteryError();
}