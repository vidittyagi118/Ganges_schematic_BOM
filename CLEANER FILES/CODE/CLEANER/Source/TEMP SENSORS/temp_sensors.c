#include "temp_sensors.h"
#include "i2c_init.h"
#define TEMP_SENSOR_1_ADDRESS 0x48
#define TEMP_SENSOR_2_ADDRESS 0x49
#define TEMP_SENSOR_3_ADDRESS 0x4B

#define BOARD_TEMP_I2C_BASEADDR I2C0   

#define TEMP_SENSOR_READ_AGAIN  2000 //ms

static stBoardTemperature maxBoardTemperature;

static float tempSense1_pvalue = 0;
static float tempSense2_pvalue = 0;
static uint32_t tempSense1_ptime = 0;
static uint32_t tempSense2_ptime = 0;

float GetTemperatureSensorData(eTempSensors sensorId)
{
//  uint8_t slaveAddress;
//  uint32_t currTime = GetTempSenseI2CTimer();
//  switch(sensorId)
//  {
//  case TEMPSENSOR_1: if((currTime - tempSense1_ptime) >= TEMP_SENSOR_READ_AGAIN)
//  { 
//    slaveAddress = TEMP_SENSOR_1_ADDRESS;
//    tempSense1_ptime = GetTempSenseI2CTimer();
//    break;
//  }
//  else
//  {
//    return tempSense1_pvalue;
//  }
//  case TEMPSENSOR_2: if((currTime - tempSense2_ptime) >= TEMP_SENSOR_READ_AGAIN)
//  {
//    slaveAddress = TEMP_SENSOR_2_ADDRESS;                      
//    tempSense2_ptime = GetTempSenseI2CTimer();
//    break;
//  }
//  else
//  {
//    return tempSense2_pvalue;
//  }
//  default:
//    return 0;
//  }
//  
//  uint8_t rxBuff[4];
//  I2C_ReadKeypadRegs(BOARD_TEMP_I2C_BASEADDR,slaveAddress,0,0,rxBuff,2);
//  int TempSum = (((rxBuff[0] << 8) | rxBuff[1]) >> 4); 
//  // From Datasheet the TMP75 has a quantisation value of 0.0625 degreesC per bit
//  float temp = (TempSum*0.0625); 
//  if(sensorId == TEMPSENSOR_1)
//  {
//    tempSense1_pvalue = temp;
//  }
//  else if(sensorId == TEMPSENSOR_2)
//  {
//    tempSense2_pvalue = temp;
//  }
//  return temp;
  return 40;
}

stBoardTemperature *GetSetMaxBoardTemperature(void)
{
  return(&maxBoardTemperature);
}
