#ifndef _TEMP_SENSORS_H_
#define _TEMP_SENSORS_H_

#include "board.h"

typedef struct {
  float maxTempSensorOne;
  float maxTempSensorTwo;
} stBoardTemperature;

typedef enum tempSensor {
  TEMPSENSOR_1,
  TEMPSENSOR_2,
  TEMPSENSOR_3                          // Not available in New Board
}eTempSensors;

float GetTemperatureSensorData(eTempSensors);
stBoardTemperature *GetSetMaxBoardTemperature(void);

#endif