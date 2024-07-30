#ifndef _PWM_DRIVE_SPI_H_
#define _PWM_DRIVE_SPI_H_

#include "spi.h"

#include <stdint.h>
#include <stdbool.h>

typedef enum eDriveReg_def{
 FAULT_STATUS,
 VDS_GDS,
 MAIN_REG, 
 IDRIVE_WD,
 VDS,
 CONFIG 
}eDriveReg;

bool GetDriveData (uint8_t device, eDriveReg driveReg, uint16_t * data);
bool GetDriveAllData (uint8_t device, uint16_t * data, uint16_t datalen);

#endif





