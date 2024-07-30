#ifndef __EEPROM_MAIN_H
#define __EEPROM_MAIN_H

#include "eeprom.h"

typedef enum eEepromStatus_def {
  EEPROM_OK             = 0x00,
  EEPROM_READ_ERROR     = 0x01,
  EEPROM_WRITE_ERROR    = 0x02,
  EEPROM_LOCK_ERROR     = 0x04,
  EEPROM_UNLOCK_ERROR   = 0x08
}eEepromStatus;

eEepromStatus ReadEEPROM (uint16_t storeAddress, float *readData, uint16_t noOfData);
eEepromStatus WriteEEPROM (uint16_t storeAddress, float *storeData, uint16_t noOfData);
eEepromStatus ReadEEPROM_Byte (uint16_t storeAddress, uint8_t *readData, uint16_t noOfData);
eEepromStatus WriteEEPROM_Byte (uint16_t storeAddress, uint8_t *storeData, uint16_t noOfData);
bool EEPROM_Init (void);


#endif