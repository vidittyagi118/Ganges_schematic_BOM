#ifndef __EEPROM_STD_TO_HAL_DRIVER_H
#define __EEPROM_STD_TO_HAL_DRIVER_H

#include "stdbool.h"
//#include "stm32f3xx_hal.h"
#include "fsl_common.h"

typedef enum
{ 
  FLASH_BUSY = 1,
  FLASH_ERROR_WRP,
  FLASH_ERROR_PROGRAM,
  FLASH_COMPLETE,
  FLASH_TIMEOUT
}FLASH_Status;

/* FLASH Memory Programming functions *****************************************/   
FLASH_Status FLASH_Initialisation(void);
FLASH_Status FLASH_Unlock(void);
FLASH_Status FLASH_Lock(void);
FLASH_Status FLASH_ErasePage(uint32_t Page_Address);
FLASH_Status FLASH_EraseAllPages(void);
FLASH_Status FLASH_ProgramWord(uint32_t Address, uint32_t Data);
FLASH_Status FLASH_ProgramHalfWord(uint32_t Address, uint16_t Data);


FLASH_Status FLASH_ProgramWordArray(uint32_t Address, uint32_t* Data, uint8_t noOfData);

void FLASH_PageErase(uint32_t PageAddress);
void FLASH_Program_HalfWord(uint32_t Address, uint16_t Data);
void FLASH_SetErrorCode(void);
static FLASH_Status GetStdError(status_t status);

#endif