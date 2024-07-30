#include "eepromStdtoHalDriver.h"
#include "fsl_flash.h"
#include "usr_flash.h"
#define FLASH_ER_PRG_TIMEOUT         ((uint32_t)0x000B0000)

FLASH_Status FLASH_Initialisation(void)
{
  status_t HALstatus;
  HALstatus = kStatus_FLASH_Success;
  HALstatus = flash_init();
  return(GetStdError(HALstatus));
}

FLASH_Status FLASH_Unlock(void)
{
  status_t HALstatus;
  HALstatus = kStatus_FLASH_Success;
  //HALstatus = HAL_FLASH_Unlock();
  return(GetStdError(HALstatus));
}

FLASH_Status FLASH_Lock(void)
{
  status_t HALstatus;
  HALstatus = kStatus_FLASH_Success;
 // HALstatus = HAL_FLASH_Lock();  
  return(GetStdError(HALstatus));
}

FLASH_Status FLASH_ErasePage(uint32_t Page_Address)
{
  FLASH_Status status = FLASH_COMPLETE;
  status_t HALstatus;
  HALstatus = sectorErasing(Page_Address);
  status = GetStdError(HALstatus);
  /* Return the Erase Status */
  return status;  
}

FLASH_Status FLASH_ProgramWord(uint32_t Address, uint32_t Data)
{
  status_t status = kStatus_FLASH_Success;
  //status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD , Address, Data);
  return(GetStdError(status));
}

FLASH_Status FLASH_ProgramWordArray(uint32_t Address, uint32_t* Data, uint8_t noOfData)
{
  status_t status;
 // status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD , Address, Data);
  status = WriteArray(Address,Data,noOfData);
  return(GetStdError(status));
}

FLASH_Status FLASH_ProgramHalfWord(uint32_t Address, uint16_t Data)
{
  status_t status;
 // status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD , Address, Data);
  status = WritingToOneAddress(Address,Data);
  return(GetStdError(status));
}

static FLASH_Status GetStdError(status_t status)
{
  if(status == kStatus_FLASH_Success)
  {
    return FLASH_COMPLETE;
  }
  if(status == kStatus_FLASH_InvalidArgument)
  {
    return FLASH_ERROR_PROGRAM;
  }
//  if(status == HAL_BUSY)
//  {
//    return FLASH_BUSY;
//  }
//  if(status == HAL_TIMEOUT)
//  {
//    return FLASH_TIMEOUT;
//  }
  return FLASH_ERROR_PROGRAM;
}
