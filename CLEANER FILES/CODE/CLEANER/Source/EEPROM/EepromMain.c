#include "eeprommain.h"
#include "Serial_Debug.h"
//#include "EventLogger.h"

bool EEPROM_Init (void)
{
  bool status = true;
  
  if(FLASH_Initialisation() != FLASH_COMPLETE)
  {
    status = false;
  }
  else
  {
    /* Unlock the Flash Program Erase controller */
    if(FLASH_Unlock() != FLASH_COMPLETE)
    {
      status = false;
    }
    else
    {
      /* EEPROM Init */
      if(EE_Init() != FLASH_COMPLETE)
      {
        status = false;
      }
    }
    /* Lock the Flash Program Erase controller */
    if(FLASH_Lock() != FLASH_COMPLETE)
    {
      status = false;
    } 
  }
  return status;  
}

eEepromStatus WriteEEPROM (uint16_t storeAddress, float *storeData, uint16_t noOfData)
{
  union {
    float storeFloat;
    uint16_t storeInt[2];
    uint32_t storeInt32;
  } unStoreData;
  
  eEepromStatus status = EEPROM_OK;
  /* Unlock the Flash Program Erase controller */
  if(FLASH_Unlock() != FLASH_COMPLETE)
  {
    status = EEPROM_UNLOCK_ERROR;
  }
  else
  {
    for(uint16_t storeIndex = 0; storeIndex < noOfData; storeIndex++)
    {
      unStoreData.storeFloat = storeData[storeIndex];
      if(EE_WriteVariable(storeAddress, unStoreData.storeInt32) == FLASH_COMPLETE)
      {
        storeAddress++;
//        if(EE_WriteVariable(storeAddress, unStoreData.storeInt[1]) == FLASH_COMPLETE)
//        {
//          storeAddress++;
//        }
//        else
//        {
//          status = EEPROM_WRITE_ERROR;
//          break;
//        }
      }
      else 
      {
        status = EEPROM_WRITE_ERROR; 
        break;
      }
    }
  }
  /* Lock the Flash Program Erase controller */
  if(FLASH_Lock() != FLASH_COMPLETE)
  {
    status = EEPROM_LOCK_ERROR;
  }
  if(status != EEPROM_OK)
  {
    //SetLogEvent(EV_LOG_EEPROM, (uint8_t)EV_MEM_WRITE_ERROR); 
    Serial_Debug("\n Eeprom Write Error :");
    Serial_Debug_Num(status);
  }
  else
  {
      //SetLogEvent(EV_LOG_EEPROM, (uint8_t)EV_MEM_WRITE); 
  }
  return status;
}

eEepromStatus ReadEEPROM (uint16_t storeAddress, float *readData, uint16_t noOfData)
{
  union {
    float readFloat;
    uint16_t readInt[2];
    uint32_t readInt32;
  } unReadData;
  
  eEepromStatus status = EEPROM_OK;
  /* Unlock the Flash Program Erase controller */
  if(FLASH_Unlock() != FLASH_COMPLETE)
  {
    status = EEPROM_UNLOCK_ERROR;
  }
  else
  {
    for(uint16_t storeIndex = 0; storeIndex < noOfData; storeIndex++)
    {
      /* read the last stored variables data*/
      if(EE_ReadVariable(storeAddress, &unReadData.readInt32)== 0)
      {
        storeAddress++;
//        if(EE_ReadVariable(storeAddress, &unReadData.readInt[1]) == 0)
//        {
//          storeAddress++;
          readData[storeIndex] = unReadData.readFloat;
//        }
//        else
//        {
//          status = EEPROM_READ_ERROR;  
//          break;
//        }
      }
      else
      {
        status = EEPROM_READ_ERROR;    
        break;
      }
    }
  }
  /* Lock the Flash Program Erase controller */
  if(FLASH_Lock() != FLASH_COMPLETE)
  {
    status = EEPROM_LOCK_ERROR;
  }
  if(status != EEPROM_OK)
  {
    //SetLogEvent(EV_LOG_EEPROM, (uint8_t)EV_MEM_READ_ERROR);  
    Serial_Debug("\n Eeprom Read Error :");
    Serial_Debug_Num(status);
  }
  return status;
}

eEepromStatus WriteEEPROM_Byte (uint16_t storeAddress, uint8_t *storeData, uint16_t noOfData)
{
 
  uint32_t storeWord;
  eEepromStatus status = EEPROM_OK;
  /* Unlock the Flash Program Erase controller */
  if(FLASH_Unlock() != FLASH_COMPLETE)
  {
    status = EEPROM_UNLOCK_ERROR;
  }
  else
  {
    for(uint16_t storeIndex = 0; storeIndex < noOfData; storeIndex++)
    {
      storeWord = (uint32_t)storeData[storeIndex];
      if(EE_WriteVariable(storeAddress, storeWord) == FLASH_COMPLETE)
      {
        storeAddress++;
      }
      else 
      {
        status = EEPROM_WRITE_ERROR; 
        break;
      }
    }
  }
  /* Lock the Flash Program Erase controller */
  if(FLASH_Lock() != FLASH_COMPLETE)
  {
    status = EEPROM_LOCK_ERROR;
  }
  if(status != EEPROM_OK)
  {
    //SetLogEvent(EV_LOG_EEPROM, (uint8_t)EV_MEM_WRITE_ERROR);  
    Serial_Debug("\n Eeprom Byte Write Error :");
    Serial_Debug_Num(status);
  }
  else
  {
      //SetLogEvent(EV_LOG_EEPROM, (uint8_t)EV_MEM_WRITE); 
  }
  return status;
}

eEepromStatus ReadEEPROM_Byte (uint16_t storeAddress, uint8_t *readData, uint16_t noOfData)
{
 // SetLogEvent(EV_LOG_EEPROM, (uint8_t)EV_MEM_READ);  
  uint32_t storeWord;
  eEepromStatus status = EEPROM_OK;
  /* Unlock the Flash Program Erase controller */
  if(FLASH_Unlock() != FLASH_COMPLETE)
  {
    status = EEPROM_UNLOCK_ERROR;
  }
  else
  {
    for(uint16_t storeIndex = 0; storeIndex < noOfData; storeIndex++)
    {
      /* read the last stored variables data*/
      if(EE_ReadVariable(storeAddress, &storeWord)== 0)
      {
        storeAddress++;
        readData[storeIndex] = (uint8_t)storeWord ;
      }
      else
      {
        status = EEPROM_READ_ERROR;    
        break;
      }
    }
  }
  /* Lock the Flash Program Erase controller */
  if(FLASH_Lock() != FLASH_COMPLETE)
  {
    status = EEPROM_LOCK_ERROR;
  }
  if(status != EEPROM_OK)
  {
    //SetLogEvent(EV_LOG_EEPROM, (uint8_t)EV_MEM_READ_ERROR); 
    Serial_Debug("\n Eeprom Byte Read Error :");
    Serial_Debug_Num(status);
  }
  return status;
}