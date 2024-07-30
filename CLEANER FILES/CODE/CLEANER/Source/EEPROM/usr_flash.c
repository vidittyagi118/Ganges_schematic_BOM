#include "usr_flash.h"
#include "Serial_Debug.h"
#include <stdio.h>
/*! @brief Flash driver Structure */
static flash_config_t s_flashDriver;
flash_security_state_t securityStatus = kFLASH_SecurityStateNotSecure; /* Return protection status */
uint32_t destAdrss[NOOFSECTORSINUSE]; /* Address of the target location */
uint32_t pflashBlockBase = 0;
uint32_t pflashTotalSize = 0;
uint32_t pflashSectorSize = 0;
uint32_t failAddr, failDat;

void error_trap(void)
{
  Serial_Debug("\r\n\r\n\r\n\t---- **** FLASH ERROR! ----");
}

status_t flash_init(void)
{
  status_t result = kStatus_FLASH_Success;
  /* Clean up Flash, Cache driver Structure*/
  memset(&s_flashDriver, 0, sizeof(flash_config_t));

  /* Setup flash driver structure for device and initialize variables. */
  result = FLASH_Init(&s_flashDriver);
  if (kStatus_FLASH_Success != result)
  {
    error_trap();
    return result;
  }
    /* Get flash properties*/
  FLASH_GetProperty(&s_flashDriver, kFLASH_PropertyPflashBlockBaseAddr, &pflashBlockBase);
  FLASH_GetProperty(&s_flashDriver, kFLASH_PropertyPflashTotalSize, &pflashTotalSize);
  FLASH_GetProperty(&s_flashDriver, kFLASH_PropertyPflashSectorSize, &pflashSectorSize); 
  
  /* Print flash information - PFlash. */
  Serial_Debug("\r\n PFlash Information: ");
  char dispstring[200];
  snprintf(dispstring, sizeof dispstring, "\r\n Total Program Flash Size:\t%d KB, Hex: (0x%x)", (pflashTotalSize / 1024), pflashTotalSize);
  Serial_Debug(dispstring);
  snprintf(dispstring, sizeof dispstring, "\r\n Program Flash Sector Size:\t%d KB, Hex: (0x%x) ", (pflashSectorSize / 1024), pflashSectorSize);
  Serial_Debug(dispstring);
    
        /* Check security status. */
  result = FLASH_GetSecurityState(&s_flashDriver, &securityStatus);
  if (kStatus_FLASH_Success != result)
  {
    error_trap();
    return result;
  }
  switch (securityStatus)
  {
      case kFLASH_SecurityStateNotSecure:
          Serial_Debug("\r\n Flash is UNSECURE!");
          break;
      case kFLASH_SecurityStateBackdoorEnabled:
          Serial_Debug("\r\n Flash is SECURE, BACKDOOR is ENABLED!");
          break;
      case kFLASH_SecurityStateBackdoorDisabled:
          Serial_Debug("\r\n Flash is SECURE, BACKDOOR is DISABLED!");
          break;
      default:
          break;
  }
  
    
#ifndef SECTOR_INDEX_FROM_END1
#define SECTOR_INDEX_FROM_END1 1U     //SECTOR 1 STATUS STORING
#endif
  
#ifndef SECTOR_INDEX_FROM_END2 
#define SECTOR_INDEX_FROM_END2 2U     //SECTOR 0 STATUS STORING 
#endif
#ifndef SECTOR_INDEX_FROM_END3
#define SECTOR_INDEX_FROM_END3 3U     //SECTOR 1 STATUS STORING
#endif
  
#ifndef SECTOR_INDEX_FROM_END4 
#define SECTOR_INDEX_FROM_END4 4U     //SECTOR 0 STATUS STORING 
#endif  
  
/* Erase a sector from destAdrss. */
#if defined(FSL_FEATURE_FLASH_HAS_PFLASH_BLOCK_SWAP) && FSL_FEATURE_FLASH_HAS_PFLASH_BLOCK_SWAP
  /* Note: we should make sure that the sector shouldn't be swap indicator sector*/
  destAdrss[0] = pflashBlockBase + (pflashTotalSize - (SECTOR_INDEX_FROM_END1 * pflashSectorSize * 2));
  destAdrss[1] = pflashBlockBase + (pflashTotalSize - (SECTOR_INDEX_FROM_END2 * pflashSectorSize * 2));
  destAdrss[2] = pflashBlockBase + (pflashTotalSize - (SECTOR_INDEX_FROM_END3 * pflashSectorSize * 2));
  destAdrss[3] = pflashBlockBase + (pflashTotalSize - (SECTOR_INDEX_FROM_END4 * pflashSectorSize * 2));
#else
  destAdrss[0] = pflashBlockBase + (pflashTotalSize - (SECTOR_INDEX_FROM_END1 * pflashSectorSize));  
  destAdrss[1] = pflashBlockBase + (pflashTotalSize - (SECTOR_INDEX_FROM_END2 * pflashSectorSize));
  destAdrss[2] = pflashBlockBase + (pflashTotalSize - (SECTOR_INDEX_FROM_END3 * pflashSectorSize));  
  destAdrss[3] = pflashBlockBase + (pflashTotalSize - (SECTOR_INDEX_FROM_END4 * pflashSectorSize));  
#endif
    
  flash_protection_state_t protectStat;
  FLASH_IsProtected(&s_flashDriver, destAdrss[0], pflashSectorSize*SECTOR_INDEX_FROM_END1, &protectStat);
  if (kStatus_FLASH_Success != result)
  {
    error_trap();
    return result;
  }
  if (kFLASH_ProtectionStateUnprotected == protectStat)
  { 
    Serial_Debug("\r\n Flash is UN-PROTECTED");
  }
  else
  {
    Serial_Debug("\r\n Erase/Program operation will not be executed, as Flash is SECURE!");
    error_trap();
    result = kStatus_FLASH_ProtectionViolation;
  }
  return result;
}

status_t sectorErasing(uint32_t sectorAddress)
{
  status_t result; 
  status_t eraseResult = kStatus_FLASH_Success;
  uint32_t intmask = DisableGlobalIRQ();  
  result = FLASH_Erase(&s_flashDriver, sectorAddress,pflashSectorSize, kFLASH_ApiEraseKey);
  EnableGlobalIRQ(intmask); 
  if (kStatus_FLASH_Success != result)
  {
    error_trap();
    eraseResult = result;
  }
  intmask = DisableGlobalIRQ();
  result = FLASH_VerifyErase(&s_flashDriver,sectorAddress, pflashSectorSize, kFLASH_MarginValueUser);
  EnableGlobalIRQ(intmask);
  if (kStatus_FLASH_Success != result)
  {
    error_trap();
    eraseResult = result;
  }
  return eraseResult;
}

status_t WritingToOneAddress(uint32_t phyAdd, uint16_t writeData)
{
  //Serial_Debug("\nWRITING TO ONE ADD LOC");
  uint32_t s_buffer[2];
  s_buffer[0]= writeData;
  s_buffer[1]= 0;
//  Serial_Debug("\n Address->");
//  Serial_Debug_Hex(phyAdd);
//    Serial_Debug("\nData->");
//  Serial_Debug_Num(writeData);
  status_t retVal = write_flash(s_buffer, (BYTECASE*ONEADDSIZE), phyAdd);
//  Serial_Debug("\r\nError Code from WritingToOneAddress-->");
//  Serial_Debug_Num(retVal);
  return retVal;
}

status_t WriteArray(uint32_t phyAdd, uint32_t* writeData,uint8_t noOfData )
{
//  Serial_Debug("\nWRITING TO Array");
////  uint32_t s_buffer[1];
////  s_buffer[0]= writeData;
//
//  Serial_Debug("\n Address->");
//  Serial_Debug_Hex(phyAdd);
//    Serial_Debug("\nData->");
//  Serial_Debug_Num(writeData[0]);
  status_t retVal = write_flash(writeData, (BYTECASE*ONEADDSIZE), phyAdd);
//  Serial_Debug("\r\nError Code from WRITING TO Array-->");
//  Serial_Debug_Num(retVal);
  return retVal;
}

status_t write_flash(uint32_t * buffer, uint8_t noofAddLocs, uint32_t statAddress)   
{
  status_t result; 
  status_t tempResult = kStatus_Success;
  
//  Serial_Debug("\r\n Program a buffer to a sector of flash ");
//  Serial_Debug("\r\n");
  uint32_t intmask = DisableGlobalIRQ(); 
  result = FLASH_Program(&s_flashDriver, statAddress, buffer, noofAddLocs);//sizeof(s_buffer)
  EnableGlobalIRQ(intmask);
  if (kStatus_FLASH_Success != result)
  {
    Serial_Debug("\nPROGRAM ERROR");
    error_trap();
    tempResult = result;
  }
  
  intmask = DisableGlobalIRQ(); 
  result = FLASH_VerifyProgram(&s_flashDriver, statAddress, noofAddLocs, buffer, kFLASH_MarginValueUser,
                                     &failAddr, &failDat);
  EnableGlobalIRQ(intmask);
  if (kStatus_FLASH_Success != result)
  {
    Serial_Debug("\nVERIFY PROGRAM ERROR");
    error_trap();
    tempResult = result;
  }
//  Serial_Debug("\n FAIL ADD:");
//  Serial_Debug_Num(failAddr);
//  Serial_Debug("\n FAIL DAT:");
//  Serial_Debug_Num(failDat);
//  Serial_Debug("\n"); 
 // FTFx_CACHE_ClearCachePrefetchSpeculation(&s_cacheDriver, false);    
  return tempResult;
}

void ReadFlash(uint8_t* data, uint32_t destAddress,uint32_t noOfData)
{
//Serial_Debug("\r\nIn Read Flash-->");  
for(int i =0; i<noOfData; i++) 
{
data[i] = *(volatile uint8_t *)(destAddress+i*1);
//Serial_Debug_Hex(data[i]);
//Serial_Debug(" ");
}
}

status_t EraseFlash(uint32_t address, uint32_t lengthInBytes)
{
  status_t status;
  uint32_t  intmask = DisableGlobalIRQ(); 
  status = FLASH_Erase(&s_flashDriver,address,lengthInBytes,kFLASH_ApiEraseKey);
  EnableGlobalIRQ(intmask);
  return status;
}