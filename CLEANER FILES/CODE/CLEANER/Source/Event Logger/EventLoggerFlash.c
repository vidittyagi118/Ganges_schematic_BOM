#include "EventLoggerFlash.h"
//#include "SpiFlashLib.h"
#include "Rtc.h"
#include "Serial_Debug.h"
#include "usr_flash.h"
#include "eepromStdtoHalDriver.h"

static eStoreEventCurSector allLogActiveSector = EV_STORE_SECTOR_1;
static uint32_t allLogActiveSectorStartAdd[] = {FLASH_LOG_ALL_SEC1_START_ADDRESS, FLASH_LOG_ALL_SEC2_START_ADDRESS};

static uint16_t allLogEventIndex = 0;
//static uint32_t MaxStoreSizeInSingleSector = 0;
// uint32_t MAX_NO_OF_STORE_LOG_DATA = 0;
static uint32_t NoofDataStored = 0;

static void MoveToNextSector_Write (void);
static eStoreEventCurSector MoveToNextSector_Read (eStoreEventCurSector allLogCurReadSector);

uint32_t ReadRTCBackup(uint8_t index)
{
//return RFVBAT->REG[index];
  uint32_t data;
ReadDataForKey((uint16_t)index, &data);
return data;
}

void WriteRTCBackup(uint8_t index,uint32_t data)
{
//RFVBAT->REG[index] = data;
  CheckAndWriteData((uint16_t)index,data);
}
void EventLoggerAllLogFlash_Init (void)
{
  uint32_t nextIndexAndSector = ReadRTCBackup(RTC_BACKUP_ALL_LOG_STORE_INDEX_SECTOR_ADD);
  allLogActiveSector = (eStoreEventCurSector)((nextIndexAndSector >> 16) & 0xFFFF);
  allLogEventIndex = nextIndexAndSector & 0xFFFF; 
  NoofDataStored = ReadRTCBackup(RTC_BACKUP_ALL_LOG_STORE_NO_OF_DATA_ADD);
  if(((allLogActiveSector != EV_STORE_SECTOR_1) && (allLogActiveSector != EV_STORE_SECTOR_2)) 
     || (NoofDataStored == 0) || (NoofDataStored > MAX_NO_OF_STORE_LOG_DATA))
  {
    ClearFlashAllLog();                                                       /* Erase Both Sectors if no data. Kind of New Board */
  }
  Serial_Debug("\n All Log Initial Active Sector : ");
  Serial_Debug_Num(allLogActiveSector);
  Serial_Debug("\n All Log Initial Active Index : ");
  Serial_Debug_Num(allLogEventIndex);
  Serial_Debug("\n All Log Initial No of Data Stored : ");
  Serial_Debug_Num(NoofDataStored);
  Serial_Debug("\n All Log Initial Active Store Address : ");
  Serial_Debug_Num(allLogActiveSectorStartAdd[allLogActiveSector]);
/*  
  Serial_Debug("\n\nFlash Logger TEst ****");
  uint16_t logStoreAddress = allLogActiveSectorStartAdd[EV_STORE_SECTOR_1];
  uint8_t logdata[SINGLE_STORE_BYTE_COUNT];
  for(uint16_t startNo=0; startNo <= 100; startNo++)
  {
    Serial_Debug("\n Event Log Store Bytes1: ");
    uint16_t logDatachar = startNo*SINGLE_STORE_BYTE_COUNT;
    for(uint8_t i = 0; i < SINGLE_STORE_BYTE_COUNT; i++)
    {
      logdata[i] = logDatachar+i+1;
      Serial_Debug_Num((uint8_t)logdata[i]);
      Serial_Debug(",");
    }
    
    SpiFlashWriteContinuous(logStoreAddress,logdata, SINGLE_STORE_BYTE_COUNT); 
    char dispstring[100];
    snprintf(dispstring, sizeof dispstring, "\nLog Store Info: logStoreAddress: %d, startNo: %d, NoofDataStored: %d", logStoreAddress, startNo+1, SINGLE_STORE_BYTE_COUNT);
    Serial_Debug(dispstring);
    Serial_Debug("\n Event Log Store Bytes: ");
    for(uint8_t i = 0; i < SINGLE_STORE_BYTE_COUNT; i++)
    {
    Serial_Debug_Num((uint8_t)logdata[i]);
    Serial_Debug(",");
    }
    HAL_Delay(200);
    
    uint8_t logData1[SINGLE_STORE_BYTE_COUNT];
    for(uint8_t i = 0; i < SINGLE_STORE_BYTE_COUNT; i++)
    {
      logData1[i] = 0;
    }
    SpiFlashReadContinuous(logStoreAddress, logData1, SINGLE_STORE_BYTE_COUNT);
    Serial_Debug("\n Log Store Read Address : ");
    Serial_Debug_Num(logStoreAddress);
    Serial_Debug("\n Event Log Store Bytes 2 : ");
    for(uint8_t i = 0; i < SINGLE_STORE_BYTE_COUNT; i++)
    {
      Serial_Debug_Num((uint8_t)logData1[i]);
      Serial_Debug(",");
    }
    HAL_Delay(2000);
    logStoreAddress = logStoreAddress + SINGLE_STORE_BYTE_COUNT;
  }
*/
}

void ClearFlashAllLog (void)
{
    allLogActiveSector = EV_STORE_SECTOR_1;
    allLogEventIndex = 0;
    NoofDataStored = 0;
    sectorErasing(allLogActiveSectorStartAdd[EV_STORE_SECTOR_1]);    
    sectorErasing(allLogActiveSectorStartAdd[EV_STORE_SECTOR_2]);
    sectorErasing(FLASH_LOG_COUNT_SEC_START_ADDRESS);
    WriteRTCBackup(RTC_BACKUP_ALL_LOG_STORE_INDEX_SECTOR_ADD, 0x0000);
    WriteRTCBackup(RTC_BACKUP_ALL_LOG_STORE_NO_OF_DATA_ADD, 0x0000);
    Serial_Debug("\nErase All Log Sector 1 and Sector 2");  
}

static void MoveToNextSector_Write (void)
{
  if(allLogActiveSector == EV_STORE_SECTOR_1)
  {
     allLogActiveSector = EV_STORE_SECTOR_2;
  }  
  else
  {
     allLogActiveSector = EV_STORE_SECTOR_1;
  }
  sectorErasing(allLogActiveSectorStartAdd[allLogActiveSector]);
}

static eStoreEventCurSector MoveToNextSector_Read (eStoreEventCurSector allLogCurReadSector)
{
    eStoreEventCurSector allLogReadSector;
    if(allLogCurReadSector == EV_STORE_SECTOR_1)
    {
      allLogReadSector = EV_STORE_SECTOR_2;
    }  
    else
    {
      allLogReadSector = EV_STORE_SECTOR_1;
    }
    return allLogReadSector;
}

bool StoreFlashAllLog(uint8_t * logData, uint16_t logDataLen)
{
  if(logDataLen > SINGLE_STORE_BYTE_COUNT)
  {
    return false;      
  }
  else
  {
    if(allLogEventIndex >= MAX_NO_OF_STORE_LOG_DATA)
    {
      allLogEventIndex = 0;
      MoveToNextSector_Write();
    }
    uint32_t nextIndexAndSector = (((uint32_t)allLogActiveSector) << 16) | (allLogEventIndex+1);       /* Consider the present data is stored and Write the next index to backup */
    uint32_t logStoreAddress = allLogActiveSectorStartAdd[allLogActiveSector] + (allLogEventIndex*SINGLE_STORE_BYTE_COUNT);
    WriteRTCBackup(RTC_BACKUP_ALL_LOG_STORE_INDEX_SECTOR_ADD, nextIndexAndSector); 
    NoofDataStored++;
    if(NoofDataStored > MAX_NO_OF_STORE_LOG_DATA)
    {
      NoofDataStored = MAX_NO_OF_STORE_LOG_DATA;
    }
    allLogEventIndex++;
    WriteRTCBackup(RTC_BACKUP_ALL_LOG_STORE_NO_OF_DATA_ADD, NoofDataStored);
    write_flash(logData, (uint32_t)logDataLen,logStoreAddress); 
    
#ifdef ENABLE_SERIAL_DEBUG 
    char dispstring[100];
    snprintf(dispstring, sizeof dispstring, "\nLog Store Info: allLogEventIndex: %d, NoofDataStored: %d, allLogActiveSector: %d, logStoreAddress: %ld", allLogEventIndex, NoofDataStored, allLogActiveSector, logStoreAddress);
    Serial_Debug(dispstring);
    uint32_t utcTime =  ((uint32_t)logData[0] << 24)  | ((uint32_t)logData[1] << 16) 
                | ((uint32_t)logData[2] << 8) | logData[3];
    snprintf(dispstring, sizeof dispstring, "\n Log Store Data:[ %ld, %d, %d ]", utcTime, logData[4], (logData[5]<<8)&logData[6]);
    Serial_Debug(dispstring);
#endif
//    Serial_Debug("\n Store Log Address :");
//    Serial_Debug_Num(logStoreAddress);
//    HAL_Delay(200);
//    uint8_t logData1[SINGLE_STORE_BYTE_COUNT];
//      for(uint8_t i = 0; i < SINGLE_STORE_BYTE_COUNT; i++)
//  {
//      logData1[i] = 0;
//  }
//    SpiFlashReadContinuous(logStoreAddress, logData1, SINGLE_STORE_BYTE_COUNT); 
//  Serial_Debug("\n Event Log Store Bytes 2 : ");
//  for(uint8_t i = 0; i < SINGLE_STORE_BYTE_COUNT; i++)
//  {
//    Serial_Debug_Num((uint8_t)logData1[i]);
//    Serial_Debug(",");
//  }
  
    return true;
  }
}

uint32_t ReadFlashAllLog(uint8_t *logStoreData, uint16_t maxNoofData, uint16_t startIndex, uint16_t endIndex)
{
  int16_t noofReqData = endIndex - startIndex + 1;
  char dispstring[100];
//  snprintf(dispstring, sizeof dispstring, "\n1. StartIndex: %d, EndIndex: %d, NoofReqData: %d", startIndex, endIndex, noofReqData);
//  Serial_Debug(dispstring);
  if(noofReqData <= 0)
  {
    noofReqData = 0;
  }
  if(endIndex > MAX_NO_OF_STORE_LOG_DATA)
  {
    noofReqData = 0;
  }
  if(startIndex > NoofDataStored)
  {
     noofReqData = 0;
  }
//  snprintf(dispstring, sizeof dispstring, "\n2. StartIndex: %d, EndIndex: %d, NoofReqData: %d", startIndex, endIndex, noofReqData);
//  Serial_Debug(dispstring);
  
  /* Calculate Index of First Stored Data */
  int32_t allLogEventStartIndex = allLogEventIndex - NoofDataStored;
  eStoreEventCurSector allLogEventStartSector = allLogActiveSector;
  if(allLogEventStartIndex < 0 )
  {
    allLogEventStartSector = MoveToNextSector_Read(allLogActiveSector);
    allLogEventStartIndex = MAX_NO_OF_STORE_LOG_DATA - (allLogEventStartIndex*-1) + 1;
  }
  
  /* Calculate Index of First Stored Data to be Read*/
  int32_t allLogEventReadStartIndex = allLogEventStartIndex + startIndex-1;
  eStoreEventCurSector allLogEventReadStartSector = allLogEventStartSector;
  if(allLogEventReadStartIndex > MAX_NO_OF_STORE_LOG_DATA)
  {
    allLogEventReadStartSector = MoveToNextSector_Read(allLogEventStartSector);
    allLogEventReadStartIndex = allLogEventReadStartIndex - MAX_NO_OF_STORE_LOG_DATA-1; 
  }
  
  /* Calculate Available Data between the given Start and End Index */
  uint32_t noOfAvailableReadData = 0;
  if(NoofDataStored >= MAX_NO_OF_STORE_LOG_DATA)
  {
    noOfAvailableReadData = noofReqData;
  }
  else
  {
    noOfAvailableReadData = NoofDataStored - allLogEventReadStartIndex;
  }
  
//  /* Calculate Index of End Stored Data to be Read*/
//  int32_t allLogEventReadEndIndex = allLogEventReadStartIndex + noofReqData -1;
//  if(allLogEventReadEndIndex > MAX_NO_OF_STORE_LOG_DATA)
//  {
//    allLogEventReadEndSector = MoveToNextSector_Read(allLogEventReadStartSector);
//    allLogEventReadEndIndex = allLogEventReadEndIndex - MAX_NO_OF_STORE_LOG_DATA-1; 
//  }
//   
  /* Adjust Current Log Index to End Index First */
//  int32_t allLogEventReadEndIndex = allLogEventIndex - endIndex;
//  if(allLogEventReadEndIndex < 0 )
//  {
//    allLogEventReadEndIndex = MAX_NO_OF_STORE_LOG_DATA - (allLogEventReadEndIndex*-1);
//  }
//  uint32_t NoofEffectiveStoredData = NoofDataStored-(MAX_NO_OF_STORE_LOG_DATA - endIndex);
//  if(noofReqData > NoofEffectiveStoredData)
//  {
//    noofReqData = NoofEffectiveStoredData;
//  }
  
//  snprintf(dispstring, sizeof dispstring, "\n3. MAX_NO_OF_STORE_LOG_DATA: %d, NoofDataStored: %d", MAX_NO_OF_STORE_LOG_DATA, NoofDataStored);
//  Serial_Debug(dispstring);
//  
//  snprintf(dispstring, sizeof dispstring, "\n4. allLogEventIndex: %d, allLogEventReadStartIndex: %d, noOfAvailableReadData: %d", allLogEventIndex, allLogEventReadStartIndex, noOfAvailableReadData);
//  Serial_Debug(dispstring);

//  if(allLogEventReadEndIndex < noofReqData)
//  {
//    if(allLogEventReadEndIndex == 0)
//    {
//      readIndex = 0;
//    }
//    else
//    {
//    readIndex = MAX_NO_OF_STORE_LOG_DATA-(noofReqData - allLogEventReadEndIndex)-1;
//    }
//    allLogReadSector = MoveToNextSector_Read(allLogActiveSector);
//  }
//  else
//  {
//    allLogReadSector = allLogActiveSector;
//    readIndex = noofReqData-allLogEventReadEndIndex;
//  }
  
  uint16_t readIndex = allLogEventReadStartIndex;
  eStoreEventCurSector readSector = allLogEventReadStartSector;
  uint16_t readEventIndex = 0;
//  snprintf(dispstring, sizeof dispstring, "\n5. readIndex: %d, readSector: %d, NoofReqData: %d", readIndex, readSector, noofReqData);
//  Serial_Debug(dispstring);

  while(noofReqData--)
  {
    if((readEventIndex >= maxNoofData) || (readEventIndex >= noOfAvailableReadData))
    {
      break;
    }
    if(readIndex >= MAX_NO_OF_STORE_LOG_DATA)
    {
      readIndex = 0;
      readSector = MoveToNextSector_Read(readSector);
    }
    uint32_t logReadAddress = allLogActiveSectorStartAdd[readSector] + (readIndex*SINGLE_STORE_BYTE_COUNT);
    //SpiFlashReadContinuous(logReadAddress, (uint8_t *)&logStoreData[(readEventIndex*SINGLE_STORE_BYTE_COUNT)], SINGLE_STORE_BYTE_COUNT); 
    ReadFlash((uint8_t *)&logStoreData[(readEventIndex*SINGLE_STORE_BYTE_COUNT)],logReadAddress,SINGLE_STORE_BYTE_COUNT);
    //snprintf(dispstring, sizeof dispstring, "\nLog Read Info: allLogReadEventIndex: %d, readIndex: %d, allLogActiveSector: %d, logReadAddress: %ld", readEventIndex, readIndex, readSector, logReadAddress);
    //Serial_Debug(dispstring);
    uint8_t * rxbytes = (uint8_t *)&logStoreData[(readEventIndex*SINGLE_STORE_BYTE_COUNT)];
    uint32_t utcTime =  ((uint32_t)rxbytes[0] << 24)  | ((uint32_t)rxbytes[1] << 16) 
                | ((uint32_t)rxbytes[2] << 8) | rxbytes[3];
    //snprintf(dispstring, sizeof dispstring, "\n Log Read Data:[ %ld, %d, %d ]", utcTime, rxbytes[4], (rxbytes[5]<<8)|rxbytes[6]);
    //Serial_Debug(dispstring);

    
    readIndex++;
    readEventIndex++;
  }
  return readEventIndex;
}
uint16_t CheckAndWriteData(uint16_t key, uint32_t data)
{
  uint8_t retryCount = 0;
writeagain:if(WriteKeyandDatatoFlash(key,data)!=1)
{
  if(retryCount<3)
  {
  uint32_t data1,data2;
  ReadDataForKey(RTC_BACKUP_ALL_LOG_STORE_INDEX_SECTOR_ADD,&data1);
  ReadDataForKey(RTC_BACKUP_ALL_LOG_STORE_NO_OF_DATA_ADD,&data2);
  sectorErasing(FLASH_LOG_COUNT_SEC_START_ADDRESS);
  WriteKeyandDatatoFlash(RTC_BACKUP_ALL_LOG_STORE_INDEX_SECTOR_ADD,data1);
  WriteKeyandDatatoFlash(RTC_BACKUP_ALL_LOG_STORE_NO_OF_DATA_ADD,data2);
  retryCount++;
  goto writeagain;
  }
  else 
  {
  return 0;
  }
}
else
{
  return 1;
}
//return 0;
}
uint16_t WriteKeyandDatatoFlash(uint16_t key, uint32_t data)
{
  uint8_t stat =0;
  uint32_t dataToWrite[2];
  dataToWrite[0] = key;
  dataToWrite[1] = data;
  uint32_t address = FLASH_LOG_COUNT_SEC_START_ADDRESS;
  uint32_t pageEndAddress = FLASH_LOG_COUNT_SEC_END_ADDRESS-2;
  while(address<pageEndAddress)
  {
    if((*(__IO uint64_t*)address) == 0xFFFFFFFFFFFFFFFF)
    {
      FLASH_ProgramWordArray(address, dataToWrite,2);
      stat = 1;
      break;
    }
    else
    {
      address+=8;
    }
  }
  //  if(stat!=1)
  //  {
  //    uint32_t data1,data2;
  //    ReadDataForKey(0x1234,&data1);
  //    ReadDataForKey(0x1235,&data2);
  //    FLASH_ErasePage(0x79000);
  //    WriteKeyandDatatoFlash(0x1234,data1);
  //    WriteKeyandDatatoFlash(0x1235,data2);
  //    Serial_Debug("\r\nactual data");
  //    char dispstring[10];
  //    snprintf(dispstring,10,"%lu\r\n",data);
  //    Serial_Debug(dispstring);
  //    snprintf(dispstring,10,"data1->%lu\r\n",data1);
  //    Serial_Debug(dispstring);
  //    snprintf(dispstring,10,"data2->%lu\r\n",data2);
  //    Serial_Debug(dispstring);
  //    goto writeagain;
  //  }
  return stat;
}

uint16_t ReadDataForKey(uint16_t key, uint32_t* data)
{
  uint32_t address = (uint32_t)FLASH_LOG_COUNT_SEC_END_ADDRESS;
  uint32_t pageStartAddress = (uint32_t)FLASH_LOG_COUNT_SEC_START_ADDRESS;
  uint16_t keyValue = (uint16_t)0x1234;
  *data = 0;
  while(address+1>=pageStartAddress)
  {
    keyValue = (*(__IO uint16_t*)(address+1));
    if(keyValue==key)
    {
      //      Serial_Debug("\r\n Data found at Address->");
      //      Serial_Debug_Hex(address);
      
      *data = (*(__IO uint32_t*)(address + 5));
      break;
    }
    else
    {
      address-=8;
    }
  }
  return 1;
}
