#ifndef __EVENT_LOGGER_FLASH_H
#define __EVENT_LOGGER_FLASH_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

  /* We are Using 2 Sectors (4K +4K = 8K) for All Event Log Storage. Data is stored in One sector Only at any time.
     Before Storing in any Sector its erased. */ 
  /* The index of Storage and Current Active Sector are all stored in RTC NVM registers */
  
  /* In the same manner as All Log, we are logging only the crtical events in another secor */
      
   /* Always have the start address as Even Number. As we use continuous read and Write , most operations happens as two bytes(Even First and Odd Next) */

  
#define SECTOR_SIZE_KB     4U                                                      /* 4KBytes */
#define BLOCK_SIZE_KB      64U                                                     /* 64KBytes */
#define NO_OF_BLOCKS       16U                                                     /* No of 64KBytes Blocks in Upper 1MByte is 16 */

#define FLASH_LOG_ALL_SEC1_START_ADDRESS        0x0007A000UL                         /* Start From Second Sector  */   
#define FLASH_LOG_ALL_SEC1_END_ADDRESS          0x0007AFFFUL                         
#define FLASH_LOG_ALL_SEC2_START_ADDRESS        0x0007B000UL                         /* Third Sector  */   
#define FLASH_LOG_ALL_SEC2_END_ADDRESS          0x0007BFFFUL                         /* Lets Have two Sectors for Log Totally*/
#define FLASH_LOG_ALL_SECTOR_SIZE               0x1000UL                           /* 4 Kbytes */
#define FLASH_LOG_ALL_TOTAL_SIZE                0x2000UL                           /* FLASH_LOG_END_ADDRESS - FLASH_LOG_START_ADDRESS + 1*/ /* 8KByte */

#define FLASH_LOG_CRITICAL_SEC1_START_ADDRESS   0x003000UL                         /* Start From Fourth Sector  */   
#define FLASH_LOG_CRITICAL_SEC1_END_ADDRESS     0x003FFFUL                         /
#define FLASH_LOG_CRITICAL_SEC2_START_ADDRESS   0x004000UL                         /* Start From Fifth Sector  */   
#define FLASH_LOG_CRITICAL_SEC2_END_ADDRESS     0x004FFFUL                         /* Lets Have two Sectors for Log */
#define FLASH_LOG_CRITICAL_SECTOR_SIZE          0x1000UL                           /* 4 Kbytes */
#define FLASH_LOG_CRITICAL_TOTAL_SIZE           0x2000UL                           /* FLASH_LOG_CRITICAL_END_ADDRESS - FLASH_LOG_CRITICAL_START_ADDRESS + 1*/ /* 8KByte */


#define FLASH_LOG_COUNT_SEC_START_ADDRESS       0x00079000UL
#define FLASH_LOG_COUNT_SEC_END_ADDRESS         0x00079FFFUL
  
#define FLASH_SINGLE_SECTOR_END_ADDRESS         (FLASH_LOG_SECTOR_SIZE -1)  
       
       
#define SINGLE_STORE_BYTE_COUNT                8                                   /*  Always have the this as Even Number. As we use continuous read and Write , most operations happens as two bytes(Even First and Odd Next) */

#define MAX_STORE_SIZE_IN_SINGLE_SECTOR        (((uint32_t)(FLASH_LOG_ALL_SECTOR_SIZE/SINGLE_STORE_BYTE_COUNT))*SINGLE_STORE_BYTE_COUNT)
#define MAX_NO_OF_STORE_LOG_DATA               ((uint32_t)(MAX_STORE_SIZE_IN_SINGLE_SECTOR/SINGLE_STORE_BYTE_COUNT))    /* Its 512 as per the calculation */

#define RTC_BACKUP_ALL_LOG_STORE_INDEX_SECTOR_ADD                   0x03
#define RTC_BACKUP_ALL_LOG_STORE_NO_OF_DATA_ADD                     0x04
  
typedef enum eStoreEventType_def
{
  EV_STORE_TYPE_ALL,
  EV_STORE_TYPE_CRITICAL
}eStoreEventType; 

typedef enum eStoreEventCurSector_def
{
  EV_STORE_SECTOR_1,
  EV_STORE_SECTOR_2
}eStoreEventCurSector; 

//typedef struct{
//  uint32_t utcTime;
//  uint8_t event;
//  uint8_t eventStatus;
//}stLogStoreStruct;

void EventLoggerAllLogFlash_Init (void);
bool StoreFlashAllLog(uint8_t * logData, uint16_t logDataLen);
uint32_t ReadFlashAllLog(uint8_t  *logStoreData, uint16_t maxNoofData, uint16_t startIndex, uint16_t endIndex);
void ClearFlashAllLog (void);
uint16_t WriteKeyandDatatoFlash(uint16_t key, uint32_t data);
uint16_t ReadDataForKey(uint16_t key, uint32_t* data);
uint16_t CheckAndWriteData(uint16_t key, uint32_t data);
#ifdef __cplusplus
}
#endif

#endif //__EVENT_LOGGER_FLASH_H