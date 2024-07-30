/**
******************************************************************************
* @file    STM32F3xx_EEPROM_Emulation/inc/eeprom.h
* @author  MCD Application Team
* @version V1.0.0
* @date    02-October-2012
* @brief   Main program body.
******************************************************************************
* @attention
*
* <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
*
* Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
* You may not use this file except in compliance with the License.
* You may obtain a copy of the License at:
*
*        http://www.st.com/software_license_agreement_liberty_v2
*
* Unless required by applicable law or agreed to in writing, software 
* distributed under the License is distributed on an "AS IS" BASIS, 
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*
******************************************************************************
*/ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __EEPROM_H
#define __EEPROM_H

#include "fsl_common.h"
#include <stdint.h>
#include <stdbool.h>
#include "eepromStdtoHalDriver.h"

/* Exported constants --------------------------------------------------------*/
/* Define the Flash page size depending on the used device */
#define PAGE_SIZE  (uint16_t)0x1000  /* Page size = 4KByte */

/* EEPROM start address in Flash */
#define EEPROM_START_ADDRESS    ((uint32_t)0x0007C000) //((uint32_t)0x000FC000) /* EEPROM emulation start address:
/* after 252KByte of used Flash memory -> The last 2 pages(4K+4K) is used for EEPROM */
/* Pages 0 and 1 base and end addresses */
#define PAGE0_BASE_ADDRESS      ((uint32_t)(EEPROM_START_ADDRESS + 0x000))
#define PAGE0_END_ADDRESS       ((uint32_t)(EEPROM_START_ADDRESS + (PAGE_SIZE - 1)))
#define PAGE1_BASE_ADDRESS      ((uint32_t)(EEPROM_START_ADDRESS + PAGE_SIZE))
#define PAGE1_END_ADDRESS       ((uint32_t)(EEPROM_START_ADDRESS + (2 * PAGE_SIZE - 1)))
#define PAGE2_BASE_ADDRESS      ((uint32_t)(EEPROM_START_ADDRESS + PAGE_SIZE*2))
#define PAGE2_END_ADDRESS       ((uint32_t)(EEPROM_START_ADDRESS + (3 * PAGE_SIZE - 1)))
#define PAGE3_BASE_ADDRESS      ((uint32_t)(EEPROM_START_ADDRESS + PAGE_SIZE*3))
#define PAGE3_END_ADDRESS       ((uint32_t)(EEPROM_START_ADDRESS + (4 * PAGE_SIZE - 1)))

/* Used Flash pages for EEPROM emulation */
#define PAGE0                   ((uint16_t)0x0000)
#define PAGE1                   ((uint16_t)0x0001)
/* No valid page define */
#define NO_VALID_PAGE           ((uint16_t)0x00AB)
/* Page status definitions */
#define ERASED                  ((uint16_t)0xFFFF)     /* PAGE is empty */
#define RECEIVE_DATA            ((uint16_t)0xEEEE)     /* PAGE is marked to receive data */
#define VALID_PAGE              ((uint16_t)0x0000)     /* PAGE containing valid data */
/* Valid pages in read and write defines */
#define READ_FROM_VALID_PAGE    ((uint8_t)0x00)
#define WRITE_IN_VALID_PAGE     ((uint8_t)0x01)
/* Page full define */
#define PAGE_FULL               ((uint8_t)0x80)
/* Variables' number */
#define NB_OF_VAR               ((uint8_t)0x03)
/* Exported types ------------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
uint16_t EE_Init(void);
uint16_t EE_ReadVariable(uint16_t VirtAddress, uint32_t* Data);
uint16_t EE_WriteVariable(uint16_t VirtAddress, uint32_t Data);
uint16_t GetMaxVirtualAddress (void);
static FLASH_Status EE_Format(void);
static uint16_t EE_VerifyPageFullWriteVariable(uint16_t VirtAddress, uint32_t Data);
static uint16_t EE_PageTransfer(uint16_t VirtAddress, uint32_t Data);
static uint16_t EE_FindValidPage(uint8_t Operation);
#endif /* __EEPROM_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
