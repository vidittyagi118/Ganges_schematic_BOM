#ifndef _SPI_H_
#define _SPI_H_

#include <stdint.h>
#include <stdbool.h>

enum {
  DRIVE_1,
  DRIVE_2,
  DRIVE_3,
  FLS,  
  LCD  
};
 
bool SpiInit (void);
bool SpiTxData (uint8_t device, uint8_t *txdata, uint16_t txdatalen);
bool SpiRxData (uint8_t device, uint8_t *rxdata, uint16_t rxdatalen);
bool SpiConsecutiveTxRxData (uint8_t device, uint8_t *txdata, uint16_t txdatalen, uint8_t *rxdata, uint16_t rxdatalen);
bool SpiTxRxData (uint8_t device, uint8_t *txdata, uint8_t *rxdata, uint16_t datalen);
void Direct_SPI_Init();
bool SD_M_IN_Init (void);
void SD_Write_Byte(char SPIData);
unsigned char SD_READ_Byte (const unsigned char regAddr);
void SDdelay1( unsigned int Timet);
void SDCARDMount();
void PINCHECK_MIN();

#endif