#include <stdbool.h>
#include <stdint.h>

//enum {
//  DRIVE_1,
//  DRIVE_2,
//  DRIVE_3,
//  FLS,  
//  SDC  
//};
void int_SDC_SPI (void);
void SDC_SPI_Tx (uint8_t * SDCSpiTxData , uint8_t * SDCSpiRxData, uint8_t SDC_SPI_TXData_Len);
//
//bool SpiTxRxData (uint8_t device, uint8_t *txdata, uint8_t *rxdata, uint16_t datalen);
