#ifndef SDC_TXRX_SPI_H		//Do only once the first time this file is used
#define	SDC_TXRX_SPI_H	

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>


extern void SDC_spitx (unsigned char);
extern unsigned char SDC_spirx (void);

//bool SpiTxRxData (uint8_t device, uint8_t *txdata, uint8_t *rxdata, uint16_t datalen);

extern volatile bool isTransferCompleted;

extern void msdelay_SDC_prog (unsigned int msdel2);

#endif