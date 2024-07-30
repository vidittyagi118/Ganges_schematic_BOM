#include "fsl_flash.h"
#include "serial_debug.h"
#include "fsl_common.h"



#define NOOFSECTORSINUSE        4//four sectors are totally in use

#define ONEADDSIZE              8//one data storing it takes 4 address locations

#define BYTECASE        1       //for 8/16 bits
#define LONGCASE        2       //for 32 bits
#define DOUBLECASE      4       //for 64 bits

void error_trap(void);

status_t flash_init(void);
status_t sectorErasing(uint32_t sectorAddress);
status_t write_flash(uint32_t * buffer, uint8_t noofAddLocs, uint32_t statAddress);
status_t WritingToOneAddress(uint32_t phyAdd, uint16_t writeData);
status_t EraseFlash(uint32_t address, uint32_t lengthInBytes);
status_t WriteArray(uint32_t phyAdd, uint32_t* writeData,uint8_t noOfData );

void ReadFlash(uint8_t* data, uint32_t destAddress,uint32_t noOfData);