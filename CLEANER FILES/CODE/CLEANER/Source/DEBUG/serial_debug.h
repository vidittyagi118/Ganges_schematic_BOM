#ifndef SERIAL_DEBUG_H
#define SERIAL_DEBUG_H

#include <stdint.h>
#include "customtypedefs.h"

#ifdef __cplusplus
extern "C" {
#endif
  
/**
* @brief Uncomment the line below to Use Serial Debug Feature
*/
#define ENABLE_SERIAL_DEBUG

void Serial_Debug (const char *txdata_buf);
void Serial_Debug_Char (const char txdata_buf);
void Serial_Debug_Bytes (const char *txdata_buf, uint16_t txdata_buf_strlen);
extern ErrorStatus DebugUartTransmit (uint8_t * txBuffer, uint16_t txBufferLength);
extern ErrorStatus ZigbeeTxData (const uint8_t *txData, const uint16_t txDatalen); 
void Serial_Debug_Num(int32_t message_int);
void Serial_Debug_Hex(int32_t message_int);
void Serial_Debug_Float(float message_float, uint8_t noOfDecimal);
  
#ifdef __cplusplus
}
#endif

#endif //SERIAL_DEBUG_H