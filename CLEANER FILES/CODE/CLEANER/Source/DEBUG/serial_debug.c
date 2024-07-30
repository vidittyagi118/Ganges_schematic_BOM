#include <string.h>
#include <stdio.h>
#include "Serial_Debug.h"
#include "NumStrConversion.h"

void Serial_Debug (const char *txdata_buf)
{
  Serial_Debug_Bytes(txdata_buf, strlen(txdata_buf));  
}

void Serial_Debug_Char (const char txdata_buf)
{
  Serial_Debug_Bytes(&txdata_buf, 1); 
}

void Serial_Debug_Bytes (const char *txdata_buf, uint16_t txdata_buf_strlen)
{
#ifdef ENABLE_SERIAL_DEBUG
 DebugUartTransmit ((uint8_t *) txdata_buf, txdata_buf_strlen);
 //ZigbeeTxData ((uint8_t *) txdata_buf, txdata_buf_strlen);   // Can be used Only when Zigbee in API Mode.
#endif 
}

void Serial_Debug_Num(int32_t message_int)
{
#ifdef ENABLE_SERIAL_DEBUG
  static char dispstring[50];
  snprintf(dispstring, sizeof dispstring, "%d", message_int);
  Serial_Debug_Bytes((const char *)dispstring, strlen(dispstring));
#endif 
}

void Serial_Debug_Hex(int32_t message_int)
{
#ifdef ENABLE_SERIAL_DEBUG
  static char dispstring[50];
  snprintf(dispstring, sizeof dispstring, "%x", message_int);
  Serial_Debug_Bytes((const char *)dispstring, strlen(dispstring));
#endif 
}
void Serial_Debug_Float(float message_float, uint8_t noOfDecimal)
{
#ifdef ENABLE_SERIAL_DEBUG
  char dispstring[20];
  ftoa(message_float, dispstring, noOfDecimal);
  Serial_Debug(dispstring);
#endif 
}