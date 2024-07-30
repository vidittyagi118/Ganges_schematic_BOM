#ifndef __COMM_COMMON_H_
#define __COMM_COMMON_H_

#include "Customtypedefs.h"
#include <stdint.h>
#include <stdbool.h>

typedef enum Uart_type_def
{
  ZIGBEE_UART,
  DEBUG_UART,
  CONFIG_UART,
  UNKNOWN_UART  
}eUartType;

typedef struct{
  bool enable;
  long int interval_ms;
  long int noOfMessages;
  uint32_t maxReconnectTime;
}stHeartbeatConfig;

void CommTimeIncrement_ms (void);
void CommTimerStop (void);
bool IsCommTimeOver (void);

#ifdef __cplusplus
extern "C" {
#endif
  
ErrorStatus UartTransmit (uint8_t * txBuffer, uint16_t txBufferLength);
ErrorStatus UartTransmitType (eUartType uartType, uint8_t * txBuffer, uint16_t txBufferLength);
ErrorStatus UartAllTransmit (uint8_t * txBuffer, uint16_t txBufferLength);
void SetUartType (eUartType uartRxType);
stHeartbeatConfig* GetSetHeartbeatConfig(void);
void RestartCommTimer (void);
void ClearCommTimeOver (void);
static void CommTimerOn (uint32_t setCommTimeCount_ms);
eUartType GetUartType (void);
#ifdef __cplusplus
}
#endif

#endif