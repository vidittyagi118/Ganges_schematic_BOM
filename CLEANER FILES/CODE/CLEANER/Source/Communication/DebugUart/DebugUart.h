#ifndef _DEBUGUART_H_
#define _DEBUGUART_H_

#include <stdint.h>
#include "DebugUart_Config.h"
#include "Customtypedefs.h"

#ifdef __cplusplus
extern "C" {
#endif
  
ErrorStatus DebugUartInit (void);
ErrorStatus DebugUartDeInit (void);
ErrorStatus DebugUartTransmit (uint8_t * txBuffer, uint16_t txBufferLength);
ErrorStatus DebugUartReceive (uint8_t * rxBuffer, uint16_t rxBufferLength);
static void DebugUartReceive_Callback (uint8_t rxData);
static void DebugUartTransmit_Callback (void);
static void DebugUartError_Callback (void);
void DebugRxDataReceive (uint8_t * rxBuffer, uint16_t rxBufferLength);
static void CombineDebugUartReceiveData (uint8_t rxData);
void DisableDebugUartRxIT(void);
void EnableDebugUartRxIT(void); 
ITStatus * GetDebugUartTxCompleteFlagPtr (void);

#ifdef __cplusplus
}
#endif

#endif