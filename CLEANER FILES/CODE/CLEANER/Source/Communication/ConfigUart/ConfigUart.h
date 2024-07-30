#ifndef _CONFIGUART_H_
#define _CONFIGUART_H_

#include <stdint.h>
#include "ConfigUart_Config.h"
#include "Customtypedefs.h"

#ifdef __cplusplus
extern "C" {
#endif
  
ErrorStatus ConfigUartInit (void);
ErrorStatus ConfigUartDeInit (void);
ErrorStatus ConfigUartTransmit (uint8_t * txBuffer, uint16_t txBufferLength);
ErrorStatus ConfigUartReceive (uint8_t * rxBuffer, uint16_t rxBufferLength);
static void ConfigUartReceive_Callback (uint8_t rxData);
static void ConfigUartTransmit_Callback (void);
static void ConfigUartError_Callback (void);
void ConfigRxDataReceive (uint8_t * rxBuffer, uint16_t rxBufferLength);
static void CombineConfigUartReceiveData (uint8_t rxData);
void DisableConfigUartRxIT(void);
void EnableConfigUartRxIT(void); 
ITStatus * GetConfigUartTxCompleteFlagPtr (void);

#ifdef __cplusplus
}
#endif

#endif