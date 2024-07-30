#ifndef _ZIGBEEUART_H_
#define _ZIGBEEUART_H_

#include <stdint.h>
#include "ZigbeeUart_Config.h"
#include "Customtypedefs.h"


#ifdef __cplusplus
extern "C" {
#endif
    
ErrorStatus ZigbeeUartInit (void);
ErrorStatus ZigbeeUartDeInit (void);
ErrorStatus ZigbeeUartTransmit (uint8_t * txBuffer, uint16_t txBufferLength);
ErrorStatus ZigbeeUartReceive (uint64_t *rxMacAddr, uint8_t * rxBuffer, uint16_t rxBufferLength);
static void ZigbeeUartReceive_Callback (uint8_t rxData);
static void ZigbeeUartTransmit_Callback (void);
static void ZigbeeUartError_Callback (void);
static void CombineZigbeeUartReceiveData (uint64_t rxMacAddr, uint8_t rxData);
void DisableZigbeeUartIT(void);
void EnableZigbeeUartIT(void); 
ITStatus * GetZigbeeUartTxCompleteFlagPtr (void);
uint64_t GetZigbeeUartRxMacAddr (void);

#ifdef ZIGBEE_API_MODE_ENABLE
void ZigbeeRxDataReceive (uint64_t rxMacAddr, uint8_t * rxBuffer, uint16_t rxBufferLength);
#endif

#ifdef ENABLE_ZIGBEE_RESET
void ZigbeeResetInit(void);
void ZigbeeResetOn(void);
void ZigbeeResetOff(void);
#endif
  
#ifdef __cplusplus
}
#endif

#endif