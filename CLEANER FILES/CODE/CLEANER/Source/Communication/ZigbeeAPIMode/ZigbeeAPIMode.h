#ifndef __ZIGBEE_API_MODE_H
#define __ZIGBEE_API_MODE_H

#include "ZigbeeUart.h"
#include <stdint.h>
#include <stdbool.h>
#include "ZigbeeAPI.h"
#include "Serial_Debug.h"

#ifdef __cplusplus
extern "C" {
#endif
  
#ifdef ENABLE_SERIAL_DEBUG
  #ifdef ZIGBEE_API_MODE_ENABLE
    #define ENABLE_LIBRARY_DEBUG false  /* Make this as true when Debugging Needed Inside Zigbee API Library */
  #else
    #define ENABLE_LIBRARY_DEBUG false
  #endif
#else 
  #define ENABLE_LIBRARY_DEBUG false
#endif
  

#define DEFAULT_RX_MAC_ADDR                  0xFFFFFFFFFFFFFFFF

#define MAC_ID_HEX_LEN          (8U) 
#define PAN_ID_HEX_LEN          (8U)    
#define PAN_ID_16BIT_HEX_LEN    (2U) 
#define LINK_KEY_HEX_LEN        (32U)               
#define NETWORK_OPTIONS_HEX_LEN (1U)  
#define ASSOC_IND_HEX_LEN       (1U) 
#define ENC_ENABLE_HEX_LEN      (1U) 
#define JV_ENABLE_HEX_LEN       (1U) 
#define OP_CHANNEL_HEX_LEN      (1U)
#define STACK_PROFILE_HEX_LEN   (1U)

#define MAC_ID_STR_LEN          ((MAC_ID_HEX_LEN*2)+1)                  // 8 Hex Byte + 1 null      
#define PAN_ID_STR_LEN          ((PAN_ID_HEX_LEN*2)+1)                  // 8 Hex Bytes + 1 null
#define PAN_ID_16BIT_STR_LEN    ((PAN_ID_16BIT_HEX_LEN*2)+1)            // 2 Hex Byte + 1 null
#define LINK_KEY_STR_LEN        ((LINK_KEY_HEX_LEN*2)+1)                // 32 Hex Bytes + 1 null
#define NETWORK_OPTIONS_STR_LEN ((NETWORK_OPTIONS_HEX_LEN*2)+1)         // 1 Hex Byte + 1 null
#define ASSOC_IND_STR_LEN       ((ASSOC_IND_HEX_LEN*2)+1)               // 1 Hex Byte + 1 null
#define ENC_ENABLE_STR_LEN      ((ENC_ENABLE_HEX_LEN*2)+1)              // 1 Hex Byte + 1 null
#define JV_ENABLE_STR_LEN       ((JV_ENABLE_HEX_LEN*2)+1)               // 1 Hex Byte + 1 null
#define OP_CHANNEL_STR_LEN      ((OP_CHANNEL_HEX_LEN*2)+1)              // 1 Hex Byte + 1 null
#define STACK_PROFILE_STR_LEN   ((STACK_PROFILE_HEX_LEN*2)+1)           // 1 Hex Byte + 1 null  

typedef enum eZigNwReset_def {
  NW_RESET_IDLE = -1,
  NW_RESET_LOCAL = 0,
  NW_RESET_GLOBAL = 1
}eZigNwReset;
  
typedef struct
{
  uint8_t panID[PAN_ID_STR_LEN];
  uint8_t encryptionLinkKey[LINK_KEY_STR_LEN];
  uint8_t encryptionOptions[NETWORK_OPTIONS_STR_LEN];
  uint8_t encryptionEnable;
  uint8_t joinVerifyEnable;
  uint8_t writeEnable;
}stZigbeeConfig;

typedef struct
{
  uint8_t configPanID[PAN_ID_STR_LEN];
  uint8_t encryptionOptions[NETWORK_OPTIONS_STR_LEN];
  uint8_t AssocStatus[ASSOC_IND_STR_LEN];
  uint8_t encryptionEnable[ENC_ENABLE_STR_LEN];
  uint8_t joinVerifyEnable[JV_ENABLE_STR_LEN];
}stZigbeeConfigRead;

typedef struct
{
  uint8_t opPanID[PAN_ID_STR_LEN];
  uint8_t opPan16BitID[PAN_ID_16BIT_STR_LEN];
  uint8_t opChannel[OP_CHANNEL_STR_LEN];
  uint8_t stackProfile[STACK_PROFILE_STR_LEN];
}stZigbeeNetworkParamRead;

bool ZigbeeAPIModeInit (void);
void ZigbeeAPIModeTimer_ms(void);
void ZigbeeUartRx(uint8_t rxData); 
ErrorStatus ZigbeeTxData (const uint8_t *txData, const uint16_t txDatalen);        /* This sends data Only to the coordinator */
ErrorStatus ZigbeeApiTxData (const uint64_t destAddress, const uint8_t *txData, const uint16_t txDatalen);
bool IsZigbeeNetworkJoined (void);
uint64_t GetZigbeeMacAddress (void);
void ZigbeePoll (void);

stZigbeeConfig *GetSetZigbeeConfig(void);
void UpdateZigbeeConfigChangeFlag (bool status);
void UpdateZigbeeNetworkResetFlag (eZigNwReset  status);
bool GetZigbeeParameters (stZigbeeConfigRead * zigbeeReadParams);
bool GetZigbeeNetworkParameters (stZigbeeNetworkParamRead * zigbeeNwReadParams);

static void ZigbeeAPIRxDataFunc (sZigbeeAPIRxData zigbeeRxData);
#if ENABLE_LIBRARY_DEBUG == true
static void DebugUartTx(const uint8_t * txBuffer, const uint16_t txBufferLength);
#endif
static bool ZigbeeUartTx(uint8_t * txBuffer, uint16_t txBufferLength);       

#ifdef ZIGBEE_API_MODE_ENABLE
static inline void ResetZigbeeNetworkJoinedStatus (void);
static inline void UpdateZigbeeNetworkJoinedStatus (void);
static bool CheckAndUpdateZigbeeConfig (void);
static bool CheckAndResetZigbeeNetwork (void);
#endif
#ifdef __cplusplus
}
#endif

#endif /* __ZIGBEE_API_H */