/*
This file is the main Header File for Zigbee API Library.
Its used in the main application program along with the Library file.
This file should NOT be modified as its used both in the ZigbeeAPI Library and in the main application program.

It has function declarations that are used in the Library.
*/


/* *****************   THIS FILE SHOULD NOT BE MODIFIED *************************/
/* *****************   THIS FILE SHOULD NOT BE MODIFIED *************************/
/* *****************   THIS FILE SHOULD NOT BE MODIFIED *************************/
/* *****************   THIS FILE SHOULD NOT BE MODIFIED *************************/
/* *****************   THIS FILE SHOULD NOT BE MODIFIED *************************/

#ifndef __ZIGBEE_API_H
#define __ZIGBEE_API_H

#include <stdint.h>
#include <stdbool.h>

/*  IMPORTANAT ******************************************

  Debug Uart SHOULD NOT be same as the Zigbee Uart. It will cause the program to hang 
* *******************************************************************************************/

/** Buils an uint16_t out of 2 single bytes */
#define UINT16(msb,lsb)     (uint16_t)(((msb) << 8) | (lsb))
/** Buils an uint64_t out of 2 uint32_t */
#define UINT64(msb,lsb)     (uint64_t)(((uint64_t)(msb) << 32) | (lsb))

#define UINT64_HI32(u64)     (uint32_t)((u64) >> 32)
#define UINT64_LO32(u64)     (uint32_t)((u64) & 0xFFFFFFFF)

#define MAX_ZIGBEE_UNICAST_TX_FRAME_BYTES       255             // This value is as per Zigbee Specs (as per command -> NP)
#define MAX_ZIGBEE_BROADCAST_TX_FRAME_BYTES     74              // This is as per Zigbee specs.

enum eRxDataType 
{
 UNICAST = 0,               // This values should not be changed as library uses these values
 BROADCAST = 1
};

typedef struct sZigbeeAPIRxData_def {
  uint64_t rxRemote64bitAddress;
  uint16_t rxRemote16bitAddress;
  uint16_t rxDataLength;
  uint8_t *rxData;
  enum eRxDataType rxDataType; 
}sZigbeeAPIRxData;

typedef void (*ZigbeeAPIRxDataCallback_ptr)(sZigbeeAPIRxData);                                          // Zigbee Received Data Call Back Pointer
typedef bool (*ZigbeeAPITxDataCallback_ptr)(uint8_t *txData, uint16_t txDataLength);                    // Zigbee Function to Transmit Data Over Uart
typedef void (*ZigbeeAPIRxIntEnable_ptr)(void);                                                         // Zigbee Uart Receive Interrupt Enable
typedef void (*ZigbeeAPIRxIntDisable_ptr)(void);                                                        // Zigbee Uart Receive Interrupt Disable
typedef void (*ZigbeeAPIModResetOn_ptr)(void);                                                          // Zigbee Reset Pin Enable (Reset ON)
typedef void (*ZigbeeAPIModResetOff_ptr)(void);                                                         // Zigbee Reset Pin Disable (Reset OFF)
typedef void (*ZigbeeAPIDebugSerialTx_ptr)(const uint8_t *txdata_buf, const uint16_t txdatalen);        // Zigbee Function to Transmit Debug Data Over Uart
                                                                                                        // Debug Uart SHOULD NOT be same as the Zigbee Uart. It will cause the program to hang 

typedef struct sZigbeeAPICallback_def {
  ZigbeeAPIRxDataCallback_ptr ZigbeeAPIrxDataCallback;
  ZigbeeAPITxDataCallback_ptr ZigbeeAPITxDataCallback;
  ZigbeeAPIRxIntEnable_ptr ZigbeeAPIRxIntEnable;
  ZigbeeAPIRxIntDisable_ptr ZigbeeAPIRxIntDisable;
  ZigbeeAPIModResetOn_ptr ZigbeeAPIModResetOn;
  ZigbeeAPIModResetOff_ptr ZigbeeAPIModResetOff;
  ZigbeeAPIDebugSerialTx_ptr ZigbeeAPIDebugTxDataCallback;
}sZigbeeAPICallback;

#ifdef __cplusplus
extern "C" {
#endif
  
bool ZigbeeAPIInit (sZigbeeAPICallback *ZigbeeAPICallbacks, bool enableDebug);

bool isZigbeeAPINetworkJoined (void);
bool SendDatatoCoordinator (const uint8_t *data, const uint16_t data_len);
bool SendBroadcastData(const uint8_t *data, const uint16_t data_len);

bool SetZigbeeAPIPanId (uint64_t panId);
bool SetZigbeeAPIEncryptionLinkKey(const uint8_t * const key, const uint16_t length);
bool SetZigbeeAPINetworkEncryptionKey(const uint8_t * const key, const uint16_t length);                // NOT POSSIBLE TO UPDATE THIS VALUE FOR ROUTER
bool SetZigbeeAPISecurityEnable(bool enable);
bool SetZigbeeAPIEncryptionOptions(const uint8_t options);
bool SetZigbeeAPIWriteConfig(void);
bool SetZigbeeAPIJoinVerification (bool enable);
bool SetZigbeeAPIResetNetwork(bool nrParam);

bool GetOperatingPanId(uint64_t * opPanId);
bool GetOperating16bitPanId(uint16_t * opPanId);
bool GetConfiguredPanId(uint64_t * opPanId);
bool GetZigbeeAPIEncryptionOptions(uint8_t *options);
bool GetZigbeeAPISecurityState(bool *enable);
bool GetZigbeeAPIJoinVerifyEnableState(bool *enable);
uint64_t GetZigbeeAPIMacAddress(void);
bool GetAssociationStatus (uint8_t *assocStatus);   
bool GetOperatingChannel(uint8_t * opChannel);
bool GetStackProfile(uint8_t * stackProfile);


bool SendDatatoRemoteNode(const uint64_t remoteAddr, const uint8_t *data, const uint16_t data_len);               // Not checked
bool SendExplicitDatatoRemoteNode(const uint64_t remoteAddr, const uint8_t *data, const uint16_t data_len);        // Not checked

void ZigbeeAPIPoll (void);                                                              // This has to be frequenctly and continuously called inorder to receive data
void ZibeeAPIUartRxChar(uint8_t rxData);                                                // This has to be called from Uart Receive Routine for every Data Char reception
void ZigbeeAPImsTimerHandler (void);                                                    // This has to be called every millisecond.
   
#ifdef __cplusplus
}
#endif

#endif /* __ZIGBEE_API_H */