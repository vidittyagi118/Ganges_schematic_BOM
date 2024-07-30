#include "ZigbeeAPIMode.h"
#include "ZigbeeAPI.h"
#include "NumStrConversion.h"
#include "Serial_Debug.h"
//#include "EventLogger.h"
#include "CommCommon.h"
#include "ApiProcessing.h"
#include "FaultProcess.h"
#include "Serial_Debug.h"

#include <stdio.h>

enum {
  NOT_JOINED = 0,
  JOINED = 1
};

#define ZIGBEE_NETWORK_STATUS_UPDATE_INTERVAL_MS  2000  

static stZigbeeConfig zigbeeConfigValues;
static bool ZigbeeNetworkJoinedStatus = NOT_JOINED;
static uint32_t ZigNetStatusUpdateTimeinterval = 0;

#ifdef ZIGBEE_API_MODE_ENABLE
static sZigbeeAPICallback ZigbeeAPICallbackFunc;
static bool zigbeeConfigChangeFlag = false;
static eZigNwReset zigbeeNetworkResetFlag = NW_RESET_IDLE;
static bool zigbeeInitStatus = false;
#endif

stZigbeeConfig *GetSetZigbeeConfig(void)
{
  return(&zigbeeConfigValues);  
}

bool ZigbeeAPIModeInit (void)
{
 #ifdef ZIGBEE_API_MODE_ENABLE
  ZigbeeAPICallbackFunc.ZigbeeAPIrxDataCallback = &ZigbeeAPIRxDataFunc;
  ZigbeeAPICallbackFunc.ZigbeeAPITxDataCallback = &ZigbeeUartTx;
  ZigbeeAPICallbackFunc.ZigbeeAPIRxIntEnable = &EnableZigbeeUartIT;
  ZigbeeAPICallbackFunc.ZigbeeAPIRxIntDisable = &DisableZigbeeUartIT;
#ifdef ENABLE_ZIGBEE_RESET
  ZigbeeAPICallbackFunc.ZigbeeAPIModResetOn = &ZigbeeResetOn;
  ZigbeeAPICallbackFunc.ZigbeeAPIModResetOff = &ZigbeeResetOff;
#else
  ZigbeeAPICallbackFunc.ZigbeeAPIModResetOn = NULL;
  ZigbeeAPICallbackFunc.ZigbeeAPIModResetOff = NULL;
#endif
#if ENABLE_LIBRARY_DEBUG == true
  ZigbeeAPICallbackFunc.ZigbeeAPIDebugTxDataCallback = &DebugUartTx;
#else
  ZigbeeAPICallbackFunc.ZigbeeAPIDebugTxDataCallback = NULL;
#endif  
  if(ZigbeeAPIInit(&ZigbeeAPICallbackFunc, ENABLE_LIBRARY_DEBUG) == false)
  {
    Serial_Debug("\nZigbee API Init Error ");
    zigbeeInitStatus = false;
    return false;
  }
  else
  {
    Serial_Debug("\nZigbee API Init OK ");
    zigbeeInitStatus = true;
    return true;
  }   
#else
   zigbeeInitStatus = true;
   return true; 
#endif
}

uint64_t GetZigbeeMacAddress (void)
{
  if(zigbeeInitStatus == true)
  {
    return GetZigbeeAPIMacAddress(); 
  }
  else
  {
    return DEFAULT_RX_MAC_ADDR;
  }
}

static inline void UpdateZigbeeNetworkJoinedStatus (void)                                    
{
#ifdef ZIGBEE_API_MODE_ENABLE
  if(ZigNetStatusUpdateTimeinterval >= ZIGBEE_NETWORK_STATUS_UPDATE_INTERVAL_MS)
  {
    ZigbeeNetworkJoinedStatus = isZigbeeAPINetworkJoined()? JOINED : NOT_JOINED;
    ZigNetStatusUpdateTimeinterval = 0;
    if(ZigbeeNetworkJoinedStatus == JOINED)
    {
      TransmitWelcomeMessage(ZIGBEE_UART);                                      /* This is done here just to transmit welcome message after some delay */
      ReportErrorOnZigeeStart();
    }
  }
#endif
}

bool IsZigbeeNetworkJoined (void)
{
  if(ZigbeeNetworkJoinedStatus == NOT_JOINED)
  {
    UpdateZigbeeNetworkJoinedStatus();
  }
  return ZigbeeNetworkJoinedStatus;
}

void ZigbeePoll (void)
{
#ifdef ZIGBEE_API_MODE_ENABLE
  if(zigbeeInitStatus == true)
  {
    if(IsZigbeeNetworkJoined() == JOINED)
    {
    ZigbeeAPIPoll();
    }
    (void)CheckAndResetZigbeeNetwork();
    (void)CheckAndUpdateZigbeeConfig();
  }
#endif
}

#ifdef ZIGBEE_API_MODE_ENABLE
static inline void ResetZigbeeNetworkJoinedStatus (void)
{
  ZigbeeNetworkJoinedStatus = false;
}
#endif

#ifdef ZIGBEE_API_MODE_ENABLE
static void ZigbeeAPIRxDataFunc (sZigbeeAPIRxData zigbeeRxData)
{
  Serial_Debug("\nZigbee Data Received -> \n");  
  char dispstring[200];
  snprintf(dispstring, sizeof dispstring, "\r\nReceived a %s RX packet [%08x:%08x|%04x], len %d\r\nData: ", 
           zigbeeRxData.rxDataType ? "BROADCAST" : "UNICAST", 
           UINT64_HI32(zigbeeRxData.rxRemote64bitAddress), UINT64_LO32(zigbeeRxData.rxRemote64bitAddress), 
           zigbeeRxData.rxRemote16bitAddress, zigbeeRxData.rxDataLength);
  Serial_Debug(dispstring);
  Serial_Debug("\n ** ");
  for (int i = 0; i < zigbeeRxData.rxDataLength; i++)
  {
/*    Serial_Debug_Char(zigbeeRxData.rxData[i]);           */                     /* Enable this to print ALL Received Characters in API Mode */
  }
  Serial_Debug(" **\n");
  ZigbeeRxDataReceive(zigbeeRxData.rxRemote64bitAddress, zigbeeRxData.rxData, zigbeeRxData.rxDataLength);
}
#endif

void ZigbeeAPIModeTimer_ms(void)
{
  ZigbeeAPImsTimerHandler(); 
  ZigNetStatusUpdateTimeinterval++;
}

void ZigbeeUartRx(uint8_t rxData)
{
  ZibeeAPIUartRxChar(rxData);
}

#ifdef ZIGBEE_API_MODE_ENABLE
bool ZigbeeUartTx(uint8_t * txBuffer, uint16_t txBufferLength)
{
  ErrorStatus  status  = ZigbeeUartTransmit(txBuffer, txBufferLength);
  return (status == SUCCESS);
}
#endif

#if ENABLE_LIBRARY_DEBUG == true
static void DebugUartTx(const uint8_t * txBuffer, const uint16_t txBufferLength)
{
  Serial_Debug_Bytes((const char *)txBuffer, txBufferLength);
}
#endif

ErrorStatus ZigbeeTxData (const uint8_t *txData, const uint16_t txDatalen)
{
  //SetLogEvent(EV_LOG_ZB_UART, (uint8_t)EV_UART_WRITE); 
#ifdef ZIGBEE_API_MODE_ENABLE
  bool status = false;
  if(ZigbeeNetworkJoinedStatus == JOINED)
  {
    uint16_t transmitIndex = 0;
    uint16_t noOfTxBytes = 0;
    uint16_t noOfRemainingTxBytes = txDatalen;
    while(noOfRemainingTxBytes)
    {
      if(noOfRemainingTxBytes <= MAX_ZIGBEE_UNICAST_TX_FRAME_BYTES)
      {
        noOfTxBytes = noOfRemainingTxBytes;
      }
      else
      {
        noOfTxBytes = MAX_ZIGBEE_UNICAST_TX_FRAME_BYTES;
      }
      status = SendDatatoCoordinator(&txData[transmitIndex], noOfTxBytes);
      if(status == true)
      {
        noOfRemainingTxBytes = noOfRemainingTxBytes - noOfTxBytes;
        transmitIndex = transmitIndex + noOfTxBytes;
      }
      else
      {
        break;
      }
    }
  }
  else
  {
    status = false;
  }
  if(status == false)
  {
    Serial_Debug("\n Zigbee Transmit Error");
  }
  return (status ? SUCCESS : ERROR);  
#else
  ErrorStatus status = ZigbeeUartTransmit((uint8_t *)txData, txDatalen);
  return status;  
#endif
}

ErrorStatus ZigbeeApiTxData (const uint64_t destAddress, const uint8_t *txData, const uint16_t txDatalen)
{
  //SetLogEvent(EV_LOG_ZB_UART, (uint8_t)EV_UART_WRITE); 
#ifdef ZIGBEE_API_MODE_ENABLE
  bool status = false;
  if(ZigbeeNetworkJoinedStatus == JOINED)
  {
    uint16_t transmitIndex = 0;
    uint16_t noOfTxBytes = 0;
    uint16_t noOfRemainingTxBytes = txDatalen;
    while(noOfRemainingTxBytes)
    {
      if(noOfRemainingTxBytes <= MAX_ZIGBEE_UNICAST_TX_FRAME_BYTES)
      {
        noOfTxBytes = noOfRemainingTxBytes;
      }
      else
      {
        noOfTxBytes = MAX_ZIGBEE_UNICAST_TX_FRAME_BYTES;
      }
      status = SendDatatoRemoteNode(destAddress, &txData[transmitIndex], noOfTxBytes);
      if(status == true)
      {
        noOfRemainingTxBytes = noOfRemainingTxBytes - noOfTxBytes;
        transmitIndex = transmitIndex + noOfTxBytes;
      }
      else
      {
        break;
      }
    }
  }
  else
  {
    status = false;
  }
  if(status == false)
  {
    Serial_Debug("\n Zigbee Transmit Error");
  }
  return (status ? SUCCESS : ERROR);  
#else
  ErrorStatus status = ZigbeeUartTransmit((uint8_t *)txData, txDatalen);
  return status;  
#endif
}

void UpdateZigbeeConfigChangeFlag (bool status)
{
  #ifdef ZIGBEE_API_MODE_ENABLE
  zigbeeConfigChangeFlag = status;
  #endif
}

void UpdateZigbeeNetworkResetFlag (eZigNwReset  status)
{
  #ifdef ZIGBEE_API_MODE_ENABLE
  zigbeeNetworkResetFlag = status;
  #endif
}

#ifdef ZIGBEE_API_MODE_ENABLE
static bool IsZigbeeConfigChanged (void)
{
  return zigbeeConfigChangeFlag;
}
#endif

#ifdef ZIGBEE_API_MODE_ENABLE
static bool CheckAndResetZigbeeNetwork (void)
{
  bool status = true;
  if(zigbeeNetworkResetFlag != NW_RESET_IDLE)
  {
    status = SetZigbeeAPIResetNetwork((bool)zigbeeNetworkResetFlag);
    zigbeeNetworkResetFlag = NW_RESET_IDLE;
  } 
  return status;
}
#endif

#ifdef ZIGBEE_API_MODE_ENABLE
static bool CheckAndUpdateZigbeeConfig (void)
{
  if(IsZigbeeConfigChanged() == false)
    return true;
  
  bool status = false;
  stZigbeeConfig *zigbeeConfigValues = GetSetZigbeeConfig();
  if(zigbeeConfigValues->panID[0] != NULL)
  {
    uint64_t panID = HexStringtoInt((uint8_t const *)zigbeeConfigValues->panID);
    status = SetZigbeeAPIPanId(panID);
    if(status == false)
    {
      Serial_Debug("\n ** PAN -ID Set - Error ");
      return false;
    }
    else
    {      
      Serial_Debug("\nPAN -ID Set - OK ");
    }
  }
  if(zigbeeConfigValues->encryptionLinkKey[0] != NULL)
  {
    uint8_t byteArray[LINK_KEY_HEX_LEN];
    uint16_t byteArrayLength = LINK_KEY_HEX_LEN;
    bool status;
    status = HexStringtoByteArray(zigbeeConfigValues->encryptionLinkKey, byteArray, &byteArrayLength);
    if(status == true)
    {
      status = SetZigbeeAPIEncryptionLinkKey(byteArray, byteArrayLength);
    }
    if(status == false)
    {
      Serial_Debug("\n ** Zigbee Link Key Set - Error ");
      return false;
    }
    else
    {
      Serial_Debug("\nZigbee Link Key Set - OK "); 
    }
  }
  if(zigbeeConfigValues->encryptionOptions[0] != NULL)
  {
    if(zigbeeConfigValues->encryptionOptions[1] == NULL)
    {
      zigbeeConfigValues->encryptionOptions[1] = zigbeeConfigValues->encryptionOptions[0];
      zigbeeConfigValues->encryptionOptions[0] = '0';
    }
    uint8_t options = HexStringtoHexByte(zigbeeConfigValues->encryptionOptions[0], zigbeeConfigValues->encryptionOptions[1]);
    bool status = SetZigbeeAPIEncryptionOptions(options);
    if(status == false)
    {
      Serial_Debug("\n ** Zigbee Net Options Set - Error ");
      return false;
    }
    else
    {
      Serial_Debug("\nZigbee Net Options Set - OK "); 
    }
  }
  if(zigbeeConfigValues->encryptionEnable != NULL)
  {
    bool status = SetZigbeeAPISecurityEnable(zigbeeConfigValues->encryptionEnable - '0');
    if(status == false)
    {
      Serial_Debug("\n ** Zigbee Sec Enable Set - Error ");
      return false;
    }
    else
    {
      Serial_Debug("\nZigbee Sec Enable Set - OK "); 
    }
  }
  if(zigbeeConfigValues->joinVerifyEnable != NULL)
  {
    bool status = SetZigbeeAPIJoinVerification(zigbeeConfigValues->joinVerifyEnable - '0');
    if(status == false)
    {
      Serial_Debug("\n ** Zigbee Join Verify Enable Set - Error ");
      return false;
    }
    else
    {
      Serial_Debug("\nZigbee Join Verify Enable Set - OK "); 
    }
  }
  if((zigbeeConfigValues->writeEnable != NULL) && (zigbeeConfigValues->writeEnable == '1'))
  {
    bool status = SetZigbeeAPIWriteConfig();
    if(status == false)
    {
      Serial_Debug("\n ** Zigbee Write Config Set - Error ");
      return false;
    }
    else
    {
      Serial_Debug("\nZigbee Write Config Set - OK "); 
    }
  }
  ResetZigbeeNetworkJoinedStatus();
  UpdateZigbeeConfigChangeFlag(false);
  return true;
}  
#endif

bool GetZigbeeParameters (stZigbeeConfigRead * zigbeeReadParams)
{
  zigbeeReadParams->AssocStatus[0] = '\0';
  zigbeeReadParams->configPanID[0] = '\0';
  zigbeeReadParams->encryptionOptions[0] = '\0';
  zigbeeReadParams->encryptionEnable[0] = '\0';
  zigbeeReadParams->joinVerifyEnable[0] = '\0';
  
#ifdef ZIGBEE_API_MODE_ENABLE
  
  uint64_t panID;
  if(GetConfiguredPanId(&panID) == false)
  {
    Serial_Debug("\nGetting Config Pan ID Failed");
    return false;
  }
  uint16_t maxStrLength = PAN_ID_STR_LEN;
  Int64ToHexString (panID, (char *)zigbeeReadParams->configPanID, &maxStrLength);
  
  uint8_t options;
  if(GetZigbeeAPIEncryptionOptions(&options) == false)
  {
    return false;
  }
  maxStrLength = NETWORK_OPTIONS_STR_LEN;
  Int64ToHexString (options, (char *)zigbeeReadParams->encryptionOptions, &maxStrLength);
  
  bool enable;
  if(GetZigbeeAPISecurityState(&enable) == false)
  {
    Serial_Debug("\nGetting Security Status Failed");
    return false;
  }
  maxStrLength = ENC_ENABLE_STR_LEN;
  zigbeeReadParams->encryptionEnable[0] = (uint8_t)enable + '0';
  zigbeeReadParams->encryptionEnable[1] = '\0';
  
  bool jvEnable;
  if(GetZigbeeAPIJoinVerifyEnableState(&jvEnable) == false)
  {
    Serial_Debug("\nGetting Join Verify Status Failed");
    return false;
  }
  maxStrLength = JV_ENABLE_STR_LEN;
  zigbeeReadParams->joinVerifyEnable[0] = (uint8_t)jvEnable + '0';
  zigbeeReadParams->joinVerifyEnable[1] = '\0';
  
  uint8_t assocStatus;
  if(GetAssociationStatus(&assocStatus) == false)
  {
    Serial_Debug("\nGetting Association Status Failed");
    return false;
  }
  maxStrLength = ASSOC_IND_STR_LEN;
  Int64ToHexString (assocStatus, (char *)zigbeeReadParams->AssocStatus, &maxStrLength);
  return true;
#else
  return true;
#endif
}

bool GetZigbeeNetworkParameters (stZigbeeNetworkParamRead * zigbeeNwReadParams)
{
  zigbeeNwReadParams->opPanID[0] = '\0';
  zigbeeNwReadParams->opPan16BitID[0] = '\0';
  zigbeeNwReadParams->opChannel[0] = '\0';
  zigbeeNwReadParams->stackProfile[0] = '\0';
  
#ifdef ZIGBEE_API_MODE_ENABLE
  
  uint64_t panID;
  if(GetOperatingPanId(&panID) == false)
  {
    Serial_Debug("\nGetting Op Pan ID Failed");
    return false;
  }
  uint16_t maxStrLength = PAN_ID_STR_LEN;
  Int64ToHexString (panID, (char *)zigbeeNwReadParams->opPanID, &maxStrLength);
  
  uint16_t opPanID16Bit;
  if(GetOperating16bitPanId(&opPanID16Bit) == false)
  {
    Serial_Debug("\nGetting Op Pan ID 16Bit Failed");
    return false;
  }
  maxStrLength = PAN_ID_16BIT_STR_LEN;
  Int64ToHexString (opPanID16Bit, (char *)zigbeeNwReadParams->opPan16BitID, &maxStrLength);
  
  uint8_t opChannel;
  if(GetOperatingChannel(&opChannel) == false)
  {
    return false;
  }
  maxStrLength = OP_CHANNEL_STR_LEN;
  Int64ToHexString (opChannel, (char *)zigbeeNwReadParams->opChannel, &maxStrLength);
  
  uint8_t stackProfile;
  if(GetStackProfile(&stackProfile) == false)
  {
    return false;
  }
  maxStrLength = STACK_PROFILE_STR_LEN;
  Int64ToHexString (stackProfile, (char *)zigbeeNwReadParams->stackProfile, &maxStrLength);
  
  return true;
#else
  return true;
#endif
}