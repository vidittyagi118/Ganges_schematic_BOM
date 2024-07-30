#include "CommCommon.h"
#include "ZigbeeAPIMode.h"
#include "DebugUart.h"
#include "ConfigUart.h"
#include "ConfigAPIProcess.h"
#include "APIProcessing.h"
//#include "FaultProcess.h"

static volatile bool commTimeOver     = false;
static int32_t comm_timecount         = 0;
static int32_t commTimeoutCount       = 0;
static eUartType uartType = UNKNOWN_UART;
static stHeartbeatConfig heartbeatConfigValues;

ErrorStatus UartTransmit (uint8_t * txBuffer, uint16_t txBufferLength)
{
  ErrorStatus status = ERROR;
  if(GetUartType() == ZIGBEE_UART)
  {
#ifdef ZIGBEE_JSON_MODE 
    ConvertHexAndProcess(txBuffer, &txBufferLength);
#endif
    status =ZigbeeApiTxData(GetReceivedMACAddress(),txBuffer, txBufferLength);
  }
  else if(GetUartType() == DEBUG_UART)
  {
    status = DebugUartTransmit(txBuffer, txBufferLength); 
  }
  else if(GetUartType() == CONFIG_UART)
  {
    ConvertHexAndProcess(txBuffer, &txBufferLength);
    status = ConfigUartTransmit(txBuffer, txBufferLength); 
  }
  return status;
}

ErrorStatus UartTransmitType (eUartType uartType, uint8_t * txBuffer, uint16_t txBufferLength)
{
  ErrorStatus status = ERROR;
  if(uartType == ZIGBEE_UART)
  {
    #ifdef ZIGBEE_JSON_MODE 
    ConvertHexAndProcess(txBuffer, &txBufferLength);
    #endif
    status = ZigbeeApiTxData(GetReceivedMACAddress(),txBuffer, txBufferLength);
  }
  else if(uartType == DEBUG_UART)
  {
    status = DebugUartTransmit(txBuffer, txBufferLength); 
  }
  else if(uartType == CONFIG_UART)
  {
   // ConvertHexAndProcess(txBuffer, &txBufferLength);
    status = ConfigUartTransmit(txBuffer, txBufferLength); 
  }
  return status;
}

ErrorStatus UartAllTransmit (uint8_t * txBuffer, uint16_t txBufferLength)
{
  bool status = true;
  if(ZigbeeApiTxData(GetReceivedMACAddress(),txBuffer, txBufferLength) == ERROR)
  {
    status = false;
  }
  if(DebugUartTransmit(txBuffer, txBufferLength) == ERROR)
  {
    status = false;
  }
  if(ConfigUartTransmit(txBuffer, txBufferLength) == ERROR)
  {
    status = false;
  }
  if(status == true)
  {
    return SUCCESS;
  }
  else
  {
    return ERROR;
  }
}

eUartType GetUartType (void)
{
  return uartType;
}

void SetUartType (eUartType uartRxType)
{
  uartType = uartRxType;
}

inline void CommTimerStop (void)
{
  comm_timecount = 0;
}

bool IsCommTimeOver (void)
{
  stHeartbeatConfig *heartBeatconfig = GetSetHeartbeatConfig();
  if(heartBeatconfig->enable == true)
  {
    return (commTimeOver);
  }
  else
  {
    return (false);
  }
}

void ClearCommTimeOver (void)
{
  commTimeOver = false; 
}

static void CommTimerOn (uint32_t setCommTimeCount_ms)
{
  if(setCommTimeCount_ms == 0)
  {
    commTimeOver = true;
    CommTimerStop();
  }
  else
  {
    comm_timecount = setCommTimeCount_ms;
    commTimeOver = false;
  }
}

void CommTimeIncrement_ms (void)
{
  if(comm_timecount)
  {
    if(--comm_timecount <= 0)
    {
      if(--commTimeoutCount <= 0)
      {
        commTimeOver = true;
        CommTimerStop();
      }
      else
      {
        CommTimerOn(heartbeatConfigValues.interval_ms);
      }
    }
    else
    {
      commTimeOver = false;
    }
  }
  else
  {
    commTimeOver = true;
  }
}

void RestartCommTimer (void)
{
  commTimeOver = false;
  CommTimerStop();
  commTimeoutCount = heartbeatConfigValues.noOfMessages;
  CommTimerOn(heartbeatConfigValues.interval_ms);
  //ClearCommunicationFaultFlag();
}

stHeartbeatConfig *GetSetHeartbeatConfig(void)
{
  return(&heartbeatConfigValues);  
}




