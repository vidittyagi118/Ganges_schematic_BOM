#include "fsl_uart.h"
#include "DebugUart.h"
#include "Serial_Debug.h"

#include <string.h>

typedef struct {
  uint8_t txBuffer[MAX_DEBUG_UART_TX_BUFFER];
  volatile uint16_t txBufferLength;
  volatile uint16_t txBufferIndex;
  volatile ITStatus txCompleteFlag;  
} sUartTxdef;

typedef struct {
  uint8_t rxBuffer[MAX_DEBUG_UART_RX_BUFFER];
  uint16_t rxBufferLength;
  uint16_t Rxdata_Count_End;
  volatile ITStatus rxCompleteFlag;
  volatile ITStatus rxStartFlag;
} sUartRxdef;


static sUartTxdef debugUartTx;
static sUartRxdef debugUartRx;

static void EnableDebugUartIT (void);
static void DisableDebugUartIT (void);

ErrorStatus DebugUartInit (void)
{
  uart_config_t config;
  /*
 * config.baudRate_Bps = 115200U;
 * config.parityMode = kUART_ParityDisabled;
 * config.stopBitCount = kUART_OneStopBit;
 * config.txFifoWatermark = 0;
 * config.rxFifoWatermark = 1;
 * config.enableTx = false;
 * config.enableRx = false;
 */
  UART_GetDefaultConfig(&config);
  config.baudRate_Bps = DEBUG_UART_BAUDRATE;
  config.enableTx = true;
  config.enableRx = true;
  
  if(UART_Init(DEBUG_UART_BASE, &config, DEBUG_UART_CLK_FREQ) != kStatus_Success)
  {
    return ERROR;
  }
  
  debugUartTx.txCompleteFlag = SET;
  debugUartRx.rxCompleteFlag = RESET;
  UART_EnableInterrupts(DEBUG_UART_BASE, kUART_RxDataRegFullInterruptEnable | kUART_RxOverrunInterruptEnable ); 
  EnableDebugUartIT();
  return SUCCESS;
}

ErrorStatus DebugUartDeInit (void)
{
 UART_Deinit(DEBUG_UART_BASE);
 return SUCCESS;  
}

ErrorStatus DebugUartTransmit (uint8_t * txBuffer, uint16_t txBufferLength)
{
  if(!txBufferLength)
  {
    return SUCCESS;
  }
  while(debugUartTx.txCompleteFlag == RESET);                                        /* Wait for Previous Transmission Completion */
  if((sizeof(debugUartTx.txBuffer)/sizeof(debugUartTx.txBuffer[0])) >= txBufferLength)
  {
    memcpy(debugUartTx.txBuffer, txBuffer, txBufferLength);
    debugUartTx.txBufferLength = txBufferLength;
    debugUartTx.txBufferIndex = 0;
    debugUartTx.txCompleteFlag = RESET;
    UART_EnableInterrupts(DEBUG_UART_BASE, kUART_TxDataRegEmptyInterruptEnable);   
  }
  else
  {
    return ERROR;
  }
  return SUCCESS;
}

ErrorStatus DebugUartReceive (uint8_t * rxBuffer, uint16_t rxBufferLength)
{
  ErrorStatus status;
  if(debugUartRx.rxCompleteFlag == RESET)
  {
    status = ERROR;
  }
  else
  {
    DisableDebugUartIT();
    if(debugUartRx.rxCompleteFlag == RESET)
    {
      status = ERROR;
    }
    else
    {
      if(debugUartRx.Rxdata_Count_End <= rxBufferLength)
      {
        memcpy(rxBuffer, debugUartRx.rxBuffer, debugUartRx.Rxdata_Count_End);
        status = SUCCESS;
      }
      else
      {
        status = ERROR;
      }      
      debugUartRx.rxCompleteFlag = RESET;
    }
   EnableDebugUartIT();
  }
  return status; 
}

void DebugRxDataReceive (uint8_t * rxBuffer, uint16_t rxBufferLength)
{
    for(uint16_t rxDataCount = 0; rxDataCount < rxBufferLength; rxDataCount++)
    {
      CombineDebugUartReceiveData(rxBuffer[rxDataCount]);
    }  
}

void DEBUG_UART_IRQHandler(void)
{
  /* If new data arrived. */
  if (((kUART_RxDataRegFullFlag | kUART_RxOverrunFlag) & UART_GetStatusFlags(DEBUG_UART_BASE)) &&
    (UART_GetEnabledInterrupts(DEBUG_UART_BASE) & kUART_RxDataRegFullInterruptEnable))
  {
      uint8_t data = UART_ReadByte(DEBUG_UART_BASE);
      DebugUartReceive_Callback(data);
  }
  if((kUART_TxDataRegEmptyFlag & UART_GetStatusFlags(DEBUG_UART_BASE)) && 
     (kUART_TxDataRegEmptyInterruptEnable & UART_GetEnabledInterrupts(DEBUG_UART_BASE)))
  {
    if(debugUartTx.txBufferLength)
    {
      UART_WriteByte(DEBUG_UART_BASE, debugUartTx.txBuffer[debugUartTx.txBufferIndex++]);
      debugUartTx.txBufferLength--;
      if(!debugUartTx.txBufferLength)
      {
        UART_DisableInterrupts(DEBUG_UART_BASE,kUART_TxDataRegEmptyInterruptEnable);
        DebugUartTransmit_Callback();
      }
    }
    else
    {
      UART_DisableInterrupts(DEBUG_UART_BASE,kUART_TxDataRegEmptyInterruptEnable);
      DebugUartTransmit_Callback();
    }
  }
}

void DebugUartError_Callback(void)
{
  /* Handle Transfer Error */
//  (void)HAL_UART_EnableReceive(&DebugUartHandle);                      /* Enable reception that was disabled on error by the hal routine.  */
}

void DebugUartTransmit_Callback(void)
{
  /* Set transmission flag: trasfer complete*/
  debugUartTx.txCompleteFlag = SET;
}

void DebugUartReceive_Callback(uint8_t rxData)
{
  CombineDebugUartReceiveData(rxData);
}

static void CombineDebugUartReceiveData (uint8_t rxData)
{
  if(rxData == DEBUG_UART_RxStartChar)
  {
    debugUartRx.rxBufferLength = 0;
    debugUartRx.rxStartFlag = SET;
    debugUartRx.rxCompleteFlag = RESET;
    debugUartRx.rxBuffer[debugUartRx.rxBufferLength++] = rxData;
  }
  else if(rxData == DEBUG_UART_RxEndChar)
  {
    debugUartRx.rxBuffer[debugUartRx.rxBufferLength++] = rxData;
    debugUartRx.rxBuffer[debugUartRx.rxBufferLength++] = 0x00;           /* Just add null character at the end     */
    debugUartRx.Rxdata_Count_End = debugUartRx.rxBufferLength;
    debugUartRx.rxCompleteFlag = SET;
    debugUartRx.rxBufferLength = 0;
    debugUartRx.rxStartFlag = RESET;
  }
  else if(debugUartRx.rxStartFlag == SET)
  {
    if(debugUartRx.rxBufferLength <  MAX_DEBUG_UART_RX_BUFFER-1)
    {
      debugUartRx.rxBuffer[debugUartRx.rxBufferLength++] = rxData;
    }
    else
    {
      debugUartRx.rxBufferLength = 0;
      debugUartRx.rxStartFlag = RESET;
    }
  }  
}


ITStatus * GetDebugUartTxCompleteFlagPtr (void)
{
  return ((ITStatus *)&debugUartTx.txCompleteFlag);
}

static void DisableDebugUartIT(void)
{
    DisableIRQ(DEBUG_UART_IRQn);
}

static void EnableDebugUartIT(void)
{
    EnableIRQ(DEBUG_UART_IRQn);
}

