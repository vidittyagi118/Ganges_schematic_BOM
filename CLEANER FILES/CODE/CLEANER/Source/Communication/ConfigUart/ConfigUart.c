#include "fsl_uart.h"
#include "ConfigUart.h"
#include "Serial_Debug.h"
#include "switch.h"

#include <string.h>

typedef struct {
  uint8_t txBuffer[MAX_CONFIG_UART_TX_BUFFER];
  volatile uint16_t txBufferLength;
  volatile uint16_t txBufferIndex;
  volatile ITStatus txCompleteFlag;  
} sUartTxdef;

typedef struct {
  uint8_t rxBuffer[MAX_CONFIG_UART_RX_BUFFER];
  uint16_t rxBufferLength;
  uint16_t Rxdata_Count_End;
  volatile ITStatus rxCompleteFlag;
  volatile ITStatus rxStartFlag;
} sUartRxdef;


static sUartTxdef configUartTx;
static sUartRxdef configUartRx;

static void EnableConfigUartIT (void);
static void DisableConfigUartIT (void);

ErrorStatus ConfigUartInit (void)
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
  config.baudRate_Bps = CONFIG_UART_BAUDRATE;
  config.enableTx = true;
  config.enableRx = true;
  
  if(UART_Init(CONFIG_UART_BASE, &config, CONFIG_UART_CLK_FREQ) != kStatus_Success)
  {
    return ERROR;
  }
  
  configUartTx.txCompleteFlag = SET;
  configUartRx.rxCompleteFlag = RESET;
  UART_EnableInterrupts(CONFIG_UART_BASE, kUART_RxDataRegFullInterruptEnable | kUART_RxOverrunInterruptEnable ); 
  EnableConfigUartIT();
  return SUCCESS;
}

ErrorStatus ConfigUartDeInit (void)
{
 UART_Deinit(CONFIG_UART_BASE);
 return SUCCESS;  
}

ErrorStatus ConfigUartTransmit (uint8_t * txBuffer, uint16_t txBufferLength)
{
  if(!txBufferLength)
  {
    return SUCCESS;
  }
  while(configUartTx.txCompleteFlag == RESET);                                        /* Wait for Previous Transmission Completion */
  if((sizeof(configUartTx.txBuffer)/sizeof(configUartTx.txBuffer[0])) >= txBufferLength)
  {
    memcpy(configUartTx.txBuffer, txBuffer, txBufferLength);
    configUartTx.txBufferLength = txBufferLength;
    configUartTx.txBufferIndex = 0;
    configUartTx.txCompleteFlag = RESET;
    UART_EnableInterrupts(CONFIG_UART_BASE, kUART_TxDataRegEmptyInterruptEnable);   
  }
  else
  {
    return ERROR;
  }
  return SUCCESS;
}

ErrorStatus ConfigUartReceive (uint8_t * rxBuffer, uint16_t rxBufferLength)
{
  ErrorStatus status;
  if(configUartRx.rxCompleteFlag == RESET)
  {
    status = ERROR;
  }
  else
  {
    DisableConfigUartIT();
    if(configUartRx.rxCompleteFlag == RESET)
    {
      status = ERROR;
    }
    else
    {
      if(configUartRx.Rxdata_Count_End <= rxBufferLength)
      {
        memcpy(rxBuffer, configUartRx.rxBuffer, configUartRx.Rxdata_Count_End);
        status = SUCCESS;
      }
      else
      {
        status = ERROR;
      }      
      configUartRx.rxCompleteFlag = RESET;
    }
   EnableConfigUartIT();
  }
  return status; 
}

void ConfigRxDataReceive (uint8_t * rxBuffer, uint16_t rxBufferLength)
{
    for(uint16_t rxDataCount = 0; rxDataCount < rxBufferLength; rxDataCount++)
    {
      CombineConfigUartReceiveData(rxBuffer[rxDataCount]);
    }  
}

void CONFIG_UART_IRQHandler(void)
{
  /* If new data arrived. */
  if (((kUART_RxDataRegFullFlag | kUART_RxOverrunFlag) & UART_GetStatusFlags(CONFIG_UART_BASE)) &&
    (UART_GetEnabledInterrupts(CONFIG_UART_BASE) & kUART_RxDataRegFullInterruptEnable))
  {
      uint8_t data = UART_ReadByte(CONFIG_UART_BASE);
      ConfigUartReceive_Callback(data);
  }
  if((kUART_TxDataRegEmptyFlag & UART_GetStatusFlags(CONFIG_UART_BASE)) && 
     (kUART_TxDataRegEmptyInterruptEnable & UART_GetEnabledInterrupts(CONFIG_UART_BASE)))
  {
    if(configUartTx.txBufferLength)
    {
      UART_WriteByte(CONFIG_UART_BASE, configUartTx.txBuffer[configUartTx.txBufferIndex++]);
      configUartTx.txBufferLength--;
      if(!configUartTx.txBufferLength)
      {
        UART_DisableInterrupts(CONFIG_UART_BASE,kUART_TxDataRegEmptyInterruptEnable);
        ConfigUartTransmit_Callback();
      }
    }
    else
    {
      UART_DisableInterrupts(CONFIG_UART_BASE,kUART_TxDataRegEmptyInterruptEnable);
      ConfigUartTransmit_Callback();
    }
  }
}

void ConfigUartError_Callback(void)
{
  /* Handle Transfer Error */
//  (void)HAL_UART_EnableReceive(&ConfigUartHandle);                      /* Enable reception that was disabled on error by the hal routine.  */
}

void ConfigUartTransmit_Callback(void)
{
  /* Set transmission flag: trasfer complete*/
  configUartTx.txCompleteFlag = SET;
}

void ConfigUartReceive_Callback(uint8_t rxData)
{
  if((GetSwitchState()==MANUAL_CONTROL)||(GetSwitchState() == AUTO_CONTROL )) 
  {
  CombineConfigUartReceiveData(rxData);
  }
}

static void CombineConfigUartReceiveData (uint8_t rxData)
{
  if((GetSwitchState()==MANUAL_CONTROL))  
  {
    if(rxData == CONFIG_UART_RxStartChar)
    {
      configUartRx.rxBufferLength = 0;
      configUartRx.rxStartFlag = SET;
      configUartRx.rxCompleteFlag = RESET;
      configUartRx.rxBuffer[configUartRx.rxBufferLength++] = rxData;
    }
    else if(rxData == CONFIG_UART_RxEndChar)
    {
      configUartRx.rxBuffer[configUartRx.rxBufferLength++] = rxData;
      configUartRx.rxBuffer[configUartRx.rxBufferLength++] = 0x00;           /* Just add null character at the end     */
      configUartRx.Rxdata_Count_End = configUartRx.rxBufferLength;
      configUartRx.rxCompleteFlag = SET;
      configUartRx.rxBufferLength = 0;
      configUartRx.rxStartFlag = RESET;
    }
    else if(configUartRx.rxStartFlag == SET)
    {
      if(configUartRx.rxBufferLength <  MAX_CONFIG_UART_RX_BUFFER-1)
      {
        configUartRx.rxBuffer[configUartRx.rxBufferLength++] = rxData;  
      }
      else
      {
        configUartRx.rxBufferLength = 0;
        configUartRx.rxStartFlag = RESET;
      }   
    }
  }
  else if(GetSwitchState() == AUTO_CONTROL)
  {
    if(rxData == SlaveADD)
    {
      configUartRx.rxBufferLength = 0;
      configUartRx.rxStartFlag = SET;
      configUartRx.rxCompleteFlag = RESET;
      configUartRx.rxBuffer[configUartRx.rxBufferLength++] = rxData;  
    
    }
    else if(rxData == Slaveend)
    {
      configUartRx.rxBuffer[configUartRx.rxBufferLength++] = rxData;
      configUartRx.rxBuffer[configUartRx.rxBufferLength++] = 0x00;           /* Just add null character at the end     */
      configUartRx.Rxdata_Count_End = configUartRx.rxBufferLength;
      configUartRx.rxCompleteFlag = SET;
      configUartRx.rxBufferLength = 0;
      configUartRx.rxStartFlag = RESET;
    }
    else if(configUartRx.rxStartFlag == SET)
    {
      if(configUartRx.rxBufferLength <  MAX_CONFIG_UART_RX_BUFFER-1)
      {
        configUartRx.rxBuffer[configUartRx.rxBufferLength++] = rxData;  
      }
      else
      {
        configUartRx.rxBufferLength = 0;
        configUartRx.rxStartFlag = RESET;
      } 
    }
  } 
}  



ITStatus * GetConfigUartTxCompleteFlagPtr (void)
{
  return ((ITStatus *)&configUartTx.txCompleteFlag);
}

static void DisableConfigUartIT(void)
{
    DisableIRQ(CONFIG_UART_IRQn);
}

static void EnableConfigUartIT(void)
{
    EnableIRQ(CONFIG_UART_IRQn);
}

