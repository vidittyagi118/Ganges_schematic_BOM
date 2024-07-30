#include "fsl_uart.h"
#include "ZigbeeUart.h"
#include "ZigbeeAPIMode.h"
#include "Serial_Debug.h"
#include "board.h"
#include "switch.h"

#include <string.h>


typedef struct {
  uint8_t txBuffer[MAX_ZIGBEE_UART_TX_BUFFER];
  uint16_t txBufferLength;
  uint16_t txBufferIndex;
  volatile ITStatus txCompleteFlag;  
} sUartTxdef;

typedef struct {
  uint8_t rxBuffer[MAX_ZIGBEE_UART_RX_BUFFER];
  uint16_t rxBufferLength;
  uint16_t Rxdata_Count_End;
  volatile ITStatus rxCompleteFlag;
  volatile ITStatus rxStartFlag;
} sUartRxdef;


static sUartTxdef zigbeeUartTx;
static sUartRxdef zigbeeUartRx;
static uint64_t zigbeeUartRxMacAddress = DEFAULT_RX_MAC_ADDR;
static volatile bool zigbeeRxDataReadInProgress = false;

ErrorStatus ZigbeeUartInit (void)
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
  config.baudRate_Bps = ZIGBEE_UART_BAUDRATE;
  config.enableTx = true;
  config.enableRx = true;
  
  if(UART_Init(ZIGBEE_UART_BASE, &config, ZIGBEE_UART_CLK_FREQ) != kStatus_Success)
  {
    return ERROR;
  }
  
#ifdef ENABLE_ZIGBEE_RESET
  ZigbeeResetInit();
#endif
  
  zigbeeUartTx.txCompleteFlag = SET;
  zigbeeUartRx.rxCompleteFlag = RESET;
  UART_EnableInterrupts(ZIGBEE_UART_BASE, kUART_RxDataRegFullInterruptEnable | kUART_RxOverrunInterruptEnable ); 
  EnableZigbeeUartIT();

  
#ifdef ZIGBEE_API_MODE_ENABLE
  if(ZigbeeAPIModeInit() == false)
  {
    return ERROR;
  }
#endif
 return SUCCESS;
}

ErrorStatus ZigbeeUartDeInit (void)
{
 UART_Deinit(ZIGBEE_UART_BASE);
 return SUCCESS;  
}

ErrorStatus ZigbeeUartTransmit (uint8_t * txBuffer, uint16_t txBufferLength)
{
  if(!txBufferLength)
  {
    return SUCCESS;
  }
  while(zigbeeUartTx.txCompleteFlag == RESET);                                        /* Wait for Previous Transmission Completion */
  if((sizeof(zigbeeUartTx.txBuffer)/sizeof(zigbeeUartTx.txBuffer[0])) >= txBufferLength)
  {
    memcpy(zigbeeUartTx.txBuffer, txBuffer, txBufferLength);
    zigbeeUartTx.txBufferLength = txBufferLength;
    zigbeeUartTx.txBufferIndex = 0;
    zigbeeUartTx.txCompleteFlag = RESET;
    UART_EnableInterrupts(ZIGBEE_UART_BASE, kUART_TxDataRegEmptyInterruptEnable);   
  }
  else
  {
    return ERROR;
  }
  return SUCCESS;
}

ErrorStatus ZigbeeUartReceive (uint64_t *rxMacAddr, uint8_t * rxBuffer, uint16_t rxBufferLength)
{
  ErrorStatus status;
  if(zigbeeUartRx.rxCompleteFlag == RESET)
  {
    status = ERROR;
  }
  else
  {
#ifdef ZIGBEE_API_MODE_ENABLE
    zigbeeRxDataReadInProgress = true;
#else
    DisableZigbeeUartIT();
#endif
    if(zigbeeUartRx.rxCompleteFlag == RESET)
    {
      status = ERROR;
    }
    else
    {
      if(zigbeeUartRx.Rxdata_Count_End <= rxBufferLength)
      {
        memcpy(rxBuffer, zigbeeUartRx.rxBuffer, zigbeeUartRx.Rxdata_Count_End);
        *rxMacAddr = GetZigbeeUartRxMacAddr();
        status = SUCCESS;
      }
      else
      {
        status = ERROR;
      }      
      zigbeeUartRx.rxCompleteFlag = RESET;
    }
#ifdef ZIGBEE_API_MODE_ENABLE
    zigbeeRxDataReadInProgress = false;
#else
   EnableZigbeeUartIT();
#endif
    
  }
  return status; 
}

#ifdef ZIGBEE_API_MODE_ENABLE
void ZigbeeRxDataReceive (uint64_t rxMacAddr, uint8_t * rxBuffer, uint16_t rxBufferLength)
{
  static uint64_t zigbeeRxMacAddr = DEFAULT_RX_MAC_ADDR;
  if(zigbeeRxDataReadInProgress == false)
  {
    if(zigbeeRxMacAddr != rxMacAddr)
    {
      zigbeeRxMacAddr = rxMacAddr;
      zigbeeUartRx.rxStartFlag = RESET;                                                 /* Restart Reception If data Received From New Module */
    }
    for(uint16_t rxDataCount = 0; rxDataCount < rxBufferLength; rxDataCount++)
    {
      if(GetSwitchState()==AUTO_CONTROL)
      {
      CombineZigbeeUartReceiveData(zigbeeRxMacAddr, rxBuffer[rxDataCount]);
      }
    }  
  }
}
#endif

void ZIGBEE_UART_IRQHandler(void)
{
  /* If new data arrived. */
  if (((kUART_RxDataRegFullFlag | kUART_RxOverrunFlag) & UART_GetStatusFlags(ZIGBEE_UART_BASE)) &&
    (UART_GetEnabledInterrupts(ZIGBEE_UART_BASE) & kUART_RxDataRegFullInterruptEnable))
  {
      uint8_t data = UART_ReadByte(ZIGBEE_UART_BASE);
      ZigbeeUartReceive_Callback(data);
  }
  if((kUART_TxDataRegEmptyFlag & UART_GetStatusFlags(ZIGBEE_UART_BASE)) && 
     (kUART_TxDataRegEmptyInterruptEnable & UART_GetEnabledInterrupts(ZIGBEE_UART_BASE)))
  {
    if(zigbeeUartTx.txBufferLength)
    {
      UART_WriteByte(ZIGBEE_UART_BASE, zigbeeUartTx.txBuffer[zigbeeUartTx.txBufferIndex++]);
      zigbeeUartTx.txBufferLength--;
      if(!zigbeeUartTx.txBufferLength)
      {
        UART_DisableInterrupts(ZIGBEE_UART_BASE,kUART_TxDataRegEmptyInterruptEnable);
        ZigbeeUartTransmit_Callback();
      }
    }
    else
    {
      UART_DisableInterrupts(ZIGBEE_UART_BASE,kUART_TxDataRegEmptyInterruptEnable);
      ZigbeeUartTransmit_Callback();
    }
  }
}

/**
* @brief  Transmission Error callback
* @param  None
* @note   This shows a simple way to report Uart error.
* @retval None
*/
void ZigbeeUartError_Callback(void)
{
  /* Handle Transfer Error */
//  (void)HAL_UART_EnableReceive(&ZigbeeUartHandle);                      /* Enable reception that was disabled on error by the hal routine.  */
}

/**
* @brief  Tx Transfer completed callback
* @param  ZigbeeUartHandle: UART handle. 
* @note   This shows a simple way to report end of DMA Tx transfer.
* @retval None
*/
void ZigbeeUartTransmit_Callback(void)
{
  /* Set transmission flag: trasfer complete*/
  zigbeeUartTx.txCompleteFlag = SET;
}

/**
* @brief  Rx Transfer completed callback
* @param  ZigbeeUartHandle: UART handle
* @note   This shows a simple way to report end of DMA Rx transfer.
* @retval None
*/
void ZigbeeUartReceive_Callback(uint8_t rxData)
{
#ifdef ZIGBEE_API_MODE_ENABLE
  ZigbeeUartRx(rxData);
#else
  CombineZigbeeUartReceiveData(DEFAULT_RX_MAC_ADDR, rxData);
#endif
}

static void CombineZigbeeUartReceiveData (uint64_t rxMacAddr, uint8_t rxData)
{
  if(rxData == ZIGBEE_UART_RxStartChar)
  {
    zigbeeUartRx.rxBufferLength = 0;
    zigbeeUartRx.rxStartFlag = SET;
    zigbeeUartRx.rxCompleteFlag = RESET;
    zigbeeUartRx.rxBuffer[zigbeeUartRx.rxBufferLength++] = rxData;
  }
  else if(zigbeeUartRx.rxStartFlag == SET)
  {
    if(rxData == ZIGBEE_UART_RxEndChar)
    {
      zigbeeUartRx.rxBuffer[zigbeeUartRx.rxBufferLength++] = rxData;
      zigbeeUartRx.rxBuffer[zigbeeUartRx.rxBufferLength++] = 0x00;           /* Just add null character at the end     */
      zigbeeUartRx.Rxdata_Count_End = zigbeeUartRx.rxBufferLength;
      zigbeeUartRx.rxCompleteFlag = SET;
      zigbeeUartRx.rxBufferLength = 0;
      zigbeeUartRx.rxStartFlag = RESET;
      zigbeeUartRxMacAddress = rxMacAddr;
    }
    else if(zigbeeUartRx.rxBufferLength <  MAX_ZIGBEE_UART_RX_BUFFER-1)
    {
      zigbeeUartRx.rxBuffer[zigbeeUartRx.rxBufferLength++] = rxData;
    }
    else
    {
      zigbeeUartRx.rxBufferLength = 0;
      zigbeeUartRx.rxStartFlag = RESET;
    }
  }  
}

uint64_t GetZigbeeUartRxMacAddr (void)
{
  return zigbeeUartRxMacAddress;
}


ITStatus * GetZigbeeUartTxCompleteFlagPtr (void)
{
  return ((ITStatus *)&zigbeeUartTx.txCompleteFlag);
}

void DisableZigbeeUartIT(void)
{
    DisableIRQ(ZIGBEE_UART_IRQn);
}

void EnableZigbeeUartIT(void)
{
    EnableIRQ(ZIGBEE_UART_IRQn);
}

#ifdef ENABLE_ZIGBEE_RESET

void ZigbeeResetInit(void)
{
  gpio_pin_config_t out_config = {
    kGPIO_DigitalOutput, 1,
  };
  GPIO_PinInit(ZIGBEE_RESET_GPIO, ZIGBEE_RESET_PIN, &out_config);
  ZigbeeResetOff();
} 

void ZigbeeResetOn(void)
{
   GPIO_PortClear(ZIGBEE_RESET_GPIO, 1U << ZIGBEE_RESET_PIN);
}

void ZigbeeResetOff(void)
{
   GPIO_PortSet(ZIGBEE_RESET_GPIO, 1U << ZIGBEE_RESET_PIN); 
}

#endif
