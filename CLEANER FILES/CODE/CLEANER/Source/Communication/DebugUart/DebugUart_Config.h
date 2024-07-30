/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DEBUGUART_CONFIG_H_
#define __DEBUGUART_CONFIG_H_

/* Includes ------------------------------------------------------------------*/
#include "board.h"

#define ENABLE_DEBUG_RESET                     /* Enable Debug Reset Pin */

#define MAX_DEBUG_UART_TX_BUFFER            1500
#define MAX_DEBUG_UART_RX_BUFFER            500

#define DEBUG_UART_RxStartChar              0x01        
#define DEBUG_UART_RxEndChar                0x04    

#define DEBUG_UART_BASE                      UART1
#define DEBUG_UART_CLKSRC                    kCLOCK_CoreSysClk //kCLOCK_BusClk
#define DEBUG_UART_CLK_FREQ                  CLOCK_GetFreq(DEBUG_UART_CLKSRC)
#define DEBUG_UART_IRQn                      UART1_RX_TX_IRQn
#define DEBUG_UART_IRQHandler                UART1_RX_TX_IRQHandler

#define DEBUG_UART_BAUDRATE                  115200

#endif