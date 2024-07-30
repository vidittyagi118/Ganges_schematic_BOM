/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CONFIGUART_CONFIG_H_
#define __CONFIGUART_CONFIG_H_

/* Includes ------------------------------------------------------------------*/
#include "board.h"

#define ENABLE_CONFIG_RESET                     /* Enable Debug Reset Pin */

#define MAX_CONFIG_UART_TX_BUFFER            10000           //1500
#define MAX_CONFIG_UART_RX_BUFFER            500

#define CONFIG_UART_RxStartChar              '{'      
#define CONFIG_UART_RxEndChar                '}'   

#define SlaveADD      ':'
#define Slaveend      '\n'

#define CONFIG_UART_BASE                      UART5
#define CONFIG_UART_CLKSRC                    BUS_CLK //kCLOCK_BusClk
#define CONFIG_UART_CLK_FREQ                  CLOCK_GetFreq(CONFIG_UART_CLKSRC)
#define CONFIG_UART_IRQn                      UART5_RX_TX_IRQn
#define CONFIG_UART_IRQHandler                UART5_RX_TX_IRQHandler

#define CONFIG_UART_BAUDRATE                  9600

#endif