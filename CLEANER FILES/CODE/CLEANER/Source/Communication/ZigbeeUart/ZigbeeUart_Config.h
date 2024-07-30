/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ZIGBEEUART_CONFIG_H_
#define __ZIGBEEUART_CONFIG_H_

/* Includes ------------------------------------------------------------------*/
#include "board.h"

#define ENABLE_ZIGBEE_RESET                     /* Enable Zigbee Reset Pin */

#define ZIGBEE_API_MODE_ENABLE                  /* Put Zigbee in API Mode */

//#define ZIGBEE_JSON_MODE

#ifdef ZIGBEE_API_MODE_ENABLE
#define MAX_ZIGBEE_UART_TX_BUFFER            500
#define MAX_ZIGBEE_UART_RX_BUFFER            500
#else
#define MAX_ZIGBEE_UART_TX_BUFFER            1500
#define MAX_ZIGBEE_UART_RX_BUFFER            500
#endif

#ifdef ZIGBEE_JSON_MODE
#define ZIGBEE_UART_RxStartChar              '{'        
#define ZIGBEE_UART_RxEndChar                '}'   
#else
#define ZIGBEE_UART_RxStartChar              0x01        
#define ZIGBEE_UART_RxEndChar                0x04   
#endif 

#define ZIGBEE_UART_BASE                      UART2
#define ZIGBEE_UART_CLKSRC                    kCLOCK_BusClk
#define ZIGBEE_UART_CLK_FREQ                  CLOCK_GetFreq(kCLOCK_BusClk)
#define ZIGBEE_UART_IRQn                      UART2_RX_TX_IRQn
#define ZIGBEE_UART_IRQHandler                UART2_RX_TX_IRQHandler

#define ZIGBEE_UART_BAUDRATE                 9600

#ifdef ENABLE_ZIGBEE_RESET
  #define ZIGBEE_RESET_GPIO                BOARD_ZIGBEE_RESET_GPIO                                   
  #define ZIGBEE_RESET_PIN                 BOARD_ZIGBEE_RESET_PIN                            
#endif
#endif
