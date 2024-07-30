/*
 * The Clear BSD License
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided
 * that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _BOARD_H_
#define _BOARD_H_

#include "clock_config.h"
#include "fsl_gpio.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*! @brief The board name */
//#define BOARD_NAME "TWR-K21F120M"

/*! @brief The UART to use for debug messages. */
#define BOARD_USE_UART
#define BOARD_DEBUG_UART_TYPE DEBUG_CONSOLE_DEVICE_TYPE_UART
#define BOARD_DEBUG_UART_BASEADDR (uint32_t) UART5
#define BOARD_DEBUG_UART_INSTANCE 5U
#define BOARD_DEBUG_UART_CLKSRC kCLOCK_BusClk
#define BOARD_DEBUG_UART_CLK_FREQ CLOCK_GetBusClkFreq()
#define BOARD_UART_IRQ UART5_RX_TX_IRQn
#define BOARD_UART_IRQ_HANDLER UART5_RX_TX_IRQHandler

#ifndef BOARD_DEBUG_UART_BAUDRATE
#define BOARD_DEBUG_UART_BAUDRATE 19200         //115200
#endif /* BOARD_DEBUG_UART_BAUDRATE */

/*! @brief The instances of peripherals used for board. */
#define BOARD_DAC_DEMO_DAC_BASEADDR DAC0
#define BOARD_DAC_DEMO_ADC_BASEADDR ADC0
#define BOARD_DAC_DEMO_ADC_CHANNEL 23U

/*! @brief The CAN instance used for board */
#define BOARD_CAN_BASEADDR CAN0

/*! @brief The i2c instance used for i2c connection by default */
#define BOARD_I2C_BASEADDR I2C1
#define BOARD_ACCEL_I2C_BASEADDR I2C1

/*! @brief The SDHC instance/channel used for board */
#define BOARD_SDHC_BASEADDR SDHC
#define BOARD_SDHC_CD_GPIO_IRQ_HANDLER PORTC_IRQHandler

/*! @brief The CMP instance/channel used for board. */
#define BOARD_CMP_BASEADDR CMP0
#define BOARD_CMP_CHANNEL 0U

/*! @brief The rtc instance used for rtc_func */
#define BOARD_RTC_FUNC_BASEADDR RTC

/*! @brief The i2c instance used for sai demo */
#define BOARD_SAI_DEMO_I2C_BASEADDR I2C1

/*! @brief Define the port interrupt number for the board switches */
#ifndef BOARD_SW2_GPIO
#define BOARD_SW2_GPIO GPIOC
#endif
#ifndef BOARD_SW2_PORT
#define BOARD_SW2_PORT PORTC
#endif
#ifndef BOARD_SW2_GPIO_PIN
#define BOARD_SW2_GPIO_PIN 7U
#endif
#define BOARD_SW2_IRQ PORTC_IRQn
#define BOARD_SW2_IRQ_HANDLER PORTC_IRQHandler
#define BOARD_SW2_NAME "SW2"

#ifndef BOARD_SW3_GPIO
#define BOARD_SW3_GPIO GPIOC
#endif
#ifndef BOARD_SW3_PORT
#define BOARD_SW3_PORT PORTC
#endif
#ifndef BOARD_SW3_GPIO_PIN
#define BOARD_SW3_GPIO_PIN 6U
#endif
#define BOARD_SW3_IRQ PORTC_IRQn
#define BOARD_SW3_IRQ_HANDLER PORTC_IRQHandler
#define BOARD_SW3_NAME "SW3"

/* Board Encoder mapping */
#define BOARD_ENCODER_GPIO GPIOA
#define BOARD_ENCODER_PORT PORTA
#define BOARD_ENCODER_MOTOR1_PIN 8U
#define BOARD_ENCODER_MOTOR2_PIN 9U
   
   /* Board Edgesensor mapping */

#define BOARD_EDGESENSOR1_GPIO GPIOA
#define BOARD_EDGESENSOR1_PORT PORTA
#define BOARD_EDGESENSOR1_GPIO_PIN 10U

#define BOARD_EDGESENSOR2_GPIO GPIOA
#define BOARD_EDGESENSOR2_PORT PORTA
#define BOARD_EDGESENSOR2_GPIO_PIN 11U

/* Board Relay mapping */

#define BOARD_RELAY_GPIO GPIOB
#define BOARD_RELAY_PORT PORTB
#define BOARD_RELAY_ON_GPIO_PIN 10U
#define BOARD_RELAY_OFF_GPIO_PIN 11U
   
/* Board led mapping */
#define BOARD_LED_GPIO GPIOB
#define BOARD_LED_PORT PORTB
#define BOARD_LED1_GPIO_PIN 16U
#define BOARD_LED2_GPIO_PIN 17U
#define BOARD_LED3_GPIO_PIN 20U
#define BOARD_LED4_GPIO_PIN 21U

/* Direct spi port pin method*/

#define BOARD_SD_GPIO GPIOC
#define BOARD_SD_PORT PORTC

#define BOARD_SD_CS_GPIO_PIN 4U
#define BOARD_SD_CLK_GPIO_PIN 5U
#define BOARD_SD_MOUT_GPIO_PIN 6U  //6
#define BOARD_SD_MIN_GPIO_PIN 7U  //7


/* Board motor mapping */
#define BOARD_MOTOR1_GPIO GPIOE
#define BOARD_MOTOR1_PORT PORTE
#define BOARD_MOTOR1_PWM_PIN_1 11U
#define BOARD_MOTOR1_PWM_PIN_2 12U

#define BOARD_MOTOR2_GPIO GPIOD
#define BOARD_MOTOR2_PORT PORTD
#define BOARD_MOTOR2_PWM_PIN_1 7U
#define BOARD_MOTOR2_PWM_PIN_2 6U

#define BOARD_BRUSHMOTOR_GPIO GPIOB
#define BOARD_BRUSHMOTOR_PORT PORTB
#define BOARD_BRUSHMOTOR_PWM_PIN_1 18U
#define BOARD_BRUSHMOTOR_PWM_PIN_2 19U

#define BOARD_MOTOR_ENABLE_GPIO GPIOD
#define BOARD_MOTOR_ENABLE_PORT PORTD
#define BOARD_MOTOR1_ENABLE_GPIO_PIN 15U
#define BOARD_MOTOR2_ENABLE_GPIO_PIN 14U
#define BOARD_BRUSHMOTOR_ENABLE_GPIO_PIN 13U


/* Zigbee Uart Pin Mapping */

#define BOARD_ZIGBEE_RESET_GPIO GPIOA
#define BOARD_ZIGBEE_RESET_PORT PORTA
#define BOARD_ZIGBEE_RESET_PIN 17U

#define BOARD_ZIGBEE_UART_GPIO GPIOD
#define BOARD_ZIGBEE_UART_PORT PORTD
#define BOARD_ZIGBEE_UART_TX_PIN 2U
#define BOARD_ZIGBEE_UART_RX_PIN 3U

/* Debug Uart Pin Mapping */

#define BOARD_DEBUG_UART_GPIO GPIOE
#define BOARD_DEBUG_UART_PORT PORTE
#define BOARD_DEBUG_UART_TX_PIN 0U
#define BOARD_DEBUG_UART_RX_PIN 1U

/* Config Uart Pin Mapping */

#define BOARD_CONFIG_UART_GPIO GPIOD
#define BOARD_CONFIG_UART_PORT PORTD
#define BOARD_CONFIG_UART_TX_PIN 9U
#define BOARD_CONFIG_UART_RX_PIN 8U
  
/* Board led color mapping */
#define LOGIC_LED_ON 1U
#define LOGIC_LED_OFF 0U
#ifndef BOARD_LED_ORANGE_GPIO
#define BOARD_LED_ORANGE_GPIO GPIOD
#endif
#define BOARD_LED_ORANGE_GPIO_PORT PORTD
#ifndef BOARD_LED_ORANGE_GPIO_PIN
#define BOARD_LED_ORANGE_GPIO_PIN 6U
#endif
#ifndef BOARD_LED_GREEN_GPIO
#define BOARD_LED_GREEN_GPIO GPIOD
#endif
#define BOARD_LED_GREEN_GPIO_PORT PORTD
#ifndef BOARD_LED_GREEN_GPIO_PIN
#define BOARD_LED_GREEN_GPIO_PIN 4U
#endif
#ifndef BOARD_LED_BLUE_GPIO
#define BOARD_LED_BLUE_GPIO GPIOD
#endif
#define BOARD_LED_BLUE_GPIO_PORT PORTD
#ifndef BOARD_LED_BLUE_GPIO_PIN
#define BOARD_LED_BLUE_GPIO_PIN 7U
#endif
#ifndef BOARD_LED_YELLOW_GPIO
#define BOARD_LED_YELLOW_GPIO GPIOD
#endif
#define BOARD_LED_YELLOW_GPIO_PORT PORTD
#ifndef BOARD_LED_YELLOW_GPIO_PIN
#define BOARD_LED_YELLOW_GPIO_PIN 5U
#endif

/*----------Battery Charger----------*/
//#define BOARD_BAT_I2C_BASEADDR I2C1   
//#define BAT_I2C_CLK_SRC I2C1_CLK_SRC
//#define BAT_I2C_CLK_FREQ CLOCK_GetFreq(I2C1_CLK_SRC)
//#define BAT_I2C_RELEASE_SDA_PORT PORTC
//#define BAT_I2C_RELEASE_SCL_PORT PORTC
//#define BAT_I2C_RELEASE_SDA_GPIO GPIOC
//#define BAT_I2C_RELEASE_SDA_PIN 11U
//#define BAT_I2C_RELEASE_SCL_GPIO GPIOC
//#define BAT_I2C_RELEASE_SCL_PIN 10U
//#define BAT_I2C_BAUDRATE 100000U

#define LTC4015_PORT PORTC
#define LTC4015_GPIO GPIOC
#define SMBALERT_PIN 9U

#define MANUAL_CONTROL_SWITCH_GPIO GPIOC
#define MANUAL_CONTROL_SWITCH_PORT PORTC
#define MANUAL_CONTROL_SWITCH_PIN 16U

   /* SPI Mapping */

#define BOARD_SPI0_PORT         PORTC
#define BOARD_SPI0_SCLK         5U
#define BOARD_SPI0_SDO          6U
#define BOARD_SPI0_SDI          7U

    
#define BOARD_CHIPSELECT_GPIO GPIOC
#define BOARD_CHIPSELECT_PORT PORTC
#define SPI_CHIPSELECT_PIN_0 0U
#define SPI_CHIPSELECT_PIN_1 1U
#define SPI_CHIPSELECT_PIN_2 2U
#define SPI_CHIPSELECT_PIN_3 3U
#define SPI_CHIPSELECT_PIN_4 4U
   
#define LED_GREEN_INIT(output)                                             \
    GPIO_PinWrite(BOARD_LED_GREEN_GPIO, BOARD_LED_GREEN_GPIO_PIN, output); \
    BOARD_LED_GREEN_GPIO->PDDR |= (1U << BOARD_LED_GREEN_GPIO_PIN) /*!< Enable target LED_GREEN */
#define LED_GREEN_ON() \
    GPIO_PortClear(BOARD_LED_GREEN_GPIO, 1U << BOARD_LED_GREEN_GPIO_PIN) /*!< Turn on target LED_GREEN */
#define LED_GREEN_OFF() \
    GPIO_PortSet(BOARD_LED_GREEN_GPIO, 1U << BOARD_LED_GREEN_GPIO_PIN) /*!< Turn off target LED_GREEN */
#define LED_GREEN_TOGGLE() \
    GPIO_PortToggle(BOARD_LED_GREEN_GPIO, 1U << BOARD_LED_GREEN_GPIO_PIN) /*!< Toggle on target LED_GREEN */

#define LED_BLUE_INIT(output)                                            \
    GPIO_PinWrite(BOARD_LED_BLUE_GPIO, BOARD_LED_BLUE_GPIO_PIN, output); \
    BOARD_LED_BLUE_GPIO->PDDR |= (1U << BOARD_LED_BLUE_GPIO_PIN) /*!< Enable target LED_BLUE */
#define LED_BLUE_ON()                                                                                   \
    GPIO_PortClear(BOARD_LED_BLUE_GPIO, 1U << BOARD_LED_BLUE_GPIO_PIN) /*!< Turn on target LED_BLUE \ \ \
                                                                            */
#define LED_BLUE_OFF()                                                                                 \
    GPIO_PortSet(BOARD_LED_BLUE_GPIO, 1U << BOARD_LED_BLUE_GPIO_PIN) /*!< Turn off target LED_BLUE \ \ \
                                                                          */
#define LED_BLUE_TOGGLE() \
    GPIO_PortToggle(BOARD_LED_BLUE_GPIO, 1U << BOARD_LED_BLUE_GPIO_PIN) /*!< Toggle on target LED_BLUE */

#define LED_YELLOW_INIT(output)                                              \
    GPIO_PinWrite(BOARD_LED_YELLOW_GPIO, BOARD_LED_YELLOW_GPIO_PIN, output); \
    BOARD_LED_YELLOW_GPIO->PDDR |= (1U << BOARD_LED_YELLOW_GPIO_PIN) /*!< Enable target LED_YELLOW */
#define LED_YELLOW_ON() \
    GPIO_PortClear(BOARD_LED_YELLOW_GPIO, 1U << BOARD_LED_YELLOW_GPIO_PIN) /*!< Turn on target LED_YELLOW */
#define LED_YELLOW_OFF() \
    GPIO_PortSet(BOARD_LED_YELLOW_GPIO, 1U << BOARD_LED_YELLOW_GPIO_PIN) /*!< Turn off target LED_YELLOW */
#define LED_YELLOW_TOGGLE() \
    GPIO_PortToggle(BOARD_LED_YELLOW_GPIO, 1U << BOARD_LED_YELLOW_GPIO_PIN) /*!< Toggle on target LED_YELLOW */

#define LED_ORANGE_INIT(output)                                              \
    GPIO_PinWrite(BOARD_LED_ORANGE_GPIO, BOARD_LED_ORANGE_GPIO_PIN, output); \
    BOARD_LED_ORANGE_GPIO->PDDR |= (1U << BOARD_LED_ORANGE_GPIO_PIN) /*!< Enable target LED_ORANGE */
#define LED_ORANGE_ON() \
    GPIO_PortClear(BOARD_LED_ORANGE_GPIO, 1U << BOARD_LED_ORANGE_GPIO_PIN) /*!< Turn on target LED_ORANGE */
#define LED_ORANGE_OFF() \
    GPIO_PortSet(BOARD_LED_ORANGE_GPIO, 1U << BOARD_LED_ORANGE_GPIO_PIN) /*!< Turn off target LED_ORANGE */
#define LED_ORANGE_TOGGLE() \
    GPIO_PortToggle(BOARD_LED_ORANGE_GPIO, 1U << BOARD_LED_ORANGE_GPIO_PIN) /*!< Toggle on target LED_ORANGE */
      
#define LED1_INIT(output)                                              \
    GPIO_PinWrite(BOARD_LED_GPIO, BOARD_LED1_GPIO_PIN, output); \
    BOARD_LED_GPIO->PDDR |= (1U << BOARD_LED1_GPIO_PIN) /*!< Enable target LED1 */
#define LED1_OFF() \
    GPIO_PortClear(BOARD_LED_GPIO, 1U << BOARD_LED1_GPIO_PIN) /*!< Turn on target LED1 */
#define LED1_ON() \
    GPIO_PortSet(BOARD_LED_GPIO, 1U << BOARD_LED1_GPIO_PIN) /*!< Turn off target LED1 */
#define LED1_TOGGLE() \
    GPIO_PortToggle(BOARD_LED_GPIO, 1U << BOARD_LED1_GPIO_PIN) /*!< Toggle on target LED1 */

#define LED2_INIT(output)                                              \
    GPIO_PinWrite(BOARD_LED_GPIO, BOARD_LED2_GPIO_PIN, output); \
    BOARD_LED_GPIO->PDDR |= (1U << BOARD_LED2_GPIO_PIN) /*!< Enable target LED1 */
#define LED2_OFF() \
    GPIO_PortClear(BOARD_LED_GPIO, 1U << BOARD_LED2_GPIO_PIN) /*!< Turn on target LED1 */
#define LED2_ON() \
    GPIO_PortSet(BOARD_LED_GPIO, 1U << BOARD_LED2_GPIO_PIN) /*!< Turn off target LED1 */
#define LED2_TOGGLE() \
    GPIO_PortToggle(BOARD_LED_GPIO, 1U << BOARD_LED2_GPIO_PIN) /*!< Toggle on target LED1 */

      
#define LED3_INIT(output)                                              \
    GPIO_PinWrite(BOARD_LED_GPIO, BOARD_LED3_GPIO_PIN, output); \
    BOARD_LED_GPIO->PDDR |= (1U << BOARD_LED3_GPIO_PIN) /*!< Enable target LED1 */
#define LED3_OFF() \
    GPIO_PortClear(BOARD_LED_GPIO, 1U << BOARD_LED3_GPIO_PIN) /*!< Turn on target LED1 */
#define LED3_ON() \
    GPIO_PortSet(BOARD_LED_GPIO, 1U << BOARD_LED3_GPIO_PIN) /*!< Turn off target LED1 */
#define LED3_TOGGLE() \
    GPIO_PortToggle(BOARD_LED_GPIO, 1U << BOARD_LED3_GPIO_PIN) /*!< Toggle on target LED1 */

      
#define LED4_INIT(output)                                              \
    GPIO_PinWrite(BOARD_LED_GPIO, BOARD_LED4_GPIO_PIN, output); \
    BOARD_LED_GPIO->PDDR |= (1U << BOARD_LED4_GPIO_PIN) /*!< Enable target LED1 */
#define LED4_OFF() \
    GPIO_PortClear(BOARD_LED_GPIO, 1U << BOARD_LED4_GPIO_PIN) /*!< Turn on target LED1 */
#define LED4_ON() \
    GPIO_PortSet(BOARD_LED_GPIO, 1U << BOARD_LED4_GPIO_PIN) /*!< Turn off target LED1 */
#define LED4_TOGGLE() \
    GPIO_PortToggle(BOARD_LED_GPIO, 1U << BOARD_LED4_GPIO_PIN) /*!< Toggle on target LED1 */
      
#define MOTOR2_ENABLE() \
    GPIO_PortSet(BOARD_MOTOR_ENABLE_GPIO, 1U << BOARD_MOTOR2_ENABLE_GPIO_PIN) 
#define MOTOR2_DISABLE() \
    GPIO_PortClear(BOARD_MOTOR_ENABLE_GPIO, 1U << BOARD_MOTOR2_ENABLE_GPIO_PIN) 
      
#define MOTOR3_ENABLE() \
    GPIO_PortSet(BOARD_MOTOR_ENABLE_GPIO, 1U << BOARD_MOTOR3_ENABLE_GPIO_PIN); \
      (Motor_3_En_Bit   = true)
#define MOTOR3_DISABLE() \
    GPIO_PortClear(BOARD_MOTOR_ENABLE_GPIO, 1U << BOARD_MOTOR3_ENABLE_GPIO_PIN); \
    (Motor_3_En_Bit   = false)
      
//#define MOTOR1_OFF() \
//    GPIO_PortClear(BOARD_MOTOR1_GPIO, 1U << BOARD_MOTOR1_PWM_PIN_1); \
//    GPIO_PortClear(BOARD_MOTOR1_GPIO, 1U << BOARD_MOTOR1_PWM_PIN_2) 
//#define MOTOR2_OFF() \
//    GPIO_PortClear(BOARD_MOTOR2_GPIO, 1U << BOARD_MOTOR2_PWM_PIN_1); \
//    GPIO_PortClear(BOARD_MOTOR2_GPIO, 1U << BOARD_MOTOR2_PWM_PIN_2) 
#define MOTOR3_OFF() \
    GPIO_PortClear(BOARD_MOTOR3_GPIO, 1U << BOARD_MOTOR3_PWM_PIN_1); \
    GPIO_PortClear(BOARD_MOTOR3_GPIO, 1U << BOARD_MOTOR3_PWM_PIN_2) 
      
/*! @brief The SMARTCARD interface. */
#define BOARD_SMARTCARD_MODULE (UART0)
#define BOARD_SMARTCARD_MODULE_IRQ (UART0_RX_TX_IRQn)
#define BOARD_SMARTCARD_MODULE_ERRIRQ (UART0_ERR_IRQn)
#define BOARD_SMARTCARD_CLOCK_MODULE (0u) /* FTM0 */
#define BOARD_SMARTCARD_CLOCK_MODULE_CHANNEL (0u)
#define BOARD_SMARTCARD_CLOCK_MODULE_SOURCE_CLK (kCLOCK_BusClk)
#define BOARD_SMARTCARD_CLOCK_MODULE_CLK_FREQ CLOCK_GetFreq((kCLOCK_BusClk))
#define BOARD_SMARTCARD_CLOCK_VALUE (5000000u)
#define BOARD_SMARTCARD_CONTROL_PORT (3u) /* PORTD */
#define BOARD_SMARTCARD_CONTROL_PIN (10u)
#ifndef BOARD_SMARTCARD_RST_PORT
#define BOARD_SMARTCARD_RST_PORT (2u) /* PORTC */
#endif
#ifndef BOARD_SMARTCARD_RST_PIN
#define BOARD_SMARTCARD_RST_PIN (3u)
#endif
#define BOARD_SMARTCARD_IRQ_PORT (3u) /* PORTD */
#define BOARD_SMARTCARD_IRQ_PIN (14u)
#define BOARD_SMARTCARD_IRQ_PIN_IRQ (PORTD_IRQn)
#define BOARD_SMARTCARD_VSEL0_PORT (0u) /* PORTA */
#define BOARD_SMARTCARD_VSEL0_PIN (5u)
#define BOARD_SMARTCARD_VSEL1_PORT (3u) /* PORTD */
#define BOARD_SMARTCARD_VSEL1_PIN (11u)
#define BOARD_SMARTCARD_TS_TIMER_ID (0) /* PIT0 timer channel 0 */
#define BOARD_SMARTCARD_TS_TIMER_IRQ (PIT0_IRQn)
/* SDHC base address, clock and card detection pin */
#define BOARD_SDHC_BASEADDR SDHC
#define BOARD_SDHC_CLKSRC kCLOCK_CoreSysClk
#define BOARD_SDHC_CLK_FREQ CLOCK_GetFreq(kCLOCK_CoreSysClk)
#define BOARD_SDHC_IRQ SDHC_IRQn
#define BOARD_SDHC_CD_GPIO_BASE GPIOC
#ifndef BOARD_SDHC_CD_GPIO_PIN
#define BOARD_SDHC_CD_GPIO_PIN 18U
#endif
#define BOARD_SDHC_CD_PORT_BASE PORTC
#define BOARD_SDHC_CD_PORT_IRQ PORTC_IRQn
#define BOARD_SDHC_CD_PORT_IRQ_HANDLER PORTC_IRQHandler
#define BOARD_MMC_VCC_SUPPLY kMMC_VoltageWindows270to360

#define BOARD_ACCEL_I2C_BASEADDR I2C1

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

/*******************************************************************************
 * API
 ******************************************************************************/

void BOARD_InitDebugConsole(void);

#if defined(__cplusplus)
}
#endif /* __cplusplus */

#endif /* _BOARD_H_ */
