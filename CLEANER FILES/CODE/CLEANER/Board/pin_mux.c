/*
 * The Clear BSD License
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
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

/* clang-format off */
/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!GlobalInfo
product: Pins v3.0
processor: MK21FN1M0Axxx12
package_id: MK21FN1M0AVMC12
mcu_data: ksdk2_0
processor_version: 0.0.8
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */
/* clang-format on */

#include "fsl_common.h"
#include "fsl_port.h"
#include "pin_mux.h"
#include "board.h"



/* clang-format off */
/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
BOARD_InitPins:
- options: {callFromInitBoot: 'true', coreID: core0, enableClock: 'true'}
- pin_list:
  - {pin_num: A10, peripheral: UART5, signal: RX, pin_signal: PTD8/I2C0_SCL/UART5_RX/FBa_A16}
  - {pin_num: A9, peripheral: UART5, signal: TX, pin_signal: PTD9/I2C0_SDA/UART5_TX/FBa_A17}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */
/* clang-format on */

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitPins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
void BOARD_InitPins(void)
{
  

    CLOCK_EnableClock(kCLOCK_PortA);
    CLOCK_EnableClock(kCLOCK_PortB);
    CLOCK_EnableClock(kCLOCK_PortD);
    CLOCK_EnableClock(kCLOCK_PortC);
    CLOCK_EnableClock(kCLOCK_PortE);

    PORT_SetPinMux(BOARD_LED_PORT, BOARD_LED1_GPIO_PIN, kPORT_MuxAsGpio);
    PORT_SetPinMux(BOARD_LED_PORT, BOARD_LED2_GPIO_PIN, kPORT_MuxAsGpio);
    PORT_SetPinMux(BOARD_LED_PORT, BOARD_LED3_GPIO_PIN, kPORT_MuxAsGpio);
    PORT_SetPinMux(BOARD_LED_PORT, BOARD_LED4_GPIO_PIN, kPORT_MuxAsGpio);
    
    PORT_SetPinMux(BOARD_RELAY_PORT, BOARD_RELAY_ON_GPIO_PIN, kPORT_MuxAsGpio);
    PORT_SetPinMux(BOARD_RELAY_PORT, BOARD_RELAY_OFF_GPIO_PIN, kPORT_MuxAsGpio);
    
    PORT_SetPinMux(BOARD_SD_PORT, BOARD_SD_CS_GPIO_PIN, kPORT_MuxAsGpio);//new
    PORT_SetPinMux(BOARD_SD_PORT, BOARD_SD_CLK_GPIO_PIN, kPORT_MuxAsGpio);
    PORT_SetPinMux(BOARD_SD_PORT, BOARD_SD_MOUT_GPIO_PIN, kPORT_MuxAsGpio);
    PORT_SetPinMux(BOARD_SD_PORT, BOARD_SD_MIN_GPIO_PIN, kPORT_MuxAsGpio);
    
    
 //   PORT_SetPinMux(BOARD_SPI0_PORT, BOARD_SPI0_SCLK, kPORT_MuxAlt2);
  //  PORT_SetPinMux(BOARD_SPI0_PORT, BOARD_SPI0_SDO, kPORT_MuxAlt2);//modi
 //   PORT_SetPinMux(BOARD_SPI0_PORT, BOARD_SPI0_SDI, kPORT_MuxAlt2);
    
 //   PORT_SetPinMux(BOARD_CHIPSELECT_PORT, SPI_CHIPSELECT_PIN_0, kPORT_MuxAlt2);
 //   PORT_SetPinMux(BOARD_CHIPSELECT_PORT, SPI_CHIPSELECT_PIN_1, kPORT_MuxAlt2);
 //   PORT_SetPinMux(BOARD_CHIPSELECT_PORT, SPI_CHIPSELECT_PIN_2, kPORT_MuxAlt2);
 //   PORT_SetPinMux(BOARD_CHIPSELECT_PORT, SPI_CHIPSELECT_PIN_3, kPORT_MuxAlt2);
//    PORT_SetPinMux(BOARD_CHIPSELECT_PORT, SPI_CHIPSELECT_PIN_4, kPORT_MuxAlt2);
    

    
    PORT_SetPinMux(BOARD_ENCODER_PORT, BOARD_ENCODER_MOTOR1_PIN, kPORT_MuxAsGpio);
    PORT_SetPinMux(BOARD_ENCODER_PORT, BOARD_ENCODER_MOTOR2_PIN, kPORT_MuxAsGpio);
    
    PORT_SetPinMux(BOARD_EDGESENSOR1_PORT, BOARD_EDGESENSOR1_GPIO_PIN, kPORT_MuxAsGpio);
    PORT_SetPinMux(BOARD_EDGESENSOR2_PORT, BOARD_EDGESENSOR2_GPIO_PIN, kPORT_MuxAsGpio);
    

    PORT_SetPinMux(BOARD_MOTOR1_PORT, BOARD_MOTOR1_PWM_PIN_1, kPORT_MuxAlt6);
    PORT_SetPinMux(BOARD_MOTOR1_PORT, BOARD_MOTOR1_PWM_PIN_2, kPORT_MuxAlt6);
    

    
    PORT_SetPinMux(BOARD_MOTOR2_PORT, BOARD_MOTOR2_PWM_PIN_1, kPORT_MuxAlt4);
    PORT_SetPinMux(BOARD_MOTOR2_PORT, BOARD_MOTOR2_PWM_PIN_2, kPORT_MuxAlt4);
    
    PORT_SetPinMux(BOARD_BRUSHMOTOR_PORT, BOARD_BRUSHMOTOR_PWM_PIN_1, kPORT_MuxAlt3);
    PORT_SetPinMux(BOARD_BRUSHMOTOR_PORT, BOARD_BRUSHMOTOR_PWM_PIN_2, kPORT_MuxAlt3);
    
    PORT_SetPinMux(BOARD_MOTOR_ENABLE_PORT, BOARD_MOTOR1_ENABLE_GPIO_PIN, kPORT_MuxAsGpio);
    PORT_SetPinMux(BOARD_MOTOR_ENABLE_PORT, BOARD_MOTOR2_ENABLE_GPIO_PIN, kPORT_MuxAsGpio);
    PORT_SetPinMux(BOARD_MOTOR_ENABLE_PORT, BOARD_BRUSHMOTOR_ENABLE_GPIO_PIN, kPORT_MuxAsGpio);
       
                    
    PORT_SetPinMux(BOARD_ZIGBEE_UART_PORT, BOARD_ZIGBEE_UART_TX_PIN, kPORT_MuxAlt3);
    PORT_SetPinMux(BOARD_ZIGBEE_UART_PORT, BOARD_ZIGBEE_UART_RX_PIN, kPORT_MuxAlt3);
    PORT_SetPinMux(BOARD_ZIGBEE_RESET_PORT, BOARD_ZIGBEE_RESET_PIN, kPORT_MuxAsGpio);
    
    PORT_SetPinMux(BOARD_ZIGBEE_UART_PORT, BOARD_ZIGBEE_UART_TX_PIN, kPORT_MuxAlt3);
    PORT_SetPinMux(BOARD_ZIGBEE_UART_PORT, BOARD_ZIGBEE_UART_RX_PIN, kPORT_MuxAlt3);
    
    PORT_SetPinMux(BOARD_DEBUG_UART_PORT, BOARD_DEBUG_UART_TX_PIN, kPORT_MuxAlt3);    
    PORT_SetPinMux(BOARD_DEBUG_UART_PORT, BOARD_DEBUG_UART_RX_PIN , kPORT_MuxAlt3);
    
    PORT_SetPinMux(BOARD_CONFIG_UART_PORT, BOARD_CONFIG_UART_TX_PIN, kPORT_MuxAlt3);    
    PORT_SetPinMux(BOARD_CONFIG_UART_PORT, BOARD_CONFIG_UART_RX_PIN , kPORT_MuxAlt3);
    
    PORT_SetPinMux(LTC4015_PORT, SMBALERT_PIN, kPORT_MuxAsGpio);
    PORT_SetPinMux(MANUAL_CONTROL_SWITCH_PORT,MANUAL_CONTROL_SWITCH_PIN,kPORT_MuxAsGpio);

}

//void I2C1_InitPins(void)
//{
//    /* Port E Clock Gate Control: Clock enabled */
//    CLOCK_EnableClock(kCLOCK_PortC);
//
//    const port_pin_config_t portc0_pin11_config = {/* Internal pull-up resistor is enabled */
//                                                  kPORT_PullDisable,
//                                                  /* Fast slew rate is configured */
//                                                  kPORT_FastSlewRate,
//                                                  /* Passive filter is disabled */
//                                                  kPORT_PassiveFilterDisable,
//                                                  /* Open drain is enabled */
//                                                  kPORT_OpenDrainEnable,
//                                                  /* Low drive strength is configured */
//                                                  kPORT_LowDriveStrength,
//                                                  /* Pin is configured as I2C1_SDA */
//                                                  kPORT_MuxAlt2,
//                                                  /* Pin Control Register fields [15:0] are not locked */
//                                                  kPORT_UnlockRegister};
//    /* PORTC11  is configured as I2C1_SDA */
//    PORT_SetPinConfig(PORTC, 11U, &portc0_pin11_config);
//
//    const port_pin_config_t portc1_pin10_config = {/* Internal pull-up resistor is enabled */
//                                                  kPORT_PullDisable,
//                                                  /* Fast slew rate is configured */
//                                                  kPORT_FastSlewRate,
//                                                  /* Passive filter is disabled */
//                                                  kPORT_PassiveFilterDisable,
//                                                  /* Open drain is enabled */
//                                                  kPORT_OpenDrainEnable,
//                                                  /* Low drive strength is configured */
//                                                  kPORT_LowDriveStrength,
//                                                  /* Pin is configured as I2C1_SCL */
//                                                  kPORT_MuxAlt2,
//                                                  /* Pin Control Register fields [15:0] are not locked */
//                                                  kPORT_UnlockRegister};
//    /* PORTC10  is configured as I2C1_SCL */
//    PORT_SetPinConfig(PORTC, 10U, &portc1_pin10_config);
//}void I2C1_DeinitPins(void)
//{
//    /* Port E Clock Gate Control: Clock enabled */
//    CLOCK_EnableClock(kCLOCK_PortC);
//
//    /* PORTE0 (pin 1) is configured as ADC1_SE4a */
//    PORT_SetPinMux(PORTC, 10U, kPORT_PinDisabledOrAnalog);
//
//    /* PORTE1 (pin 2) is configured as ADC1_SE5a */
//    PORT_SetPinMux(PORTC, 11U, kPORT_PinDisabledOrAnalog);
//}

void I2C0_InitPins(void)
{
    /* Port E Clock Gate Control: Clock enabled */
    CLOCK_EnableClock(kCLOCK_PortB);

    const port_pin_config_t porte0_pin1_config = {/* Internal pull-up resistor is enabled */
                                                  kPORT_PullDisable,
                                                  /* Fast slew rate is configured */
                                                  kPORT_FastSlewRate,
                                                  /* Passive filter is disabled */
                                                  kPORT_PassiveFilterDisable,
                                                  /* Open drain is enabled */
                                                  kPORT_OpenDrainEnable,
                                                  /* Low drive strength is configured */
                                                  kPORT_LowDriveStrength,
                                                  /* Pin is configured as I2C1_SDA */
                                                  kPORT_MuxAlt2,
                                                  /* Pin Control Register fields [15:0] are not locked */
                                                  kPORT_UnlockRegister};
    /* PORTE0 (pin 1) is configured as I2C1_SDA */
    PORT_SetPinConfig(PORTB, 1U, &porte0_pin1_config);

    const port_pin_config_t porte1_pin2_config = {/* Internal pull-up resistor is enabled */
                                                  kPORT_PullDisable,
                                                  /* Fast slew rate is configured */
                                                  kPORT_FastSlewRate,
                                                  /* Passive filter is disabled */
                                                  kPORT_PassiveFilterDisable,
                                                  /* Open drain is enabled */
                                                  kPORT_OpenDrainEnable,
                                                  /* Low drive strength is configured */
                                                  kPORT_LowDriveStrength,
                                                  /* Pin is configured as I2C1_SCL */
                                                  kPORT_MuxAlt2,
                                                  /* Pin Control Register fields [15:0] are not locked */
                                                  kPORT_UnlockRegister};
    /* PORTE1 (pin 2) is configured as I2C1_SCL */
    PORT_SetPinConfig(PORTB, 0U, &porte1_pin2_config);
}void I2C0_DeinitPins(void)
{
    /* Port E Clock Gate Control: Clock enabled */
    CLOCK_EnableClock(kCLOCK_PortB);

    /* PORTE0 (pin 1) is configured as ADC1_SE4a */
    PORT_SetPinMux(PORTB, 0U, kPORT_PinDisabledOrAnalog);

    /* PORTE1 (pin 2) is configured as ADC1_SE5a */
    PORT_SetPinMux(PORTB, 1U, kPORT_PinDisabledOrAnalog);
}
/***********************************************************************************************************************
 * EOF
 **********************************************************************************************************************/
