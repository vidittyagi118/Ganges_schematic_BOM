#ifndef _BAT_I2C_INIT_H_
#define _BAT_I2C_INIT_H_

#include "fsl_i2c.h"
#include "LTC4015.h"

#define BOARD_BAT_I2C_BASEADDR I2C1   
#define BAT_I2C_CLK_SRC I2C1_CLK_SRC
#define BAT_I2C_CLK_FREQ CLOCK_GetFreq(I2C1_CLK_SRC)
#define BAT_I2C_RELEASE_SDA_PORT PORTC
#define BAT_I2C_RELEASE_SCL_PORT PORTC
#define BAT_I2C_RELEASE_SDA_GPIO GPIOC
#define BAT_I2C_RELEASE_SDA_PIN 11U
#define BAT_I2C_RELEASE_SCL_GPIO GPIOC
#define BAT_I2C_RELEASE_SCL_PIN 10U
#define BAT_I2C_BAUDRATE 100000U


void InitBatI2C(void);
void BOARD_BAT_I2C_ReleaseBus(void);
void BatI2CTimer(void); //has to be called every millisecond
uint32_t GetBatI2CTime(void);

int BatI2C_WriteReg(uint8_t device_addr, uint8_t reg_addr, uint16_t value,port_configuration_t* I2Cdata);
int BatI2C_ReadRegs(uint8_t device_addr, uint8_t reg_addr, uint16_t *rxBuff,port_configuration_t* I2Cdata);
void bat_i2c_master_callback(I2C_Type *base, i2c_master_handle_t *handle, status_t status, void *userData);
int BatI2C_ReadAlert(uint8_t device_addr, uint8_t reg_addr, uint8_t *rxBuff, I2C_Type* I2Cdata);
#endif