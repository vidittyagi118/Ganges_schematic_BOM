#ifndef _I2C_INIT_H_
#define _I2C_INIT_H_

#include "fsl_i2c.h"

void InitI2C(void);
void BOARD_I2C_ReleaseBus(void);
bool I2C_WriteKeypadReg(I2C_Type *base, uint8_t device_addr, uint8_t reg_addr,uint8_t reg_size, uint8_t value);
bool I2C_ReadKeypadRegs(I2C_Type *base, uint8_t device_addr, uint8_t reg_addr,uint8_t reg_size, uint8_t *rxBuff, uint32_t rxSize);
void i2c_master_callback(I2C_Type *base, i2c_master_handle_t *handle, status_t status, void *userData);
void TempSenseTimer_ms(void);
uint32_t GetTempSenseI2CTimer(void);

#endif