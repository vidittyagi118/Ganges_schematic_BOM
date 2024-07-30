#include "bat_i2c.h"
#include "board.h"
#include "fsl_i2c.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_gpio.h"
#include "fsl_port.h"
#include "LTC4015.h"
#include "Serial_Debug.h"

i2c_master_handle_t g_m_handle_b;
volatile bool completionFlag_b=false;
volatile bool nakFlag_b=false;
uint32_t batI2CTimer = 0;
uint32_t difference =0;
uint32_t currTime = 0;
/*----------Battery Charger----------*/

#define I2C_RELEASE_BUS_COUNT 100U
#define BAT_I2C_TIMEOUT       100U //ms

void I2C1_InitPins(void);
void I2C1_DeinitPins(void);

void BatI2CTimer(void)
{
  batI2CTimer++;
}
uint32_t GetBatI2CTime(void)
{
 return batI2CTimer;
}

void InitBatI2C(void)
{ 
  uint32_t sourceClock = 0;
  BOARD_BAT_I2C_ReleaseBus();
  I2C1_InitPins();
  I2C_MasterTransferCreateHandle(BOARD_BAT_I2C_BASEADDR, &g_m_handle_b, bat_i2c_master_callback, NULL);
  i2c_master_config_t masterConfig;
  I2C_MasterGetDefaultConfig(&masterConfig);
  
  masterConfig.baudRate_Bps = BAT_I2C_BAUDRATE;
  
  sourceClock = BAT_I2C_CLK_FREQ;
  
  I2C_MasterInit(BOARD_BAT_I2C_BASEADDR, &masterConfig, sourceClock);
  
}

void bat_i2c_master_callback(I2C_Type *base, i2c_master_handle_t *handle, status_t status, void *userData)
{
  /* Signal transfer success when received success status. */
  if (status == kStatus_Success)
  {
    completionFlag_b = true;
  }
  /* Signal transfer success when received success status. */
  if ((status == kStatus_I2C_Nak) || (status == kStatus_I2C_Addr_Nak))
  {
    nakFlag_b = true;
  }
}

void i2c_release_bus_delay_b(void)
{
  uint32_t i = 0;
  for (i = 0; i < I2C_RELEASE_BUS_COUNT; i++)
  {
    __NOP();
  }
}


void BOARD_BAT_I2C_ReleaseBus(void)
{
  uint8_t i = 0;
  gpio_pin_config_t pin_config;
  port_pin_config_t i2c_pin_config = {0};
  
  /* Config pin mux as gpio */
  i2c_pin_config.pullSelect = kPORT_PullUp;
  i2c_pin_config.mux = kPORT_MuxAsGpio;
  
  pin_config.pinDirection = kGPIO_DigitalOutput;
  pin_config.outputLogic = 1U;
  CLOCK_EnableClock(kCLOCK_PortB);
  PORT_SetPinConfig(BAT_I2C_RELEASE_SCL_PORT, BAT_I2C_RELEASE_SCL_PIN, &i2c_pin_config);
  PORT_SetPinConfig(BAT_I2C_RELEASE_SCL_PORT, BAT_I2C_RELEASE_SDA_PIN, &i2c_pin_config);
  
  GPIO_PinInit(BAT_I2C_RELEASE_SCL_GPIO, BAT_I2C_RELEASE_SCL_PIN, &pin_config);
  GPIO_PinInit(BAT_I2C_RELEASE_SDA_GPIO, BAT_I2C_RELEASE_SDA_PIN, &pin_config);
  
  /* Drive SDA low first to simulate a start */
  GPIO_PinWrite(BAT_I2C_RELEASE_SDA_GPIO, BAT_I2C_RELEASE_SDA_PIN, 0U);
  i2c_release_bus_delay_b();
  
  /* Send 9 pulses on SCL and keep SDA high */
  for (i = 0; i < 9; i++)
  {
    GPIO_PinWrite(BAT_I2C_RELEASE_SCL_GPIO, BAT_I2C_RELEASE_SCL_PIN, 0U);
    i2c_release_bus_delay_b();
    
    GPIO_PinWrite(BAT_I2C_RELEASE_SDA_GPIO, BAT_I2C_RELEASE_SDA_PIN, 1U);
    i2c_release_bus_delay_b();
    
    GPIO_PinWrite(BAT_I2C_RELEASE_SCL_GPIO, BAT_I2C_RELEASE_SCL_PIN, 1U);
    i2c_release_bus_delay_b();
    i2c_release_bus_delay_b();
  }
  
  /* Send stop */
  GPIO_PinWrite(BAT_I2C_RELEASE_SCL_GPIO, BAT_I2C_RELEASE_SCL_PIN, 0U);
  i2c_release_bus_delay_b();
  
  GPIO_PinWrite(BAT_I2C_RELEASE_SDA_GPIO, BAT_I2C_RELEASE_SDA_PIN, 0U);
  i2c_release_bus_delay_b();
  
  GPIO_PinWrite(BAT_I2C_RELEASE_SCL_GPIO, BAT_I2C_RELEASE_SCL_PIN, 1U);
  i2c_release_bus_delay_b();
  
  GPIO_PinWrite(BAT_I2C_RELEASE_SDA_GPIO, BAT_I2C_RELEASE_SDA_PIN, 1U);
  i2c_release_bus_delay_b();
}




int BatI2C_WriteReg(uint8_t device_addr, uint8_t reg_addr, uint16_t value,port_configuration_t* I2Cdata)
{
  nakFlag_b = false;completionFlag_b = false;
  uint8_t data_temp[2];
  data_temp[0] = value & 0x00FF;
  data_temp[1] = value>>8;
  //  Serial_Debug("\nIn Write I2C-->");
  //  Serial_Debug_Num(data_temp[1]);
  //  Serial_Debug(" ");
  //  Serial_Debug_Num(data_temp[0]);
  i2c_master_transfer_t masterXfer;
  memset(&masterXfer, 0, sizeof(masterXfer));
  
  masterXfer.slaveAddress = device_addr;
  masterXfer.direction = kI2C_Write;
  masterXfer.subaddress = reg_addr;
  masterXfer.subaddressSize = 1;
  masterXfer.data = data_temp;
  masterXfer.dataSize = 2;
  masterXfer.flags = kI2C_TransferDefaultFlag;
  
  /*  direction=write : start+device_write;cmdbuff;xBuff; */
  /*  direction=recive : start+device_write;cmdbuff;repeatStart+device_read;xBuff; */
  
  I2C_MasterTransferNonBlocking(I2Cdata->I2C_No, &g_m_handle_b, &masterXfer);
  currTime = GetBatI2CTime();
  /*  wait for transfer completed. */
  while ((!nakFlag_b) && (!completionFlag_b))
  {
    if(GetBatI2CTime()-currTime>=BAT_I2C_TIMEOUT)
    {
    return true;
    }
  }
  
  nakFlag_b = false;
  
  if (completionFlag_b == true)
  {
    completionFlag_b = false;
    return false;
  }
  else
  {
    return true;
  }
}

int BatI2C_ReadRegs(uint8_t device_addr, uint8_t reg_addr, uint16_t *rxBuff, port_configuration_t* I2Cdata)
{
  nakFlag_b = false;completionFlag_b = false;
  uint8_t readBuff[2];
  i2c_master_transfer_t masterXfer;
  memset(&masterXfer, 0, sizeof(masterXfer));
  masterXfer.slaveAddress = device_addr;
  masterXfer.direction = kI2C_Read;
  masterXfer.subaddress = reg_addr;
  masterXfer.subaddressSize = 1;
  masterXfer.data = readBuff;
  masterXfer.dataSize = 2;
  masterXfer.flags = kI2C_TransferDefaultFlag;
  
  /*  direction=write : start+device_write;cmdbuff;xBuff; */
  /*  direction=recive : start+device_write;cmdbuff;repeatStart+device_read;xBuff; */
  
  I2C_MasterTransferNonBlocking(I2Cdata->I2C_No, &g_m_handle_b, &masterXfer);
 
  currTime = GetBatI2CTime();
  difference = 0;
  /*  wait for transfer completed. */
  while ((!nakFlag_b) && (!completionFlag_b))
  {
    difference = GetBatI2CTime()-currTime;
    if(difference>=BAT_I2C_TIMEOUT)
    {
    return true;
    }
  }
 // difference++;
  nakFlag_b = false;
  
  if (completionFlag_b == true)
  {
    completionFlag_b = false;
    int tempData = (((readBuff[1] << 8) | (readBuff[0] & 0xFF)));
    *rxBuff = tempData;
    //        Serial_Debug("\r\nIn I2C-->");
    //      Serial_Debug("\nreadBUff[1]-->");
    //      Serial_Debug_Num(readBuff[1]);
    //      Serial_Debug("\nreadBUff[0]-->");
    //      Serial_Debug_Num(readBuff[0]);
    return false;
  }
  else
  {
    return true;
  }
}
int BatI2C_ReadAlert(uint8_t device_addr, uint8_t reg_addr, uint8_t *rxBuff, I2C_Type* I2Cdata)
{
  nakFlag_b = false;completionFlag_b = false;
  //uint8_t readBuff[2] = {0,0};
  i2c_master_transfer_t masterXfer;
  memset(&masterXfer, 0, sizeof(masterXfer));
  masterXfer.slaveAddress = device_addr;
  masterXfer.direction = kI2C_Read;
  masterXfer.subaddress = reg_addr;
  masterXfer.subaddressSize = 0;
  masterXfer.data = rxBuff;
  masterXfer.dataSize = 1;
  masterXfer.flags = kI2C_TransferDefaultFlag;
  
  /*  direction=write : start+device_write;cmdbuff;xBuff; */
  /*  direction=recive : start+device_write;cmdbuff;repeatStart+device_read;xBuff; */
  
  I2C_MasterTransferNonBlocking(I2Cdata, &g_m_handle_b, &masterXfer);
  currTime = GetBatI2CTime();
  /*  wait for transfer completed. */
  while ((!nakFlag_b) && (!completionFlag_b))
  {
    if(GetBatI2CTime()-currTime>=BAT_I2C_TIMEOUT)
    {
    return true;
    }
  }
  
  nakFlag_b = false;
  
  if (completionFlag_b == true)
  {
    completionFlag_b = false;
    return false;
  }
  else
  {
    
    return true;
  }
}
void I2C1_InitPins(void)
{
  /* Port C Clock Gate Control: Clock enabled */
  CLOCK_EnableClock(kCLOCK_PortC);
  
  const port_pin_config_t portc0_pin11_config = {/* Internal pull-up resistor is enabled */
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
  /* PORTC11  is configured as I2C1_SDA */
  PORT_SetPinConfig(BAT_I2C_RELEASE_SDA_PORT, BAT_I2C_RELEASE_SDA_PIN, &portc0_pin11_config);
  
  const port_pin_config_t portc1_pin10_config = {/* Internal pull-up resistor is enabled */
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
  /* PORTC10  is configured as I2C1_SCL */
  PORT_SetPinConfig(BAT_I2C_RELEASE_SCL_PORT, BAT_I2C_RELEASE_SCL_PIN, &portc1_pin10_config);
}
void I2C1_DeinitPins(void)
{
  //CLOCK_EnableClock(kCLOCK_PortC);
  
  PORT_SetPinMux(BAT_I2C_RELEASE_SCL_PORT, BAT_I2C_RELEASE_SCL_PIN, kPORT_PinDisabledOrAnalog);
  
  PORT_SetPinMux(BAT_I2C_RELEASE_SDA_PORT, BAT_I2C_RELEASE_SDA_PIN, kPORT_PinDisabledOrAnalog);
}