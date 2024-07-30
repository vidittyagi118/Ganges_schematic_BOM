#include "i2c_init.h"
#include "board.h"
#include "fsl_i2c.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_gpio.h"
#include "fsl_port.h"
#include "Serial_Debug.h"

i2c_master_handle_t g_m_handle_t;
uint8_t g_keypad_addr_found;
volatile bool completionFlag=false;
volatile bool nakFlag=false;
//volatile bool bp=false;
uint32_t TempSensorI2cTimer = 0;

#define BOARD_TEMP_I2C_BASEADDR I2C0  
#define TEMP_I2C_CLK_SRC I2C0_CLK_SRC
#define TEMP_I2C_CLK_FREQ CLOCK_GetFreq(I2C0_CLK_SRC)
#define I2C_RELEASE_SDA_PORT PORTB
#define I2C_RELEASE_SCL_PORT PORTB
#define I2C_RELEASE_SDA_GPIO GPIOB
#define I2C_RELEASE_SDA_PIN 1U
#define I2C_RELEASE_SCL_GPIO GPIOB
#define I2C_RELEASE_SCL_PIN 0U
#define I2C_RELEASE_BUS_COUNT 100U
#define I2C_BAUDRATE 100000U

#define TEMP_I2C_TIMEOUT 100 //ms

void TempSenseTimer_ms(void)
{
TempSensorI2cTimer++;
}

uint32_t GetTempSenseI2CTimer(void)
{
return TempSensorI2cTimer;
}

void InitI2C(void)
{ 
  uint32_t sourceClock = 0;
    BOARD_I2C_ReleaseBus();
    I2C0_InitPins();
    //Serial_Debug("\r\nRead Temp Value\r\n");

    I2C_MasterTransferCreateHandle(BOARD_TEMP_I2C_BASEADDR, &g_m_handle_t, i2c_master_callback, NULL);
    i2c_master_config_t masterConfig;
    I2C_MasterGetDefaultConfig(&masterConfig);

    masterConfig.baudRate_Bps = I2C_BAUDRATE;

    sourceClock = TEMP_I2C_CLK_FREQ;

    I2C_MasterInit(BOARD_TEMP_I2C_BASEADDR, &masterConfig, sourceClock);
     
}
void i2c_master_callback(I2C_Type *base, i2c_master_handle_t *handle, status_t status, void *userData)
{
    /* Signal transfer success when received success status. */
    if (status == kStatus_Success)
    {
        completionFlag = true;
    }
    /* Signal transfer success when received success status. */
    if ((status == kStatus_I2C_Nak) || (status == kStatus_I2C_Addr_Nak))
    {
        nakFlag = true;
    }
}
void i2c_release_bus_delay(void)
{
    uint32_t i = 0;
    for (i = 0; i < I2C_RELEASE_BUS_COUNT; i++)
    {
        __NOP();
    }
}


void BOARD_I2C_ReleaseBus(void)
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
    PORT_SetPinConfig(I2C_RELEASE_SCL_PORT, I2C_RELEASE_SCL_PIN, &i2c_pin_config);
    PORT_SetPinConfig(I2C_RELEASE_SCL_PORT, I2C_RELEASE_SDA_PIN, &i2c_pin_config);

    GPIO_PinInit(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, &pin_config);
    GPIO_PinInit(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, &pin_config);

    /* Drive SDA low first to simulate a start */
    GPIO_PinWrite(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, 0U);
    i2c_release_bus_delay();

    /* Send 9 pulses on SCL and keep SDA high */
    for (i = 0; i < 9; i++)
    {
        GPIO_PinWrite(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, 0U);
        i2c_release_bus_delay();

        GPIO_PinWrite(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, 1U);
        i2c_release_bus_delay();

        GPIO_PinWrite(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, 1U);
        i2c_release_bus_delay();
        i2c_release_bus_delay();
    }

    /* Send stop */
    GPIO_PinWrite(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, 0U);
    i2c_release_bus_delay();

    GPIO_PinWrite(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, 0U);
    i2c_release_bus_delay();

    GPIO_PinWrite(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, 1U);
    i2c_release_bus_delay();

    GPIO_PinWrite(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, 1U);
    i2c_release_bus_delay();
}




bool I2C_WriteKeypadReg(I2C_Type *base, uint8_t device_addr, uint8_t reg_addr,uint8_t reg_size, uint8_t value)
{
    nakFlag = false; completionFlag = false;
    i2c_master_transfer_t masterXfer;
    memset(&masterXfer, 0, sizeof(masterXfer));

    masterXfer.slaveAddress = device_addr;
    masterXfer.direction = kI2C_Write;
    masterXfer.subaddress = reg_addr;
    masterXfer.subaddressSize = reg_size;
    masterXfer.data = &value;
    masterXfer.dataSize = 1;
    masterXfer.flags = kI2C_TransferDefaultFlag;

    /*  direction=write : start+device_write;cmdbuff;xBuff; */
    /*  direction=recive : start+device_write;cmdbuff;repeatStart+device_read;xBuff; */

    I2C_MasterTransferNonBlocking(base, &g_m_handle_t, &masterXfer);

    uint32_t currI2CTime = GetTempSenseI2CTimer();
    /*  wait for transfer completed. */
    while ((!nakFlag) && (!completionFlag))
    {
      if((GetTempSenseI2CTimer()-currI2CTime)>=TEMP_I2C_TIMEOUT)
      {
      return false;
      }
    }

    nakFlag = false;

    if (completionFlag == true)
    {
        completionFlag = false;
        return true;
    }
    else
    {
        return false;
    }
}

bool I2C_ReadKeypadRegs(I2C_Type *base, uint8_t device_addr, uint8_t reg_addr,uint8_t reg_size, uint8_t *rxBuff, uint32_t rxSize)
{
    nakFlag = false; completionFlag = false;
    i2c_master_transfer_t masterXfer;
    memset(&masterXfer, 0, sizeof(masterXfer));
    masterXfer.slaveAddress = device_addr;
    masterXfer.direction = kI2C_Read;
    masterXfer.subaddress = reg_addr;
    masterXfer.subaddressSize = reg_size;
    masterXfer.data = rxBuff;
    masterXfer.dataSize = rxSize;
    masterXfer.flags = kI2C_TransferDefaultFlag;

    /*  direction=write : start+device_write;cmdbuff;xBuff; */
    /*  direction=recive : start+device_write;cmdbuff;repeatStart+device_read;xBuff; */

    I2C_MasterTransferNonBlocking(base, &g_m_handle_t, &masterXfer);
    
    uint32_t currI2CTime = GetTempSenseI2CTimer();
    /*  wait for transfer completed. */
    while ((!nakFlag) && (!completionFlag))
    {
      if((GetTempSenseI2CTimer()-currI2CTime)>=TEMP_I2C_TIMEOUT)
      {
      return false;
      }
    }

    nakFlag = false;

    if (completionFlag == true)
    {
        completionFlag = false;
        return true;
    }
    else
    {
        return false;
    }
}