#include "fsl_port.h"
#include "fsl_gpio.h"
#include "board.h"
#include "switch.h"

bool switchState = false;

void InitSwitch(void)
{
gpio_pin_config_t switch_config = {
    kGPIO_DigitalInput, 0,
  };
  
  port_pin_config_t pull_up_Config = {
    kPORT_PullUp,
    kPORT_SlowSlewRate,
    kPORT_PassiveFilterDisable,
    kPORT_OpenDrainDisable,
    kPORT_HighDriveStrength,
    kPORT_MuxAsGpio,
    kPORT_UnlockRegister,     
  };
  
  GPIO_PinInit(SWITCH_GPIO, SWITCH_PIN, &switch_config);
  PORT_SetPinConfig(SWITCH_PORT, SWITCH_PIN, &pull_up_Config);
}

eControlModes GetSwitchState(void)
{
  switchState = (GPIO_PinRead(SWITCH_GPIO, SWITCH_PIN) == SWITCH_VALID_STATE);
  if(!switchState)
  {
    return MANUAL_CONTROL;
  }
  else
  {
    return AUTO_CONTROL;
  }
}