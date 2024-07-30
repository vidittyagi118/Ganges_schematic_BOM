#include "PortInterrupt.h"
#include "EdgeSense1.h"
#include "EdgeSense2.h"
#include "RotateSense1.h"
#include "RotateSense2.h"
#include "LTC4015_main.h"
#include "fsl_gpio.h"
#include "fsl_port.h"
#include "board.h"

#define BOARD_PORT_INTERRUPT_GPIO         GPIOA
#define BOARD_PORT_INTERRUPT_PORT         PORTA

#define BOARD_SMB_PORT_INTERRUPT_GPIO     GPIOC
#define BOARD_SMB_PORT_INTERRUPT_PORT     PORTC

//  TODO Check if Interrupt Enabled Flag is Necessary inside all IRQ handler.

void PORTA_IRQHandler(void)
{
  uint32_t volatile interruptFlagsSet = 0;
  interruptFlagsSet = PORT_GetPinsInterruptFlags(BOARD_PORT_INTERRUPT_PORT);
//  interruptFlagsSet = GPIO_PortGetInterruptFlags(BOARD_PORT_GPIO);
  if(interruptFlagsSet & (1 << ROTATE_SENSE1_GPIO_PIN))
  {
    Rotate_Sense1_Irq();
  }
  if(interruptFlagsSet & (1 << ROTATE_SENSE2_GPIO_PIN))
  {
    Rotate_Sense2_Irq();
  }
  if(interruptFlagsSet & (1 << EDGE_SENSE1_GPIO_PIN))
  {
    Edge_Sense1_Irq();
  }
  if(interruptFlagsSet & (1 << EDGE_SENSE2_GPIO_PIN))
  {
    Edge_Sense2_Irq();
  }
  
#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
}

void EnablePortInterrupt (void)
{
  EnableIRQ(PORTA_IRQn);
}

void DisablePortInterrupt (void)
{
  DisableIRQ(PORTA_IRQn); 
}

void PORTC_IRQHandler(void)
{
  uint32_t volatile interruptFlagsSet = 0;
  interruptFlagsSet = PORT_GetPinsInterruptFlags(BOARD_SMB_PORT_INTERRUPT_PORT);
  if(interruptFlagsSet & (1 << SMBALERT_PIN))
  {
    SMB_Irq();
    GPIO_ClearPinsInterruptFlags(LTC4015_GPIO, 1U << SMBALERT_PIN);
  }
}

void EnableSmbPortInterrupt (void)
{
  EnableIRQ(PORTC_IRQn);
}

void DisableSmbPortInterrupt (void)
{
  DisableIRQ(PORTC_IRQn); 
}