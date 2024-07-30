#ifndef __LED_2_H
#define __LED_2_H
  
#include <stdint.h>
#include <stdbool.h>

#define LED_2_GPIO                BOARD_LED_GPIO
#define LED_2_GPIO_PIN            BOARD_LED2_GPIO_PIN

typedef enum eLed2OperateState_def
{
  LED_2_INITIAL,
  LED_2_NORMAL,
  LED_2_ERROR1,
}eLed2OperateState;

void Led2InitPins(void);
void Led2TimeIncrement_ms (void);
void OperateLed2 (void);
void SwitchLed2Off (void);
void SwitchLed2On (void);
void SwitchLed2Toggle (void);
void Led2Test (bool state);
void SetLed2OperateState(eLed2OperateState state);

#endif