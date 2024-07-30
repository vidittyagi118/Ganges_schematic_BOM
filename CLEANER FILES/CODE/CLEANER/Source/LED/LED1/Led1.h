#ifndef __LED_1_H
#define __LED_1_H
  
#include <stdint.h>
#include <stdbool.h>

#define LED_1_GPIO                BOARD_LED_GPIO
#define LED_1_GPIO_PIN            BOARD_LED1_GPIO_PIN

typedef enum eLed1OperateState_def
{
  LED_1_INITIAL,
  LED_1_NORMAL,
  LED_1_ERROR1,
  LED_1_ERROR2  
}eLed1OperateState;

void Led1InitPins(void);
void Led1TimeIncrement_ms (void);
void OperateLed1 (void);
void SwitchLed1Off (void);
void SwitchLed1On (void);
void SwitchLed1Toggle (void);
void Led1Test (bool state);

#endif