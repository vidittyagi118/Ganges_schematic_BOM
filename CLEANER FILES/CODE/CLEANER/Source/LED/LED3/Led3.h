#ifndef __LED_3_H
#define __LED_3_H
  
#include <stdint.h>
#include <stdbool.h>

#define LED_3_GPIO                BOARD_LED_GPIO
#define LED_3_GPIO_PIN            BOARD_LED3_GPIO_PIN

typedef enum eLed3OperateState_def
{
  LED_3_INITIAL,
  LED_3_NORMAL,
  LED_3_ERROR1,
  LED_3_ERROR2  
}eLed3OperateState;

void SetVAL1var(int16_t );
void SetVAL2var(int16_t );
void SetVAL3var(int16_t );
int16_t GetVAL1var(void);
int16_t GetVAL2var(void);
int16_t GetVAL3var(void);

void Led3InitPins(void);
void Led3TimeIncrement_ms (void);
void OperateLed3 (void);
void SwitchLed3Off (void);
void SwitchLed3On (void);
void SwitchLed3Toggle (void);
void Led3Test (bool state);

#endif