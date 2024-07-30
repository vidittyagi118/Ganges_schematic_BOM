#ifndef __MOTOR1CONTROL_HAL_H_
#define __MOTOR1CONTROL_HAL_H_

#include "Motor1Control_Config.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct {
  bool mot1Polarity;
} stMot1Polarity;


void Motor1Init_hal (void);
void Motor1MovePositive (void);                              // Loads the already set PWM to the Corresponsing channel for output
void Motor1MoveNegative (void);                              // Loads the already set PWM to the Corresponsing channel for output
void Motor1Break (void);
void Motor1Stop (void);
bool UpdateMotor1PositivePwm (float pwmDuty);               // Sets the PWM and outpus to the Corresponsing channel for output
bool UpdateMotor1NegativePwm (float pwmDuty);               // Sets the PWM and outpus to the Corresponsing channel for output
bool UpdateMotor1StopPwm (float pwmDuty);                   // Sets the PWM as End_PWm_Value and outpus to the Corresponsing channel for output
uint8_t GetMotor1NegativePwm (void);
uint8_t GetMotor1PositivePwm (void);  

stMot1Polarity * GetSetMot1Polarity (void);
static void Error_Handler(void);
static void Motor1InitPins(void);
static void Motor1Enable (void);
static void Motor1Disable (void);

#endif