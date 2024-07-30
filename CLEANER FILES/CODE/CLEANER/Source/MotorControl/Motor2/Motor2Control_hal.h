#ifndef __MOTOR2CONTROL_HAL_H_
#define __MOTOR2CONTROL_HAL_H_

#include "Motor2Control_Config.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct {
  bool mot2Polarity;
} stMot2Polarity;


void Motor2Init_hal (void);
void Motor2MovePositive (void);
void Motor2MoveNegative (void);
void Motor2Break (void);
void Motor2Stop (void);

bool UpdateMotor2PositivePwm (float pwmDuty);               // Sets the PWM and outpus to the Corresponsing channel for output
bool UpdateMotor2NegativePwm (float pwmDuty);               // Sets the PWM and outpus to the Corresponsing channel for output
bool UpdateMotor2StopPwm (float pwmDuty);                   // Sets the PWM as End_PWm_Value and outpus to the Corresponsing channel for output
uint8_t GetMotor2NegativePwm (void);
uint8_t GetMotor2PositivePwm (void);  

stMot2Polarity * GetSetMot2Polarity (void);
static void Error_Handler(void);
static void Motor2InitPins(void);
static void Motor2Enable (void);
static void Motor2Disable (void);

#endif