#ifndef __BRUSHMOTORCONTROL_HAL_H_
#define __BRUSHMOTORCONTROL_HAL_H_

#include "BrushMotorControl_Config.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct {
  bool mot1Polarity;
} stBrushMotPolarity;


void BrushMotorInit_hal (void);
void BrushMotorMovePositive (void);                              // Loads the already set PWM to the Corresponsing channel for output
void BrushMotorMoveNegative (void);                              // Loads the already set PWM to the Corresponsing channel for output
void BrushMotorBreak (void);
void BrushMotorStop (void);
bool UpdateBrushMotorPositivePwm (float pwmDuty);               // Sets the PWM and outpus to the Corresponsing channel for output
bool UpdateBrushMotorNegativePwm (float pwmDuty);               // Sets the PWM and outpus to the Corresponsing channel for output
bool UpdateBrushMotorStopPwm (float pwmDuty);                   // Sets the PWM as End_PWm_Value and outpus to the Corresponsing channel for output
uint8_t GetBrushMotorNegativePwm (void);
uint8_t GetBrushMotorPositivePwm (void);  

stBrushMotPolarity * GetSetBrushMotPolarity (void);
static void Error_Handler(void);
static void BrushMotorInitPins(void);
static void BrushMotorEnable (void);
static void BrushMotorDisable (void);

#endif