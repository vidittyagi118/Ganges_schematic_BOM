
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PWM1_H
#define __PWM1_H

#include <stdint.h>
#include <stdbool.h>
#include "Customtypedefs.h"

#include "Pwm1_Config.h"

bool Pwm1Init(void);
ErrorStatus SetPwm1Ch1DutyCycle (float dutyCycle);              /* This sets the Variable But DOES NOT change the Output */
ErrorStatus SetPwm1Ch2DutyCycle (float dutyCycle);              /* This sets the Variable But DOES NOT change the Output */
bool SetUpdatePwm1Ch1DutyCycle (float dutyCycle);               /* This sets the Variable and also CHANGES the Output */
bool SetUpdatePwm1Ch2DutyCycle (float dutyCycle);               /* This sets the Variable and also CHANGES the Output */
ErrorStatus StartPwm1Ch1 (void);                                /* This Loads the set Variable CHANGES the Output */
ErrorStatus StartPwm1Ch2 (void);                                /* This Loads the set Variable CHANGES the Output */
ErrorStatus StopPwm1Ch1 (void);                                 /* This DOES NOT Change the Variable But CHANGES the Output */
ErrorStatus StopPwm1Ch2 (void);                                 /* This DOES NOT Change the Variable But CHANGES the Output */
uint8_t GetPwm1Ch1DutyCycle (void);                             /* Set PWM1 value    */
uint8_t GetPwm1Ch2DutyCycle (void);                             /* Set PWM1 value    */

//uint8_t GetCurrentPwm1Duty (void);               /* Actual pin out PWM1 */

static void Error_Handler(void);

#endif