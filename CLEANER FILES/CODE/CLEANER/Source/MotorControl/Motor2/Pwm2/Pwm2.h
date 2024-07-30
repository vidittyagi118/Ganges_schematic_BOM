
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PWM2_H
#define __PWM2_H

#include <stdint.h>
#include <stdbool.h>
#include "Customtypedefs.h"

#include "Pwm2_Config.h"

bool Pwm2Init(void);
ErrorStatus SetPwm2Ch1DutyCycle (float dutyCycle);              /* This sets the Variable But DOES NOT change the Output */
ErrorStatus SetPwm2Ch2DutyCycle (float dutyCycle);              /* This sets the Variable But DOES NOT change the Output */
ErrorStatus SetUpdatePwm2Ch1DutyCycle (float dutyCycle);        /* This sets the Variable and also CHANGES the Output */
ErrorStatus SetUpdatePwm2Ch2DutyCycle (float dutyCycle);        /* This sets the Variable and also CHANGES the Output */
ErrorStatus StartPwm2Ch1 (void);                                /* This Loads the set Variable CHANGES the Output */
ErrorStatus StartPwm2Ch2 (void);                                /* This Loads the set Variable CHANGES the Output */
ErrorStatus StopPwm2Ch1 (void);                                 /* This DOES NOT Change the Variable But CHANGES the Output */
ErrorStatus StopPwm2Ch2 (void);                                 /* This DOES NOT Change the Variable But CHANGES the Output */
uint8_t GetPwm2Ch1DutyCycle (void);                             /* Set PWM2 value    */
uint8_t GetPwm2Ch2DutyCycle (void);                             /* Set PWM2 value    */

//uint8_t GetCurrentPwm2Duty (void);               /* Actual pin out PWM2 */

static void Error_Handler(void);

#endif