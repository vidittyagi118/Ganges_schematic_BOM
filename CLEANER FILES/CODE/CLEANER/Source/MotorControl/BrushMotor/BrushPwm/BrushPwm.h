
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BRUSHPWM_H
#define __BRUSHPWM_H

#include <stdint.h>
#include <stdbool.h>
#include "Customtypedefs.h"

#include "BrushPwm_Config.h"

bool BrushPwmInit(void);
ErrorStatus SetBrushPwmCh1DutyCycle (float dutyCycle);              /* This sets the Variable But DOES NOT change the Output */
ErrorStatus SetBrushPwmCh2DutyCycle (float dutyCycle);              /* This sets the Variable But DOES NOT change the Output */
bool SetUpdateBrushPwmCh1DutyCycle (float dutyCycle);               /* This sets the Variable and also CHANGES the Output */
bool SetUpdateBrushPwmCh2DutyCycle (float dutyCycle);               /* This sets the Variable and also CHANGES the Output */
ErrorStatus StartBrushPwmCh1 (void);                                /* This Loads the set Variable CHANGES the Output */
ErrorStatus StartBrushPwmCh2 (void);                                /* This Loads the set Variable CHANGES the Output */
ErrorStatus StopBrushPwmCh1 (void);                                 /* This DOES NOT Change the Variable But CHANGES the Output */
ErrorStatus StopBrushPwmCh2 (void);                                 /* This DOES NOT Change the Variable But CHANGES the Output */
uint8_t GetBrushPwmCh1DutyCycle (void);                             /* Set BRUSHPWM value    */
uint8_t GetBrushPwmCh2DutyCycle (void);                             /* Set BRUSHPWM value    */

//uint8_t GetCurrentBrushPwmDuty (void);               /* Actual pin out BRUSHPWM */

static void Error_Handler(void);

#endif