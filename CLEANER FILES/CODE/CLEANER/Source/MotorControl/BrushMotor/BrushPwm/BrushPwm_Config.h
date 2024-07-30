
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BRUSHPWM_CONFIG_H
#define __BRUSHPWM_CONFIG_H

#define SERIAL_DEBUG_BRUSHPWM

#define STOP_PWM_VALUE          0

#define BRUSHPWM_FTM_BASEADDR       FTM2
#define BRUSHPWM_CHANNEL1           0U
#define BRUSHPWM_CHANNEL2           1U

#define FTM_SOURCE_CLOCK CLOCK_GetFreq(kCLOCK_BusClk)


#endif