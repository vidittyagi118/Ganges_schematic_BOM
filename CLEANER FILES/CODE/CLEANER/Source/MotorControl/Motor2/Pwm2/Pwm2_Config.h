
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PWM2_CONFIG_H
#define __PWM2_CONFIG_H

#define SERIAL_DEBUG_PWM2

#define STOP_PWM_VALUE          0

#define PWM2_FTM_BASEADDR       FTM0
#define PWM2_CHANNEL1           7U
#define PWM2_CHANNEL2           6U

#define FTM_SOURCE_CLOCK CLOCK_GetFreq(kCLOCK_BusClk)


#endif