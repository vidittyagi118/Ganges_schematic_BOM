
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PWM1_CONFIG_H
#define __PWM1_CONFIG_H

#define SERIAL_DEBUG_PWM1

#define STOP_PWM_VALUE          0

#define PWM1_FTM_BASEADDR       FTM3
#define PWM1_CHANNEL1           6U
#define PWM1_CHANNEL2           7U

#define FTM_SOURCE_CLOCK CLOCK_GetFreq(kCLOCK_BusClk)


#endif