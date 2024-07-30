#ifndef __SWITCH_H_
#define __SWITCH_H_

typedef enum {AUTO_CONTROL,MANUAL_CONTROL}eControlModes;

#include <stdbool.h>
#include <stdint.h>

#define SWITCH_GPIO               MANUAL_CONTROL_SWITCH_GPIO
#define SWITCH_PORT               MANUAL_CONTROL_SWITCH_PORT
#define SWITCH_PIN                MANUAL_CONTROL_SWITCH_PIN

#define SWITCH_VALID_STATE        true

void InitSwitch(void);
eControlModes GetSwitchState(void);
#endif