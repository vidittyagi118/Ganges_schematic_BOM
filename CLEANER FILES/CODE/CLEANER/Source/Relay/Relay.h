#ifndef __RELAY_H_
#define __RELAY_H_

#include <stdbool.h>
#include <stdint.h>

#define RELAY_GPIO               BOARD_RELAY_GPIO
#define RELAY_PORT               BOARD_RELAY_PORT
#define RELAY_ON_GPIO_PIN        BOARD_RELAY_ON_GPIO_PIN
#define RELAY_OFF_GPIO_PIN       BOARD_RELAY_OFF_GPIO_PIN

typedef enum relayStatus_def
{
  RELAY_OFF,
  RELAY_ON,
  RELAY_UNKNOWN
} eRelayStatus;

bool RelayInit (void);
void DoRelayOn (void);
void DoRelayOff (void);
void DoInitialRelayOn (void);
void DoInitialRelayOff (void);
eRelayStatus GetRelayStatus (void);
void RelayTimeIncrement_ms (void);
void RelayFSM (void);

#endif
