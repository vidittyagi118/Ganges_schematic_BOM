#ifndef __EDGE_SENSE1_H_
#define __EDGE_SENSE1_H_

#include <stdbool.h>
#include <stdint.h>

#define EDGE_SENSE1_GPIO               BOARD_EDGESENSOR1_GPIO
#define EDGE_SENSE1_PORT               BOARD_EDGESENSOR1_PORT
#define EDGE_SENSE1_GPIO_PIN           BOARD_EDGESENSOR1_GPIO_PIN

bool EdgeSense1Init (void);

bool IsEdgeSense1Detected (void);
void SetEdgeSense1State (bool state);
void UpdateSense1State (void);
void EdgeSense1TimeIncrement_ms (void);
void Edge_Sense1_Irq (void);

char GetEdgeSense1State(void);

#endif
