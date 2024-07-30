#ifndef __EDGE_SENSE2_H_
#define __EDGE_SENSE2_H_

#include <stdbool.h>
#include <stdint.h>

#define EDGE_SENSE2_GPIO               BOARD_EDGESENSOR2_GPIO
#define EDGE_SENSE2_PORT               BOARD_EDGESENSOR2_PORT
#define EDGE_SENSE2_GPIO_PIN           BOARD_EDGESENSOR2_GPIO_PIN

bool EdgeSense2Init (void);

bool IsEdgeSense2Detected (void);
void SetEdgeSense2State (bool state);
void UpdateSense2State(void);
void EdgeSense2TimeIncrement_ms (void);
void Edge_Sense2_Irq(void);

char GetEdgeSense2State(void);

#endif
