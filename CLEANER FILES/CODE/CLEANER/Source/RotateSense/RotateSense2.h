#ifndef __ROTATE_SENSE2_H_
#define __ROTATE_SENSE2_H_

#include <stdbool.h>
#include <stdint.h>

#define ROTATE_SENSE2_GPIO               BOARD_ENCODER_GPIO
#define ROTATE_SENSE2_PORT               BOARD_ENCODER_PORT
#define ROTATE_SENSE2_GPIO_PIN           BOARD_ENCODER_MOTOR2_PIN

bool RotateSense2Init (void);

uint32_t GetRotateSense2CountValue (void);
uint32_t GetBrushRotateSense2CountValue (void);
void SetRotateSense2Count (uint32_t count);
void ClearRotateSense2Count (void);
void ClearBrushRotateSense2Count (void);
void StartRotateSense2Count (void);
void ResumeRotateSense2Count (void);
void StopRotateSense2Count (void);

void RotateSense2TimeIncrement_ms (void);
void Rotate_Sense2_Irq(void);

char GetRotateSense2State(void);

#endif
