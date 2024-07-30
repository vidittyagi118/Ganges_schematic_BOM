#ifndef __ROTATE_SENSE1_H_
#define __ROTATE_SENSE1_H_

#include <stdbool.h>
#include <stdint.h>

#define ROTATE_SENSE1_GPIO               BOARD_ENCODER_GPIO
#define ROTATE_SENSE1_PORT               BOARD_ENCODER_PORT
#define ROTATE_SENSE1_GPIO_PIN           BOARD_ENCODER_MOTOR1_PIN

bool RotateSense1Init (void);

uint32_t GetRotateSense1CountValue (void);
uint32_t GetAcelRotateCountValue (void);
void SetRotateSense1Count (uint32_t count);
void ClearRotateSense1Count (void);
void ClearAcelRotateCount (void);
void StartRotateSense1Count (void);
void ResumeRotateSense1Count (void);
void StopRotateSense1Count (void);

void RotateSense1TimeIncrement_ms (void);
void Rotate_Sense1_Irq(void);

char GetRotateSense1State(void);

#endif
