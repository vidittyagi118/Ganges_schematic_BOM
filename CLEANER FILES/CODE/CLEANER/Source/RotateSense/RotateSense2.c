#include "RotateSense2.h"
#include "PortInterrupt.h"
#include "RobotOperation.h"
#include "fsl_port.h"
#include "fsl_gpio.h"
#include "board.h"
#include "RotateSenseCommon.h"

/* IMPORTANT -> Timer Interrupt And Port Interrupt should be of Same priority */

#define ROTATE_SENSE2_VALID_STATE        true

#define SENSE2_NOISE_TIME_MS             10    /* Minimum Time For the Signal to be constant for it to be considered as Valid */

static uint32_t rotateCount = 0;
static uint32_t Brush_rotateCount = 0;
//static bool enableRotateSense2Sensor = true;

static volatile bool rotateSense2TimeOver      = false;
static int32_t rotateSense2_timecount          = 0;

static void RotateSense2TimerOn (uint32_t setRotateSense2TimeCount_ms);
static inline void RotateSense2TimerStop (void);
//static inline bool IsRotateSense2TimeOver (void);
//static inline void ClearRotateSense2TimeOver (void);
static inline void IncrementRotateCount (void);
static inline void IncrementBrushRotateCount (void);
static void EnableRotateSense2Sensor (void);
static void DisableRotateSense2Sensor (void);
static inline bool IsValidRotateSense2SensorState (void);
static void CheckAndIncrementRotateCount(void);
static void CheckAndIncrementBrushRotateCount(void);

static void EnableInterrupt (void);
static void DisableInterrupt (void);

static inline bool IsValidRotateSense2SensorState (void)
{
   return (GPIO_PinRead(ROTATE_SENSE2_GPIO, ROTATE_SENSE2_GPIO_PIN) == ROTATE_SENSE2_VALID_STATE); 
} 

char GetRotateSense2State()
{
  if(IsValidRotateSense2SensorState())
  {
    return 1;
  }
  else
  {
    return 0;
  }
}

void Rotate_Sense2_Irq(void)
{
//  if(enableRotateSense2Sensor)
//  {
    if(IsValidRotateSense2SensorState() && IsRotateSensorEnabled())
    {
      RotateSense2TimerOn(SENSE2_NOISE_TIME_MS);
    }
    else
    {
      RotateSense2TimerStop();
    }
    GPIO_PortClearInterruptFlags(ROTATE_SENSE2_GPIO, 1U << ROTATE_SENSE2_GPIO_PIN);
//  }
}

bool RotateSense2Init (void)
{
  gpio_pin_config_t RotateSense2_config = {
    kGPIO_DigitalInput, 0,
  };
  
  port_pin_config_t pull_down_Config = {
    kPORT_PullDown,
    kPORT_SlowSlewRate,
    kPORT_PassiveFilterDisable,
    kPORT_OpenDrainDisable,
    kPORT_HighDriveStrength,
    kPORT_MuxAsGpio,
    kPORT_UnlockRegister,     
  };
  
  GPIO_PinInit(ROTATE_SENSE2_GPIO, ROTATE_SENSE2_GPIO_PIN, &RotateSense2_config);
  PORT_SetPinConfig(ROTATE_SENSE2_PORT, ROTATE_SENSE2_GPIO_PIN, &pull_down_Config);
  (void)IsValidRotateSense2SensorState();
  EnableInterrupt();
  return true;
}

static void EnableInterrupt (void)
{
  PORT_SetPinInterruptConfig(ROTATE_SENSE2_PORT, ROTATE_SENSE2_GPIO_PIN, kPORT_InterruptEitherEdge);
}

static void DisableInterrupt (void)
{
  PORT_SetPinInterruptConfig(ROTATE_SENSE2_PORT, ROTATE_SENSE2_GPIO_PIN, kPORT_InterruptOrDMADisabled); 
}

static void EnableRotateSense2Sensor (void)
{
 //  enableRotateSense2Sensor = true;
   EnableInterrupt(); 
}

static void DisableRotateSense2Sensor (void)
{
   DisableInterrupt();
 //  enableRotateSense2Sensor = false;
}

void StartRotateSense2Count (void)
{
  RotateSense2TimerStop();
  
  if(GetPrevPauseState() == PA_IDLE)
  {
  rotateCount = 0;
  }
  else
  {
  rotateCount = GetPrevRotateSense2Count();
  }
  GPIO_PortClearInterruptFlags(ROTATE_SENSE2_GPIO, 1U << ROTATE_SENSE2_GPIO_PIN);
  EnableRotateSense2Sensor();
}

void ResumeRotateSense2Count (void)
{
  EnableRotateSense2Sensor();
}


void StopRotateSense2Count (void)
{
  DisableRotateSense2Sensor();
}

static inline void IncrementRotateCount (void)
{
  rotateCount++;  
}

static inline void IncrementBrushRotateCount (void)
{
  Brush_rotateCount++;  
}

uint32_t GetRotateSense2CountValue (void)
{
  return rotateCount;  
}

uint32_t GetBrushRotateSense2CountValue (void)
{
  return Brush_rotateCount;  
}

void ClearRotateSense2Count (void)
{
  rotateCount = 0;  
}
void ClearBrushRotateSense2Count (void)
{
  Brush_rotateCount = 0;  
}

void SetRotateSense2Count (uint32_t count)
{
  rotateCount = count;  
}

static void CheckAndIncrementRotateCount(void)
{
  if(IsValidRotateSense2SensorState())
  {
    IncrementRotateCount();
    IncrementBrushRotateCount();
  }  
}

static void RotateSense2TimerOn (uint32_t setRotateSense2TimeCount_ms)
{
  if(setRotateSense2TimeCount_ms == 0)
  {
    rotateSense2TimeOver = true;
    RotateSense2TimerStop();
  }
  else
  {
    rotateSense2_timecount = setRotateSense2TimeCount_ms;
    rotateSense2TimeOver = false;
  }
}

static inline void RotateSense2TimerStop (void)
{
  rotateSense2_timecount = 0;
}

//static inline bool IsRotateSense2TimeOver (void)
//{
//  return (rotateSense2TimeOver);
//}
//
//static inline void ClearRotateSense2TimeOver (void)
//{
//  rotateSense2TimeOver = false; 
//}

void RotateSense2TimeIncrement_ms (void)
{
  if(rotateSense2_timecount)
  {
    if(--rotateSense2_timecount <= 0)
    {
      rotateSense2TimeOver = true;
      CheckAndIncrementRotateCount();
      //CheckAndIncrementBrushRotateCount();
      RotateSense2TimerStop();
    }
    else
    {
      rotateSense2TimeOver = false;
    }
  }
}
