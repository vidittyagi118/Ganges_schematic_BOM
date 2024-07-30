#include "RotateSense1.h"
#include "PortInterrupt.h"
#include "RobotOperation.h"
#include "fsl_port.h"
#include "fsl_gpio.h"
#include "board.h"
#include "RotateSenseCommon.h"

/* IMPORTANT -> Timer Interrupt And Port Interrupt should be of Same priority */

#define ROTATE_SENSE1_VALID_STATE        true

#define SENSE1_NOISE_TIME_MS             10    /* Minimum Time For the Signal to be constant for it to be considered as Valid */

static uint32_t rotateCount = 0;
static uint32_t AcelrotateCount = 0;
//static bool enableRotateSense1Sensor = true;

static volatile bool rotateSense1TimeOver      = false;
static int32_t rotateSense1_timecount          = 0;

static void RotateSense1TimerOn (uint32_t setRotateSense1TimeCount_ms);
static inline void RotateSense1TimerStop (void);
//static inline bool IsRotateSense1TimeOver (void);
//static inline void ClearRotateSense1TimeOver (void);
static inline void IncrementRotateCount (void);
static inline void IncrementAcelRotateCount (void);
static void EnableRotateSense1Sensor (void);
static void DisableRotateSense1Sensor (void);
static inline bool IsValidRotateSense1SensorState (void);
static void CheckAndIncrementRotateCount(void);
static void EnableInterrupt (void);
static void DisableInterrupt (void);
extern int Cumulative_distance,distance;

uint32_t prev_rotateCount = 0;

static inline bool IsValidRotateSense1SensorState (void)
{
   return (GPIO_PinRead(ROTATE_SENSE1_GPIO, ROTATE_SENSE1_GPIO_PIN) == ROTATE_SENSE1_VALID_STATE); 
} 

char GetRotateSense1State()
{
  if(IsValidRotateSense1SensorState())
  {
    return 1;
  }
  else
  {
    return 0;
  }
}

void Rotate_Sense1_Irq(void)
{
//  if(enableRotateSense1Sensor)
//  {
    if(IsValidRotateSense1SensorState() && IsRotateSensorEnabled())
    {
      RotateSense1TimerOn(SENSE1_NOISE_TIME_MS);
    }
    else
    {
      RotateSense1TimerStop();
    }
    GPIO_PortClearInterruptFlags(ROTATE_SENSE1_GPIO, 1U << ROTATE_SENSE1_GPIO_PIN);
 // }
}

bool RotateSense1Init (void)
{
  gpio_pin_config_t RotateSense1_config = {
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
  
  GPIO_PinInit(ROTATE_SENSE1_GPIO, ROTATE_SENSE1_GPIO_PIN, &RotateSense1_config);
  PORT_SetPinConfig(ROTATE_SENSE1_PORT, ROTATE_SENSE1_GPIO_PIN, &pull_down_Config);
  (void)IsValidRotateSense1SensorState();
  EnableInterrupt();
  return true;
}

static void EnableInterrupt (void)
{
  PORT_SetPinInterruptConfig(ROTATE_SENSE1_PORT, ROTATE_SENSE1_GPIO_PIN, kPORT_InterruptEitherEdge);
}

static void DisableInterrupt (void)
{
  PORT_SetPinInterruptConfig(ROTATE_SENSE1_PORT, ROTATE_SENSE1_GPIO_PIN, kPORT_InterruptOrDMADisabled); 
}

static void EnableRotateSense1Sensor (void)
{
   //enableRotateSense1Sensor = true;
   EnableInterrupt(); 
}

static void DisableRotateSense1Sensor (void)
{
   DisableInterrupt();
   //enableRotateSense1Sensor = false;
}

void StartRotateSense1Count (void)
{
  RotateSense1TimerStop();
  if(GetPrevPauseState() == PA_IDLE)
  {
  rotateCount = 0;
  }
  else
  {
  rotateCount = GetPrevRotateSense1Count();
  }
  GPIO_PortClearInterruptFlags(ROTATE_SENSE1_GPIO, 1U << ROTATE_SENSE1_GPIO_PIN);
  EnableRotateSense1Sensor();
}

void ResumeRotateSense1Count (void)
{
  EnableRotateSense1Sensor();
}


void StopRotateSense1Count (void)
{
  DisableRotateSense1Sensor();
}

static inline void IncrementRotateCount (void)
{
  rotateCount++;
  prev_rotateCount =  rotateCount;  
  Cumulative_distance += distance;
}

static inline void IncrementAcelRotateCount (void)
{
  AcelrotateCount++;  
}

uint32_t GetRotateSense1CountValue (void)
{
  return rotateCount;  
}

uint32_t GetAcelRotateCountValue (void)
{
  return AcelrotateCount;  
}

void ClearRotateSense1Count (void)
{
  rotateCount = 0;  
}

void ClearAcelRotateCount (void)
{
  AcelrotateCount = 0;  
}

void SetRotateSense1Count (uint32_t count)
{
  rotateCount = count;  
}

static void CheckAndIncrementRotateCount(void)
{
  if(IsValidRotateSense1SensorState())
  {
    IncrementRotateCount();
    if(GetRobotState() == ROBOT_ACCELERATION_SLOW_START)
    {
      IncrementAcelRotateCount();
    }
  }  
}

static void RotateSense1TimerOn (uint32_t setRotateSense1TimeCount_ms)
{
  if(setRotateSense1TimeCount_ms == 0)
  {
    rotateSense1TimeOver = true;
    RotateSense1TimerStop();
  }
  else
  {
    rotateSense1_timecount = setRotateSense1TimeCount_ms;
    rotateSense1TimeOver = false;
  }
}

static inline void RotateSense1TimerStop (void)
{
  rotateSense1_timecount = 0;
}

//static inline bool IsRotateSense1TimeOver (void)
//{
//  return (rotateSense1TimeOver);
//}
//
//static inline void ClearRotateSense1TimeOver (void)
//{
//  rotateSense1TimeOver = false; 
//}

void RotateSense1TimeIncrement_ms (void)
{
  if(rotateSense1_timecount)
  {
    if(--rotateSense1_timecount <= 0)
    {
      rotateSense1TimeOver = true;
      CheckAndIncrementRotateCount();
      RotateSense1TimerStop();
    }
    else
    {
      rotateSense1TimeOver = false;
    }
  }
}
