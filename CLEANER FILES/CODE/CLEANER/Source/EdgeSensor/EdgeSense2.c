#include "EdgeSense2.h"
#include "PortInterrupt.h"
#include "fsl_port.h"
#include "fsl_gpio.h"
#include "board.h"

/* IMPORTANT -> Timer Interrupt And Port Interrupt should be of Same priority */

#define EDGE_SENSE2_VALID_STATE        true

#define SENSE2_NOISE_TIME_MS           200    /* Minimum Time For the Signal to be constant for it to be considered as Valid */

static bool edgeSense2Flag = false;

static volatile bool edgeSense2TimeOver      = false;
static int32_t edgeSense2_timecount          = 0;

static bool edgesensor2IntState = false;

static void EdgeSense2TimerOn (uint32_t setEdgeSense2TimeCount_ms);
static inline void EdgeSense2TimerStop (void);
static inline void SetEdgeSense2Flag (void);
static inline void ClearEdgeSense2Flag (void);
static inline bool IsValidEdgeSense2SensorState (bool state);
static void CheckAndSetEdgeSense2(void);
static void EnableInterrupt (void);
static void DisableInterrupt (void);
static void UpdateEdgeSense2 (bool state);

char Reverse_End = 0;

static inline bool IsValidEdgeSense2SensorState (bool state)
{
   return (state == EDGE_SENSE2_VALID_STATE); 
} 

static inline bool ReadEdgeSense2SensorState (void)
{
  return GPIO_PinRead(EDGE_SENSE2_GPIO, EDGE_SENSE2_GPIO_PIN);
  Reverse_End = 1;
}
  
char GetEdgeSense2State()
{
  if(ReadEdgeSense2SensorState ())
  {
    return 1;
  }
  else
  {
    return 0;
  }
 }

void Edge_Sense2_Irq(void)
{
    edgesensor2IntState = ReadEdgeSense2SensorState();
    EdgeSense2TimerOn(SENSE2_NOISE_TIME_MS);
    GPIO_PortClearInterruptFlags(EDGE_SENSE2_GPIO, 1U << EDGE_SENSE2_GPIO_PIN);
}

bool EdgeSense2Init (void)
{
  gpio_pin_config_t EdgeSense2_config = {
    kGPIO_DigitalInput, 0,
  };
  
  port_pin_config_t pull_up_Config = {
    kPORT_PullUp,
    kPORT_SlowSlewRate,
    kPORT_PassiveFilterDisable,
    kPORT_OpenDrainDisable,
    kPORT_HighDriveStrength,
    kPORT_MuxAsGpio,
    kPORT_UnlockRegister,     
  }; 
  
  GPIO_PinInit(EDGE_SENSE2_GPIO, EDGE_SENSE2_GPIO_PIN, &EdgeSense2_config);
  PORT_SetPinConfig(EDGE_SENSE2_PORT, EDGE_SENSE2_GPIO_PIN, &pull_up_Config);
  DisableInterrupt();
  UpdateSense2State();
  EnableInterrupt();
  return true;
}

void UpdateSense2State (void)
{
  edgesensor2IntState = ReadEdgeSense2SensorState();
  UpdateEdgeSense2(edgesensor2IntState);   
}

static void EnableInterrupt (void)
{
 PORT_SetPinInterruptConfig(EDGE_SENSE2_PORT, EDGE_SENSE2_GPIO_PIN, kPORT_InterruptEitherEdge);
}

static void DisableInterrupt (void)
{
 PORT_SetPinInterruptConfig(EDGE_SENSE2_PORT, EDGE_SENSE2_GPIO_PIN, kPORT_InterruptOrDMADisabled);
}

static inline void SetEdgeSense2Flag (void)
{
  edgeSense2Flag = true;  
}

static inline void ClearEdgeSense2Flag (void)
{
  edgeSense2Flag = false;  
}

bool IsEdgeSense2Detected (void)
{
  return edgeSense2Flag;
}

void SetEdgeSense2State (bool state)
{
  edgeSense2Flag = state;  
}

static void UpdateEdgeSense2 (bool state)
{
  if(IsValidEdgeSense2SensorState(state))
  {
    SetEdgeSense2Flag();
  }
  else
  {
    ClearEdgeSense2Flag();
  } 
}

static void CheckAndSetEdgeSense2(void)
{
  bool curState = ReadEdgeSense2SensorState();
  if(edgesensor2IntState == curState)
  {
    UpdateEdgeSense2(curState);
  }
}

static void EdgeSense2TimerOn (uint32_t setEdgeSense2TimeCount_ms)
{
  if(setEdgeSense2TimeCount_ms == 0)
  {
    edgeSense2TimeOver = true;
    EdgeSense2TimerStop();
  }
  else
  {
    edgeSense2_timecount = setEdgeSense2TimeCount_ms;
    edgeSense2TimeOver = false;
  }
}

static inline void EdgeSense2TimerStop (void)
{
  edgeSense2_timecount = 0;
}

//static inline bool IsEdgeSense2TimeOver (void)
//{
//  return (edgeSense2TimeOver);
//}
//
//static inline void ClearEdgeSense2TimeOver (void)
//{
//  edgeSense2TimeOver = false; 
//}

void EdgeSense2TimeIncrement_ms (void)
{
  if(edgeSense2_timecount)
  {
    if(--edgeSense2_timecount <= 0)
    {
      edgeSense2TimeOver = true;
      CheckAndSetEdgeSense2();
      EdgeSense2TimerStop();
    }
    else
    {
      edgeSense2TimeOver = false;
    }
  }
}
