#include "EdgeSense1.h"
#include "PortInterrupt.h"
#include "fsl_port.h"
#include "fsl_gpio.h"
#include "board.h"

/* IMPORTANT -> Timer Interrupt And Port Interrupt should be of Same priority */
#define EDGE_SENSE1_VALID_STATE        true

#define SENSE1_NOISE_TIME_MS           200    /* Minimum Time For the Signal to be constant for it to be considered as Valid */

static bool edgeSense1Flag = false;

static volatile bool edgeSense1TimeOver      = false;
static int32_t edgeSense1_timecount          = 0;

static bool edgesensor1IntState = false;

static void EdgeSense1TimerOn (uint32_t setEdgeSense1TimeCount_ms);
static inline void EdgeSense1TimerStop (void);
static inline void SetEdgeSense1Flag (void);
static inline void ClearEdgeSense1Flag (void);
static inline bool IsValidEdgeSense1SensorState (bool state);
static void CheckAndSetEdgeSense1(void);
static void EnableInterrupt (void);
static void DisableInterrupt (void);
static void UpdateEdgeSense1 (bool state);

char Forward_End = 0;

static inline bool IsValidEdgeSense1SensorState (bool state)
{
   return (state == EDGE_SENSE1_VALID_STATE); 
} 

static inline bool ReadEdgeSense1SensorState (void)
{
  return GPIO_PinRead(EDGE_SENSE1_GPIO, EDGE_SENSE1_GPIO_PIN);
  Forward_End = 1;
}
  
char GetEdgeSense1State()
{
  if(ReadEdgeSense1SensorState ())
  {
    return 1;
  }
  else
  {
    return 0;
  }
 }


void Edge_Sense1_Irq (void)
{
    edgesensor1IntState = ReadEdgeSense1SensorState();
    EdgeSense1TimerOn(SENSE1_NOISE_TIME_MS);
    GPIO_PortClearInterruptFlags(EDGE_SENSE1_GPIO, 1U << EDGE_SENSE1_GPIO_PIN);
}

bool EdgeSense1Init (void)
{
  gpio_pin_config_t EdgeSense1_config = {
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
  
  GPIO_PinInit(EDGE_SENSE1_GPIO, EDGE_SENSE1_GPIO_PIN, &EdgeSense1_config);
  PORT_SetPinConfig(EDGE_SENSE1_PORT, EDGE_SENSE1_GPIO_PIN, &pull_up_Config);
  DisableInterrupt();
  UpdateSense1State();
  EnableInterrupt();
  return true;
}

void UpdateSense1State (void)
{
  edgesensor1IntState = ReadEdgeSense1SensorState();
  UpdateEdgeSense1(edgesensor1IntState);   
}

static void EnableInterrupt (void)
{
 PORT_SetPinInterruptConfig(EDGE_SENSE1_PORT, EDGE_SENSE1_GPIO_PIN, kPORT_InterruptEitherEdge);
}

static void DisableInterrupt (void)
{
 PORT_SetPinInterruptConfig(EDGE_SENSE1_PORT, EDGE_SENSE1_GPIO_PIN, kPORT_InterruptOrDMADisabled);
}

static inline void SetEdgeSense1Flag (void)
{
  edgeSense1Flag = true;  
}

static inline void ClearEdgeSense1Flag (void)
{
  edgeSense1Flag = false;  
}

bool IsEdgeSense1Detected (void)
{
  return edgeSense1Flag;
}

void SetEdgeSense1State (bool state)
{
  edgeSense1Flag = state;  
}

static void UpdateEdgeSense1 (bool state)
{
  if(IsValidEdgeSense1SensorState(state))
  {
    SetEdgeSense1Flag();
  }
  else
  {
    ClearEdgeSense1Flag();
  } 
}

static void CheckAndSetEdgeSense1(void)
{
  bool curState = ReadEdgeSense1SensorState();
  if(edgesensor1IntState == curState)
  {
    UpdateEdgeSense1(curState);
  }
}

static void EdgeSense1TimerOn (uint32_t setEdgeSense1TimeCount_ms)
{
  if(setEdgeSense1TimeCount_ms == 0)
  {
    edgeSense1TimeOver = true;
    EdgeSense1TimerStop();
  }
  else
  {
    edgeSense1_timecount = setEdgeSense1TimeCount_ms;
    edgeSense1TimeOver = false;
  }
}

static inline void EdgeSense1TimerStop (void)
{
  edgeSense1_timecount = 0;
}

//static inline bool IsEdgeSense1TimeOver (void)
//{
//  return (edgeSense1TimeOver);
//}
//
//static inline void ClearEdgeSense1TimeOver (void)
//{
//  edgeSense1TimeOver = false; 
//}

void EdgeSense1TimeIncrement_ms (void)
{
  if(edgeSense1_timecount)
  {
    if(--edgeSense1_timecount <= 0)
    {
      edgeSense1TimeOver = true;
      CheckAndSetEdgeSense1();
      EdgeSense1TimerStop();
    }
    else
    {
      edgeSense1TimeOver = false;
    }
  }
}
