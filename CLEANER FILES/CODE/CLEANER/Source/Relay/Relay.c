#include "Relay.h"
#include "fsl_port.h"
#include "fsl_gpio.h"
#include "board.h"

#define RELAY_PULSE_TIME_MS           50    /* Minimum Time For the Signal to be constant for it to be considered as Valid */
#define RELAY_NO_PULSE_TIME_MS        50

typedef enum relayOpState_def
{
  RELAY_IDLE,
  RELAY_PULSE_ON,  
  RELAY_PULSE_ON_WAIT_TIME, 
  RELAY_PULSE_OFF,
  RELAY_PULSE_OFF_WAIT_TIME 
} eRelayOpState;

static eRelayOpState relayOpState       = RELAY_PULSE_OFF;

static eRelayStatus relayStatus         = RELAY_UNKNOWN;
static eRelayStatus relayTargetStatus   = RELAY_OFF;

static volatile bool relayTimeOver      = false;
static int32_t relay_timecount          = 0;

static void RelayTimerOn (uint32_t setRelayTimeCount_ms);
static inline void RelayTimerStop (void);
static inline bool IsRelayTimeOver (void);
static inline void ClearRelayTimeOver (void);

static void RelayOnPulse (void);
static void RelayOffPulse (void);
static void RelayNoPulse (void);

static void SetRelayStatus (eRelayStatus relState);
static void SetRelayTargetStatus (eRelayStatus relTarState);
static eRelayStatus GetRelayTargetStatus (void);

bool RelayInit (void)
{
  gpio_pin_config_t Relay_config = {
    kGPIO_DigitalOutput, 0,
  };
  
  GPIO_PinInit(RELAY_GPIO, RELAY_ON_GPIO_PIN, &Relay_config);
  GPIO_PinInit(RELAY_GPIO, RELAY_OFF_GPIO_PIN, &Relay_config);
  RelayNoPulse();
  DoRelayOff();
  return true;
}
 
static void RelayOnPulse (void)
{
  GPIO_PortClear(RELAY_GPIO, 1U << RELAY_OFF_GPIO_PIN);
  GPIO_PortSet(RELAY_GPIO, 1U << RELAY_ON_GPIO_PIN);
}
 
static void RelayOffPulse (void)
{
  GPIO_PortClear(RELAY_GPIO, 1U << RELAY_ON_GPIO_PIN);
  GPIO_PortSet(RELAY_GPIO, 1U << RELAY_OFF_GPIO_PIN);
}
 
static void RelayNoPulse (void)
{
  GPIO_PortClear(RELAY_GPIO, 1U << RELAY_ON_GPIO_PIN);
  GPIO_PortClear(RELAY_GPIO, 1U << RELAY_OFF_GPIO_PIN);
}

static void SetRelayTargetStatus (eRelayStatus relTarState)
{
  relayTargetStatus = relTarState;  
}

static eRelayStatus GetRelayTargetStatus (void)
{
  return relayTargetStatus;  
}
 
void RelayFSM (void)
{
  eRelayStatus tarRelayState = GetRelayTargetStatus();
  switch (relayOpState)
  {
   case RELAY_IDLE:
      if(tarRelayState == RELAY_ON)
      {
        RelayOnPulse();
        SetRelayStatus(RELAY_UNKNOWN);
        RelayTimerOn(RELAY_PULSE_TIME_MS);
        relayOpState = RELAY_PULSE_ON_WAIT_TIME;
      }
      else if(tarRelayState == RELAY_OFF)
      {
        RelayOffPulse();
        SetRelayStatus(RELAY_UNKNOWN);
        RelayTimerOn(RELAY_PULSE_TIME_MS);
        relayOpState = RELAY_PULSE_ON_WAIT_TIME;
      }
      else
      {
          /* Do Nothing. Here the target State is Relay_Known */
      }
   break;
   case RELAY_PULSE_ON_WAIT_TIME:
      if(IsRelayTimeOver())
      {
        if(tarRelayState == RELAY_ON)
        {
          SetRelayStatus(RELAY_ON);
        }
        else if(tarRelayState == RELAY_OFF)
        {
          SetRelayStatus(RELAY_OFF);          
        }
        else
        {
          SetRelayStatus(RELAY_UNKNOWN);
        }
        tarRelayState = RELAY_UNKNOWN;
        relayOpState = RELAY_PULSE_OFF;
      }
      else
      {
        break;
      }                                                         /* No Break Necessary */
   case RELAY_PULSE_OFF:
      RelayNoPulse();
      RelayTimerOn(RELAY_NO_PULSE_TIME_MS);
      relayOpState = RELAY_PULSE_OFF_WAIT_TIME;
   break;
   case RELAY_PULSE_OFF_WAIT_TIME:
      if(IsRelayTimeOver())
      {
        relayOpState = RELAY_IDLE;
      }
   break;
   default:
    break;
  }
}

void DoRelayOn (void)
{
  SetRelayTargetStatus(RELAY_ON);
  if(relayOpState != RELAY_IDLE)
  {
    relayOpState = RELAY_PULSE_OFF;
  }
}

void DoRelayOff (void)
{
  SetRelayTargetStatus(RELAY_OFF);
  if(relayOpState != RELAY_IDLE)
  {
    relayOpState = RELAY_PULSE_OFF;
  }
}

void DoInitialRelayOn (void)
{
  DoRelayOn();
  RelayFSM();
  while((GetRelayStatus() != RELAY_ON) || (relayOpState != RELAY_IDLE))
  {
    RelayFSM();
  }
}

/***  07S   ***/
void DoInitialRelayOff (void)
{
  DoRelayOff();
  RelayFSM();
  while((GetRelayStatus() != RELAY_OFF))
  {
    RelayFSM();
  }
}
/***  07S   ***/

static void SetRelayStatus (eRelayStatus relState)
{
  relayStatus = relState;  
}

eRelayStatus GetRelayStatus (void)
{
  return relayStatus;  
}

static inline bool IsRelayTimeOver (void)
{
  return (relayTimeOver);
}

static inline void ClearRelayTimeOver (void)
{
  relayTimeOver = false; 
}

static void RelayTimerOn (uint32_t setRelayTimeCount_ms)
{
  if(setRelayTimeCount_ms == 0)
  {
    relayTimeOver = true;
    RelayTimerStop();
  }
  else
  {
    relay_timecount = setRelayTimeCount_ms;
    relayTimeOver = false;
  }
}

static inline void RelayTimerStop (void)
{
  relay_timecount = 0;
}

void RelayTimeIncrement_ms (void)
{
  if(relay_timecount)
  {
    if(--relay_timecount <= 0)
    {
      relayTimeOver = true;
      RelayTimerStop();
    }
    else
    {
      ClearRelayTimeOver();
    }
  }
}
