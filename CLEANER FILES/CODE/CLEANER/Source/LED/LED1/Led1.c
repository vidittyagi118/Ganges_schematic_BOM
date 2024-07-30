#include "Led1.h"
#include "board.h"
#include "Serial_Debug.h"



/* LED Operating Timings

  |--|__|--|__|--|__________________|--|__|--|__|--|____________________

  |  |                                -> On Time
     |  |                             -> Off time
                 |                  | -> Long Off Time
  |              |                    -> No of Blinks (eg. 3)

*/

#define LED_START_TIME_MS         1000
#define LED_ON_TIME_MS            100
#define LED_OFF_TIME_NORMAL_MS    100
#define LED_OFF_TIME_ERROR1_MS    800
#define LED_OFF_TIME_ERROR2_MS    2800
#define LED_OFF_LONG_TIME_MS      1000
#define NO_OF_BLINKS_NORMAL           1
#define NO_OF_BLINKS_ERROR1           1
#define NO_OF_BLINKS_ERROR2           1

#define DEFAULT_LED_STATE       true

static void DoLedBlinkFSM(void);
static inline void LedTimerStop (void);
static inline bool IsLedTimeOver (void);
static inline void ClearLedTimeOver (void);
//static void SetLed1State (eLed1State ledState);
//static  eLed1State GetLed1State (void);
static void LedTimerOn (uint32_t setLed1TimeCount_ms);
static void LedTimerOn (uint32_t setLed1TimeCount_ms);
static eLed1OperateState  GetCurrentOperateState(void);
static void DoLedOperate(void);

typedef enum eLEDState_def
{
  LED_OFF,
  LED_ON
}eLEDState;

typedef enum eLedBlinkState_def
{
  LED_BLINK_IDLE,
  LED_BLINK_LED_ON,
  LED_ON_TIME_WAIT,
  LED_OFF_TIME_WAIT,
  LED_OFF_LONG_TIME_WAIT
}eLedBlinkState;

typedef enum eLed1State_def
{
  LED_IDLE,
  LED_START,
  LED_START_TIME_WAIT,
  LED_OPERATE
}eLed1State;

typedef struct stLedOperateParams_def {
  eLed1OperateState ledOperateState;
  uint8_t noOfSetBlink;
  uint32_t ledOnTime;
  uint32_t ledOffTime;
  uint32_t ledLongOffTime;    
}stLedOperateParams;


static uint8_t noOfSetBlink        = NO_OF_BLINKS_NORMAL;
static uint32_t ledOnTime          = LED_ON_TIME_MS;
static uint32_t ledOffTime         = LED_OFF_TIME_NORMAL_MS;
static uint32_t ledLongOffTime     = LED_OFF_LONG_TIME_MS;

static volatile bool ledTimeOver     = false;
static int32_t led_timecount         = 0;
static bool ledTestMode              = false;

static stLedOperateParams ledOperateParams[] = {
  {LED_1_INITIAL, NO_OF_BLINKS_NORMAL, LED_ON_TIME_MS, LED_OFF_TIME_NORMAL_MS, LED_OFF_LONG_TIME_MS},
  {LED_1_NORMAL,  NO_OF_BLINKS_NORMAL, LED_ON_TIME_MS, LED_OFF_TIME_NORMAL_MS, LED_OFF_LONG_TIME_MS},
  {LED_1_ERROR1,  NO_OF_BLINKS_ERROR1, LED_ON_TIME_MS, LED_OFF_TIME_ERROR1_MS, LED_OFF_LONG_TIME_MS},
  {LED_1_ERROR2,  NO_OF_BLINKS_ERROR2, LED_ON_TIME_MS, LED_OFF_TIME_ERROR2_MS, LED_OFF_LONG_TIME_MS}
};

void Led1InitPins(void)
{
  gpio_pin_config_t led_config = {
    kGPIO_DigitalOutput, 0,
  };
  
  GPIO_PinInit(LED_1_GPIO, LED_1_GPIO_PIN, &led_config);
#if DEFAULT_LED_STATE == true
  SwitchLed1On();
#else
  SwitchLed1Off();
#endif
} 

void SwitchLed1Off (void)
{
  GPIO_PortClear(LED_1_GPIO, 1U << LED_1_GPIO_PIN);
}

void SwitchLed1On (void)
{
  GPIO_PortSet(LED_1_GPIO, 1U << LED_1_GPIO_PIN); 
}

void SwitchLed1Toggle (void)
{
  GPIO_PortToggle(LED_1_GPIO, 1U << LED_1_GPIO_PIN); 
}

void OperateLed1 (void)
{
  static eLed1State redLedState = LED_IDLE;
  if(ledTestMode == true)
  {
    return;    
  }
  switch (redLedState)
  {
   case LED_IDLE:
  //  Led1InitPins();
    SwitchLed1On();
    redLedState = LED_START;
    break;
   case LED_START:
    SwitchLed1On();
    LedTimerOn(LED_START_TIME_MS);
    redLedState = LED_START_TIME_WAIT;
    break;
   case LED_START_TIME_WAIT:
    if(IsLedTimeOver() == true)
    {
      SwitchLed1Off();
      redLedState = LED_OPERATE;
    }
    break;
   case LED_OPERATE:
        DoLedOperate();
    break;
   default:
    break;
  }
}

static eLed1OperateState  GetCurrentOperateState(void)
{
  eLed1OperateState errorState = LED_1_NORMAL;
//  if(SdcardPresent()  == true && GSMStatus() == true && getPowerState() != LOW_BATTERY)
//  {
//    errorState = LED_1_NORMAL;
//  }
//  else if(GSMStatus() == true && getPowerState() != LOW_BATTERY)
//  {
//    errorState = LED_1_ERROR1;
//  }
//  else
//  {
//    errorState = LED_1_ERROR2;
//  }
 return errorState; 
}

static void DoLedOperate(void)
{
   static eLed1OperateState redLedPrevOperateState = LED_1_INITIAL;
   eLed1OperateState  redLedOperateState =  GetCurrentOperateState();
   if(redLedPrevOperateState == redLedOperateState)
   {
     DoLedBlinkFSM(); 
     return;
   }
   else
   {
     redLedPrevOperateState = redLedOperateState;     
   }
//  Serial_Debug("\nLED Operate State :");
//  Serial_Debug_Num(redLedOperateState);   
  uint16_t ledOpParamsCount = 0;
  uint16_t ledOpParamsTotalCount = sizeof(ledOperateParams)/sizeof(ledOperateParams[0]);
  for(ledOpParamsCount =0; ledOpParamsCount < ledOpParamsTotalCount; ledOpParamsCount++)
  {
    if(ledOperateParams[ledOpParamsCount].ledOperateState == redLedOperateState)
    {
      noOfSetBlink          = ledOperateParams[ledOpParamsCount].noOfSetBlink;
      ledOnTime          = ledOperateParams[ledOpParamsCount].ledOnTime;
      ledOffTime         = ledOperateParams[ledOpParamsCount].ledOffTime;
      ledLongOffTime     = ledOperateParams[ledOpParamsCount].ledLongOffTime;
      break;
    }
  }
  if(ledOpParamsCount >= ledOpParamsTotalCount)
  {
    noOfSetBlink          = NO_OF_BLINKS_NORMAL;
    ledOnTime          = LED_ON_TIME_MS;
    ledOffTime         = LED_OFF_TIME_NORMAL_MS;
    ledLongOffTime     = LED_OFF_LONG_TIME_MS;
  }
  DoLedBlinkFSM(); 
}

static void DoLedBlinkFSM(void)
{
  static eLedBlinkState redLedBlinkState = LED_BLINK_IDLE;
  static uint8_t noOfBlink = 0;
  switch(redLedBlinkState)
  {
   case LED_BLINK_IDLE:
    SwitchLed1Off();
    redLedBlinkState = LED_BLINK_LED_ON;
    noOfBlink = 0;
    break;
   case LED_BLINK_LED_ON:
    if(ledOnTime)
    {
      SwitchLed1On();
      LedTimerOn(ledOnTime);
      redLedBlinkState = LED_ON_TIME_WAIT;
    }
    break;
   case LED_ON_TIME_WAIT:
    if(IsLedTimeOver() == true)
    {
      if(ledOffTime)
      {
        SwitchLed1Off();
        LedTimerOn(ledOffTime);
        redLedBlinkState = LED_OFF_TIME_WAIT;
      }
    }
    break;
   case LED_OFF_TIME_WAIT:
    if(IsLedTimeOver() == true)
    {
      if(++noOfBlink < noOfSetBlink)
      {
        redLedBlinkState = LED_BLINK_LED_ON;
      }
      else
      {
        if(ledLongOffTime)
        {
          SwitchLed1Off();
          LedTimerOn(ledLongOffTime);
          redLedBlinkState = LED_OFF_LONG_TIME_WAIT;
        }
        else
        {
          redLedBlinkState = LED_BLINK_IDLE;
        }
      }
    }
    break;
   case LED_OFF_LONG_TIME_WAIT:
    if(IsLedTimeOver() == true)
    {
      redLedBlinkState = LED_BLINK_IDLE;
    }
    break;
   default:
    break;
  }
}


static void LedTimerOn (uint32_t setLed1TimeCount_ms)
{
  if(setLed1TimeCount_ms == 0)
  {
    ledTimeOver = true;
    LedTimerStop();
  }
  else
  {
    led_timecount = setLed1TimeCount_ms;
    ledTimeOver = false;
  }
}

static inline void LedTimerStop (void)
{
  led_timecount = 0;
}

static inline bool IsLedTimeOver (void)
{
  return (ledTimeOver);
}

static inline void ClearLedTimeOver (void)
{
  ledTimeOver = false; 
}

void Led1TimeIncrement_ms (void)
{
  if(led_timecount)
  {
    if(--led_timecount <= 0)
    {
      ledTimeOver = true;
      LedTimerStop();
    }
    else
    {
      ClearLedTimeOver();
    }
  }
}

void Led1Test (bool state)
{
  ledTestMode = true;
  if(state == true)
  {
    SwitchLed1On();
  }
  else
  {
    SwitchLed1Off();
  }   
}