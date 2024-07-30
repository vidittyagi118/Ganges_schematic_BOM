#include "Led3.h"
#include "board.h"
#include "Serial_Debug.h"
#include "switch.h"


/* LED Operating Timings

  |--|__|--|__|--|__________________|--|__|--|__|--|____________________

  |  |                                -> On Time
     |  |                             -> Off time
                 |                  | -> Long Off Time
  |              |                    -> No of Blinks (eg. 3)

*/

#define LED_START_TIME_MS         50
#define LED_ON_TIME_MS            0
#define LED_ON_TIME_ERROR_1_MS    100
#define LED_ON_TIME_ERROR_2_MS    100
#define LED_OFF_TIME_NORMAL_MS    100
#define LED_OFF_TIME_ERROR1_MS    100
#define LED_OFF_TIME_ERROR2_MS    100
#define LED_OFF_LONG_TIME_MS      1000
#define NO_OF_BLINKS_NORMAL           1
#define NO_OF_BLINKS_ERROR1           2
#define NO_OF_BLINKS_ERROR2           1

static void DoLedBlinkFSM(void);
static inline void LedTimerStop (void);
static inline bool IsLedTimeOver (void);
static inline void ClearLedTimeOver (void);
//static void SetLed3State (eLed3State ledState);
//static  eLed3State GetLed3State (void);
static void LedTimerOn (uint32_t setLed3TimeCount_ms);
static void LedTimerOn (uint32_t setLed3TimeCount_ms);
static eLed3OperateState  GetCurrentOperateState(void);
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

typedef enum eLed3State_def
{
  LED_IDLE,
  LED_START,
  LED_START_TIME_WAIT,
  LED_OPERATE
}eLed3State;

typedef struct stLedOperateParams_def {
  eLed3OperateState ledOperateState;
  uint8_t noOfSetBlink;
  uint32_t ledOnTime;
  uint32_t ledOffTime;
  uint32_t ledLongOffTime;    
}stLedOperateParams;


static uint8_t noOfSetBlink        = NO_OF_BLINKS_NORMAL;
static uint32_t ledOnTime          = LED_ON_TIME_MS;
static uint32_t ledOffTime         = LED_OFF_TIME_NORMAL_MS;
static uint32_t ledLongOffTime     = LED_OFF_LONG_TIME_MS;

#define DEFAULT_LED_STATE       false

static volatile bool ledTimeOver     = false;
static int32_t led_timecount         = 0;
static bool ledTestMode              = false;

static stLedOperateParams ledOperateParams[] = {
  {LED_3_INITIAL, NO_OF_BLINKS_NORMAL, LED_ON_TIME_MS,          LED_OFF_TIME_NORMAL_MS, LED_OFF_LONG_TIME_MS},
  {LED_3_NORMAL,  NO_OF_BLINKS_NORMAL, LED_ON_TIME_MS,          LED_OFF_TIME_NORMAL_MS, LED_OFF_LONG_TIME_MS},
  {LED_3_ERROR1,  NO_OF_BLINKS_ERROR1, LED_ON_TIME_ERROR_1_MS,  LED_OFF_TIME_ERROR1_MS, LED_OFF_LONG_TIME_MS},
  {LED_3_ERROR2,  NO_OF_BLINKS_ERROR2, LED_ON_TIME_ERROR_2_MS,  LED_OFF_TIME_ERROR2_MS, LED_OFF_LONG_TIME_MS}
};

int16_t VAL1 = -40;
int16_t VAL2 = -70;
int16_t VAL3 = -90;

void SetVAL1var(int16_t newValue)
{
  VAL1 = newValue;
//  Serial_Debug("\nVAL1 CHANGED");
//  Serial_Debug_Num(VAL1);
}

void SetVAL2var(int16_t newValue)
{
  VAL2 = newValue;
//  Serial_Debug("\nVAL2 CHANGED");
//  Serial_Debug_Num(VAL2);
}

void SetVAL3var(int16_t newValue)
{
  VAL3 = newValue;
//  Serial_Debug("\nVAL3 CHANGED");
//  Serial_Debug_Num(VAL3);
}

int16_t GetVAL1var(void)
{
  return VAL1;
}

int16_t GetVAL2var(void)
{
  return VAL2;
}

int16_t GetVAL3var(void)
{
  return VAL3;
}

void Led3InitPins(void)
{
  gpio_pin_config_t led_config = {
    kGPIO_DigitalOutput, 0,
  };
  
  GPIO_PinInit(LED_3_GPIO, LED_3_GPIO_PIN, &led_config);
#if DEFAULT_LED_STATE == true
  SwitchLed3On();
#else
  SwitchLed3Off();
#endif
} 

void SwitchLed3Off (void)
{
  GPIO_PortClear(LED_3_GPIO, 1U << LED_3_GPIO_PIN);
}

void SwitchLed3On (void)
{
  GPIO_PortSet(LED_3_GPIO, 1U << LED_3_GPIO_PIN); 
}

void SwitchLed3Toggle (void)
{
  GPIO_PortToggle(LED_3_GPIO, 1U << LED_3_GPIO_PIN); 
}

void OperateLed3 (void)
{
  static eLed3State redLedState = LED_IDLE;
  if(ledTestMode == true)
  {
    return;    
  }
  switch (redLedState)
  {
   case LED_IDLE:
  //  Led3InitPins();
    SwitchLed3On();
    redLedState = LED_START;
    break;
   case LED_START:
    SwitchLed3On();
    LedTimerOn(LED_START_TIME_MS);
    redLedState = LED_START_TIME_WAIT;
    break;
   case LED_START_TIME_WAIT:
    if(IsLedTimeOver() == true)
    {
      SwitchLed3Off();
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

static eLed3OperateState  GetCurrentOperateState(void)
{
  eLed3OperateState errorState = LED_3_NORMAL;
  if(GetSwitchState() == MANUAL_CONTROL)
  {
  errorState = LED_3_ERROR1;
  }
//  int16_t rssi = GetNwQuality();
//  if(rssi <= VAL1&& rssi > VAL2)         
//  {
//    errorState = LED_3_NORMAL;
//  }
//  else if(rssi <= VAL2 && rssi >= VAL3) 
//  {
//    errorState = LED_3_ERROR1;
//  }
//  else
//  {
//    errorState = LED_3_ERROR2;
//  }
 return errorState; 
}


static void DoLedOperate(void)
{
   static eLed3OperateState redLedPrevOperateState = LED_3_INITIAL;
   eLed3OperateState  redLedOperateState =  GetCurrentOperateState();
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
    SwitchLed3Off();
    redLedBlinkState = LED_BLINK_LED_ON;
    noOfBlink = 0;
    break;
   case LED_BLINK_LED_ON:
    if(ledOnTime)
    {
      SwitchLed3On();
      LedTimerOn(ledOnTime);
      redLedBlinkState = LED_ON_TIME_WAIT;
    }
    break;
   case LED_ON_TIME_WAIT:
    if(IsLedTimeOver() == true)
    {
      if(ledOffTime)
      {
        SwitchLed3Off();
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
          SwitchLed3Off();
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


static void LedTimerOn (uint32_t setLed3TimeCount_ms)
{
  if(setLed3TimeCount_ms == 0)
  {
    ledTimeOver = true;
    LedTimerStop();
  }
  else
  {
    led_timecount = setLed3TimeCount_ms;
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

void Led3TimeIncrement_ms (void)
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

void Led3Test (bool state)
{
  ledTestMode = true;
  if(state == true)
  {
    SwitchLed3On();
  }
  else
  {
    SwitchLed3Off();
  }   
}