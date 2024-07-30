#include "adc.h"

#include "board.h"
#include "fsl_adc16.h"
#include "fsl_vref.h"
#include "RobotControl.h"

#define ADC_IMOT1_BASE_ADDR ADC0
#define ADC_IMOT1_CHANNEL_GROUP 0U
#define ADC_IMOT1_CHANNEL 19U

#define ADC_IMOT2_BASE_ADDR ADC1
#define ADC_IMOT2_CHANNEL_GROUP 0U
#define ADC_IMOT2_CHANNEL 0U

#define ADC_IMOT3_BASE_ADDR ADC1
#define ADC_IMOT3_CHANNEL_GROUP 0U
#define ADC_IMOT3_CHANNEL 19U

#define ADC0_IRQn ADC0_IRQn
#define ADC0_IRQ_HANDLER_FUNC ADC0_IRQHandler

#define ADC1_IRQn ADC1_IRQn
#define ADC1_IRQ_HANDLER_FUNC ADC1_IRQHandler

#define Serial_Debug (void)

//#define ADC_MAX_SAMPLES 100

#define ADC_REF_VOLTAGE                 (1.2f)                                                /* adc Vref Using Internal Vref Generator */
#define ADC_MAX_VALUE                   (4095)                                                /* adc Max Value for 12 bit */
#define ADC_VOLT_CALC(x)                ((x*ADC_REF_VOLTAGE)/ADC_MAX_VALUE)                   /* Formula to Calculate input volatge at ADC pin */
#define ADC_MAX_SAMPLES                 1000

#define ADC_ILOAD_CALC(x)               ((ADC_VOLT_CALC(x)-(0.005*19.8))/(19.8*0.01))        /* Formula to Calculate Motor Load Current(with reference to datasheet pg.25) */

#define MAX_OFFSET_VALUE_TOLERANCE       (0.8f)

typedef enum eADCChannel_def
{
  IMOT1,
  IMOT2,
  IMOT3  
}eADCChannel;

typedef struct stADCChannelgroupdef
{
  eADCChannel adcChannel;
  ADC_Type *AdcBaseAddr;
  uint16_t adcChannelGroup;
  uint16_t adcChannelNo;
}stADCChannelGroup;

stADCChannelGroup ADCChannelGroup[] = {
  
  {IMOT1, ADC_IMOT1_BASE_ADDR, ADC_IMOT1_CHANNEL_GROUP, ADC_IMOT1_CHANNEL},
  {IMOT2, ADC_IMOT2_BASE_ADDR, ADC_IMOT2_CHANNEL_GROUP, ADC_IMOT2_CHANNEL},  
  {IMOT3, ADC_IMOT3_BASE_ADDR, ADC_IMOT3_CHANNEL_GROUP, ADC_IMOT3_CHANNEL}  
};

static adc16_channel_config_t adcConfigStruct; 

static volatile bool adc0ConversionDoneFlag = false;
static volatile bool adc1ConversionDoneFlag = false;

//static eADCChannel conversionChannel = IMOT1;
static uint16_t conversionChannelGroup = ADC_IMOT1_CHANNEL_GROUP;

static float Imot1AdcValue, Imot2AdcValue, Imot3AdcValue;
static volatile uint32_t ADC0SampleValue = 0;
static volatile uint32_t ADC1SampleValue = 0;

static float Imot1OffsetValue = 0;
static float Imot2OffsetValue = 0;
static float Imot3OffsetValue = 0;

void ADC0_IRQ_HANDLER_FUNC(void)
{
    adc0ConversionDoneFlag = true;

    /* Read conversion result to clear the conversion completed flag. */
    ADC0SampleValue = ADC16_GetChannelConversionValue(ADC_IMOT1_BASE_ADDR, conversionChannelGroup);

    /* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F Store immediate overlapping
      exception return operation might vector to incorrect interrupt */
#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
}

void ADC1_IRQ_HANDLER_FUNC(void)
{
    adc1ConversionDoneFlag = true;

    /* Read conversion result to clear the conversion completed flag. */
    ADC1SampleValue = ADC16_GetChannelConversionValue(ADC_IMOT2_BASE_ADDR, conversionChannelGroup);

    /* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F Store immediate overlapping
      exception return operation might vector to incorrect interrupt */
#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
}

bool ADCInit (void)
{
  
    vref_config_t vrefConfigStruct;
    
    VREF_GetDefaultConfig(&vrefConfigStruct);
    VREF_Init(VREF, &vrefConfigStruct);
    
    adc16_config_t adc16ConfigStruct;
  
     /*
     * adc16ConfigStruct.referenceVoltageSource = kADC16_ReferenceVoltageSourceVref;
     * adc16ConfigStruct.clockSource = kADC16_ClockSourceAsynchronousClock;
     * adc16ConfigStruct.enableAsynchronousClock = true;
     * adc16ConfigStruct.clockDivider = kADC16_ClockDivider8;
     * adc16ConfigStruct.resolution = kADC16_ResolutionSE12Bit;
     * adc16ConfigStruct.longSampleMode = kADC16_LongSampleDisabled;
     * adc16ConfigStruct.enableHighSpeed = false;
     * adc16ConfigStruct.enableLowPower = false;
     * adc16ConfigStruct.enableContinuousConversion = false;
     */
    ADC16_GetDefaultConfig(&adc16ConfigStruct);
#ifdef BOARD_ADC_USE_ALT_VREF
    adc16ConfigStruct.referenceVoltageSource = kADC16_ReferenceVoltageSourceValt;
#endif    
    ADC16_Init(ADC_IMOT1_BASE_ADDR, &adc16ConfigStruct);
    ADC16_EnableHardwareTrigger(ADC_IMOT1_BASE_ADDR, false); /* Make sure the software trigger is used. */
#if defined(FSL_FEATURE_ADC16_HAS_CALIBRATION) && FSL_FEATURE_ADC16_HAS_CALIBRATION
    if (kStatus_Success == ADC16_DoAutoCalibration(ADC_IMOT1_BASE_ADDR))
    {
        Serial_Debug("ADC16_IMOT1_DoAutoCalibration() Done.\r\n");
    }
    else
    {
        Serial_Debug("ADC16_IMOT1_DoAutoCalibration() Failed.\r\n");
    }
#endif /* FSL_FEATURE_ADC16_HAS_CALIBRATION */

/************************************************/
    
    ADC16_GetDefaultConfig(&adc16ConfigStruct);
#ifdef BOARD_ADC_USE_ALT_VREF
    adc16ConfigStruct.referenceVoltageSource = kADC16_ReferenceVoltageSourceValt;
#endif    
    ADC16_Init(ADC_IMOT2_BASE_ADDR, &adc16ConfigStruct);
    ADC16_EnableHardwareTrigger(ADC_IMOT2_BASE_ADDR, false); /* Make sure the software trigger is used. */
#if defined(FSL_FEATURE_ADC16_HAS_CALIBRATION) && FSL_FEATURE_ADC16_HAS_CALIBRATION
    if (kStatus_Success == ADC16_DoAutoCalibration(ADC_IMOT2_BASE_ADDR))
    {
        Serial_Debug("ADC16_IMOT2_DoAutoCalibration() Done.\r\n");
    }
    else
    {
        Serial_Debug("ADC16_IMOT2_DoAutoCalibration() Failed.\r\n");
    }
#endif /* FSL_FEATURE_ADC16_HAS_CALIBRATION */

 /************************************************/   

    ADC16_GetDefaultConfig(&adc16ConfigStruct);
#ifdef BOARD_ADC_USE_ALT_VREF
    adc16ConfigStruct.referenceVoltageSource = kADC16_ReferenceVoltageSourceValt;
#endif    
    ADC16_Init(ADC_IMOT3_BASE_ADDR, &adc16ConfigStruct);
    ADC16_EnableHardwareTrigger(ADC_IMOT3_BASE_ADDR, false); /* Make sure the software trigger is used. */
#if defined(FSL_FEATURE_ADC16_HAS_CALIBRATION) && FSL_FEATURE_ADC16_HAS_CALIBRATION
    if (kStatus_Success == ADC16_DoAutoCalibration(ADC_IMOT3_BASE_ADDR))
    {
        Serial_Debug("ADC16_IMOT3_DoAutoCalibration() Done.\r\n");
    }
    else
    {
        Serial_Debug("ADC16_IMOT3_DoAutoCalibration() Failed.\r\n");
    }
#endif /* FSL_FEATURE_ADC16_HAS_CALIBRATION */

    
    adcConfigStruct.channelNumber = ADC_IMOT1_CHANNEL;
    adcConfigStruct.enableInterruptOnConversionCompleted = true; /* Enable the interrupt. */
#if defined(FSL_FEATURE_ADC16_HAS_DIFF_MODE) && FSL_FEATURE_ADC16_HAS_DIFF_MODE
    adcConfigStruct.enableDifferentialConversion = false;
#endif /* FSL_FEATURE_ADC16_HAS_DIFF_MODE */
    
  EnableIRQ(ADC0_IRQn);
  EnableIRQ(ADC1_IRQn); 
    
  return true;
}


bool ADCChannelStart (eADCChannel adcChannel)
{
  uint16_t noOfChannels = sizeof ADCChannelGroup / sizeof ADCChannelGroup[0];
  uint16_t channelIndex = 0;
  for(channelIndex =0; channelIndex < noOfChannels ; channelIndex++)
  {
     if(ADCChannelGroup[channelIndex].adcChannel == adcChannel)
     {
       break;
     }
  }
  if(channelIndex < noOfChannels)
  {
    adcConfigStruct.channelNumber = ADCChannelGroup[channelIndex].adcChannelNo;
    //conversionChannel = ADCChannelGroup[channelIndex].adcChannel;
    conversionChannelGroup = ADCChannelGroup[channelIndex].adcChannelGroup;
    ADC16_SetChannelConfig(ADCChannelGroup[channelIndex].AdcBaseAddr, ADCChannelGroup[channelIndex].adcChannelGroup, &adcConfigStruct); 
    return true;
  }
  else
  {
    return false;
  }
}


void CheckADC (void)
{  
  uint64_t ADC0AvgValue;
  uint64_t ADC1AvgValue;

  ADC0AvgValue = 0;
  ADC1AvgValue = 0;
  for (uint16_t i=0;i < ADC_MAX_SAMPLES; i++)
  {
    adc0ConversionDoneFlag = false;
    ADCChannelStart(IMOT1);
    while(!adc0ConversionDoneFlag);
    ADC0AvgValue += ADC0SampleValue;
    
    adc1ConversionDoneFlag = false;
    ADCChannelStart(IMOT2); 
    while(!adc1ConversionDoneFlag);
    ADC1AvgValue += ADC1SampleValue;
  }
  Imot1AdcValue = ADC0AvgValue/ADC_MAX_SAMPLES;
  Imot2AdcValue = ADC1AvgValue/ADC_MAX_SAMPLES;
  

  ADC1AvgValue = 0;
  for (uint16_t i=0;i < ADC_MAX_SAMPLES; i++)
  {
    adc1ConversionDoneFlag = false;
    ADCChannelStart(IMOT3); 
    while(!adc1ConversionDoneFlag);
    ADC1AvgValue += ADC1SampleValue;
  }
  Imot3AdcValue = ADC1AvgValue/ADC_MAX_SAMPLES;
}

float GetImot1Value (void)
{
//  if(GetRobotState() != ROBOT_COMPLETED && GetRobotState() != ROBOT_IDLE)
//  {
    return (ADC_ILOAD_CALC(Imot1AdcValue) + Imot1OffsetValue);
//  }
//  else
//  {
//    return 0;
//  }
}

float GetImot2Value (void)
{
  return (ADC_ILOAD_CALC(Imot2AdcValue) + Imot2OffsetValue);
}

float GetImot3Value (void)
{
  return (ADC_ILOAD_CALC(Imot3AdcValue) + Imot3OffsetValue);
}

void UpdateImotOffsetValue (void)
{
  Imot1OffsetValue = ADC_ILOAD_CALC(Imot1AdcValue);
  Imot2OffsetValue = ADC_ILOAD_CALC(Imot2AdcValue);
  Imot3OffsetValue = ADC_ILOAD_CALC(Imot3AdcValue);  
  if((Imot1OffsetValue > MAX_OFFSET_VALUE_TOLERANCE) || (Imot1OffsetValue < (MAX_OFFSET_VALUE_TOLERANCE * -1)))
  {
    Imot1OffsetValue = 0;
  }
  if((Imot2OffsetValue > MAX_OFFSET_VALUE_TOLERANCE) || (Imot2OffsetValue < (MAX_OFFSET_VALUE_TOLERANCE * -1)))
  {
    Imot2OffsetValue = 0;
  }
  if((Imot3OffsetValue > MAX_OFFSET_VALUE_TOLERANCE) || (Imot3OffsetValue < (MAX_OFFSET_VALUE_TOLERANCE * -1)))
  {
    Imot3OffsetValue = 0;
  }
  Imot1OffsetValue = Imot1OffsetValue * -1;
  Imot2OffsetValue = Imot2OffsetValue * -1;
  Imot3OffsetValue = Imot3OffsetValue * -1; 
//  Imot1OffsetValue = Imot1OffsetValue * 1;
//  Imot2OffsetValue = Imot2OffsetValue * 1;
//  Imot3OffsetValue = Imot3OffsetValue * 1; 
  
}

