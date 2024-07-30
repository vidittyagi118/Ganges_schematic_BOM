#include "BrushControl.h"
#include "BrushMotorControl.h"
#include "RobotOperation.h"
#include "Serial_Debug.h"

#define DEFAULT_PWM_CHANGE_VALUE        1U                              
#define MINIMUM_PWM_DUTY                20U
#define STOP_PWM_DUTY                   0
#define IDLE_PWM_DUTY                   STOP_PWM_DUTY
#define PWM_CHANGE_TIMER_TICS_MS        1U  
#define DEFAULT_PWM_CHANGE_TIME         1U                               /* This value should not be less than the timer interrupt caller tics */
#define MINIMUM_PWM_CHANGE_VALUE        1U                               /* This value should not be zero. */

#if (DEFAULT_PWM_CHANGE_VALUE == 0)
#error (" Default PWM Change Value Cannot be Zero ");
#endif
#if (MINIMUM_PWM_CHANGE_VALUE == 0)
#error (" Minimum PWM Change Value Cannot be Zero ");
#endif
#if (DEFAULT_PWM_CHANGE_TIME < PWM_CHANGE_TIMER_TICS_MS)
#error (" PWM Update Time should not be less than timer tics ");
#endif

typedef enum eDecelState_def
{
  DECEL_START,
  DECEL_IN_PROGRESS,
  DECEL_COMPLETED
}eDecelState;

typedef enum eAccelState_def
{
  ACCEL_START,
  ACCEL_IN_PROGRESS,
  ACCEL_COMPLETED
}eAccelState;

static float pwmChangeValue                    = DEFAULT_PWM_CHANGE_VALUE;
static float currentPwmDuty                    = IDLE_PWM_DUTY; 

static const uint8_t minimumPwmDuty            = MINIMUM_PWM_DUTY;
static const uint32_t pwmChangeTimeCount_ms    = DEFAULT_PWM_CHANGE_TIME;
static uint32_t pwm_ChangeTimeCount            = 0;
static eBrushState brushState                  = BRUSH_IDLE;
static eDecelState decelState                  = DECEL_START;
static eAccelState accelState                  = ACCEL_START;
static eBrushDirection nextTargetDirection     = BRUSH_POSITIVE;

static bool brushEnabledState                  = true;


static stBrushPwmParam normalModePwmParam;
static stBrushPwmParam autoModePwmParam;
static stBrushParam currentModeBrushParam;
static stBrushParam currentModeBrushSetParam;

static void SetDecelState (eDecelState setDecelState);
static eDecelState GetDecelState (void);
static void SetAccelState (eAccelState setAccelState);
static eAccelState GetAccelState (void);
static float CalculatePwmIncrementValue (void);
static eDecelState ApplyDecelerationPwm (void);
static eAccelState ApplyAccelerationPwm (void);
static void PwmChangeTimerOn (uint32_t setPwmChangeTimeCount_ms);
static inline void PwmChangeTimerStop (void);
static void IncrementPwm (void);
static bool IncrementPwm1(void);
static void IncrementPwmCompleted (void);
static void DecrementPwm (void);
static bool DecrementPwm1 (void);
static void DecrementPwmCompleted (void);
static float CalculatePwmDecrementValue (void);
static void MoveBrushPositive (void);
static void MoveBrushNegative(void);
static void StopBrushMotor(void);
static bool IsDirectionChange (eBrushDirection tarDirection, eBrushDirection curDirection);
static eBrushDirection GetNextTargetDirection(void);
static void SetNextTargetDirection(eBrushDirection direction);
static void LoadBrushParameters(eBrushDirection direction, stBrushPwmParam * modePwmParam);
static void SetCurrentModeBrushParameter (stBrushParam *brushParam);
static eBrushDirection GetBrushActualDirection (void);
static bool IsBrushEnabled (void);

bool BrushInit (void)
{
  BrushMotorInit();  
  return true;
}

bool * GetSetBrushEnabledState (void)
{
  return(&brushEnabledState);
}

static bool IsBrushEnabled (void)
{
  return brushEnabledState;
}

void StartBrush (eBrushDirection dir)
{
  if(IsBrushEnabled())
  {
    SetBrushState(BRUSH_START);
  }
  else
  {
    if(GetBrushActualDirection() == BRUSH_STOP)
    {
      SetBrushState(BRUSH_DISABLEDSTATE);    
    }
    else
    {
      StopBrush(); 
    }
  }
  if(GetOperationMode() == AUTO)
  {
    LoadBrushParameters(dir, &autoModePwmParam);
  }
  else
  {
    LoadBrushParameters(dir, &normalModePwmParam);
  }
}

void StopBrush (void)
{
  SetBrushState(BRUSH_NORMAL_STOP);
}

void ImmediateStopBrush (void)
{
  SetBrushState(BRUSH_IMMEDIATE_STOP);
}

void SetBrushNormalPwmParameters (stBrushPwmParam *brushPwmParam)
{
  normalModePwmParam =  *brushPwmParam;
}

stBrushPwmParam * GetSetBrushNormalPwmParameters (void)
{
  return(&normalModePwmParam);
}

void SetBrushAutoPwmParameters (stBrushPwmParam *brushPwmParam)
{
  autoModePwmParam =  *brushPwmParam;
}

stBrushPwmParam * GetSetBrushAutoPwmParameters (void)
{
  return(&autoModePwmParam);
}

static void LoadBrushParameters(eBrushDirection direction, stBrushPwmParam * modePwmParam) 
{
  stBrushParam brushParameter;
  brushParameter.BrushPwmParam = *modePwmParam;
  brushParameter.direction = direction;
  SetCurrentModeBrushParameter(&brushParameter);
}

static void SetCurrentModeBrushParameter (stBrushParam *brushParam)
{
  currentModeBrushSetParam = *brushParam;  
  SetNextTargetDirection(currentModeBrushSetParam.direction);
}

static void SetNextTargetDirection(eBrushDirection direction)
{
  nextTargetDirection = direction;  
}

static eBrushDirection GetNextTargetDirection(void)
{
  return (nextTargetDirection);  
}

//static void SetCurrentDirection(eBrushDirection direction)
//{
//  currentDirection = direction;  
//}
//
//static eBrushDirection GetCurrentDirection(void)
//{
//  return (currentDirection);  
//}

static void GetCurrentModeBrushParameter (void)
{
  currentModeBrushParam = currentModeBrushSetParam;  
  Serial_Debug("\n Current Mode Brush Params *** ");
  Serial_Debug("\n Br Target Direction : ");  
  Serial_Debug_Num(currentModeBrushParam.direction); 
  Serial_Debug("\n Br Accel Time : "); 
  Serial_Debug_Num(currentModeBrushParam.BrushPwmParam.accelTime); 
  Serial_Debug("\n Br Decel Time : "); 
  Serial_Debug_Num(currentModeBrushParam.BrushPwmParam.decelTime); 
  Serial_Debug("\n Br Steady PWM1 : ");
  Serial_Debug_Num(currentModeBrushParam.BrushPwmParam.steadyPwm);
}

static void MoveBrushPositive (void)
{
  SetBrushMotorTargetState(BRUSHMOTOR_POSITIVE); 
}

static void MoveBrushNegative(void)
{
  SetBrushMotorTargetState(BRUSHMOTOR_NEGATIVE); 
}

static void StopBrushMotor(void)
{
  SetBrushMotorTargetState(BRUSHMOTOR_STOP); 
}

void SetBrushState (eBrushState setBrushState)
{
  brushState = setBrushState;  
}

eBrushState GetBrushState (void)
{
  return(brushState);  
}

static bool IsDirectionChange (eBrushDirection tarDirection, eBrushDirection curDirection)
{
  if(curDirection == BRUSH_POSITIVE)
  {
    if(tarDirection == BRUSH_NEGATIVE)
    {
      return true;
    }
  }
  else if(curDirection == BRUSH_NEGATIVE)
  {
    if(tarDirection == BRUSH_POSITIVE)
    {
      return true;
    }
  }
  else if(curDirection == BRUSH_UNKNOWN) 
  {
    if(tarDirection == BRUSH_POSITIVE || tarDirection == BRUSH_NEGATIVE)
    {
      return true;
    }
  }
  return false;  
}


static eBrushDirection GetBrushActualDirection (void)
{
  eBrushDirection brushCurDirection = BRUSH_UNKNOWN;
  eBrushMotorState brushMotDir = GetBrushMotorState();
  switch (brushMotDir)
  {
   case BRUSHMOTOR_POSITIVE:
        brushCurDirection = BRUSH_POSITIVE;
    break;
    case BRUSHMOTOR_NEGATIVE:
        brushCurDirection = BRUSH_NEGATIVE;
    break;
    case BRUSHMOTOR_STOP:
    case BRUSHMOTOR_IDLE:
        brushCurDirection = BRUSH_STOP;
      break;
   default:
    break;
  }
  return brushCurDirection;
}

void BrushFSM (void)
{
  eBrushState presentBrushState = GetBrushState();
  static bool dirChangeFlag = false;
  switch (presentBrushState)
  {
   case BRUSH_IDLE:
    currentPwmDuty = IDLE_PWM_DUTY;  
    SetBrushMotorAllPwm(currentPwmDuty);
    break;
   case BRUSH_START:
//    SetLogEvent (EV_LOG_MOT_STATE, BRUSH_START);                                 /* No break for this */
   case BRUSH_RE_START:                                 
    SetAccelState(ACCEL_COMPLETED);
    SetDecelState(DECEL_COMPLETED);
    currentPwmDuty = GetBrushMotorPwm();
    Serial_Debug("\n Brush Start : ");
    Serial_Debug_Float(currentPwmDuty, 3);
    dirChangeFlag = false;
    if(IsDirectionChange(GetNextTargetDirection(), GetBrushActualDirection()) == true)
    {
      dirChangeFlag = true;
      SetBrushState(BRUSH_DIR_CHANGE_STOP);
    }
    else
    {
      GetCurrentModeBrushParameter();
      if(currentModeBrushParam.direction == BRUSH_POSITIVE)
      {
        MoveBrushPositive();
        SetBrushState(BRUSH_WAIT_FOR_VALID_STATE);
      }
      else if(currentModeBrushParam.direction == BRUSH_NEGATIVE)
      {
        MoveBrushNegative(); 
        SetBrushState(BRUSH_WAIT_FOR_VALID_STATE);
      }
    }
    break;
   case BRUSH_WAIT_FOR_VALID_STATE:                                                     /* Wait till motor State changes to positive/Negative . It may be because of dead time delay */
      if((GetBrushActualDirection() == BRUSH_POSITIVE) || (GetBrushActualDirection() == BRUSH_NEGATIVE))
      {
        SetBrushState(BRUSH_ACCELERATION_START);
      }
   break;
   case BRUSH_ACCELERATION_START:
    if(currentPwmDuty < minimumPwmDuty)
    {
      currentPwmDuty = minimumPwmDuty;
      UpdateBrushMotorPwm(currentPwmDuty);
    }
    SetAccelState(ACCEL_START);
    SetBrushState(BRUSH_ACCELERATION);
    Serial_Debug("\n Brush Accel Start :");                                      /* No Break Necessary */
    Serial_Debug_Float(currentPwmDuty, 3);
   case BRUSH_ACCELERATION:
    if(ApplyAccelerationPwm() == ACCEL_COMPLETED)
    {
      Serial_Debug("\n Brush Accel Completed: ");
      Serial_Debug_Float(currentPwmDuty, 3);
      SetBrushState(BRUSH_STEADYSTATE);
    }
   break;
   case BRUSH_STEADYSTATE:
        /* Do Nothing In This */
   break;
   case BRUSH_NORMAL_STOP:
    dirChangeFlag = false;                                                      /* No break Necessary */
   case BRUSH_DIR_CHANGE_STOP:
    SetAccelState(ACCEL_COMPLETED);
    SetBrushState(BRUSH_DECELERATION_START);
    currentPwmDuty = GetBrushMotorPwm();
    Serial_Debug("\n Brush Motor Normal Stop");                                       /* No break Necessary */
   case BRUSH_DECELERATION_START:
    SetDecelState(DECEL_START);
    SetBrushState(BRUSH_DECELERATION);               
    Serial_Debug("\n Brush Decel Started : ");      
    Serial_Debug_Float(currentPwmDuty, 3);                                      /* No Break Necessary */
   case BRUSH_DECELERATION:
    if(ApplyDecelerationPwm() == DECEL_COMPLETED)
    {
        SetBrushState(BRUSH_END);
    }
    break;
   case BRUSH_IMMEDIATE_STOP:                                                  /* Clear Direction Flag (may be its set)- This prevents restart of motor.*/
    dirChangeFlag = false;                                                      /* No Break Necessary */
   case BRUSH_END:
    Serial_Debug("\n Brush End"); 
    currentPwmDuty = STOP_PWM_DUTY;
    SetBrushMotorAllPwm(currentPwmDuty);
    StopBrushMotor();  
    SetAccelState(ACCEL_COMPLETED);
    SetDecelState(DECEL_COMPLETED);
    if(dirChangeFlag == true)
    {
      dirChangeFlag = false;
      SetBrushState(BRUSH_RE_START);
      break;
    }
    else
    {
      if(IsBrushEnabled())
      {
        SetBrushState(BRUSH_COMPLETED);                                         /* No break Necessary */
      }
      else
      {
        SetBrushState(BRUSH_DISABLEDSTATE);                                         /* No break Necessary */
      }
    }
   case BRUSH_COMPLETED:
    break;
   case BRUSH_DISABLEDSTATE:
    break;
   default:
    break;
  }
  BrushMotorFSM();
}

static void SetDecelState (eDecelState setDecelState)
{
  decelState = setDecelState;  
}

static eDecelState GetDecelState (void)
{
  return(decelState);  
}

static float CalculatePwmIncrementValue (void)
{
  float pwmUpdateStep = 0;
  float diffPwm = (float)(currentModeBrushParam.BrushPwmParam.steadyPwm - GetBrushMotorPwm());
  if(diffPwm < 0)
  {
    pwmUpdateStep = 0;
  }
  else
  {
    if(currentModeBrushParam.BrushPwmParam.accelTime < PWM_CHANGE_TIMER_TICS_MS)                             /* 1ms is the minimum update time */
    {
      pwmUpdateStep = 0;
    }
    else 
    {
      if(pwmChangeTimeCount_ms > currentModeBrushParam.BrushPwmParam.accelTime)
      {
        pwmUpdateStep = currentModeBrushParam.BrushPwmParam.steadyPwm;
      }
      else
      {
        pwmUpdateStep = ((float)(diffPwm * pwmChangeTimeCount_ms))/currentModeBrushParam.BrushPwmParam.accelTime;
      }
    }
  }
  return pwmUpdateStep;
}

static float CalculatePwmDecrementValue (void)
{
  float pwmUpdateStep = 0;
  float diffPwm = (float)(GetBrushMotorPwm() - STOP_PWM_DUTY);
  if(diffPwm < 0)
  {
    pwmUpdateStep = 0;
  }
  else
  {
    if(currentModeBrushParam.BrushPwmParam.decelTime < PWM_CHANGE_TIMER_TICS_MS)                             /* 1ms is the minimum update time */
    {
      pwmUpdateStep = 0;
    }
    else 
    {
      if(pwmChangeTimeCount_ms > currentModeBrushParam.BrushPwmParam.decelTime)
      {
        pwmUpdateStep = STOP_PWM_DUTY;
      }
      else
      {
        pwmUpdateStep = ((float)(diffPwm * pwmChangeTimeCount_ms))/currentModeBrushParam.BrushPwmParam.decelTime;
      }
    }
  } 
  return pwmUpdateStep;
}

static eDecelState ApplyDecelerationPwm (void)
{
  eDecelState presentDecelState = GetDecelState();
  switch (presentDecelState)
  {
   case DECEL_START:
    pwmChangeValue = CalculatePwmDecrementValue();
    PwmChangeTimerOn(pwmChangeTimeCount_ms);
    presentDecelState = DECEL_IN_PROGRESS;
    SetDecelState(presentDecelState);
    break;
   case DECEL_IN_PROGRESS:
    break;
   case DECEL_COMPLETED:
    break;
  }
  return presentDecelState;
}

static void SetAccelState (eAccelState setAccelState)
{
  accelState = setAccelState;  
}

static eAccelState GetAccelState (void)
{
  return(accelState);  
}

static eAccelState ApplyAccelerationPwm (void)
{
  eAccelState presentAccelState = GetAccelState();
  switch (presentAccelState)
  {
   case ACCEL_START:
    pwmChangeValue = CalculatePwmIncrementValue();
    PwmChangeTimerOn(pwmChangeTimeCount_ms);
    presentAccelState = ACCEL_IN_PROGRESS;
    SetAccelState(presentAccelState);
    break;
   case ACCEL_IN_PROGRESS:
    break;
   case ACCEL_COMPLETED:
    break;
  }
  return presentAccelState;
}

static void PwmChangeTimerOn (uint32_t setPwmChangeTimeCount_ms)
{
  if(setPwmChangeTimeCount_ms == 0)
  {
    IncrementPwmCompleted();
    DecrementPwmCompleted();
  }
  else
  {
    pwm_ChangeTimeCount = setPwmChangeTimeCount_ms;
  }
}

inline void PwmChangeTimerStop (void)
{
  pwm_ChangeTimeCount = 0;
}

static bool IncrementPwm1(void)
{
  bool completeStatus = false;
  if(pwmChangeValue)
  {
    float newPwmDuty = currentPwmDuty + pwmChangeValue;
    if(newPwmDuty >= currentModeBrushParam.BrushPwmParam.steadyPwm)
    {
      currentPwmDuty = currentModeBrushParam.BrushPwmParam.steadyPwm;
      UpdateBrushMotorPwm(currentPwmDuty);
      completeStatus = true;
    }
    else
    {
      currentPwmDuty = newPwmDuty;
      UpdateBrushMotorPwm(currentPwmDuty);
      completeStatus = false;
    }
  }
  else
  {
    currentPwmDuty = currentModeBrushParam.BrushPwmParam.steadyPwm;
    UpdateBrushMotorPwm(currentPwmDuty);
    completeStatus = true;
  }  
  return completeStatus;
}

static void IncrementPwm (void)
{
 bool pwmCompleteStatus = true;
 pwmCompleteStatus = IncrementPwm1();
 if(pwmCompleteStatus == true)
 {
   IncrementPwmCompleted();
 }
 else
 {
   PwmChangeTimerOn(pwmChangeTimeCount_ms);
 }
}

static void IncrementPwmCompleted (void)
{
  PwmChangeTimerStop();
  SetAccelState(ACCEL_COMPLETED);
}

static bool DecrementPwm1 (void)
{
  bool completeStatus = false;
  if(pwmChangeValue)
  {
    float newPwmDuty = currentPwmDuty - pwmChangeValue;
    if(newPwmDuty <= STOP_PWM_DUTY)
    {
      currentPwmDuty = STOP_PWM_DUTY;
      UpdateBrushMotorPwm(currentPwmDuty);
      completeStatus = true;
    }
    else
    {
      currentPwmDuty = newPwmDuty;
      UpdateBrushMotorPwm(currentPwmDuty);
      completeStatus = false;
    }
  }
  else
  {
    currentPwmDuty = STOP_PWM_DUTY;
    UpdateBrushMotorPwm(currentPwmDuty);
    completeStatus = true;
  }
  return completeStatus;
}


static void DecrementPwm (void)
{
  bool pwmDecCompleteStatus = true;
  pwmDecCompleteStatus = DecrementPwm1();
  if(pwmDecCompleteStatus == true)
  {
    DecrementPwmCompleted();
  }
  else
  {
    PwmChangeTimerOn(pwmChangeTimeCount_ms);
  }
}

static void DecrementPwmCompleted (void)
{
  PwmChangeTimerStop();
  SetDecelState(DECEL_COMPLETED);
}


void BrushPwmChangeTimeIncrement_ms (void)
{
  BrushMotorTimeIncrement_ms();
  if(GetAccelState() == ACCEL_IN_PROGRESS)
  {
    if(--pwm_ChangeTimeCount <= 0)
    {
      IncrementPwm();
    }
  }
  else if(GetDecelState() == DECEL_IN_PROGRESS)
  {
    if(--pwm_ChangeTimeCount <= 0)
    {
      DecrementPwm();
    }
  }
}


