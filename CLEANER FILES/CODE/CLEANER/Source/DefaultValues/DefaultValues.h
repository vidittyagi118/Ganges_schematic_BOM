#ifndef __DEFAULT_VALUES_H_
#define __DEFAULT_VALUES_H_

#define MAX_DEV_INFO_FIELD_LEN          10+1    //+1 for Null Character

typedef struct{
  char devID[MAX_DEV_INFO_FIELD_LEN];
  char hwVersion[MAX_DEV_INFO_FIELD_LEN];
  char serialNo[MAX_DEV_INFO_FIELD_LEN];
}stDevInfo;

#ifdef __cplusplus
extern "C" {
#endif 
  
void SetDefaultPwmCycleAutoModeParameter (void);
void SetDefaultPwmCycleManualModeParameter (void);
void SetDefaultPwmManulModeParameter (void);
void SetDefaultPwmEmergencyModeParameter (void);
void SetDefaultMotorConditions(void);
void SetDefaultAngleRange(void);
void SetDefaultZigbeeConfig(void);
void SetDefaultBoardTemperature(void);
void SetDefaultMot1Polarity(void);
void SetDefaultMot2Polarity(void);
void SetDefaultIloadConfigValues(void);
void SetDefaultMaxMotCurrentLimits(void);
void SetDefaultCountRange(void);
void SetDefaultBrushPwmNormalModeParameter (void);
void SetDefaultBrushPwmAutoModeParameter (void);
void SetDefaultDevIDInfo(void);
void SetDefaultMinBatSoc(void);
void SetDefaultBrushEnabledState(void);
void SetDefaultEdgeSensorEnabledState(void);
void SetDefaultOperationMode (void);
void SetDefaultBrushMotPolarity(void);
void SetDefaultAutoScheduledTime(void);
void SetDefaultMaxBoardTemperature(void);
stDevInfo * GetSetDeviceIDInfo (void);
void SetDefaultHeartbeatConfig(void);

void SetDefaultNil(void);
void DefaultValuesInit (void);
//void GetDefaultRobotParameter (stRobotPwmParam *motionPwmParam);  

//07D
void SetDefaultWheelDia(void);
void SetDefaultLogInterval(void);
void SetDefaultCycleFrequency(void);

void SetDefaultContinue(void);
void SetDefaultReturnState(void);
void SetDefaultcomdistance(void);
void SetDefaultNoofRowday(void);
void SetDefaultRowLength(void);
void SetDefaultRowLength2(void);
void SetDefaultRowLength3(void);
void SetDefaultRowLength4(void);
void SetDefaultRowLength5(void);
void SetDefaultRowLength6(void);
void SetDefaultRowLength7(void);


#ifdef __cplusplus
}
#endif

#endif