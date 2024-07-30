#ifndef __EEPROM_STORE_H_
#define __EEPROM_STORE_H_

#include "eeprommain.h"


typedef enum eepromAccessType_def
{
  EEPROM_READ,
  EEPROM_WRITE  
}eEpromAccessType;

uint16_t GetMaxVirtualAddress (void);

eEepromStatus ResetEepromVariables(void);
eEepromStatus InitialiseEepromVariables(void);

typedef eEepromStatus (*EepromStoreProcess_Ptr)(eEpromAccessType eepromOp);
eEepromStatus EepromDataNil(eEpromAccessType eepromOp);
eEepromStatus EepromSetMode(eEpromAccessType eepromOp);
eEepromStatus EepromSetMotorPwmManualMode(eEpromAccessType eepromOp);
eEepromStatus EepromSetBrushEnabledState(eEpromAccessType eepromOp);
eEepromStatus EepromSetBrushPwmManualMode(eEpromAccessType eepromOp);
eEepromStatus EepromSetBrushMotorPolarity(eEpromAccessType eepromOp);
eEepromStatus EepromSetMotorPwmAutoMode(eEpromAccessType eepromOp);
eEepromStatus EepromSetBrushPwmAutoMode(eEpromAccessType eepromOp);
eEepromStatus EepromSetAutoScheduleTime(eEpromAccessType eepromOp);
eEepromStatus EepromSetEdgeSensorState(eEpromAccessType eepromOp);
eEepromStatus EepromSetCurrentLimits(eEpromAccessType eepromOp);
eEepromStatus EepromSetPulseCount(eEpromAccessType eepromOp);
eEepromStatus EepromSetLowBatSoC(eEpromAccessType eepromOp);
eEepromStatus EepromZigbeeConfiguration(eEpromAccessType eepromOp);
eEepromStatus EepromDevIDInfo(eEpromAccessType eepromOp);
eEepromStatus EepromSetMotorFaultConditions(eEpromAccessType eepromOp);
eEepromStatus EepromHeartbeatConfiguration(eEpromAccessType eepromOp);
//07D change
eEepromStatus EepromSetWheelDia(eEpromAccessType eepromOp);
eEepromStatus EepromSetCycleFrequency(eEpromAccessType eepromOp);
eEepromStatus EepromSetLogInterval(eEpromAccessType eepromOp);

eEepromStatus EepromSetContinue(eEpromAccessType eepromOp);
eEepromStatus EepromSetReturnState(eEpromAccessType eepromOp);
eEepromStatus EepromSetcomdistance(eEpromAccessType eepromOp);

eEepromStatus EepromSetNoofRowday(eEpromAccessType eepromOp);
eEepromStatus EepromStoreRowLength(eEpromAccessType eepromOp);
eEepromStatus EepromStoreRowLength2(eEpromAccessType eepromOp);
eEepromStatus EepromStoreRowLength3(eEpromAccessType eepromOp);
eEepromStatus EepromStoreRowLength4(eEpromAccessType eepromOp);
eEepromStatus EepromStoreRowLength5(eEpromAccessType eepromOp);
eEepromStatus EepromStoreRowLength6(eEpromAccessType eepromOp);
eEepromStatus EepromStoreRowLength7(eEpromAccessType eepromOp);

#endif