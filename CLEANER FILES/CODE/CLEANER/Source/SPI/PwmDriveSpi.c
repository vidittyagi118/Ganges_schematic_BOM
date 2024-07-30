#include "PwmDriveSpi.h"

#define FAULT_STATUS_REG_ADRR       0x00
#define VDS_GDS_REG_ADRR            0x01
#define MAIN_REG_ADRR               0x02
#define IDRIVE_WD_REG_ADRR          0x03
#define VDS_REG_ADRR                0x04
#define CONFIG_REG_ADRR             0x05

enum {
  READ,
  WRITE
};

typedef struct stDriveRegDetails_def
{
  eDriveReg driveReg;
  uint8_t driveRegAddr;
}stDriveRegDetails;
   
stDriveRegDetails driveRegDetail[] = {
  
  {FAULT_STATUS, FAULT_STATUS_REG_ADRR},
  {VDS_GDS, VDS_GDS_REG_ADRR},  
  {MAIN_REG, MAIN_REG_ADRR},
  {IDRIVE_WD, IDRIVE_WD_REG_ADRR},
  {VDS, VDS_REG_ADRR},  
  {CONFIG, CONFIG_REG_ADRR}  
};

bool GetDriveRegAddress (eDriveReg driveReg, uint8_t *address)
{
    uint16_t noOfRegisters = sizeof driveRegDetail / sizeof driveRegDetail[0];
    uint16_t regIndex = 0;
    for(regIndex =0; regIndex < noOfRegisters ; regIndex++)
    {
     if(driveRegDetail[regIndex].driveReg == driveReg)
     {
       break;
     }
    }
    if(regIndex < noOfRegisters)
    {
        *address = driveRegDetail[regIndex].driveRegAddr;
         return true;
    } 
    else
    {
      return false;
    }  
}

uint16_t FormatTxData (uint8_t mode, eDriveReg driveReg, uint8_t data)
{
  uint8_t address;
  bool status = GetDriveRegAddress (driveReg, &address);
  uint16_t formatData = 0;
  if(status == true)
  {
    formatData = address;
    formatData = (formatData << 11) | data;
    if(mode == READ)
    {
      formatData |= 0x8000;
    }
  }
  else
  {
    formatData = 0x8000;                // Simple Make to read address 0. It should not come to this 
  }
  return formatData;
}

bool GetDriveData (uint8_t device, eDriveReg driveReg, uint16_t * data)
{
  uint16_t txdata = FormatTxData(READ, driveReg, 0);
  uint8_t sptxdata[2];
  uint8_t spirxdata[2];
  sptxdata[0] = txdata >> 8;
  sptxdata[1] = txdata & 0xFF;
  if( SpiTxRxData (device, sptxdata, spirxdata, 2) == true)
  {
    *data = spirxdata[1] & 0x00FF;                                                // Reject the higher Byte as its invalid
    return true;
  }
  else
  {
    return false;
  }
}

bool GetDriveAllData (uint8_t device, uint16_t * data, uint16_t datalen)
{
  if(datalen >= 6)
  {
    GetDriveData (device, FAULT_STATUS, &data[0]);
    GetDriveData (device, VDS_GDS, &data[1]);
    GetDriveData (device, MAIN_REG, &data[2]);
    GetDriveData (device, IDRIVE_WD, &data[3]);
    GetDriveData (device, VDS, &data[4]);
    GetDriveData (device, CONFIG, &data[5]);
    return true;
  }
  else
  {
    return false;
  }
}