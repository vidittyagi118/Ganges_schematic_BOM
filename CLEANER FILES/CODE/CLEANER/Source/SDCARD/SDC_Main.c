#include "sdglobal.h"					//Global data type definitions (see https://github.com/ibexuk/C_Generic_Header_File )
#include "SDC_Main.h"				//(Include header file for this source file)
#include "SDCard_Spi.h"
#include "mem-ffs.h"
#include <stdio.h>					//(Needed for printf)
#include <string.h>					//Used in the example project code
#include "Serial_Debug.h"

WORD general_use_10ms_timer;
WORD general_use_100ms_timer;

const char read_access_mode[] = {"r"};
const char write_access_mode[] = {"w"};
const char append_access_mode[] = {"a"};
const char read_write_access_mode[] = {"r+"};
const char write_read_access_mode[] = {"w+"};
const char append_read_access_mode[] = {"a+"};

void InitSDCard(void)
{
  int_SDC_SPI();
}

int sdcard_routine(void)
{
  ffs_process();
  return(0);
}

int SdcardPresent()
{
  return (ffs_card_ok);
}

bool CreateFileInSDC(char* filename, uint8_t filelength)
{
  char fileName[20];
  strncpy(fileName,filename,filelength);
  fileName[filelength+1] = 0;
  if(ffs_card_ok)
  {
    FFS_FILE *tempFile;
    tempFile = ffs_fopen(fileName,write_access_mode);
    
  if(tempFile!=0)
  {
    ffs_fclose(tempFile);
    return 1;
  }
  }
  return 0;
}

bool WriteDataToSDC(char* data, uint8_t length, char* filename,uint8_t filelength)
{
  char fileName[20];
  strncpy(fileName,filename,filelength);
  fileName[filelength+1] = 0;
//  Serial_Debug(fileName);
//  Serial_Debug(data);
  if(ffs_card_ok)
  {
    FFS_FILE *tempFile;
    tempFile = ffs_fopen(fileName,append_access_mode);
    if(tempFile!=0)
    {
      ffs_fputs(data,tempFile);
      ffs_fclose(tempFile);
      return 1;
    }
  }
  return 0;
}

bool RemoveFileFromSDC(char* filename, uint8_t filelength)
{
  char fileName[20];
  strncpy(fileName,filename,filelength);
  fileName[filelength+1] = 0;
  bool ret;
//  Serial_Debug(fileName);
//  Serial_Debug(data);
  if(ffs_card_ok)
  {  
    ret = ffs_remove(fileName);
  }
  if(!ret)
  {
  return 1;
  }
  return 0;
}

bool ReadDataFromSDC(char* buffer,uint16_t length,char* filename, uint8_t filelength)
{
  char fileName[20];
  strncpy(fileName,filename,filelength);
  fileName[filelength+1] = 0;
    if(ffs_card_ok)
  {
    FFS_FILE *tempFile;
    tempFile = ffs_fopen(fileName,read_access_mode);
    if(tempFile!=0)
    {
      //ffs_fseek(tempFile,filename,FFS_SEEK_SET);
      ffs_fgets(buffer,length,tempFile);
      ffs_fclose(tempFile);
      return 1;
    }
  }
  return 0;
}

void SDC_Timer_handler (void)
{
	static BYTE hb_10ms_timer = 0;
	static BYTE hb_100ms_timer = 0;
	static WORD hb_1sec_timer = 0;


	//T0IR = 0x3f;							//Reset irq

	//-----------------------------
	//-----------------------------
	//----- HERE EVERY 1 mSec -----
	//-----------------------------
	//-----------------------------



	hb_10ms_timer++;
	if (hb_10ms_timer == 10)
	{
		//------------------------------
		//------------------------------
		//----- HERE EVERY 10 mSec -----
		//------------------------------
		//------------------------------
		hb_10ms_timer = 0;


		//----- GENERAL USE 10mS TIMER -----
		if (general_use_10ms_timer)
			general_use_10ms_timer--;


////		//----- READ SWITCHES FLAG -----
////		//read_switches_flag = 1;
////
////		//----- USER MODE 10mS TIMER -----
////		if (user_mode_10ms_timer)
////			user_mode_10ms_timer--;


		//----- FAT FILING SYSTEM DRIVER TIMER -----
		if (ffs_10ms_timer)
			ffs_10ms_timer--;

	} //if (hb_10ms_timer == 10)

	hb_100ms_timer++;
	if (hb_100ms_timer == 100)
	{
		//-------------------------------
		//-------------------------------
		//----- HERE EVERY 100 mSec -----
		//-------------------------------
		//-------------------------------
		hb_100ms_timer = 0;

		//----- GENERAL USE 100mS TIMER -----
		if (general_use_100ms_timer)
			general_use_100ms_timer--;


	} //if (hb_100ms_timer == 100)

	hb_1sec_timer++;
	if (hb_1sec_timer == 1000)
	{
		//----------------------------
		//----------------------------
		//----- HERE EVERY 1 Sec -----
		//----------------------------
		//----------------------------
		hb_1sec_timer = 0;





	} //if (hb_1sec_timer == 1000)


}	
