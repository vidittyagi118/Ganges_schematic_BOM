#ifndef _SDC_MAIN_H_
#define _SDC_MAIN_H_

#include <stdbool.h>
#include <stdint.h>

void InitSDCard(void);

int sdcard_routine(void);
bool WriteDataToSDC(char* data, uint8_t length, char* filename,uint8_t filelength);
bool CreateFileInSDC(char* fileName, uint8_t filelength);
bool RemoveFileFromSDC(char* filename, uint8_t filelength);
bool ReadDataFromSDC(char* buffer,uint16_t length,char* filename, uint8_t filelength);
void SDC_Timer_handler (void);

int SdcardPresent();

#endif