#ifndef __CONFIG_API_PROCESS_H_
#define __CONFIG_API_PROCESS_H_

#include "json.h"
#include "APIProcessing.h"


eJsonStatus ProcessReceivedJsonData_config(char * jsonReceivedData);
eJsonStatus ConvertHexAndProcess(char * jsonString,uint16_t* size);
bool ParseHeaderDataJson(char * headerBuffer, uint16_t * headerBufferLength, uint16_t command,uint64_t MACaddr);



#endif