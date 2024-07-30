#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "fun_json.h"
#include "json.h"
#include <stdbool.h>
#include "serial_debug.h"
#define SUCCESS 0
#define FAILURE 1

extern void msdelay (uint16_t);

static int jsoneq(const char *json, jsmntok_t *tok, const char *s) {                    // Checking String Types
  if (tok->type == JSMN_STRING && (int) strlen(s) == tok->end - tok->start &&
      strncmp(json + tok->start, s, tok->end - tok->start) == 0) {
        return 0;
      }
  return -1;
}

bool Parse_Json(const char * JSON_STRING, const char * json_key, char * json_value, uint16_t value_max_size)
{
  int i;
  int r;
  jsmn_parser p;
  jsmntok_t t[200];                                                               /* We expect no more than 500 tokens */
  JSON_STRING = &JSON_STRING[0];
  json_value[0] =0x00;
  jsmn_init(&p);
  r = jsmn_parse(&p, JSON_STRING, strlen(JSON_STRING), t, sizeof(t)/sizeof(t[0]));
  if (r < 0) {
    Serial_Debug("Failed to parse JSON: ");
    Serial_Debug_Num(r);
    json_value[0] = 0x00;
    //printf("Failed to parse JSON: %d\n", r);
    return FAILURE;
  }
  /* Assume the top-level element is an object */
  if (r < 1 || t[0].type != JSMN_OBJECT) {
    //printf("Object expected\n
    Serial_Debug("Object expected");
    json_value[0] = 0x00;
    return FAILURE;
  }  
  /* Loop over all keys of the root object */
  for (i = 1; i < r; i++) 
  {
    if (jsoneq(JSON_STRING, &t[i], json_key) == 0)
    {
      uint16_t len = t[i+1].end-t[i+1].start;
      if(len <= value_max_size)                                           //This check is necessary to prevent buffer overflow
      {
        strncpy(json_value, JSON_STRING + t[i+1].start, len);
        json_value[len] = 0x00;
        return SUCCESS;
      }
      else
      {
        Serial_Debug((char const *)("Length of Values Greater than allocated buffer="));  
        Serial_Debug_Char(len);
        Serial_Debug_Char(sizeof(json_value));
        json_value[0] = 0x00;                                             // Load an empty Value
        return SUCCESS;
      }
    } 
    else 
    {
      i++;
    }
  }
  Serial_Debug((char const *)("KEY-NOT FOUND"));  
  return FAILURE;
}

bool Parse_Json_Multi(const char * JSON_STRING, const JsonFormatKeys * JsonKeys, JsonFormatValues * JsonValues)
{
  int i;
  int r;
  jsmn_parser p;
  jsmntok_t t[200];                                                       /* We expect no more than 500 tokens */  
  //     Serial_Debug((unsigned char *)JSON_STRING);
  jsmn_init(&p);
  r = jsmn_parse(&p, JSON_STRING, strlen(JSON_STRING), t, sizeof(t)/sizeof(t[0]));
  if (r < 0) {
    //printf("Failed to parse JSON: %d\n", r);
    Serial_Debug("Failed to parse JSON: ");
    
    //          Serial_Debug_Num(r);
    JsonValues->No_of_Values = 0;
    return FAILURE;
  }
  /* Assume the top-level element is an object */
  if (r < 1 || t[0].type != JSMN_OBJECT) {
    //printf("Object expected\n
    //          Serial_Debug("Object expected");
    JsonValues->No_of_Values = 0;
    return FAILURE;
  }
  /* Loop over all keys of the root object */
  for(int j = 0; j < JsonKeys->No_of_Keys; j++)
  {
    JSON_STRING = &JSON_STRING[0];
    (*JsonValues->Value)[j][0] = 0;
    for (i = 1; i < r; i++) 
    {
      
      if (jsoneq(JSON_STRING, &t[i], JsonKeys->Key[j]) == 0)
      {
        uint16_t len = t[i+1].end-t[i+1].start;
        if(len <= JsonValues->Max_Value_Length)                                                         //This check is necessary to prevent buffer overflow
        {
          strncpy((*JsonValues->Value)[j], JSON_STRING + t[i+1].start, len);
          (*JsonValues->Value)[j][len] = 0x00;
          //                      Serial_Debug((unsigned char *)(*JsonValues->Value)[j]);
        }
        else
        {
          (*JsonValues->Value)[j][0] = 0x00;
          //                      Serial_Debug((unsigned char *)("Length of Values Greater than allocated buffer));                          
        }
        break;
      } 
      else 
      {
        i++;
      }
    }
  }
  return SUCCESS;
}
