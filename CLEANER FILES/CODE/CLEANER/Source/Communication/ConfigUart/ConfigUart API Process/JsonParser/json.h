#ifndef _JSON_H_
#define _JSON_H_

#include <stdbool.h>
#include <stdint.h>

#define MAX_JSON_KEYS_ONE_TIME          10
#define MAX_JSON_VALUES_ONE_TIME        MAX_JSON_KEYS_ONE_TIME
#define MAX_JSON_VALUES_ONE_TIME_LENGTH (30+1)                                      // String Length of a single Json -Value

typedef struct
{
  char * Key[MAX_JSON_KEYS_ONE_TIME];
  uint8_t No_of_Keys;
}JsonFormatKeys;
typedef char PointType[MAX_JSON_VALUES_ONE_TIME][MAX_JSON_VALUES_ONE_TIME_LENGTH+1];

typedef struct
{
  PointType * Value;
  uint8_t No_of_Values;
  uint16_t Max_Value_Length;
}JsonFormatValues;

enum Key_Types_enum
{
  //SNO_KEY_TYPE_TIME_STAMP,
  SNO_KEY_TYPE_ID_NO,
  SNO_KEY_TYPE_CMD,
  SNO_KEY_TYPE_MODE                   
};

bool Parse_Json(const char * JSON_STRING, const char * json_key, char * json_value, uint16_t value_max_size);
bool Parse_Json_Multi(const char * JSON_STRING, const JsonFormatKeys * JsonKeys, JsonFormatValues * JsonValues);

#endif