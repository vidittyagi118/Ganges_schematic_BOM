#ifndef __CUSTOM_TYPEDEFS_H_
#define __CUSTOM_TYPEDEFS_H_

typedef enum 
{
  SUCCESS = 0, 
  ERROR = !SUCCESS
} ErrorStatus;

typedef enum 
{
  RESET = 0, 
  SET = !RESET
} FlagStatus, ITStatus;

#endif