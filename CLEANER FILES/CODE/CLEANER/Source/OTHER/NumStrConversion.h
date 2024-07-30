#ifndef __NUMSTRCONVERSION_H_
#define __NUMSTRCONVERSION_H_

#include <stdbool.h>
#include <stdint.h>

/*
Converts a given integer val to string str[].  d is the number
of digits required in output. If digits is more than the number
of digits in val, then 0s are added at the beginning.
*/
int IntToStr(int val, char* str, uint8_t digits);
void ftoa(float n, char *res, int afterpoint);
void HexByte_to_String (int rx_byte, char *High_Byte_Chr, char *Low_Byte_Chr);
uint8_t SplitString (char * str, char ** split_string, char delimitChar, int maxNoStrings);
uint8_t HexStringtoHexByte(const char Hex_String_H, const char Hex_String_L);                           
bool HexStringtoByteArray(const uint8_t * hexString, uint8_t * byteArray, uint16_t * byteArrayLength);  /* This will append a zero in MSB if string length is odd  */
uint64_t HexStringtoInt (const uint8_t * hexString);
bool Int64ToHexString (const uint64_t int64Val, char *str, uint16_t *strLength);        /* This will truncate if inp str length is small.   */
void reverse(char *str, int len);

#endif