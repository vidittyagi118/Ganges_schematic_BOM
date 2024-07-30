#include "NumStrConversion.h"
#include <math.h>
#include <string.h>

int intToStr(int x, char str[], int d);

/* Int to String Conversion Libraray *******************************************************************************/

/* Converts a given integer val to string str[].  d is the number
*of digits required in output. If digits is more than the number
* of digits in val, then 0s are added at the beginning.*/
int IntToStr(int val, char* str, uint8_t digits)
{
  bool int_neg_signbit = false;
  int i = 0;
  if(val < 0)
  {
    int_neg_signbit = true;
    val = val * -1;
  }
  while (val)
  {
    str[i++] = (val%10) + '0';
    val = val/10;
  }
  
  /* If number of digits required is more, then */
  /* add 0s at the beginning */
  while (i < digits)
    str[i++] = '0';
  
  if(int_neg_signbit == true)
  {
    str[i++] = '-';
  }
  
  // reverse string
  int k=0, j=i-1, temp;                              
  while (k<j)
  {
    temp = str[k];
    str[k] = str[j];
    str[j] = temp;
    k++; j--;
  }
  str[i] = '\0';
  return i;
}

/* Float to String Conversion Libraray *******************************************************************************
* reverses a string 'str' of length 'len'   ***/
void reverse(char *str, int len)
{
  int i=0, j=len-1, temp;
  while (i<j)
  {
    temp = str[i];
    str[i] = str[j];
    str[j] = temp;
    i++; j--;
  }
}

/** Converts a given integer x to string str[].  d is the number
* of digits required in output. If d is more than the number
* of digits in x, then 0s are added at the beginning. ****/
int intToStr(int x, char str[], int d)
{
  int i = 0;
  while (x)
  {
    str[i++] = (x%10) + '0';
    x = x/10;
  }
  
  /* If number of digits required is more, then*/
  /* add 0s at the beginning  */
  while (i < d)
    str[i++] = '0';
  
  reverse(str, i);
  str[i] = '\0';
  return i;
}

/* Converts a floating point number to string. */
void ftoa(float n, char *res, int afterpoint)
{
  bool neg_signbit = false;
  if(n < 0)
  {
    neg_signbit = true;
    n = n * -1;
  }
  
  /* Extract integer part */
  int ipart = (int)n;
  
  /* Extract floating part  */
  float fpart = n - (float)ipart;
  
  /* convert integer part to string   */
  int i = intToStr(ipart, res, 1);
  
  /* check for display option after point      */
  if (afterpoint != 0)
  {
    res[i] = '.';  // add dot
    
    /* Get the value of fraction part upto given no. */
    /* of points after dot. The third parameter is needed */
    /* to handle cases like 233.007  */
    fpart = fpart * pow(10, afterpoint);
    
    intToStr((int)fpart, res + i + 1, afterpoint);
  }
  if(neg_signbit == true)
  {
    int8_t str_count = strlen(res)+1;
    while(str_count >= 0)
    {
      res[str_count+1] = res[str_count];
      str_count--;
    }
    res[0] = '-';
  }
}

/*Convert Hex Byte to string...................*/
void HexByte_to_String (int rx_byte, char *High_Byte_Chr, char *Low_Byte_Chr)
{
  uint8_t byte = (uint8_t)((rx_byte & 0xF0) >> 4);
  if(byte < 10)
  {
    *High_Byte_Chr = (byte + 0x30);
  }
  else
  {
    *High_Byte_Chr = (byte+ 0x37);                 
  }
  byte = (uint8_t)(rx_byte & 0x0F);
  if(byte < 10)
  {
    *Low_Byte_Chr = (byte + 0x30);
  }
  else
  {
    *Low_Byte_Chr = (byte+ 0x37);                     
  }
}

uint8_t SplitString (char * str, char ** split_string, char delimitChar, int maxNoStrings)
{
  uint8_t stringcount = 0;
  split_string[stringcount++] = &str[0];
  if(strlen(str) != 0)
  {
    uint8_t len = strlen(str);
    for (uint8_t i=0; i< len; i++)
    {
      if(str[i] == delimitChar)
      {
        str[i] = '\0';
        if(stringcount >= maxNoStrings)
        {
          stringcount=0;
          break;
        }
        if(stringcount < maxNoStrings)
        {
          split_string[stringcount] = &str[i+1];  
          stringcount++;
        }
        else
        {
          break;
        }
      }
      
    }
  }
  return stringcount;
}

uint8_t HexStringtoHexByte(const char Hex_String_H, const char Hex_String_L)
{
  uint8_t hexByte_H,  hexByte_L;
  if (Hex_String_H >= '0' && Hex_String_H <= '9') hexByte_H = Hex_String_H - '0';
  else if (Hex_String_H >= 'a' && Hex_String_H <='f') hexByte_H = Hex_String_H - 'a' + 10;
  else if (Hex_String_H >= 'A' && Hex_String_H <='F') hexByte_H = Hex_String_H - 'A' + 10;
  else if (Hex_String_H == 0) hexByte_H = 0;
  
  
  if (Hex_String_L >= '0' && Hex_String_L <= '9') hexByte_L = Hex_String_L - '0';
  else if (Hex_String_L >= 'a' && Hex_String_L <='f') hexByte_L = Hex_String_L - 'a' + 10;
  else if (Hex_String_L >= 'A' && Hex_String_L <='F') hexByte_L = Hex_String_L - 'A' + 10;    
  else if (Hex_String_L == 0) hexByte_L = 0;
  
  return ((hexByte_H << 4) | hexByte_L);
}

bool HexStringtoByteArray(const uint8_t * hexString, uint8_t * byteArray, uint16_t * byteArrayLength)
{
  uint16_t hexStrLen = strlen((char const *)hexString);
  uint16_t hexStrIndex = 0;
  if((hexStrLen == 0) || (((float)hexStrLen)/2  > * byteArrayLength))
  {
    * byteArrayLength = 0;
    return false;
  }
  uint16_t hexbytecount = 0;
  if((hexStrLen % 2 ) != 0)
  {
    uint8_t highByte = 0;
    uint8_t lowByte = hexString[hexStrIndex++];
    byteArray[hexbytecount] = HexStringtoHexByte(highByte, lowByte);
    hexbytecount++;
    hexStrLen++;
  }
  for(; hexbytecount < (hexStrLen/2); hexbytecount++)
  {
    uint8_t highByte = hexString[hexStrIndex++];
    uint8_t lowByte = hexString[hexStrIndex++];
    byteArray[hexbytecount] = HexStringtoHexByte(highByte, lowByte);
  }
  * byteArrayLength = hexbytecount;
  return true;
}

uint64_t HexStringtoInt (const uint8_t * hexString)
{
  uint64_t val = 0;
  while (*hexString) {
    uint8_t byte = *hexString++; 
    if (byte >= '0' && byte <= '9') byte = byte - '0';
    else if (byte >= 'a' && byte <='f') byte = byte - 'a' + 10;
    else if (byte >= 'A' && byte <='F') byte = byte - 'A' + 10;    
    /* shift 4 to make space for new digit, and add the 4 bits of the new digit  */
    val = (val << 4) | (byte & 0xF);
  }
  return val;
}

bool Int64ToHexString (const uint64_t int64Val, char *str, uint16_t *strLength)
{
  char highChar, lowChar;
  uint8_t strIndex = 0;
  str[strIndex] = '\0';
  uint64_t intValue = int64Val;
  for(uint8_t intSize =0; intSize < (sizeof(intValue)/sizeof((uint8_t)intValue)); intSize++)
  {
    HexByte_to_String((uint8_t)intValue, &highChar, &lowChar);
    if((int8_t)(*strLength - strIndex) >= 3)                                     /* min required = 3 -> hightbyte + lowbyte + null char; */
    {
      str[strIndex++] = lowChar;
      str[strIndex++] = highChar;    
      intValue = intValue >> 8;
    }
    else
    {
      break;
    }
  }
  str[strIndex] = '\0';
  reverse(str, strIndex);
  *strLength = strIndex;
  return true;
}
