#include "Led1.h"
#include "Led2.h"
#include "Led3.h"


void OperateLeds (void)
{
  OperateLed1();
  OperateLed2();  
  OperateLed3();
}

void LedTimeIncrement_ms (void)
{
  Led1TimeIncrement_ms();
  Led2TimeIncrement_ms();
  Led3TimeIncrement_ms();
}

void LedInit (void)
{
  Led1InitPins();
  Led2InitPins();  
  Led3InitPins();
}