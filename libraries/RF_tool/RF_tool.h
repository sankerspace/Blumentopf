#ifndef _RF_TOOL_H_
#define _RF_TOOL_H_

#if defined(ARDUINO) && (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif
#include <RF24.h>

void resetRFChip(RF24& radio);





#endif
