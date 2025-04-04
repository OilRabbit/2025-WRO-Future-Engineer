#ifndef TIMESTAMP_H
#define TIMESTAMP_H

#include "oled.h"
#include "Timer.h"

void showInternalClock(bool clearDisplay);

extern Timer internalClock;

#endif
