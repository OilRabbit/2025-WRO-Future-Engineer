#include "battery.h"

int getBatteryPercentage(){ return MiniR4.PWR.getBattPercentage(); }

void showBattPercentage(bool clearDisplay){ display.oledDisplayRightln(0, 1, String(getBatteryPercentage()) + "%", clearDisplay); }