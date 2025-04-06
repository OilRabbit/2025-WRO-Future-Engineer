#include "battery.h"

int getBatteryPercentage(){ return MiniR4.PWR.getBattPercentage(); }

void showBattPercentage(bool clearDisplay){
  static String batt_text = String(getBatteryPercentage()) + "%";
  display.oledSetTextColour(BLACK);
  display.oledDisplayRightln(0, 1, batt_text, clearDisplay);
  display.oledSetTextColour(WHITE);
  batt_text = String(getBatteryPercentage()) + "%";
  display.oledDisplayRightln(0, 1, batt_text, clearDisplay);
}