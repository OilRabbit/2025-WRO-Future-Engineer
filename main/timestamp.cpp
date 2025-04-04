#include "timestamp.h"

Timer internalClock;

void showInternalClock(bool clearDisplay){
  static bool init = false;
  static long milliseconds = 0;
  static long seconds = 0;
  static long minutes = 0;
  // if(!init){
  
  //   init = true;
  // }
  milliseconds = internalClock.read();
  minutes = milliseconds / 60000;
  milliseconds = milliseconds - 60000 * minutes;
  seconds = milliseconds / 1000;
  milliseconds = milliseconds - 1000 * seconds;
  // display.oledDisplayCenterln(0, 1, String(internalClock.read()) + "\0", clearDisplay);
  display.oledDisplayCenterln(0, 1, String(minutes) + ":" + String(seconds) + ":" + String(milliseconds), clearDisplay);
}