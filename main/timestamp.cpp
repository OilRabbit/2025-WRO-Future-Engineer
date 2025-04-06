#include "timestamp.h"

Timer internalClock;

void showInternalClock(bool clearDisplay){
  static long milliseconds = 0;
  static long seconds = 0;
  static long minutes = 0;
  milliseconds = internalClock.read();
  minutes = milliseconds / 60000;
  milliseconds = milliseconds - 60000 * minutes;
  seconds = milliseconds / 1000;
  milliseconds = (milliseconds - 1000 * seconds);
  static String curr_time_text = String(minutes) + ":" + String(seconds) + ":" + String(milliseconds);
  display.oledSetTextColour(BLACK);
  display.oledDisplayLeftln(0, 1, curr_time_text, clearDisplay);
  display.oledSetTextColour(WHITE);
  curr_time_text = String(minutes) + ":" + String(seconds) + ":" + String(milliseconds);
  display.oledDisplayLeftln(0, 1, curr_time_text, clearDisplay);
}