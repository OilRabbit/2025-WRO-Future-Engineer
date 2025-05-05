#include "timestamp.h"

/* Global variable of the internal clock. For time reading purpose only. Never reset while running. */
Timer internalClock;

/**
 * @brief A function to put into display thread for showing the time elapsed from the start of the smart car
 * 
 * @param clearDisplay; bool; Set true to clear the whole OLED display everytime before displaying the battery percentage
 */
void showInternalClock(bool clearDisplay){
  static long total_ms = 0;
  static long seconds = 0;
  static long milliseconds = 0;
  total_ms = internalClock.read();
  seconds = total_ms / 1000;
  milliseconds = total_ms % 1000;
  static String curr_time_text = String(seconds) + "." + String(milliseconds);
  display.oledSetTextColour(BLACK);
  display.oledDisplayLeftln(0, 1, curr_time_text, clearDisplay);
  display.oledSetTextColour(WHITE);
  curr_time_text = String(seconds) + "." + String(milliseconds);
  display.oledDisplayLeftln(0, 1, curr_time_text, clearDisplay);
}
// void showInternalClock(bool clearDisplay){
//   static long milliseconds = 0;
//   static long seconds = 0;
//   static long minutes = 0;
//   milliseconds = internalClock.read();
//   minutes = milliseconds / 60000;
//   milliseconds = milliseconds - 60000 * minutes;
//   seconds = milliseconds / 1000;
//   milliseconds = (milliseconds - 1000 * seconds);
//   static String curr_time_text = String(minutes) + ":" + String(seconds) + ":" + String(milliseconds);
//   display.oledSetTextColour(BLACK);
//   display.oledDisplayLeftln(0, 1, curr_time_text, clearDisplay);
//   display.oledSetTextColour(WHITE);
//   curr_time_text = String(minutes) + ":" + String(seconds) + ":" + String(milliseconds);
//   display.oledDisplayLeftln(0, 1, curr_time_text, clearDisplay);
// }