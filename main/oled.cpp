#include <sys/_stdint.h>
#include "oled.h"

void Display::oledClear(){
  MiniR4.OLED.clearDisplay();
}

void Display::oledSetTextColour(uint16_t colour){
  MiniR4.OLED.setTextColor(colour);
}

void Display::oledDisplay(int x_pos, int y_pos, int size, const char text[], bool clearDisplay){
  if(clearDisplay) MiniR4.OLED.clearDisplay();
  MiniR4.OLED.setCursor(x_pos, y_pos);
  MiniR4.OLED.setTextSize(size);
  MiniR4.OLED.print(text);
  MiniR4.OLED.display();
}

void Display::oledDisplay(int x_pos, int y_pos, int size, String &text, bool clearDisplay){
  if(clearDisplay) MiniR4.OLED.clearDisplay();
  MiniR4.OLED.setCursor(x_pos, y_pos);
  MiniR4.OLED.setTextSize(size);
  MiniR4.OLED.print(text);
  MiniR4.OLED.display();
}

void Display::oledDisplayln(int x_pos, int line_number, int size, const char text[], bool clearDisplay){
  if(clearDisplay) MiniR4.OLED.clearDisplay();
  MiniR4.OLED.setCursor(x_pos, line_number * TEXT_HEIGHT);
  MiniR4.OLED.setTextSize(size);
  MiniR4.OLED.print(text);
  MiniR4.OLED.display();
}

void Display::oledDisplayln(int x_pos, int line_number, int size, String &text, bool clearDisplay){
  if(clearDisplay) MiniR4.OLED.clearDisplay();
  MiniR4.OLED.setCursor(x_pos, line_number * TEXT_HEIGHT);
  MiniR4.OLED.setTextSize(size);
  MiniR4.OLED.print(text);
  MiniR4.OLED.display();
}

void Display::oledDisplayLeft(int y_pos, int size, const char text[], bool clearDisplay){
  if(clearDisplay) MiniR4.OLED.clearDisplay();
  oledDisplay(6, y_pos, size, text, clearDisplay);
}

void Display::oledDisplayLeft(int y_pos, int size, String &text, bool clearDisplay){
  if(clearDisplay) MiniR4.OLED.clearDisplay();
  oledDisplay(6, y_pos, size, text, clearDisplay);
}

void Display::oledDisplayLeftln(int line_number, int size, const char text[], bool clearDisplay){
  if(clearDisplay) MiniR4.OLED.clearDisplay();
  oledDisplayln(6, line_number, size, text, clearDisplay);
}

void Display::oledDisplayLeftln(int line_number, int size, String &text, bool clearDisplay){
  if(clearDisplay) MiniR4.OLED.clearDisplay();
  oledDisplayln(6, line_number, size, text, clearDisplay);
}

void Display::oledDisplayCenter(int y_pos, int size, const char text[], bool clearDisplay){
  if(clearDisplay) MiniR4.OLED.clearDisplay();
  int num_space = ceil((MAX_HTEXT - strlen(text)) / 2.0) * TEXT_LENGTH;
  oledDisplay(num_space, y_pos, size, text, clearDisplay);
}

void Display::oledDisplayCenter(int y_pos, int size, String &text, bool clearDisplay){
  if(clearDisplay) MiniR4.OLED.clearDisplay();
  int num_space = ceil((MAX_HTEXT - text.length()) / 2.0) * TEXT_LENGTH;
  oledDisplay(num_space, y_pos, size, text, clearDisplay);
}

void Display::oledDisplayCenterln(int line_number, int size, const char text[], bool clearDisplay){
  if(clearDisplay) MiniR4.OLED.clearDisplay();
  int num_space = ceil((MAX_HTEXT - strlen(text)) / 2.0) * TEXT_LENGTH;
  oledDisplayln(num_space, line_number, size, text, clearDisplay);
}

void Display::oledDisplayCenterln(int line_number, int size, String &text, bool clearDisplay){
  if(clearDisplay) MiniR4.OLED.clearDisplay();
  int num_space = ceil((MAX_HTEXT - text.length()) / 2.0) * TEXT_LENGTH;
  oledDisplayln(num_space, line_number, size, text, clearDisplay);
}

void Display::oledDisplayRight(int y_pos, int size, const char text[], bool clearDisplay){
  if(clearDisplay) MiniR4.OLED.clearDisplay();
  int num_space = (MAX_HTEXT - strlen(text)) * TEXT_LENGTH;
  oledDisplay(num_space, y_pos, size, text, clearDisplay);
}

void Display::oledDisplayRight(int y_pos, int size, String &text, bool clearDisplay){
  if(clearDisplay) MiniR4.OLED.clearDisplay();
  int num_space = (MAX_HTEXT - text.length()) * TEXT_LENGTH ;
  oledDisplay(num_space, y_pos, size, text, clearDisplay);
}

void Display::oledDisplayRightln(int line_number, int size, const char text[], bool clearDisplay){
  if(clearDisplay) MiniR4.OLED.clearDisplay();
  int num_space = (MAX_HTEXT - strlen(text)) * TEXT_LENGTH;
  oledDisplayln(num_space, line_number, size, text, clearDisplay);
}

void Display::oledDisplayRightln(int line_number, int size, String &text, bool clearDisplay){
  if(clearDisplay) MiniR4.OLED.clearDisplay();
  int num_space = (MAX_HTEXT - text.length()) * TEXT_LENGTH;
  oledDisplayln(num_space, line_number, size, text, clearDisplay);
}

Display display;