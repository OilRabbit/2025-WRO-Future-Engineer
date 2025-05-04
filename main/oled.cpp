#include <sys/_stdint.h>
#include "oled.h"

/* Global variable of the OLED display */
Display display;

/**
 * @brief Clear the entire display
 * 
 */
void Display::oledClear(){
  MiniR4.OLED.clearDisplay();
}

/**
 * @brief Set the colour of the text being displaced
 * 
 * @param colour; Only `BLACK` or `WHITE` can be chosen
 */
void Display::oledSetTextColour(uint16_t colour){
  MiniR4.OLED.setTextColor(colour);
}

/**
 * @brief Display text in char type at a specified location (x-pos & y-pos)
 * 
 * @param x_pos; int; The x-pos of the text being displaced
 * @param y_pos; int; The y-pos of the text being displaced 
 * @param size; int; The size of the text being displaced (1 ~ 2)
 * @param text; char; The text being displayed
 * @param clearDisplay; bool; Set true to clear the entire display
 */
void Display::oledDisplay(int x_pos, int y_pos, int size, const char text[], bool clearDisplay){
  if(clearDisplay) MiniR4.OLED.clearDisplay();
  MiniR4.OLED.setCursor(x_pos, y_pos);
  MiniR4.OLED.setTextSize(size);
  MiniR4.OLED.print(text);
  MiniR4.OLED.display();
}

/**
 * @brief Display text in String type at a specified location (x-pos & y-pos)
 * 
 * @param x_pos; int; The x-pos of the text being displaced
 * @param y_pos; int; The y-pos of the text being displaced 
 * @param size; int; The size of the text being displaced (1 ~ 2)
 * @param text; String; The text being displayed
 * @param clearDisplay; bool; Set true to clear the entire display
 */
void Display::oledDisplay(int x_pos, int y_pos, int size, String &text, bool clearDisplay){
  if(clearDisplay) MiniR4.OLED.clearDisplay();
  MiniR4.OLED.setCursor(x_pos, y_pos);
  MiniR4.OLED.setTextSize(size);
  MiniR4.OLED.print(text);
  MiniR4.OLED.display();
}

/**
 * @brief Display text in char type at a stated line number and a specified x-pos
 * 
 * @param x_pos; int; The x-pos of the text being displaced
 * @param line_number; int; The line number of the text being displayed (0 ~ 4)
 * @param size; int; The size of the text being displaced (1 ~ 2)
 * @param text; char; The text being displayed
 * @param clearDisplay; bool; Set true to clear the entire display
 */
void Display::oledDisplayln(int x_pos, int line_number, int size, const char text[], bool clearDisplay){
  if(clearDisplay) MiniR4.OLED.clearDisplay();
  MiniR4.OLED.setCursor(x_pos, line_number * TEXT_HEIGHT);
  MiniR4.OLED.setTextSize(size);
  MiniR4.OLED.print(text);
  MiniR4.OLED.display();
}

/**
 * @brief Display text in String type at a stated line number and a specified x-pos
 * 
 * @param x_pos; int; The x-pos of the text being displaced
 * @param line_number; int; The line number of the text being displayed (0 ~ 4)
 * @param size; int; The size of the text being displaced (1 ~ 2)
 * @param text; String; The text being displayed
 * @param clearDisplay; bool; Set true to clear the entire display
 */
void Display::oledDisplayln(int x_pos, int line_number, int size, String &text, bool clearDisplay){
  if(clearDisplay) MiniR4.OLED.clearDisplay();
  MiniR4.OLED.setCursor(x_pos, line_number * TEXT_HEIGHT);
  MiniR4.OLED.setTextSize(size);
  MiniR4.OLED.print(text);
  MiniR4.OLED.display();
}

/**
 * @brief Display the text in char type at LHS column and a specified y-pos
 * 
 * @param y_pos; int; The y-pos of the text being displaced
 * @param size; int; The size of the text being displaced (1 ~ 2)
 * @param text; char; The text being displayed
 * @param clearDisplay; bool; Set true to clear the entire display
 */
void Display::oledDisplayLeft(int y_pos, int size, const char text[], bool clearDisplay){
  if(clearDisplay) MiniR4.OLED.clearDisplay();
  oledDisplay(6, y_pos, size, text, clearDisplay);
}

/**
 * @brief Display the text in String type at LHS column and a specified y-pos
 * 
 * @param y_pos; int; The y-pos of the text being displaced
 * @param size; int; The size of the text being displaced (1 ~ 2)
 * @param text; String; The text being displayed
 * @param clearDisplay; bool; Set true to clear the entire display
 */
void Display::oledDisplayLeft(int y_pos, int size, String &text, bool clearDisplay){
  if(clearDisplay) MiniR4.OLED.clearDisplay();
  oledDisplay(6, y_pos, size, text, clearDisplay);
}

/**
 * @brief Display the text in char type at LHS column and a stated line number
 * 
 * @param line_number; int; The line number of the text being displayed (0 ~ 4)
 * @param size; int; The size of the text being displaced (1 ~ 2)
 * @param text; char; The text being displayed
 * @param clearDisplay; bool; Set true to clear the entire display
 */
void Display::oledDisplayLeftln(int line_number, int size, const char text[], bool clearDisplay){
  if(clearDisplay) MiniR4.OLED.clearDisplay();
  oledDisplayln(6, line_number, size, text, clearDisplay);
}

/**
 * @brief Display the text in String type at LHS column and a stated line number
 * 
 * @param line_number; int; The line number of the text being displayed (0 ~ 4)
 * @param size; int; The size of the text being displaced (1 ~ 2)
 * @param text; String; The text being displayed
 * @param clearDisplay; bool; Set true to clear the entire display
 */
void Display::oledDisplayLeftln(int line_number, int size, String &text, bool clearDisplay){
  if(clearDisplay) MiniR4.OLED.clearDisplay();
  oledDisplayln(6, line_number, size, text, clearDisplay);
}

/**
 * @brief Display the text in char type at Center column and a specified y-pos
 * 
 * @param y_pos; int; The y-pos of the text being displaced
 * @param size; int; The size of the text being displaced (1 ~ 2)
 * @param text; char; The text being displayed
 * @param clearDisplay; bool; Set true to clear the entire display
 */
void Display::oledDisplayCenter(int y_pos, int size, const char text[], bool clearDisplay){
  if(clearDisplay) MiniR4.OLED.clearDisplay();
  int num_space = ceil((MAX_HTEXT - strlen(text)) / 2.0) * TEXT_LENGTH;
  oledDisplay(num_space, y_pos, size, text, clearDisplay);
}

/**
 * @brief Display the text in String type at Center column and a specified y-pos
 * 
 * @param y_pos; int; The y-pos of the text being displaced
 * @param size; int; The size of the text being displaced (1 ~ 2)
 * @param text; String; The text being displayed
 * @param clearDisplay; bool; Set true to clear the entire display
 */
void Display::oledDisplayCenter(int y_pos, int size, String &text, bool clearDisplay){
  if(clearDisplay) MiniR4.OLED.clearDisplay();
  int num_space = ceil((MAX_HTEXT - text.length()) / 2.0) * TEXT_LENGTH;
  oledDisplay(num_space, y_pos, size, text, clearDisplay);
}

/**
 * @brief Display the text in char type at Center column and a stated line number
 * 
 * @param line_number; int; The line number of the text being displayed (0 ~ 4)
 * @param size; int; The size of the text being displaced (1 ~ 2)
 * @param text; char; The text being displayed
 * @param clearDisplay; bool; Set true to clear the entire display
 */
void Display::oledDisplayCenterln(int line_number, int size, const char text[], bool clearDisplay){
  if(clearDisplay) MiniR4.OLED.clearDisplay();
  int num_space = ceil((MAX_HTEXT - strlen(text)) / 2.0) * TEXT_LENGTH;
  oledDisplayln(num_space, line_number, size, text, clearDisplay);
}

/**
 * @brief Display the text in String type at Center column and a stated line number
 * 
 * @param line_number; int; The line number of the text being displayed (0 ~ 4)
 * @param size; int; The size of the text being displaced (1 ~ 2)
 * @param text; String; The text being displayed
 * @param clearDisplay; bool; Set true to clear the entire display
 */
void Display::oledDisplayCenterln(int line_number, int size, String &text, bool clearDisplay){
  if(clearDisplay) MiniR4.OLED.clearDisplay();
  int num_space = ceil((MAX_HTEXT - text.length()) / 2.0) * TEXT_LENGTH;
  oledDisplayln(num_space, line_number, size, text, clearDisplay);
}

/**
 * @brief Display the text in char type at RHS column and a specified y-pos
 * 
 * @param y_pos; int; The y-pos of the text being displaced
 * @param size; int; The size of the text being displaced (1 ~ 2)
 * @param text; char; The text being displayed
 * @param clearDisplay; bool; Set true to clear the entire display
 */
void Display::oledDisplayRight(int y_pos, int size, const char text[], bool clearDisplay){
  if(clearDisplay) MiniR4.OLED.clearDisplay();
  int num_space = (MAX_HTEXT - strlen(text)) * TEXT_LENGTH;
  oledDisplay(num_space, y_pos, size, text, clearDisplay);
}

/**
 * @brief Display the text in String type at RHS column and a specified y-pos
 * 
 * @param y_pos; int; The y-pos of the text being displaced
 * @param size; int; The size of the text being displaced (1 ~ 2)
 * @param text; String; The text being displayed
 * @param clearDisplay; bool; Set true to clear the entire display
 */
void Display::oledDisplayRight(int y_pos, int size, String &text, bool clearDisplay){
  if(clearDisplay) MiniR4.OLED.clearDisplay();
  int num_space = (MAX_HTEXT - text.length()) * TEXT_LENGTH ;
  oledDisplay(num_space, y_pos, size, text, clearDisplay);
}

/**
 * @brief Display the text in char type at RHS column and a stated line number
 * 
 * @param line_number; int; The line number of the text being displayed (0 ~ 4)
 * @param size; int; The size of the text being displaced (1 ~ 2)
 * @param text; char; The text being displayed
 * @param clearDisplay; bool; Set true to clear the entire display
 */
void Display::oledDisplayRightln(int line_number, int size, const char text[], bool clearDisplay){
  if(clearDisplay) MiniR4.OLED.clearDisplay();
  int num_space = (MAX_HTEXT - strlen(text)) * TEXT_LENGTH;
  oledDisplayln(num_space, line_number, size, text, clearDisplay);
}

/**
 * @brief Display the text in String type at RHS column and a stated line number
 * 
 * @param line_number; int; The line number of the text being displayed (0 ~ 4)
 * @param size; int; The size of the text being displaced (1 ~ 2)
 * @param text; String; The text being displayed
 * @param clearDisplay; bool; Set true to clear the entire display
 */
void Display::oledDisplayRightln(int line_number, int size, String &text, bool clearDisplay){
  if(clearDisplay) MiniR4.OLED.clearDisplay();
  int num_space = (MAX_HTEXT - text.length()) * TEXT_LENGTH;
  oledDisplayln(num_space, line_number, size, text, clearDisplay);
}