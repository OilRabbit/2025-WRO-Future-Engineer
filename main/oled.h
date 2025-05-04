#ifndef OLED_H
#define OLED_H

#include <Arduino.h>
#include <MatrixMiniR4.h>
#include <cstring>

#define LENGTH_MAX 120
#define TEXT_LENGTH 6
#define MAX_HTEXT LENGTH_MAX / TEXT_LENGTH + 1
#define WIDTH_MAX 24
#define TEXT_HEIGHT 8
#define MAX_VTEXT WIDTH_MAX / TEXT_HEIGHT + 1

/* A enum storing the available column which text can be displayed at the OLED display */
typedef enum{
  LEFT,
  MID,
  RIGHT
} COLUMN;

/* The class of the display */
class Display
{
  public:
    void oledClear();
    void oledSetTextColour(uint16_t colour);
    void oledDisplay(int x_pos, int y_pos, int size, const char text[], bool clearDisplay);
    void oledDisplay(int x_pos, int y_pos, int size, String &text, bool clearDisplay);
    void oledDisplayln(int line_number, int y_pos, int size, const char text[], bool clearDisplay);
    void oledDisplayln(int line_number, int y_pos, int size, String &text, bool clearDisplay);
    void oledDisplayLeft(int y_pos, int size, const char text[], bool clearDisplay);
    void oledDisplayLeft(int y_pos, int size, String &text, bool clearDisplay);
    void oledDisplayLeftln(int line_number, int size, const char text[], bool clearDisplay);
    void oledDisplayLeftln(int line_number, int size, String &text, bool clearDisplay);
    void oledDisplayCenter(int y_pos, int size, const char text[], bool clearDisplay);
    void oledDisplayCenter(int y_pos, int size, String &text, bool clearDisplay);
    void oledDisplayCenterln(int line_number, int size, const char text[], bool clearDisplay);
    void oledDisplayCenterln(int line_number, int size, String &text, bool clearDisplay);
    void oledDisplayRight(int y_pos, int size, const char text[], bool clearDisplay);
    void oledDisplayRight(int y_pos, int size, String &text, bool clearDisplay);
    void oledDisplayRightln(int line_number, int size, const char text[], bool clearDisplay);
    void oledDisplayRightln(int line_number, int size, String &text, bool clearDisplay);
};

extern Display display;

#endif