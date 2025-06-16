#include <sys/_intsup.h>
#ifndef COLOR_H
#define COLOR_H

#include <Arduino.h>
#include <MatrixMiniR4.h>
#include <vector>
#include <algorithm>
#include "oled.h"

extern int colorNo;

void colorInit();
int getColorRGB(ColorType colour);
int getColorType();
int getGreyScale();
void getColorTypeloop();
void showColorType(COLUMN column, int line_number, int size, bool clearDisplay);


#endif