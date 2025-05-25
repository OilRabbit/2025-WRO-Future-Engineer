#ifndef HUSKYLENS_H
#define HUSKYLENS_H

#include <Arduino.h>
#include <MatrixMiniR4.h>
#include <DFRobot_HuskyLens.h>
#include <HUSKYLENS.h>
#include <HUSKYLENSMindPlus.h>
#include <HuskyLensProtocolCore.h>
#include <Wire.h>
#include <movingAvg.h>
#include "oled.h"

/* A enum storing the ID of different colour */
typedef enum {
  RED = 2,
  GREEN = 1,
  // MAGENTA = 3,
  NO_COLOUR = -1
}COLOUR_BLOCK;

/* A struct for storing all the info of a coloured object detected */
typedef struct{
  COLOUR_BLOCK colour;
  int xpos;
  int ypos;
  int height;
  int width;
  int area;
}COLOURED_OBJ;

extern HUSKYLENS huskylens;
extern movingAvg xCenter;
extern COLOURED_OBJ nearestPillarGlobal;

void huskylensInit();
void huskylensColorRegTest(COLUMN column, int line_number, int size, bool clearDisplay);
int getHuskyResultID();
int getHuskyXPos(COLOUR_BLOCK colour);
int getHuskyHeight(COLOUR_BLOCK colour);
void showHuskyRed(COLUMN column, int line_number, int size, bool clearDisplay);
void showHuskyGreen(COLUMN column, int line_number, int size, bool clearDisplay);
void showHuskyMagenta(COLUMN column, int line_number, int size, bool clearDisplay);
void showHuskyRG(COLUMN column, int line_number, int size, bool clearDisplay);
COLOURED_OBJ getNearestPillar();
void getNearestPillarGlobal();
void showNearestPillar(COLUMN column, int line_number, int size, bool clearDisplay);
COLOURED_OBJ getNearestColour();
void showNearestColour(COLUMN column, int line_number, int size, bool clearDisplay);

#endif