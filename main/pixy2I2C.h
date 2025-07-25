#ifndef PIXY2I2C_H
#define PIXY2I2C_H

#include <Arduino.h>
#include <MatrixMiniR4.h>
#include <Pixy2I2C.h>
#include <Pixy2CCC.h>
#include <TPixy2.h>
#include <Wire.h>
#include <movingAvg.h>
#include "oled.h"
#include "huskylens.h"

/* A enum storing the ID of different colour */
// typedef enum {
//   RED = 2,
//   GREEN = 1,
//   MAGENTA = 3,
//   NO_COLOUR = -1
// }COLOUR_BLOCK;

// /* A struct for storing all the info of a coloured object detected */
// typedef struct{
//   COLOUR_BLOCK colour;
//   int xpos;
//   int ypos;
//   int height;
//   int width;
//   int area;
// }COLOURED_OBJ;

extern Pixy2I2C pixy;
extern COLOURED_OBJ nearestBlockGlobal;

void pixy2Init();
void pixy2ColorRegTest(COLUMN column, int line_number, int size, bool clearDisplay);

#endif