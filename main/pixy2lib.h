#pragma once
#include <Arduino.h>
#include "tft.h"
#include <Pixy2I2C.h>
#include <Pixy2CCC.h>
#include "calculation.h"

#define PIXY2_SDA 8
#define PIXY2_SCL 9

// Note that the upper left corner of the Pixy is its origin. The positive x- and y-axis is to the right and down respectively
#define CAM_MID_XPOS 145
#define CAM_MID_YPOS 102

typedef enum {
  RED = 1,
  GREEN = 2,
  MAGENTA = 3,
  NO_COLOUR = -1
} COLOUR_BLOCK;

typedef struct{
  COLOUR_BLOCK colour;
  uint16_t xpos;
  uint16_t ypos;
  uint16_t width;
  uint16_t height;
  int16_t angle;
  uint8_t index;
  uint8_t age;
  int area;
  int num_signitures;
} COLOURED_OBJ;

extern Pixy2I2C pixy;
extern COLOURED_OBJ nearestPillarGlobal;

int pixy2_init();
COLOURED_OBJ getNearestPillar();
void getNearestBlkloop(void *);
void showNearestBlk(TFT_COLUMN column, int line_number, int text_size, uint16_t text_colour, bool clearDisplay);