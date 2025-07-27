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

extern Pixy2I2C pixy;
extern COLOURED_OBJ nearestBlockGlobal;

void pixy2Init();
void pixy2ColorRegTest(COLUMN column, int line_number, int size, bool clearDisplay);

#endif