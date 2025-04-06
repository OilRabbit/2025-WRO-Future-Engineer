#ifndef LASER_H
#define LASER_H

#include <Arduino.h>
#include <MatrixMiniR4.h>
#include "oled.h"

typedef enum {
  I2CPORT1,
  I2CPORT2
}LASERPORT;

void laserInit(LASERPORT port);
int getLaserDist(LASERPORT port);
void showLaser1Dist(COLUMN column, int line_number, int size, bool clearDisplay);
void showLaser2Dist(COLUMN column, int line_number, int size, bool clearDisplay);

#endif