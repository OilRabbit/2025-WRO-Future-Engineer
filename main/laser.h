#ifndef LASER_H
#define LASER_H

#include <Arduino.h>
#include <MatrixMiniR4.h>
#include <vector>
#include "oled.h"

typedef enum {
  I2CPORT1,
  I2CPORT2
}LASERPORT;

void laserInit(LASERPORT port);
int getLaserDist(LASERPORT port);
int getLaser1DistMean(int max_element);
int getLaser2DistMean(int max_element);
void showLaser1Dist(COLUMN column, int line_number, int size, bool clearDisplay);
void showLaser2Dist(COLUMN column, int line_number, int size, bool clearDisplay);

#endif