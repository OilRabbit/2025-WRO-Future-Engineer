#ifndef LASER_H
#define LASER_H

#include <Arduino.h>
#include <MatrixMiniR4.h>
#include <vector>
#include <algorithm>
#include "oled.h"

extern double laser1Dist;
extern double laserDist;

extern LASERMODE LaserMode;
extern int LaserElemtents;

/* A enum storing the port number of Laser 1 and Laser 2 */
typedef enum {
  I2CPORT1,
  I2CPORT2
} LASERPORT;

/* A enum storing the mode for the laser to return distance */
typedef enum{
  RAWDIST,
  MEANDIST,
  MEDIANDIST
} LASERMODE;

void laserInit(LASERPORT port);
int getLaserDist(LASERPORT port);
int getLaser1DistMean(int max_element);
int getLaser2DistMean(int max_element);
void getLaser1Distloop(LASERMODE mode, int max_element);
void getLaser2Distloop(LASERMODE mode, int max_element);
void showLaser1Dist(COLUMN column, int line_number, int size, bool clearDisplay);
void showLaser2Dist(COLUMN column, int line_number, int size, bool clearDisplay);

#endif