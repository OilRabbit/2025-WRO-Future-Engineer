// #include <sys/_intsup.h>
// #ifndef LASER_H
// #define LASER_H

// #include <Arduino.h>
// #include <MatrixMiniR4.h>
// #include <vector>
// #include <algorithm>
// #include "oled.h"

// extern int laser1Dist;
// extern int laser2Dist;

// /* A enum storing the port number of Laser 1 and Laser 2 */
// typedef enum {
//   I2CPORT1,
//   I2CPORT2
// } LASERPORT;

// /* A enum storing the mode for the laser to return distance */
// typedef enum{
//   RAW_DIST,
//   MEAN_DIST,
//   MEDIAN_DIST,
//   CISTERN_DIST,
//   FILTER_DIST,
//   EMA_DIST
// } LASERMODE;

// extern LASERMODE LaserMode;
// extern int LaserElements;

// void laserInit(LASERPORT port);
// int getLaserDist(LASERPORT port);
// int getLaser1DistMean(int max_element);
// int getLaser2DistMean(int max_element);
// int getLaser1DistMedian(int max_element);
// int getLaser2DistMedian(int max_element);
// int getLaser1DistCistern(int max_element, int dropCount);
// int getLaser2DistCistern(int max_element, int dropCount);
// int getFilteredLaser1Dist(int samples, int max_jump);
// int getFilteredLaser2Dist(int samples, int max_jump);
// void getLaser1Distloop();
// void getLaser2Distloop();
// void showLaser1Dist(COLUMN column, int line_number, int size, bool clearDisplay);
// void showLaser2Dist(COLUMN column, int line_number, int size, bool clearDisplay);

// #endif