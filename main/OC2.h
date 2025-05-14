#ifndef OC2_H
#define OC2_H

#include <Arduino.h>
#include <MatrixMiniR4.h>
#include "laser.h"
#include "steering.h"
#include "Timer.h"
#include "imu.h"
#include "timestamp.h"
#include <algorithm>
#include "ultra.h"
#include "main.h"
#include "OC1.h"

extern float steering_percentage;

void OC2Huskylens(double right_ang, int dist_threshold, int power);
void OC2main();
void showSteeringOC2(COLUMN column, int line_number, int size, bool clearDisplay);
void showOC2Time(COLUMN column, int line_number, int size, bool clearDisplay);

#endif