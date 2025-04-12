#ifndef OC1_H
#define OC1_H

#include <Arduino.h>
#include <MatrixMiniR4.h>
#include "laser.h"
#include "steering.h"
#include "Timer.h"
#include "imu.h"
#include "timestamp.h"
#include <algorithm>

extern float steering_percentage;

void OpenChallenge300();
void OpenChallengeMeanDist();
void showSteeringOC1(COLUMN column, int line_number, int size, bool clearDisplay);

#endif