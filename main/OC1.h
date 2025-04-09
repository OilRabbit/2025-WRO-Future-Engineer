#ifndef OC1_H
#define OC1_H

#include <Arduino.h>
#include <MatrixMiniR4.h>
#include "laser.h"
#include "steering.h"
#include <algorithm>

extern steering_percentage;

void OpenChallenge();
void showSteeringOC1(COLUMN column, int line_number, int size, bool clearDisplay);

#endif