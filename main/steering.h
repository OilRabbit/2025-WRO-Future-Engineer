#ifndef STEERING_H
#define STEERING_H

#include <Arduino.h>
#include <MatrixMiniR4.h>
#include "oled.h"

#define MAX_STEERING_ANGLE 14

extern float steering_percentage; 

void steeringInit();
void reset_steering();
void steering(int steering_percentage);
void showSteering(COLUMN column, int line_number, int size, bool clearDisplay);

#endif