#ifndef STEERING_H
#define STEERING_H

#include <Arduino.h>
#include <MatrixMiniR4.h>

#define MAX_STEERING_ANGLE 15

void steeringInit();
void reset_steering();
void steering(int steering_percentage);

#endif