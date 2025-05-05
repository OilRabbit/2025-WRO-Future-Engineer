
#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include <MatrixMiniR4.h>
#include <Kalman.h>
#include "oled.h"

/* A enum storing the method to get the angle from IMU */
typedef enum{
  IMU_ORIGIN,
  IMU_KALMAN
}IMU_METHOD;

void imuInit();
void resetIMU();
double unwrapAngle(double current_angle);
double getIMU();
double getIMUKalman();
void showIMU(IMU_METHOD method, COLUMN column, int line_number, int size, bool clearDisplay);

#endif