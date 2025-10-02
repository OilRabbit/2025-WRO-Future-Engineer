#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include <Wire.h>
#include <ICM_20948.h>
#include <math.h>
#include "tft.h"

extern ICM_20948_I2C icm;

extern double imu_yaw;
extern double imu_pitch;
extern double imu_roll;

void imu_init();
bool get_ypr(double& yaw, double& pitch, double& roll);
void getYPRloop(void *parameters);
void imu_resetYaw();
void showIMU(TFT_COLUMN column, int line_number, int text_size, uint16_t text_colour, bool clearDisplay);

#endif