#ifndef DISPLAY_H
#define DISPLAY_H

#include <Arduino.h>
#include <MatrixMiniR4.h>
#include "battery.h"
#include "oled.h"
#include "timestamp.h"
#include "laser.h"
#include "huskylens.h"
#include "imu.h"
#include "OC1.h"
#include "ultra.h"
// #include "color.h"
#include "steering.h"

void displayData();

#endif