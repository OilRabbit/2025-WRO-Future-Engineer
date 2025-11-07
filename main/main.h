#ifndef MAIN_H
#define MAIN_H

#define FENZY_MODE 1

#include <string.h>
#include "LED.h"
#include "tft.h"
#include "timestamp.h"
#include "ultra.h"
#include "buttons.h"
#include "imu.h"
#include "steering.h"
#include "motor.h"
#include "pixy2lib.h"
#include "OC1.h"
#include "OC2.h"

extern TaskHandle_t blinkledThread;
extern TaskHandle_t displayThread;
extern TaskHandle_t Ultra1Thread;
extern TaskHandle_t Ultra2Thread;
extern TaskHandle_t IMUThread;
extern TaskHandle_t SteeringThread;
extern TaskHandle_t MotorEncThread;
extern TaskHandle_t Pixy2Thread;
extern TaskHandle_t OC1Thread;
extern TaskHandle_t OC2Thread;

// #include "Timer.h"
// #include "display.h"
// #include "battery.h"
// #include "steering.h"
// #include "laser.h"
// #include "OC1.h"
// #include "OC2.h"
// #include "huskylens.h"
// #include "imu.h"
// #include "color.h"

// extern ThreadController controller;
// extern Thread displayThread;
// extern Thread OC1Thread;
// extern Thread OC2Thread;
// extern Thread Ultra1Thread;
// extern Thread Ultra2Thread;
// extern Thread huskylensThread;
// extern Thread ColorThread;

#endif