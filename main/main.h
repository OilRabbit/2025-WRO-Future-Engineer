#ifndef MAIN_H
#define MAIN_H

#define FENZY_MODE 1

#include <string.h>
#include "LED.h"
#include "tft.h"
#include "timestamp.h"
#include "buttons.h"
#include "imu.h"
#include "steering.h"
#include "motor.h"
#include "pixy2lib.h"
#include "OC1.h"
#include "OC2.h"
#include "tof.h"

extern TaskHandle_t blinkledThread;
extern TaskHandle_t displayThread;
extern TaskHandle_t ToF1Thread;
extern TaskHandle_t ToF2Thread;
extern TaskHandle_t IMUThread;
extern TaskHandle_t SteeringThread;
extern TaskHandle_t MotorEncThread;
extern TaskHandle_t Pixy2Thread;
extern TaskHandle_t OC1Thread;
extern TaskHandle_t OC2Thread;

#endif