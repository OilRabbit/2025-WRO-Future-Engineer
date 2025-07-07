#ifndef MAIN_H
#define MAIN_H

#include <DFRobot_HuskyLens.h>
#include <HUSKYLENS.h>
#include <HUSKYLENSMindPlus.h>
#include <HuskyLensProtocolCore.h>
#include <MatrixMiniR4.h>
#include <Thread.h>
#include <ThreadController.h>
#include <string.h>
#include "Timer.h"
#include "oled.h"
#include "display.h"
#include "battery.h"
#include "timestamp.h"
#include "steering.h"
#include "laser.h"
#include "OC1.h"
#include "OC2.h"
#include "huskylens.h"
#include "imu.h"
#include "ultra.h"
#include "LED.h"
// #include "color.h"

extern ThreadController controller;
extern Thread displayThread;
extern Thread OC1Thread;
extern Thread OC2Thread;
extern Thread Ultra1Thread;
extern Thread Ultra2Thread;
extern Thread huskylensThread;
extern Thread ColorThread;
extern Thread Laser1Thread;

#endif