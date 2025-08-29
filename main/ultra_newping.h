#ifndef ULTRA_NEWPING_H
#define ULTRA_NEWPING_H

#include <NewPing.h>
#include <Arduino.h>
#include <MatrixMiniR4.h>
#include <vector>
#include <algorithm>
#include "oled.h"

#define SONAR_NUM 2      // Number of sensors.
#define MAX_DISTANCE 300 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm. 

void ultra_testing();
int ultra1Dist_cm();
int ultra2Dist_cm();
void ultra1_event_timer_sketch();
void echoCheck();

#endif