#ifndef OC1_H
#define OC1_H

#include <Arduino.h>
#include "steering.h"
#include "Timer.h"
#include "imu.h"
#include "timestamp.h"
#include <algorithm>
#include "ultra.h"
#include "tof.h"
#include "main.h"
#include "motor.h"
#include "buttons.h"
#include "calculation.h"

typedef enum {
  DETECT_STATE,
  WAIT_TURN_STATE,
  TURNING_STATE,
  DASH_AFTER_TURNING_STATE,
  RUN_SECTOR_STATE,
  ENDING_STATE,
  DEBUG_STATE
} OC1_STATES;

void OC1_ultra(double right_ang, int dist_threshold, int power);
void OC1_fixed(double right_ang, int dist_threshold, int power, int hypower);
void OC1main(void *parameters);
void showOC1Time(TFT_COLUMN column, int line_number, int text_size, uint16_t text_colour, bool clearDisplay);

#endif