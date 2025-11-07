#pragma once
#ifndef OC2_H
#define OC2_H

#include <Arduino.h>
#include <vector>
#include "steering.h"
#include "Timer.h"
#include "imu.h"
#include "timestamp.h"
#include <algorithm>
#include "ultra.h"
#include "main.h"
#include "OC1.h"
#include <cmath>
#include "motor.h"
#include "pixy2lib.h"
#include "buttons.h"
#include "calculation.h"
using namespace std;

#define CAM_MID_XPOS 145

struct POINT {
  float x;
  float y;
};

typedef enum {
  INIT_STATE_OC2,
  DETECT_STATE_OC2,
  TURNING_STATE_OC2,
  CURVE_BLK_STATE_OC2,
  SKIP_BLK_STATE_OC2,
  TURN_STRAIGHT_STATE_OC2,
  ENDING_STATE_OC2,
  STOP_OC2,
  // MID_P1_STATE,
  // MID_P2_STATE,
  // MID_P3_STATE,
  // MID_DASH_STATE,
  // CURVE_P1_STATE, 
  // CURVE_P2_STATE, 
  // CURVE_P2_5_STATE,
  // CURVE_P3_STATE,
  // CURVE_P4_STATE,
  // CHECK_MID_RACINGLN_STATE,
  // CHECK_MID_TWICE,
  // TESTING_1,
  // TESTING_OC2,
  // TEST_LINE_TRACK,
  // CHECK_B4_TURNING,
  // TURNING_P1,
  // TURNING_P1_0,
  // TURNING_P2,
  // TURNING_P1_5,
  // TURNING_P2_5,
  // TURNING_P3,
  // TURNING_N_AVOIDING_STATE,
  // WAIT_TURN_STATE,
  // TURNING_STATE,
  // SP_TURNING_STATE,
  // OUT_PARKING_P1_STATE,
  // OUT_PARKING_P2_STATE,
  // OUT_PARKING_P3_STATE,
  // OUT_PARKING_P4_STATE,
  // OUT_PARKING_P5_STATE,
  // OUT_PARKING_P6_STATE,
  // OUT_PARKING_P7_STATE,
} OC2_STATES;

extern float steering_percentage;

float min_xpos_Gcase(int area);
float min_xpos_Rcase(int area);
void OC2_pixy2(double right_ang, int dist_threshold, int power);
void OC2main(void *parameters);
void showOC2Time(TFT_COLUMN column, int line_number, int text_size, uint16_t text_colour, bool clearDisplay);

#endif