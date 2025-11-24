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
#include "tof.h"
#include "main.h"
#include "OC1.h"
#include <cmath>
#include "motor.h"
#include "pixy2lib.h"
#include "buttons.h"
#include "calculation.h"
using namespace std;

// Stucture for storing information when detecting a block
struct BLK_INFO {
  COLOURED_OBJ pillar;
  int enc_value;
  int tof1_dist;
  int tof2_dist;
};

// States for OC1 FSM
typedef enum {
  INIT_STATE_OC2,
  SCAN_SECTOR_STATE_OC2,
  FW2CORNER_OC2,
  Backward_until_threshold,
  Forward_for_turn,
  SCAN_CORNER_OC2,
  CHANGE_LANE_P1_OC2,
  CHANGE_LANE_P2_OC2,
  CHANGE_LANE_P3_OC2,
  PURE_TURNING_OC2,
  UNKNOWN_TURNING_OC2,
  CURVE_BLK_STATE_OC2,
  SKIP_BLK_STATE_OC2,
  TURN_STRAIGHT_STATE_OC2,
  WAIT_TURN_OC2,
  INTO_SECTOR_OC2,
  CHECK_BLK_BESIDES_OC2,
  ENDING_STATE_OC2,
  OUT_PARKING_P1_STATE,
  OUT_PARKING_P2_STATE,
  OUT_PARKING_P3_STATE,
  OUT_PARKING_P4_STATE,
  OUT_PARKING_P5_STATE,
  OUT_PARKING_ANTI_RED_P1,
  OUT_PARKING_ANTI_RED_P2,
  OUT_PARKING_CLKW_RED_P1,
  OUT_PARKING_CLKW_RED_P2,
  OUT_PARKING_CLKW_GREEN_P1,
  OUT_PARKING_CLKW_GREEN_P2,
  STOP_OC2,
  anti_in_parking_p1,
  anti_in_parking_p2,
  anti_in_parking_p3,
  anti_in_parking_p4,
  anti_in_parking_p5,
  anti_in_parking_p6,
  clkw_in_parking_p1,
  clkw_in_parking_p2,
  clkw_in_parking_p3,
  clkw_in_parking_p4,
  clkw_in_parking_p5
  // clkw_in_parking_p6
} OC2_STATES;

void OC2main(void *parameters);
void showOC2Time(TFT_COLUMN column, int line_number, int text_size, uint16_t text_colour, bool clearDisplay);

#endif