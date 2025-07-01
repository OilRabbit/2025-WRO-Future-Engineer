#ifndef OC2_H
#define OC2_H

#include <Arduino.h>
#include <MatrixMiniR4.h>
#include <vector>
#include "laser.h"
#include "steering.h"
#include "Timer.h"
#include "imu.h"
#include "timestamp.h"
#include <algorithm>
#include "ultra.h"
#include "main.h"
#include "OC1.h"
#include <cmath>
#include "huskylens.h"
#include "led.h"

#define CAM_MID_XPOS 152
#define CAM_LOWEST_YPOS 220
#define MID_RACINGLN_POS 420
#define PI 3.14159265

struct POINT {
  float x;
  float y;
};

typedef enum {
  DETECT_DRIFTING_STATE,
  DRIFTING_TUNING_STATE,
  RESET_IMU_STATE,
  BW_AFTER_RESET_IMU_STATE,
  FW_AFTER_RESET_IMU_STATE,
  DETECT_STATE,
  TURNING_N_AVOIDING_STATE,
  WAIT_TURN_STATE,
  TURNING_STATE,
  CURVE_P1_STATE, 
  CURVE_P2_STATE, 
  CURVE_P2_5_STATE,
  CURVE_P3_STATE,
  CURVE_P4_STATE,
  DASH_AFTER_TURNING_STATE,
  LAST_SECTOR_DASH,
  PARKING_STATE, // not yet written
  CHECK_FRONT_BLK_STATE,
  ST_FW_WITH_BLK_STATE,
  CHECK_MID_RACINGLN_STATE,
  MID_P1_STATE,
  MID_P2_STATE,
  MID_P3_STATE,
  MID_DASH_STATE,
  DEBUG_STATE
} OC2_STATES;

extern float steering_percentage;

float min_xpos_Gcase(int area);
float min_xpos_Rcase(int area);
void OC2Huskylens(double right_ang, int dist_threshold, int power);
void OC2main();
void showOC2Time(COLUMN column, int line_number, int size, bool clearDisplay);

#endif