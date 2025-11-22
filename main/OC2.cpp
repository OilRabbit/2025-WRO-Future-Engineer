#include <iterator>
#include <cmath>
#include "motor.h"
#include "pixy2lib.h"
#include "OC2.h"
#include "calculation.h"
using namespace std;

bool reset_OC2 = true;
long OC2_starttime = 0; 
long OC2_endtime = 0; 
bool run_OC2 = false;

static inline void safeSuspend(TaskHandle_t h){ if (h) vTaskSuspend(h); }
static inline void safeResume(TaskHandle_t h){ if (h) vTaskResume(h); }

/**
 * @brief Algorithm to calculate the safe xpos of the Red pillar seen according to its area from the huskylens
 * @param area; int; the area of the pillar from the huskylens
 *
 * @return float; the safe xpos of the pillar seen
*/
float min_xpos_Rcase_detect(int area){
  return -0.0008 * pow(area, 3) + 0.0948 * area * area - 4.6858 * area + 134.52; // - 70 * (1 - 1 / (area + 1))
}

float min_xpos_Rcase_skip(int area){
  return -0.0008 * pow(area, 3) + 0.0948 * area * area - 4.6858 * area + 134.52 - 70; // - 70
}

/**
 * @brief Algorithm to calculate the safe xpos of the Green pillar seen according to its area from the huskylens
 * @param area; int; the area of the pillar from the huskylens
 *
 * @return float; the safe xpos of the pillar seen
*/
float min_xpos_Gcase(int area){
  return 0.0003 * pow(area, 3) - 0.051 * pow(area, 2) + 3.3764 * area + 170.59 + 45;
}

float area2dist_red(int area){
  return 169.46 * pow(area, -0.486) - 10 + 25 * (7 / area);
}

float area2dist_green(int area){
  return 0.000000007 * pow(area, 6) - 0.000001983 * pow(area, 5) + 0.000239396 * pow(area, 4) - 0.014922766 * pow(area, 3) + 0.512699658 * pow(area, 2) - 9.728327963 * area + 102.428377560 + 15 * (1 - area / 55);
}
/**
 * @brief Algorithm to calculate the mirror everything on the Red pillar case to the Green pillar case
 * @param xpos; float; the xpos of Green pillar from the huskylens
 *
 * @return float; the corresponding xpos when translate to Red case
*/
float mirror(float xpos){
  return CAM_MID_XPOS * 2 - xpos;
}

float change_lane_dist_algo(float tof_dist, float init_ang, float final_ang){
  return (tof_dist * cos(deg2rad(init_ang)) - 15 - 10 * cos(deg2rad(90 - init_ang))) / sin(deg2rad(final_ang));
}

/**
 * @brief Function for OC2
 * 
 * @param right_ang; double; the value of an "right angle" for the IMU (as the value of IMU is not consistent)
 * @param dist_threshold; int; the threshold that the car sees for turning (in mm)
 * @param power; int; the power of the driving motor (0 ~ -100)
 * 
 */
void OC2_pixy2(double right_ang, int dist_threshold, int power){
  safeSuspend(blinkledThread);
  safeSuspend(OC1Thread);

  float imu_kp = 4.5;
  static OC2_STATES state = INIT_STATE_OC2; // A variable storing the current state of OC2 run
  static OC2_STATES prev_state = INIT_STATE_OC2;
  static bool end_game = false;               // A flag determining whether the run has ended
  vector<BLK_INFO> pillars_array;    // An array storing all the pillars detected
  BLK_INFO pillar_front_info;
  static COLOURED_OBJ pillar_front;           // A struct storing all the info of the nearest pillar detected during DETECT_STATE
  static bool is_anticlockwise = true;          // A boolean storing whether the car is racing in clockwise or anti-clockwise direction
  static int tar_power = power;
  static String OC2_text = "INIT_STATE_OC2";
  static bool blk_while_turning = false;
  static double tar_ang = 0;                  // The target angle which the car should be facing
  static double tar_ang_calibrate = 0;
  static int num_turn = 0;
  static float blk_area_after_turn = 0;
  int turn_st_ang = 15;
  static int clone_blk_xpos = 0;
  static int inner_wall_dist = 0;
  static float change_lane_run_dist = 0.0;
  static int change_lane_run_ang = 45;
  static float skip_blk_dist = 0;
  static bool outpark_hv_blk = false;
  static int time_count = 0;

  if (reset_OC2){
    tar_ang = 0;
    tar_ang_calibrate = 0;
    is_anticlockwise = false;
    num_turn = 0;
    steering_percentage = 0;
    end_game = false;
    prev_state = INIT_STATE_OC2;
    state = INIT_STATE_OC2;
    OC2_text = "INIT_STATE_OC2";
    pillars_array.clear();
    tar_power = power;
    blk_area_after_turn = 0;
    clone_blk_xpos = 0;
    inner_wall_dist = 0;
    change_lane_run_dist = 0.0;
    change_lane_run_ang = 50;
    skip_blk_dist = 0;
    outpark_hv_blk = false;
    OC2_starttime = internalClock.read();
    safeSuspend(blinkledThread);
    imu_resetYaw();
    reset_encoder();
    reset_OC2 = false;
  }

  if (!end_game){
    OC2_endtime = internalClock.read();

    switch (state) {
      case INIT_STATE_OC2:
        Serial.println("INIT_STATE_OC2");
        OC2_text = "INIT_STATE_OC2";
        motor_move(tar_power);
        tar_power = 0;
        if (dist_t1 > dist_t2) is_anticlockwise = true; //dist_t1
        else is_anticlockwise = false;
        prev_state = INIT_STATE_OC2;
        state = OUT_PARKING_P1_STATE;
        tar_power = power;
        break;

      case SCAN_SECTOR_STATE_OC2:
        Serial.println("SCAN_SECTOR_STATE_OC2");
        OC2_text = "SCAN_SECTOR_STATE_OC2";
        tar_power = power;
        motor_move(tar_power);
        if (num_turn >= 12){
          prev_state = SCAN_SECTOR_STATE_OC2;
          state = ENDING_STATE_OC2;
          break;
        }
        if (nearestPillarGlobal.colour == RED || nearestPillarGlobal.colour == GREEN){
          pillar_front = nearestPillarGlobal;
          pillar_front_info.pillar = pillar_front;
          pillar_front_info.enc_value = abs(MOTOR_ENCODER_COUNT);
          pillar_front_info.tof1_dist = dist_t1;
          pillar_front_info.tof2_dist = dist_t2;
          pillars_array.push_back(pillar_front_info);
          if (pillar_front.colour == RED){
            if (pillar_front.xpos >= min_xpos_Rcase_detect(pillar_front.area)){
              prev_state = SCAN_SECTOR_STATE_OC2;
              state = CHANGE_LANE_P1_OC2;
              change_lane_run_dist = change_lane_dist_algo(pillar_front_info.tof2_dist, imu_yaw - tar_ang, change_lane_run_ang);
              break;
            } else {
              prev_state = SCAN_SECTOR_STATE_OC2;
              state = FW2CORNER_OC2;
              break;
            }
          } else {
            if (pillar_front.xpos <= min_xpos_Gcase(pillar_front.area)){
              prev_state = SCAN_SECTOR_STATE_OC2;
              state = CHANGE_LANE_P1_OC2;
              change_lane_run_dist = change_lane_dist_algo(pillar_front_info.tof1_dist, tar_ang - imu_yaw, change_lane_run_ang);
              break; 
            } else {
              prev_state = SCAN_SECTOR_STATE_OC2;
              state = FW2CORNER_OC2;
              break;
            }
          }
        } else {
          if (millis() - time_count < 2000){
            steering_percentage = 0;
            motor_move(0);
            break;
          } else {
            prev_state = SCAN_SECTOR_STATE_OC2;
            state = FW2CORNER_OC2;
            break;
          }
        }
        break;

      case FW2CORNER_OC2:
        Serial.println("FW2CORNER_OC2");
        OC2_text = "FW2CORNER_OC2";
        tar_power = power;
        motor_move(tar_power);
        if (((is_anticlockwise && dist_t1 > dist_threshold) || (!is_anticlockwise && dist_t2 > dist_threshold)) && (abs(MOTOR_ENCODER_COUNT) > 70 || (num_turn == 0 && abs(MOTOR_ENCODER_COUNT) > 10))){
          prev_state = FW2CORNER_OC2;
          state = SCAN_CORNER_OC2;
          tar_ang += (is_anticlockwise == true) ? -right_ang : right_ang;
          num_turn += 1;
          break;
        } else steering_percentage = -(imu_yaw - (tar_ang + tar_ang_calibrate)) * imu_kp;
        break;
      
      case SCAN_CORNER_OC2:
        Serial.println("SCAN_CORNER_OC2");
        OC2_text = "SCAN_CORNER_OC2";
        tar_power = power;
        motor_move(tar_power);
        if (nearestPillarGlobal.colour == RED || nearestPillarGlobal.colour == GREEN){
          pillar_front = nearestPillarGlobal;
          pillar_front_info.pillar = pillar_front;
          pillar_front_info.enc_value = abs(MOTOR_ENCODER_COUNT);
          pillar_front_info.tof1_dist = dist_t1;
          pillar_front_info.tof2_dist = dist_t2;
          pillars_array.push_back(pillar_front_info);
          prev_state = SCAN_CORNER_OC2;
          state = WAIT_TURN_OC2;
          break;
        } else {
          prev_state = SCAN_CORNER_OC2;
          state = UNKNOWN_TURNING_OC2;
          break;
        }
        break;

      case CHANGE_LANE_P1_OC2:
        Serial.println("CHANGE_LANE_P1_OC2");
        OC2_text = "CHANGE_LANE_P1_OC2";
        tar_power = power;
        motor_move(tar_power);
        if (pillar_front.colour == RED){
          if (imu_yaw - tar_ang < change_lane_run_ang) steering_percentage = 100;
          else {
            prev_state = CHANGE_LANE_P1_OC2;
            state = CHANGE_LANE_P2_OC2;
            break;
          }
        } else {
          if (tar_ang - imu_yaw < change_lane_run_ang) steering_percentage = -100;
          else {
            prev_state = CHANGE_LANE_P1_OC2;
            state = CHANGE_LANE_P2_OC2;
            break;
          }
        }
        break;

      case CHANGE_LANE_P2_OC2:
        Serial.println("CHANGE_LANE_P2_OC2");
        OC2_text = "CHANGE_LANE_P2_OC2";
        tar_power = power;
        if (pillar_front.colour == RED){
          if (!motor_degree_accel(change_lane_run_dist - 25, (change_lane_run_dist - 25) / 2, (change_lane_run_dist - 25) / 2, tar_power, tar_power + 2)){
            steering_percentage = -(imu_yaw - (tar_ang + change_lane_run_ang)) * imu_kp;
          } else {
            prev_state = CHANGE_LANE_P2_OC2;
            state = CHANGE_LANE_P3_OC2;
            break;
          }
        } else {
          if (!motor_degree_accel((change_lane_run_dist - 25), (change_lane_run_dist - 25) / 2, (change_lane_run_dist - 25) / 2, tar_power, tar_power + 2)){
            steering_percentage = -(imu_yaw - (tar_ang - change_lane_run_ang)) * imu_kp;
          } else {
            prev_state = CHANGE_LANE_P2_OC2;
            state = CHANGE_LANE_P3_OC2;
            break;
          }
        }
        break;
      
      case CHANGE_LANE_P3_OC2:
        Serial.println("CHANGE_LANE_P3_OC2");
        OC2_text = "CHANGE_LANE_P3_OC2";
        tar_power = power;
        motor_move(tar_power);
        if (pillar_front.colour == RED) {
          if (abs(tar_ang - imu_yaw) > 10 && is_anticlockwise && imu_yaw > tar_ang) steering_percentage = -100;
          else if (abs(tar_ang - imu_yaw) > 10) steering_percentage = -100;
          else {
            steering_percentage = 0;
            prev_state = CHANGE_LANE_P3_OC2;
            state = FW2CORNER_OC2;
            break;
          }
        } else if (pillar_front.colour == GREEN){
          if (abs(tar_ang - imu_yaw) > 10 && !is_anticlockwise && tar_ang > imu_yaw) steering_percentage = 100;
          else if (abs(tar_ang - imu_yaw) > 10) steering_percentage = 100;
          else {
            steering_percentage = 0;
            prev_state = CHANGE_LANE_P3_OC2;
            state = FW2CORNER_OC2;
            break;
          }
        }
        break;
      
      case PURE_TURNING_OC2:
        Serial.println("PURE_TURNING_OC2");
        OC2_text = "PURE_TURNING_OC2";
        tar_power = power;
        motor_move(tar_power);
        reset_encoder();
        if ((abs(tar_ang - imu_yaw) > 10) && (abs(tar_ang) > abs(imu_yaw))){
          steering_percentage = (tar_ang > 0) ? 100 : -100;
          break;
        } else{
          prev_state = PURE_TURNING_OC2;
          state = INTO_SECTOR_OC2;
          reset_encoder();
          break;
        }
        break;
      
      case UNKNOWN_TURNING_OC2:
        Serial.println("UNKNOWN_TURNING_OC2");
        OC2_text = "UNKNOWN_TURNING_OC2";
        tar_power = power;
        motor_move(tar_power);
        reset_encoder();
        if (nearestPillarGlobal.colour != NO_COLOUR && nearestPillarGlobal.area >= 7){
          blk_while_turning = true;
          pillar_front = nearestPillarGlobal;
          pillar_front_info.pillar = pillar_front;
          pillar_front_info.enc_value = abs(MOTOR_ENCODER_COUNT);
          pillar_front_info.tof1_dist = dist_t1;
          pillar_front_info.tof2_dist = dist_t2;
          pillars_array.push_back(pillar_front_info);
          if (nearestPillarGlobal.colour == RED){
            if (nearestPillarGlobal.xpos >= min_xpos_Rcase_detect(nearestPillarGlobal.area) - 5){
              prev_state = UNKNOWN_TURNING_OC2;
              state = CURVE_BLK_STATE_OC2;
              break;
            } else {
              if (nearestPillarGlobal.area < 7) blk_area_after_turn = 7;
              else blk_area_after_turn = nearestPillarGlobal.area;
              prev_state = UNKNOWN_TURNING_OC2;
              state = SKIP_BLK_STATE_OC2;
              clone_blk_xpos = nearestPillarGlobal.xpos;
              break;
            }
          } else {
            if (nearestPillarGlobal.xpos <= min_xpos_Gcase(nearestPillarGlobal.area) + 5){
              prev_state = UNKNOWN_TURNING_OC2;
              state = CURVE_BLK_STATE_OC2;
              break; 
            } else {
              if (nearestPillarGlobal.area < 7) blk_area_after_turn = 7;
              else blk_area_after_turn = nearestPillarGlobal.area;
              prev_state = UNKNOWN_TURNING_OC2;
              state = SKIP_BLK_STATE_OC2;
              clone_blk_xpos = nearestPillarGlobal.xpos;
              break;
            }
          }
        } else if ((abs(tar_ang - imu_yaw) > 5) && (abs(tar_ang) > abs(imu_yaw))){
          steering_percentage = (tar_ang > 0) ? 100 : -100;
          break;
        } else{
          steering_percentage = 0;
          prev_state = UNKNOWN_TURNING_OC2;
          state = INTO_SECTOR_OC2;
          reset_encoder();
          break;
        }
        break;
      
      case CURVE_BLK_STATE_OC2:
        Serial.println("CURVE_BLK_STATE_OC2");
        OC2_text = "CURVE_BLK_STATE_OC2";
        if (num_turn == 0) tar_power = 8;
        else tar_power = power;
        motor_move(power);
        if (pillar_front.colour == RED){
          if (nearestPillarGlobal.xpos >= min_xpos_Rcase_detect(nearestPillarGlobal.area) + 5) blk_area_after_turn = nearestPillarGlobal.area;
          if (nearestPillarGlobal.xpos >= min_xpos_Rcase_detect(nearestPillarGlobal.area) - 5) {
            steering_percentage = 100;
            break;
          }
          else {
            steering_percentage = 0;
            prev_state = (prev_state == UNKNOWN_TURNING_OC2) ? UNKNOWN_TURNING_OC2 : CURVE_BLK_STATE_OC2;
            // prev_state = CURVE_BLK_STATE_OC2;
            state = SKIP_BLK_STATE_OC2;
            clone_blk_xpos = nearestPillarGlobal.xpos;
            if (blk_while_turning) reset_encoder();
            break;
          }
        } else {
          if (nearestPillarGlobal.xpos <= min_xpos_Gcase(nearestPillarGlobal.area) - 5) blk_area_after_turn = nearestPillarGlobal.area;
          if (nearestPillarGlobal.xpos <= min_xpos_Gcase(nearestPillarGlobal.area)) {
            steering_percentage = -100;
            break;
          } else {
            steering_percentage = 0;
            prev_state = (prev_state == UNKNOWN_TURNING_OC2) ? UNKNOWN_TURNING_OC2 : CURVE_BLK_STATE_OC2;
            // prev_state = CURVE_BLK_STATE_OC2;
            state = SKIP_BLK_STATE_OC2;
            clone_blk_xpos = nearestPillarGlobal.xpos;
            if (blk_while_turning) reset_encoder();
            break;
          }
        }
        break;
      
      case SKIP_BLK_STATE_OC2:
        Serial.println("SKIP_BLK_STATE_OC2");
        OC2_text = "SKIP_BLK_STATE_OC2";
        Serial.println(blk_area_after_turn);
        if (num_turn == 0) tar_power = 8;
        else tar_power = power;
        if (pillar_front.colour == RED){
          if (prev_state == UNKNOWN_TURNING_OC2 && is_anticlockwise) skip_blk_dist = area2dist_red(blk_area_after_turn) * ENC_PER_CM - 30;
          else skip_blk_dist = area2dist_red(blk_area_after_turn) * ENC_PER_CM - 20;
          if (skip_blk_dist < 0) skip_blk_dist = 5;
          Serial.println(skip_blk_dist);
          if (!motor_degree_accel(skip_blk_dist, skip_blk_dist / 2, skip_blk_dist / 2, tar_power, tar_power + 2)) {
            if (clone_blk_xpos - nearestPillarGlobal.xpos < 100 && nearestPillarGlobal.colour != NO_COLOUR) steering_percentage = -(min_xpos_Rcase_skip(nearestPillarGlobal.area) - nearestPillarGlobal.xpos) * 0.8;
            else steering_percentage = 0;
          } else {
            prev_state = SKIP_BLK_STATE_OC2;
            state = TURN_STRAIGHT_STATE_OC2;
            break;
          }
        } else {
          if (prev_state == UNKNOWN_TURNING_OC2 && !is_anticlockwise) skip_blk_dist = area2dist_green(blk_area_after_turn) - 20;
          skip_blk_dist = area2dist_green(blk_area_after_turn) - 15;
          if (skip_blk_dist < 0) skip_blk_dist = 5;
          Serial.println(skip_blk_dist);
          if (!motor_degree_accel(skip_blk_dist, skip_blk_dist / 2, skip_blk_dist / 2, tar_power, tar_power + 2)) {
            if (nearestPillarGlobal.xpos - clone_blk_xpos < 100 && nearestPillarGlobal.colour != NO_COLOUR) steering_percentage = -(min_xpos_Gcase(nearestPillarGlobal.area) - nearestPillarGlobal.xpos) * 0.8;
            else steering_percentage = 0;
          } else {
            prev_state = SKIP_BLK_STATE_OC2;
            state = TURN_STRAIGHT_STATE_OC2;
            break;
          }
        }
        clone_blk_xpos = nearestPillarGlobal.xpos;
        break;

      case TURN_STRAIGHT_STATE_OC2:
        Serial.println("TURN_STRAIGHT_STATE_OC2");
        OC2_text = "TURN_STRAIGHT_STATE_OC2";
        motor_move(tar_power);
        if (abs(tar_ang - imu_yaw) > turn_st_ang){
          steering_percentage = -(imu_yaw - tar_ang) * imu_kp;
        } else{
          prev_state = TURN_STRAIGHT_STATE_OC2;
          state = INTO_SECTOR_OC2;
          break;
        }
        break;

      case WAIT_TURN_OC2:
        Serial.println("WAIT_TURN_OC2");
        OC2_text = "WAIT_TURN_OC2";
        Serial.println(pillar_front.area);
        if (pillar_front.colour == RED){
          if (is_anticlockwise) {
            if (num_turn % 4 == 0) {
              if (!motor_degree_accel(50, 50 / 2, 50 / 2, tar_power, tar_power + 2)) {
                steering_percentage = 0;
              } else {
                prev_state = WAIT_TURN_OC2;
                state = PURE_TURNING_OC2;
                break;
              }
            } else {
              if (!motor_degree_accel(75, 75 / 2, 75 / 2, tar_power, tar_power + 2)) {
                steering_percentage = 0;
              } else {
                prev_state = WAIT_TURN_OC2;
                state = PURE_TURNING_OC2;
                break;
              }
            }
          } else {
            if (!motor_degree_accel(15, 15 / 2, 15 / 2, tar_power, tar_power + 2)) {
              steering_percentage = 0;
            } else {
              prev_state = WAIT_TURN_OC2;
              state = PURE_TURNING_OC2;
              break;
            }
          }
        } else {
          if (!is_anticlockwise){
            if (num_turn % 4 == 0){
              if (!motor_degree_accel(50, 50 / 2, 50 / 2, tar_power, tar_power + 2)) {
                steering_percentage = 0;
              } else {
                prev_state = WAIT_TURN_OC2;
                state = PURE_TURNING_OC2;
                break;
              }
            } else {
              if (!motor_degree_accel(75, 75 / 2, 75 / 2, tar_power, tar_power + 2)) {
                steering_percentage = 0;
              } else {
                prev_state = WAIT_TURN_OC2;
                state = PURE_TURNING_OC2;
                break;
              }
            }
          } else {
            if (!motor_degree_accel(15, 15 / 2, 15 / 2, tar_power, tar_power + 2)) {
              steering_percentage = 0;
            } else {
              prev_state = WAIT_TURN_OC2;
              state = PURE_TURNING_OC2;
              break;
            }
          }
        }
        break;

      case INTO_SECTOR_OC2:
        Serial.println("INTO_SECTOR_OC2");
        OC2_text = "INTO_SECTOR_OC2";
        motor_move(tar_power);
        if (dist_t1 > dist_threshold || dist_t2 > dist_threshold) steering_percentage = -(imu_yaw - (tar_ang + tar_ang_calibrate)) * imu_kp;
        else {
          prev_state = INTO_SECTOR_OC2;
          reset_encoder();
          if (dist_t1 + dist_t2 < 80) state = CHECK_BLK_BESIDES_OC2;
          else {
            state = SCAN_SECTOR_STATE_OC2;
            time_count = millis();
          }
          break;
        }
        break;

      case CHECK_BLK_BESIDES_OC2:
        Serial.println("CHECK_BLK_BESIDES_OC2");
        OC2_text = "CHECK_BLK_BESIDES_OC2";
        motor_move(tar_power);
        if (dist_t1 + dist_t2 < 80) steering_percentage = -(imu_yaw - (tar_ang + tar_ang_calibrate)) * imu_kp;
        else {
          prev_state = CHECK_BLK_BESIDES_OC2;
          time_count = millis();
          state = SCAN_SECTOR_STATE_OC2;
          break;
        }
        break;
      
      case ENDING_STATE_OC2:
        Serial.println("ENDING_STATE_OC2");
        OC2_text = "ENDING_STATE_OC2";
        // if (MOTOR_ENCODER_COUNT < abs(lane_length) / 2){
        //   steering_percentage = -(imu_yaw - tar_ang) * imu_kp;
        // } else {
          motor_stop(BRAKE);
          steering_percentage = 0;
          safeResume(blinkledThread);
          safeResume(OC1Thread);
          if (!is_anticlockwise) safeResume(ToF1Thread); //safeResume(ToF1Thread);
          else safeResume(ToF2Thread);
          end_game = true;
          run_OC2 = false;
          OC2_endtime = internalClock.read();
        // }
        break;

      case OUT_PARKING_P1_STATE:
        Serial.println("OUT_P1");
        if (abs(MOTOR_ENCODER_VALUE) < 25){
          steering_percentage = 0;
          motor_move(-8);
          break;
        } else {
          motor_stop(BRAKE);
          state = OUT_PARKING_P2_STATE;
          break;
        }
        break;

      case OUT_PARKING_P2_STATE:
        Serial.println("OUT_P2");
        if (abs(imu_yaw) < 94){
          if ((nearestPillarGlobal.colour == RED || nearestPillarGlobal.colour == GREEN) && nearestPillarGlobal.area > 60) outpark_hv_blk = true;
          steering_percentage = (is_anticlockwise) ? -100 : 100;
          motor_move(10);
          break;
        } else {
          motor_stop(BRAKE);
          reset_encoder();
          state = OUT_PARKING_P3_STATE;
          break;
        }
        break;

      case OUT_PARKING_P3_STATE:
        Serial.println("OUT_P3");
        if (abs(MOTOR_ENCODER_VALUE) < 45){
          steering_percentage = (is_anticlockwise) ? 25 : -25;
          motor_move(8);
          break;
        } else {
          motor_stop(BRAKE);
          state = OUT_PARKING_P4_STATE;
          break;
        }
        break;

      case OUT_PARKING_P4_STATE:
        Serial.println("OUT_P4");
        if (abs(imu_yaw) > 30){
          steering_percentage = (is_anticlockwise) ? -100 : 100;
          motor_move(-10);
          break;
        } else {
          motor_stop(BRAKE);
          reset_encoder();
          state = (outpark_hv_blk) ? OUT_PARKING_P5_STATE : FW2CORNER_OC2;
          break;
        }
        break;

      case OUT_PARKING_P5_STATE:
        Serial.println("OUT_P5");
        if (abs(MOTOR_ENCODER_VALUE) < 50){
          steering_percentage = 0;
          motor_move(-8);
          break;
        } else {
          motor_stop(BRAKE);
          reset_encoder();
          state = FW2CORNER_OC2;
          break;
        }
        break;

      case STOP_OC2:
        Serial.println("STOP_OC2");
        steering_percentage = 0;
        motor_stop(BRAKE);
        break;
    }
  }
  vTaskDelay(5 / portTICK_PERIOD_MS);
}

void OC2main(void *){
  while (1){
    if (is_btn_bumped(TFT_BTN2)){
      Serial.println("Hello World");
      run_OC2 = !run_OC2;
      reset_OC2 = true;
    }
    if (run_OC2){
      OC2_pixy2(90, 85, 10);
      // OC2_slow(90, 85, 8); //-80
    } else {
      // safeResume(displayThread);
      // safeResume(ToF1Thread);
      safeResume(ToF1Thread);
      safeResume(ToF2Thread);
      safeResume(OC1Thread);
      motor_stop(BRAKE);
      steering_percentage = 0;
    }
    vTaskDelay(5 / portTICK_PERIOD_MS);
  }
}

void showOC2Time(TFT_COLUMN column, int line_number, int text_size, uint16_t text_colour = TFT_WHITE, bool clearDisplay = false){
  static long total_ms = 0;
  static long seconds = 0;
  static long milliseconds = 0;
  total_ms = OC2_endtime - OC2_starttime;
  seconds = total_ms / 1000;
  milliseconds = total_ms % 1000;
  String OC2_time_text = String(seconds) + "." + String(milliseconds);
  String OC2_onoff_text = run_OC2 ? "OC2 ON" : "OC2 OFF";
  tft.clearln(TFT_LEFT_CLN, line_number);
  tft.clearln(TFT_RIGHT_CLN, line_number);
  tft.displayLeftln(line_number, text_size, OC2_time_text.c_str(), text_colour, false);
  tft.displayRightln(line_number, text_size, OC2_onoff_text.c_str(), run_OC2 ? TFT_GREEN : TFT_RED, false);
}