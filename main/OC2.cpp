#include <iterator>
#include <cmath>
#include "motor.h"
#include "pixy2lib.h"
#include "OC2.h"
#include "calculation.h"
using namespace std;

// A global variables for OC2
bool reset_OC2 = true;
long OC2_starttime = 0; 
long OC2_endtime = 0; 
bool run_OC2 = false;

// Functions for safely suspend and resume threads
static inline void safeSuspend(TaskHandle_t h){ if (h) vTaskSuspend(h); }
static inline void safeResume(TaskHandle_t h){ if (h) vTaskResume(h); }

/**
 * @brief Algorithm to calculate the safe xpos of the Red pillar seen according to its area from the Pixy2
 * @param area; int; the area of the pillar from the Pixy2
 *
 * @return float; the safe xpos of the pillar seen
*/
float min_xpos_Rcase_detect(int area){
  return -0.0008 * pow(area, 3) + 0.0948 * area * area - 4.6858 * area + 134.52; // - 70 * (1 - 1 / (area + 1))
}

/**
 * @brief Algorithm to calculate the safe xpos of the Red pillar seen according to its area from the Pixy2
 * @param area; int; the area of the pillar from the Pixy2
 *
 * @return float; the safe xpos of the pillar seen
*/
float min_xpos_Rcase_skip(int area){
  return -0.0008 * pow(area, 3) + 0.0948 * area * area - 4.6858 * area + 134.52 - 70; // - 70
}

/**
 * @brief Algorithm to calculate the safe xpos of the Green pillar seen according to its area from the Pixy2
 * @param area; int; the area of the pillar from the Pixy2
 *
 * @return float; the safe xpos of the pillar seen
*/
float min_xpos_Gcase(int area){
  return 0.0003 * pow(area, 3) - 0.051 * pow(area, 2) + 3.3764 * area + 170.59 + 55; // +45
}

/**
 * @brief Algorithm to calculate distance require the car to move such that it can pass through the Red block
 * @param area; int; the area of the pillar from the Pixy2
 *
 * @return float; the encoder value for the car to move
*/
float area2dist_red(int area){
  return 169.46 * pow(area, -0.486) - 10 + 25 * (7 / area);
}

/**
 * @brief Algorithm to calculate distance require the car to move such that it can pass through the Green block
 * @param area; int; the area of the pillar from the Pixy2
 *
 * @return float; the encoder value for the car to move
*/
float area2dist_green(int area){
  return 173 * pow(area, -0.477447023);
  // return 0.000000007 * pow(area, 6) - 0.000001983 * pow(area, 5) + 0.000239396 * pow(area, 4) - 327963 * area + 102.428377560 + 15 * (1 - area / 55);
}

/**
327963 * area + 102.428377560 + 15 * (1 - area / 55);
}

/**
 * @brief Algorithm to calculate angle required to turn for the car to do a successful lane changing
 * @param tof_dist; float; the innerwall distance returned by the ToF 
 *
 * @return float; the angle in degree for the car to turn
*/
float change_lane_ang(float tof_dist){
  float final_ang = rad2deg(atan((tof_dist - 15) / 40));
  if (final_ang < 15) final_ang = 15;
  return final_ang;
}

/**
 * @brief Algorithm to calculate distance required for the car to do a successful lane changing
 * @param tof_dist; float; the innerwall distance returned by the ToF 
 * @param init_ang; float; initial yaw angle of the car
 * @param final_ang; float; yaw angle the car should be facing
 *
 * @return float; the distance that the car has to travel
*/
float change_lane_dist_algo(float tof_dist, float init_ang, float final_ang){
  return (tof_dist * cos(deg2rad(init_ang)) - 15 - 10 * cos(deg2rad(90 - init_ang))) / sin(deg2rad(final_ang));
  // return (((tof_dist-15)) /cos(deg2rad(final_ang))) - 15 * cos(deg2rad(90 - init_ang));
}

/**
 * @brief Function for OC2 using Pixy2 camera over the run
 * 
 * @param right_ang; double; the value of an "right angle" for the IMU (if the value returned by the IMU is not consistent)
 * @param dist_threshold; int; the threshold that the ToFs consider as a turnable corner (in mm)
 * @param power; int; the power of the driving motor (0 ~ -100)
 * 
 */
void OC2_pixy2(double right_ang, int dist_threshold, int power){
  // Disable unecessary threads for efficiency
  safeSuspend(blinkledThread);
  safeSuspend(OC1Thread);

  // Variables required for OC1
  float imu_kp = 4.5;
  static OC2_STATES state = INIT_STATE_OC2; 
  static OC2_STATES prev_state = INIT_STATE_OC2;
  static bool end_game = false;             
  vector<BLK_INFO> pillars_array;    
  BLK_INFO pillar_front_info;
  static COLOURED_OBJ pillar_front;  
  static bool is_anticlockwise = true;
  static int tar_power = power;
  static String OC2_text = "INIT_STATE_OC2";
  static bool blk_while_turning = false;
  static double tar_ang = 0;          
  static double tar_ang_calibrate = -5;
  static int num_turn = 0;
  static float blk_area_after_turn = 0;
  int turn_st_ang = 15;
  static int clone_blk_xpos = 0;
  static int inner_wall_dist = 0;
  static int outer_wall_dist = 0;
  static float change_lane_run_dist = 0.0;
  static int change_lane_run_ang = 45;
  static float skip_blk_dist = 0;
  static bool outpark_hv_blk = false;
  static int time_count = 0;
  static float idk = 0;
  static int idk_color = 0;

  // Initializing variables for this function
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
    outer_wall_dist = 0;
    change_lane_run_dist = 0.0;
    change_lane_run_ang = 60;
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

    // Finite State Machine of this funciton
    switch (state) {
      // Initialize the run. Also detect the direction of the circuit
      case INIT_STATE_OC2:
        Serial.println("INIT_STATE_OC2");
        OC2_text = "INIT_STATE_OC2";
        motor_move(tar_power);
        tar_power = 0;
        if (dist_t1 > dist_t2) is_anticlockwise = true;
        else is_anticlockwise = false;
        prev_state = INIT_STATE_OC2;
        state = OUT_PARKING_P1_STATE;
        // idk_color = 2;
        // outer_wall_dist = dist_t2;
        // state = anti_in_parking_p1;
        tar_power = power;
        break;

      // Scan whether there is a block on this sector
      case SCAN_SECTOR_STATE_OC2:
        Serial.println("SCAN_SECTOR_STATE_OC2");
        OC2_text = "SCAN_SECTOR_STATE_OC2";
        tar_power = power;
        motor_move(tar_power);
        // if (num_turn >= 12){
        //   prev_state = SCAN_SECTOR_STATE_OC2;
        //   state = ENDING_STATE_OC2;
        //   break;
        // }
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
              if(is_anticlockwise){
                if (num_turn % 4 == 0) change_lane_run_ang = min_val(46, change_lane_ang(105 - pillar_front_info.tof1_dist));
                else change_lane_run_ang = change_lane_ang(112 - pillar_front_info.tof1_dist);
                change_lane_run_dist = change_lane_dist_algo(pillar_front_info.tof1_dist, imu_yaw - tar_ang, change_lane_run_ang);
                Serial.println(pillar_front_info.tof1_dist);
              } else {
                change_lane_run_ang = change_lane_ang(pillar_front_info.tof2_dist - 15);
                change_lane_run_dist = change_lane_dist_algo(pillar_front_info.tof2_dist, imu_yaw - tar_ang, change_lane_run_ang);
                Serial.println(pillar_front_info.tof2_dist);
              }
              Serial.println(change_lane_run_ang);
              Serial.println(change_lane_run_dist);
              if (change_lane_run_dist == 0) change_lane_run_dist = 1;//idk
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
              if(is_anticlockwise){
                change_lane_run_ang = change_lane_ang(pillar_front_info.tof1_dist - 10);
                change_lane_run_dist = change_lane_dist_algo(pillar_front_info.tof1_dist - 5, tar_ang - imu_yaw, change_lane_run_ang);
              } else {
                if (num_turn % 4 == 0) change_lane_run_ang = min_val(40, change_lane_ang(100 - pillar_front_info.tof2_dist));
                else change_lane_run_ang = change_lane_ang(95 - pillar_front_info.tof2_dist);
                change_lane_run_dist = change_lane_dist_algo(pillar_front_info.tof2_dist, tar_ang - imu_yaw, change_lane_run_ang);
              }
              Serial.println(change_lane_run_ang);
              Serial.println(change_lane_run_dist);
              if (change_lane_run_dist == 0) change_lane_run_dist = 1;//idk
              break; 
            } else {
              prev_state = SCAN_SECTOR_STATE_OC2;
              state = FW2CORNER_OC2;
              break;
            }
          }
        } else {
          // if (internalClock.read() - time_count < 1000){
          //   steering_percentage = 0;
          //   motor_stop(BRAKE);
          //   break;
          // } else {
            Serial.println("cant_see");
            prev_state = SCAN_SECTOR_STATE_OC2;
            state = FW2CORNER_OC2;
            break;
          // }
        }
        break;

      // Feedforward until ToF scans a corner
      case FW2CORNER_OC2:
        Serial.println("FW2CORNER_OC2");
        OC2_text = "FW2CORNER_OC2";
        tar_power = 10;
        motor_move(tar_power);
        if (((is_anticlockwise && dist_t1 > dist_threshold) || (!is_anticlockwise && dist_t2 > dist_threshold)) && (abs(MOTOR_ENCODER_COUNT) > 50 || (num_turn == 0 && abs(MOTOR_ENCODER_COUNT) > 10))){
          prev_state = FW2CORNER_OC2;
          state = Backward_until_threshold;
          tar_ang += (is_anticlockwise == true) ? -right_ang : right_ang;
          num_turn += 1;
          break;
        } else {
          steering_percentage = -(imu_yaw - (tar_ang + tar_ang_calibrate)) * imu_kp;
          motor_move(tar_power);
          break;
        }
        break;
      
      // Move backward until ToF scans the innerwall, which makes the turning later more accurate
      case Backward_until_threshold:
        Serial.println("Backward_until_threshold");
        if (num_turn >= 13){
          if (pillar_front.colour == RED) outer_wall_dist = dist_t2;
          else inner_wall_dist = dist_t1;
          prev_state = Backward_until_threshold;
          state = (is_anticlockwise) ? anti_in_parking_p1 : clkw_in_parking_p1;
          reset_encoder();
          tar_ang += (is_anticlockwise == true) ? right_ang : -right_ang;
          break;
        }
        if ((is_anticlockwise && dist_t1 > dist_threshold) || (!is_anticlockwise && dist_t2 > dist_threshold)){
          steering_percentage = 0;
          motor_move(-9);
          break;
        } else {
          prev_state = Backward_until_threshold;
          state = Forward_for_turn;
          reset_encoder();
          break;
        }
        break;

      // Move forward for a certain encoder value to prevent crashing the innerwall while turning
      case Forward_for_turn:
        Serial.println("Forward_for_turn");
        OC2_text = "Forard_for_turn";
        if(abs(MOTOR_ENCODER_COUNT) < 22){
          steering_percentage = 0;
          motor_move(9);
          break;
        } else{
          prev_state = Forward_for_turn;
          state = SCAN_CORNER_OC2;
          reset_encoder();
        }
        break;

      // Scan if there is any blocks at the start of the next sector
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

      // Turn to a certain yaw angle to prepare for a lane changing
      case CHANGE_LANE_P1_OC2:
        Serial.println("CHANGE_LANE_P1_OC2");
        OC2_text = "CHANGE_LANE_P1_OC2";
        tar_power = power;
        motor_move(tar_power);
        if (pillar_front.colour == RED){
          if (abs(imu_yaw - tar_ang) < change_lane_run_ang) steering_percentage = 100;
          else {
            reset_encoder();
            prev_state = CHANGE_LANE_P1_OC2;
            state = CHANGE_LANE_P2_OC2;
            break;
          }
        } else {
          if (abs(tar_ang - imu_yaw) < change_lane_run_ang) steering_percentage = -100;
          else {
            reset_encoder();
            prev_state = CHANGE_LANE_P1_OC2;
            state = CHANGE_LANE_P2_OC2;
            break;
          }
        }
        break;

      // Move diagonally (after turning in the previos state) to another lane
      case CHANGE_LANE_P2_OC2:
        Serial.println("CHANGE_LANE_P2_OC2");
        Serial.println(change_lane_run_dist);
        OC2_text = "CHANGE_LANE_P2_OC2";
        tar_power = power;
        if (pillar_front.colour == RED){
          //motor_on_degree(change_lane_run_dist - 0, tar_power, BRAKE)
          if (abs(MOTOR_ENCODER_COUNT) < change_lane_run_dist - 0){
            steering_percentage = -(imu_yaw - (tar_ang + change_lane_run_ang)) * imu_kp;
            motor_move(tar_power);
          } else {
            motor_stop(COAST);
            reset_encoder();
            prev_state = CHANGE_LANE_P2_OC2;
            state = CHANGE_LANE_P3_OC2;
            break;
          }
        } else {
          //motor_on_degree(change_lane_run_dist - 0, tar_power, BRAKE)
          if (abs(MOTOR_ENCODER_COUNT) < change_lane_run_dist - 0){
            steering_percentage = -(imu_yaw - (tar_ang - change_lane_run_ang)) * imu_kp;
            motor_move(tar_power);
          } else {
            motor_stop(COAST);
            reset_encoder();
            prev_state = CHANGE_LANE_P2_OC2;
            state = CHANGE_LANE_P3_OC2;
            break;
          }
        }
        break;
      
      // Turn back to the target angle
      case CHANGE_LANE_P3_OC2:
        Serial.println("CHANGE_LANE_P3_OC2");
        OC2_text = "CHANGE_LANE_P3_OC2";
        tar_power = power;
        motor_move(tar_power);
        if (pillar_front.colour == RED) {
          if (abs(tar_ang - imu_yaw) > 5 && is_anticlockwise && imu_yaw > tar_ang) steering_percentage = -100;
          else if (abs(tar_ang - imu_yaw) > 5) steering_percentage = -100;
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
      
      // Since a block is being detected at the start of the next sector, the car will first move to an optimal position before turning. This state
      // is responsible for the turning without doing any block scanning
      case PURE_TURNING_OC2:
        Serial.println("PURE_TURNING_OC2");
        OC2_text = "PURE_TURNING_OC2";
        tar_power = power;
        motor_move(tar_power);
        reset_encoder();
        if ((abs(tar_ang - imu_yaw) > 8) && (abs(tar_ang) > abs(imu_yaw))){
          steering_percentage = (tar_ang > 0) ? 100 : -100;
          break;
        } else {
          prev_state = PURE_TURNING_OC2;
          state = INTO_SECTOR_OC2;
          reset_encoder();
          break;
        }
        break;
      
      // Since no block is being detected at the start of the next sector, the car will scan for blocks while turning, in case there is any missing blocks
      case UNKNOWN_TURNING_OC2:
        Serial.println("UNKNOWN_TURNING_OC2");
        OC2_text = "UNKNOWN_TURNING_OC2";
        tar_power = power;
        motor_move(tar_power);
        reset_encoder();
        if (nearestPillarGlobal.colour != NO_COLOUR && nearestPillarGlobal.area >= 7 && ((is_anticlockwise && nearestPillarGlobal.xpos > 75) || (!is_anticlockwise && nearestPillarGlobal.xpos < 225))){
          Serial.print(nearestPillarGlobal.colour);
          Serial.print(" ");
          Serial.println(nearestPillarGlobal.xpos);
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
      
      // If a block is being detected while turning, this state will aid for a block passing turn
      case CURVE_BLK_STATE_OC2:
        Serial.println("CURVE_BLK_STATE_OC2");
        OC2_text = "CURVE_BLK_STATE_OC2";
        if (num_turn == 0) tar_power = 8;
        else tar_power = power;
        motor_move(power);
        if (pillar_front.colour == RED){
          if (nearestPillarGlobal.xpos >= min_xpos_Rcase_detect(nearestPillarGlobal.area) + 15) blk_area_after_turn = nearestPillarGlobal.area;
          if (nearestPillarGlobal.xpos >= min_xpos_Rcase_detect(nearestPillarGlobal.area) + 15) {
            steering_percentage = 100;
            break;
          }
          else {
            steering_percentage = 0;
            reset_encoder();
            prev_state = (prev_state == UNKNOWN_TURNING_OC2) ? UNKNOWN_TURNING_OC2 : CURVE_BLK_STATE_OC2;
            // prev_state = CURVE_BLK_STATE_OC2;
            state = SKIP_BLK_STATE_OC2;
            clone_blk_xpos = nearestPillarGlobal.xpos;
            if (blk_while_turning) reset_encoder();
            break;
          }
        } else {
          if (nearestPillarGlobal.xpos <= min_xpos_Gcase(nearestPillarGlobal.area) - 15) blk_area_after_turn = nearestPillarGlobal.area;
          if (nearestPillarGlobal.xpos <= min_xpos_Gcase(nearestPillarGlobal.area) - 15) {
            steering_percentage = -100;
            break;
          } else {
            steering_percentage = 0;
            reset_encoder();
            prev_state = (prev_state == UNKNOWN_TURNING_OC2) ? UNKNOWN_TURNING_OC2 : CURVE_BLK_STATE_OC2;
            // prev_state = CURVE_BLK_STATE_OC2;
            state = SKIP_BLK_STATE_OC2;
            clone_blk_xpos = nearestPillarGlobal.xpos;
            if (blk_while_turning) reset_encoder();
            break;
          }
        }
        break;
      
      // Move forward for a certain amount of encoder value to ensure the block is being passed
      case SKIP_BLK_STATE_OC2:
        Serial.println("SKIP_BLK_STATE_OC2");
        OC2_text = "SKIP_BLK_STATE_OC2";
        Serial.println(blk_area_after_turn);
        if (num_turn == 0) tar_power = 8;
        else tar_power = power;
        if (pillar_front.colour == RED){
          if (prev_state == UNKNOWN_TURNING_OC2 && is_anticlockwise) skip_blk_dist = area2dist_red(blk_area_after_turn) * ENC_PER_CM - 20;
          else skip_blk_dist = area2dist_red(blk_area_after_turn) * ENC_PER_CM - 10;
          if (skip_blk_dist <= 0) skip_blk_dist = 5;
          Serial.println(skip_blk_dist);
          skip_blk_dist = min_val(skip_blk_dist, 80);
          Serial.println(skip_blk_dist);
          //motor_on_degree(skip_blk_dist, tar_power, BRAKE)
          if (abs(MOTOR_ENCODER_COUNT) < skip_blk_dist) {
            if (clone_blk_xpos - nearestPillarGlobal.xpos < 100 && nearestPillarGlobal.colour != NO_COLOUR) steering_percentage = -(min_xpos_Rcase_skip(nearestPillarGlobal.area) - nearestPillarGlobal.xpos) * 0.8;
            else steering_percentage = 0;
            motor_move(tar_power);
          } else {
            motor_stop(COAST);
            // reset_encoder();
            prev_state = SKIP_BLK_STATE_OC2;
            state = TURN_STRAIGHT_STATE_OC2;
            break;
          }
        } else {
          if (prev_state == UNKNOWN_TURNING_OC2 && !is_anticlockwise) skip_blk_dist = area2dist_green(blk_area_after_turn) - 20;
          skip_blk_dist = area2dist_green(blk_area_after_turn) - 25;
          if (skip_blk_dist <= 0) skip_blk_dist = 5;
          Serial.println(skip_blk_dist);
          skip_blk_dist = min_val(skip_blk_dist, 80);
          Serial.println(skip_blk_dist);
          //motor_on_degree(skip_blk_dist, tar_power, BRAKE)
          if (abs(MOTOR_ENCODER_COUNT) < skip_blk_dist) {
            if (nearestPillarGlobal.xpos - clone_blk_xpos < 100 && nearestPillarGlobal.colour != NO_COLOUR) steering_percentage = -(min_xpos_Gcase(nearestPillarGlobal.area) - nearestPillarGlobal.xpos) * 0.8;
            else steering_percentage = 0;
            motor_move(tar_power);
          } else {
            motor_stop(COAST);
            reset_encoder();
            prev_state = SKIP_BLK_STATE_OC2;
            state = TURN_STRAIGHT_STATE_OC2;
            break;
          }
        }
        clone_blk_xpos = nearestPillarGlobal.xpos;
        break;

      // Turn back to the target angle after passing a block
      case TURN_STRAIGHT_STATE_OC2:
        Serial.println("TURN_STRAIGHT_STATE_OC2");
        OC2_text = "TURN_STRAIGHT_STATE_OC2";
        motor_move(tar_power);
        if (abs(tar_ang - imu_yaw) > turn_st_ang){
          steering_percentage = -(imu_yaw - tar_ang) * imu_kp;
        } else{
          reset_encoder();
          prev_state = TURN_STRAIGHT_STATE_OC2;
          state = INTO_SECTOR_OC2;
          break;
        }
        break;

      // Move forward for a certain amount of encoder value for turning with the existance of a block
      case WAIT_TURN_OC2:
        Serial.println("WAIT_TURN_OC2");
        OC2_text = "WAIT_TURN_OC2";
        Serial.println(pillar_front.area);
        if (pillar_front.colour == RED){
          if (is_anticlockwise) {
            if (num_turn % 4 == 0) {
              if (abs(MOTOR_ENCODER_COUNT) < 60) {
                steering_percentage = 0;
                motor_move(tar_power);
              } else {
                motor_stop(COAST);
                reset_encoder();
                prev_state = WAIT_TURN_OC2;
                state = PURE_TURNING_OC2;
                break;
              }
            } else {
              if (abs(MOTOR_ENCODER_COUNT) < 70) {
                steering_percentage = 0;
                motor_move(tar_power);
              } else {
                motor_stop(COAST);
                reset_encoder();
                prev_state = WAIT_TURN_OC2;
                state = PURE_TURNING_OC2;
                break;
              }
            }
          } else {
            // if (!motor_on_degree(30, tar_power, BRAKE)) {
            //   steering_percentage = 0;
            // } else {
              reset_encoder();
              prev_state = WAIT_TURN_OC2;
              state = PURE_TURNING_OC2;
              break;
            // }
          }
        } else {
          if (!is_anticlockwise){
            if (num_turn % 4 == 0){
              if (abs(MOTOR_ENCODER_COUNT) < 30) {
                steering_percentage = 0;
                motor_move(tar_power);
              } else {
                motor_stop(COAST);
                reset_encoder();
                prev_state = WAIT_TURN_OC2;
                state = PURE_TURNING_OC2;
                break;
              }
            } else {
              if (abs(MOTOR_ENCODER_COUNT) < 77) {
                steering_percentage = 0;
                motor_move(tar_power);
              } else {
                motor_stop(COAST);
                reset_encoder();
                prev_state = WAIT_TURN_OC2;
                state = PURE_TURNING_OC2;
                break;
              }
            }
          } else {
            // if (!motor_on_degree(5, tar_power, BRAKE)) {
            //   steering_percentage = 0;
            // } else {
              reset_encoder();
              prev_state = WAIT_TURN_OC2;
              state = PURE_TURNING_OC2;
              break;
            // }
          }
        }
        break;

      // Move forward after turning until the ToFs see the walls to prevent false detection
      case INTO_SECTOR_OC2:
        Serial.println(prev_state);
        Serial.println(" INTO_SECTOR_OC2");
        OC2_text = "INTO_SECTOR_OC2";
        motor_move(tar_power);
        if (dist_t1 > dist_threshold || dist_t2 > dist_threshold) {
          steering_percentage = -(imu_yaw - (tar_ang + tar_ang_calibrate)) * imu_kp;
          reset_encoder();
        } else {
          prev_state = INTO_SECTOR_OC2;
          if (dist_t1 + dist_t2 < 80 || abs(MOTOR_ENCODER_COUNT) < 14) steering_percentage = -(imu_yaw - (tar_ang + tar_ang_calibrate)) * imu_kp;
          else {
            state = SCAN_SECTOR_STATE_OC2;
            time_count = internalClock.read();
            break;
          }
          break;
        }
        break;

      // Check if there is any block blocking the ToF detection. If so, move forward until there is no block besides the car
      case CHECK_BLK_BESIDES_OC2:
        Serial.println("CHECK_BLK_BESIDES_OC2");
        OC2_text = "CHECK_BLK_BESIDES_OC2";
        motor_move(tar_power);
        if (dist_t1 + dist_t2 < 80) steering_percentage = -(imu_yaw - (tar_ang + tar_ang_calibrate)) * imu_kp;
        else {
          prev_state = CHECK_BLK_BESIDES_OC2;
          time_count = internalClock.read();
          state = SCAN_SECTOR_STATE_OC2;
          break;
        }
        break;
      
      // End the run by braking the car
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

      // Move backward for a preset encoder value to leave the parking lot
      case OUT_PARKING_P1_STATE:
        Serial.println("OUT_P1");
        if (abs(MOTOR_ENCODER_VALUE) < 28){
          steering_percentage = 0;
          motor_move(-10);
          break;
        } else {
          motor_stop(BRAKE);
          state = OUT_PARKING_P2_STATE;
          break;
        }
        break;

      // Turn to a certain degree, while scanning if there is any block in front
      case OUT_PARKING_P2_STATE:
        Serial.println("OUT_P2");
        idk = (is_anticlockwise) ? 97 : 88;
        if (abs(imu_yaw) < idk){
          if (nearestPillarGlobal.colour == RED && nearestPillarGlobal.area > 60){
            idk_color = 1;
            outpark_hv_blk = true;
          } else if (nearestPillarGlobal.colour == GREEN && nearestPillarGlobal.area > 40){
            idk_color = 2;
            outpark_hv_blk = true;
          }
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

      // Move forward while avoding any places with blocks
      case OUT_PARKING_P3_STATE:
        Serial.println("OUT_P3");
        if (abs(MOTOR_ENCODER_VALUE) < 43){
          steering_percentage = (is_anticlockwise) ? 30 : -30;
          motor_move(10);
          break;
        } else {
          motor_stop(BRAKE);
          reset_encoder();
          if (is_anticlockwise){
            state = OUT_PARKING_P4_STATE;
          } else {
            if (outpark_hv_blk){
              if (idk_color == 1) state = OUT_PARKING_CLKW_RED_P1;
              else state = OUT_PARKING_CLKW_GREEN_P1;
            } else {
              state = OUT_PARKING_P4_STATE;
            }
          }
          break;
        }
        break;

      // Turn to the target angle
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

      // Move backward for more spaces in front of the car
      case OUT_PARKING_P5_STATE:
        Serial.println("OUT_P5");
        inner_wall_dist = (is_anticlockwise) ? 60 : 15;
        if (abs(MOTOR_ENCODER_VALUE) < inner_wall_dist){
          steering_percentage = 0;
          motor_move(-10);
          break;
        } else {
          motor_stop(BRAKE);
          reset_encoder();
          state = (outpark_hv_blk && idk_color == 1) ? OUT_PARKING_ANTI_RED_P1 : SCAN_SECTOR_STATE_OC2;
          break;
        }
        break;

      case OUT_PARKING_ANTI_RED_P1:
        Serial.println("OUT_PARKING_ANTI_RED_P1");
        if (abs(imu_yaw) < 25){
          steering_percentage = 100;
          motor_move(10);
          break;
        } else {
          motor_stop(BRAKE);
          reset_encoder();
          state = OUT_PARKING_ANTI_RED_P2;
          break;
        }
        break;

      case OUT_PARKING_ANTI_RED_P2:
        Serial.println("OUT_PARKING_ANTI_RED_P2");
        if (abs(imu_yaw) > 10){
          steering_percentage = -100;
          motor_move(10);
          break;
        } else {
          motor_stop(BRAKE);
          reset_encoder();
          state = FW2CORNER_OC2;
          break;
        }
        break;

      // Turn to pass through the Red block (if any) after leaving the parking lot
      case OUT_PARKING_CLKW_RED_P1:
        Serial.println("OUT_PARKING_CLKW_RED_P1");
        if (abs(imu_yaw) > 75){
          steering_percentage = 100;
          motor_move(-10);
          break;
        } else {
          motor_stop(BRAKE);
          reset_encoder();
          state = OUT_PARKING_CLKW_RED_P2;
          break;
        }
        break;

      // Turn to pass through the Red block (if any) after leaving the parking lot
      case OUT_PARKING_CLKW_RED_P2:
        Serial.println("OUT_PARKING_CLKW_RED_P2");
        if (abs(imu_yaw) > 10){
          steering_percentage = -100;
          motor_move(10);
          break;
        } else {
          motor_stop(BRAKE);
          reset_encoder();
          state = FW2CORNER_OC2;
          break;
        }
        break;

      // Pass through the Green block (if any) after leaving the parking lot
      case OUT_PARKING_CLKW_GREEN_P1:
        Serial.println("OUT_PARKING_CLKW_GREEN_P1");
        if (abs(imu_yaw) > 10){
          steering_percentage = 100;
          motor_move(-10);
          break;
        } else {
          motor_stop(BRAKE);
          reset_encoder();
          state = OUT_PARKING_CLKW_GREEN_P2;
          break;
        }
        break;

      // Pass through the Green block (if any) after leaving the parking lot
      case OUT_PARKING_CLKW_GREEN_P2:
        Serial.println("OUT_PARKING_CLKW_GREEN_P2");
        if (abs(MOTOR_ENCODER_COUNT) < 30){
          steering_percentage = 0;
          motor_move(9);
          break;
        } else {
          motor_stop(BRAKE);
          reset_encoder();
          state = FW2CORNER_OC2;
          break;
        }
        break;

      // Debug state for stopping the OC2 run
      case STOP_OC2:
        Serial.println("STOP_OC2");
        steering_percentage = 0;
        motor_stop(BRAKE);
        break;

      // A parking state if the run direction is anti-clockwise. Move backward for a certain encoder value
      case anti_in_parking_p1:
        if (abs(MOTOR_ENCODER_COUNT) < 35){
          steering_percentage = 0;
          motor_move(-8);
          break;
        } else {
          motor_stop(BRAKE);
          if (pillar_front.colour == GREEN) inner_wall_dist = dist_t1;
          else outer_wall_dist = dist_t2; //if (idk_color == 2) 
          Serial.println(inner_wall_dist);
          state = anti_in_parking_p2;
          reset_encoder();
          break;
        }
        break;

      // A parking state if the run direction is anti-clockwise. Move forward for a certain encoder value
      case anti_in_parking_p2:
        if (abs(MOTOR_ENCODER_COUNT) < 38){
          steering_percentage = -(imu_yaw - (tar_ang + tar_ang_calibrate)) * imu_kp;
          motor_move(8);
          break;
        } else {
          motor_stop(BRAKE);
          state = anti_in_parking_p3;
          reset_encoder();
          break;
        }
        break;

      // A parking state if the run direction is anti-clockwise. Make a turn to get into the parking lot
      case anti_in_parking_p3:
        if (abs(imu_yaw - tar_ang) < 82){
          steering_percentage = 100;
          motor_move(-9);
          break;
        } else {
          motor_stop(BRAKE);
          state = anti_in_parking_p4;
          reset_encoder();
          break;
        }
        break;

      // A parking state if the run direction is anti-clockwise. Make a turn to get into the parking
      case anti_in_parking_p4:
        idk = (pillar_front.colour == RED) ? 1.0 * outer_wall_dist : 1.2 * abs(42.0 - inner_wall_dist);
        Serial.println(idk);
        if (abs(MOTOR_ENCODER_COUNT) < idk){
          steering_percentage = 0;
          if (idk_color == 1) motor_move(9);
          else motor_move(-9);
          break;
        } else {
          motor_stop(BRAKE);
          state = anti_in_parking_p5;
          reset_encoder();
          break;
        }
        break;

      // A parking state if the run direction is anti-clockwise. Make a turn to get into the parking lot for a parallel parking
      case anti_in_parking_p5:
        if (abs(imu_yaw - tar_ang) < 172){
          steering_percentage = 100;
          motor_move(-9);
          break;
        } else {
          motor_stop(BRAKE);
          state = anti_in_parking_p6;
          reset_encoder();
          break;
        }
        break;

      // A parking state if the run direction is anti-clockwise. Move forward until the car is fully into the parking lot
      case anti_in_parking_p6:
        if (abs(MOTOR_ENCODER_COUNT) < 20){
          steering_percentage = 0;
          motor_move(8);
          break;
        } else {
          motor_stop(BRAKE);
          state = ENDING_STATE_OC2;
          reset_encoder();
          break;
        }
        break;

      // A parking state if the run direction is clockwise. Move backward for a certain encoder value
      case clkw_in_parking_p1:
        if (abs(MOTOR_ENCODER_COUNT) < 25){
          steering_percentage = 0;
          motor_move(-8);
          break;
        } else {
          motor_stop(BRAKE);
          inner_wall_dist = dist_t2;
          state = clkw_in_parking_p2;
          reset_encoder();
          break;
        }
        break;

      // A parking state if the run direction is clockwise. Move forward for a certain encoder value
      case clkw_in_parking_p2:
        if (abs(MOTOR_ENCODER_COUNT) < 25){
          steering_percentage = 0;
          motor_move(-8);
          break;
        } else {
          motor_stop(BRAKE);
          state = clkw_in_parking_p3;
          reset_encoder();
          break;
        }
        break;

      // A parking state if the run direction is clockwise. Make a turn to get into the parking lot
      case clkw_in_parking_p3:
        if (abs(imu_yaw - tar_ang) < 73){
          steering_percentage = -100;
          motor_move(-10);
          break;
        } else {
          motor_stop(BRAKE);
          state = clkw_in_parking_p4;
          reset_encoder();
          break;
        }
        break;

      // A parking state if the run direction is clockwise. Make a turn to get into the parking for a parallel parking
      case clkw_in_parking_p4:
        if (abs(MOTOR_ENCODER_COUNT) < 1.25 * abs(41.5 - inner_wall_dist)){
          steering_percentage = 0;
          if (45 - inner_wall_dist > 0) motor_move(-9);
          else motor_move(9);
          break;
        } else {
          motor_stop(BRAKE);
          state = clkw_in_parking_p5;
          reset_encoder();
          break;
        }
        break;

      // A parking state if the run direction is clockwise. Move forward until the car is fully into the parking lot
      case clkw_in_parking_p5:
        if (abs(imu_yaw - tar_ang) > 9){
          steering_percentage = 100;
          motor_move(-9);
          break;
        } else {
          motor_stop(BRAKE);
          state = anti_in_parking_p6;
          reset_encoder();
          break;
        }
        break;

    }
  }
  vTaskDelay(5 / portTICK_PERIOD_MS);
}

/**
 * @brief The main thread function for OC1
 */
void OC2main(void *){
  while (1){
    if (is_btn_bumped(TFT_BTN2)){
      Serial.println("Hello World");
      run_OC2 = !run_OC2;
      reset_OC2 = true;
    }
    if (run_OC2){
      OC2_pixy2(90, 100, 11);
    } else {
      safeResume(ToF1Thread);
      safeResume(ToF2Thread);
      safeResume(OC1Thread);
      motor_stop(BRAKE);
      steering_percentage = 0;
    }
    vTaskDelay(5 / portTICK_PERIOD_MS);
  }
}

/**
 * @brief Function to print the time and status of OC2 run on the LCD monitor
 * 
 * @param column; TFT_COLUMN; the column on the LCD where the info is being printed
 * @param line_number; int; the line number on the LCD where the info is being printed
 * @param text_size; int; text size of the info on the LCD (1 or 2)
 * @param text_colour; uint16_t; text colour printed on the LCD
 * @param clearDisplay; bool; (UNUSED) whether the display will be cleared before running
 * 
 */
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