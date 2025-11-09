#include "motor.h"
#include "pixy2lib.h"
#include "OC2.h"

bool reset_OC2 = true;
long OC2_starttime = 0; 
long OC2_endtime = 0; 

static inline void safeSuspend(TaskHandle_t h){ if (h) vTaskSuspend(h); }
static inline void safeResume(TaskHandle_t h){ if (h) vTaskResume(h); }

/**
 * @brief Algorithm to calculate the safe xpos of the Red pillar seen according to its area from the huskylens
 * @param area; int; the area of the pillar from the huskylens
 *
 * @return float; the safe xpos of the pillar seen
*/
float min_xpos_Rcase(int area){
  return -35.34 * log(area) + 179.99;
}

/**
 * @brief Algorithm to calculate the safe xpos of the Green pillar seen according to its area from the huskylens
 * @param area; int; the area of the pillar from the huskylens
 *
 * @return float; the safe xpos of the pillar seen
*/
float min_xpos_Gcase(int area){
  return 33.605 * log(area) + 131.5;
}

float area2dist_red(int area){
  return -19.22 * log(area) + 97.475;
}

float area2dist_green(int area){
  return -15.98 * log(area) + 94.5;
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
  std::vector<COLOURED_OBJ> pillars_array;    // An array storing all the pillars detected
  static COLOURED_OBJ pillar_front;           // A struct storing all the info of the nearest pillar detected during DETECT_STATE
  const int area_dangerzone = 40;             // A variable representing the area of a pillar which is identified as danger when its area is larger than this value
  const float steering_amp_fact1 = 250; // 150       // An amplification factor for calculating the steering percentage during CURVE_P1_STATE
  const float steering_amp_fact2 = 3;         // An amplification factor for calculating the steering percentage during CURVE_P3_STATE
  static int gyro_ang_detect_phase = 0;       // A variable storing the current IMU angle
  static int pillar_avoid_save_t = 0;         // A variable storing the instant time for avoiding pillars
  static float curve_phase2_chk_t = 300;      // A variable storing the time required for the car to move straight forward during CURVE_P2_STATE. Non-editable as it will be recalculated during the run
  static float curve_sphase3_chk_t = 300;     // A variable storing the time required for the car to move straight forward during CURVE_P2_5_STATE. Non-editable as it will be recalculated during the run
  static int curve_phase1_gyro_chkpt = 45;    // A variable storing the IMU value for the car to turn to during CURVE_P1_STATE. Non-editable as it will be recalculated during the run
  static int mid_phase1_gyro_chkpt = 60;      // A variable storing the IMU value for the car to turn to during MID_P1_STATE. Non-editable as it will be recalculated during the run
  static float inner_wall_dist = 0;           // A variable storing the distance between the car and the inner wall (measured using ultrasonic sensor)
  static int mid_save_t = 0;                  // A variable storing the instant time for going back to mid racing line
  static float mid_phase2_chk_t = 0;          // A variable storing the time required for the car to move straight forward during MID_P2_STATE. Non-editable as it will be recalculated during the run
  static int mid_dash_save_t = 0;             // A variable storing the time required for the car to move straight forward during MID_DASH_STATE. Non-editable as it will be recalculated during the run
  static double tar_ang = 0;                  // The target angle which the car should be facing
  static double tar_ang_calibrate = 0;
  static long dash_timeZero = 0;              // Variable to store the instant time from the internal clock for dashing
  static long dash_time = 0;                  // The time required for the car to run before using the ultrasonic sensors for detection again after turning
  static bool is_anticlockwise = true;          // A boolean storing whether the car is racing in clockwise or anti-clockwise direction
  static int num_turn = 0;                    // Variable storing the number of turns the car has made
  static long turn_waittimeZero = 0;          // Variable to store the instant time from the internal clock for waiting to turning
  static int turn_waittime = 150;             // Variable storing the time in ms that required to wait before the car turn 
  static bool blk_while_turning = false;      // A flag determining whether any pillars in close distance is detected during the turning
  static int change_state_time = 0;
  static double prev_inner_dist = 0;
  static double cur_inner_dist = 0;
  static double drift_ang = 0;
  static double steering_for_drift = 0;
  static double detect_drifting_t = 0;
  static double drift_tuning_t = 0;
  static int num_reset_imu = 0;
  static double reset_imu_t = 0;
  static bool initial_drift_detect = true;
  static double speed_amp_factor = -80 / power;
  static double encoder_counter = 0;
  static bool last_sector_no_blk = false;
  static int CP4_t = 0;
  static bool from_tNa_state = false;
  static int u1_ind = 0;
  static int u2_ind = 0;
  static int motor_last_counter = 0;
  static int dir_color = 2; // 8 // 2: blue, 8: orange
  static int front_wall_dist = 0;
  static int side_wall_dist = 0;
  static double mid_dist = 0;
  static int check_front_t = 0;
  static int gyro_offset = 0;
  static bool blk_be4_turning = false;
  static int check_mid_t = 0;
  static bool mid_already = false;
  static int turn4_deg = 0;
  static int tar_power = power;

  if (reset_OC2){
    tar_ang = 0;
    tar_ang_calibrate = 0;
    mid_already = false;
    gyro_offset = 0;
    turn4_deg = 0;
    check_mid_t = 0;
    blk_be4_turning = false;
    is_anticlockwise = false;
    num_turn = 0;
    steering_percentage = 0;
    end_game = false;
    state = INIT_STATE_OC2;
    gyro_ang_detect_phase = 0;
    pillars_array.clear();
    pillar_avoid_save_t = 0;
    curve_phase2_chk_t = 300;
    curve_sphase3_chk_t = 300;
    curve_phase1_gyro_chkpt = 45;
    mid_phase1_gyro_chkpt = 50;
    inner_wall_dist = 0;
    mid_save_t = 0;
    mid_phase2_chk_t = 0;
    mid_dash_save_t = 0;
    dash_timeZero = 0;
    dash_time = 0;
    num_turn = 0;
    turn_waittimeZero = 0;
    turn_waittime = 150;
    blk_while_turning = false;
    tar_power = power;
    OC2_starttime = internalClock.read();
    safeSuspend(blinkledThread);
    imu_resetYaw();
    reset_OC2 = false;
  }

  if (!end_game){
    OC2_endtime = internalClock.read();

    switch (state) {
      case INIT_STATE_OC2:
        Serial.println("INIT_STATE_OC2");
        motor_move(tar_power);
        tar_power = 0;
        if (ultra1Dist > ultra2Dist) is_anticlockwise = true;
        else is_anticlockwise = false;
        prev_state = INIT_STATE_OC2;
        state = DETECT_STATE_OC2;
        motor_last_counter = abs(MOTOR_ENCODER_COUNT);
        break;

      case DETECT_STATE_OC2:
        Serial.print("DETECT_STATE_OC2");
        motor_move(tar_power);
        tar_power = 10;
        if (!is_anticlockwise){
          if (imu_yaw >= 315){
            prev_state = DETECT_STATE_OC2;
            state = ENDING_STATE_OC2;
            tar_ang_calibrate = 0;
            break;
          }
        } else {
          if (imu_yaw <= -315){
            prev_state = DETECT_STATE_OC2;
            state = ENDING_STATE_OC2;
            tar_ang_calibrate = 0;
            break;
          }
        }
        if (((is_anticlockwise && ultra1Dist > dist_threshold) || (!is_anticlockwise && ultra2Dist > dist_threshold)) && (abs(MOTOR_ENCODER_COUNT) > 50 * ENC_PER_CM)){
          prev_state = DETECT_STATE_OC2;
          state = TURNING_STATE_OC2;
          tar_ang_calibrate = 0;
          tar_ang += (is_anticlockwise == true) ? -right_ang : right_ang;
          break;
        } else if (nearestPillarGlobal.colour != NO_COLOUR && nearestPillarGlobal.area >= 3){
          if (nearestPillarGlobal.colour == RED){
            if (nearestPillarGlobal.xpos >= min_xpos_Rcase(nearestPillarGlobal.area)){
              pillars_array.push_back(nearestPillarGlobal);
              pillar_front = nearestPillarGlobal;
              prev_state = DETECT_STATE_OC2;
              state = CURVE_BLK_STATE_OC2;
              tar_ang_calibrate = 0;
              break;
            } else {
              prev_state = DETECT_STATE_OC2;
              state = SKIP_BLK_STATE_OC2;
              tar_ang_calibrate = 0;
              break;
            }
          } else {
            if (nearestPillarGlobal.xpos <= min_xpos_Gcase(nearestPillarGlobal.area)){
              pillars_array.push_back(nearestPillarGlobal);
              pillar_front = nearestPillarGlobal;
              prev_state = DETECT_STATE_OC2;
              state = CURVE_BLK_STATE_OC2;
              tar_ang_calibrate = 0;
              break; 
            } else {
              prev_state = DETECT_STATE_OC2;
              state = SKIP_BLK_STATE_OC2;
              tar_ang_calibrate = 0;
              break;
            }
          }
        } 
        else if (ultra3Dist < 20 && nearestPillarGlobal.colour == NO_COLOUR){
          prev_state = DETECT_STATE_OC2;
          state = BACKWARD_STATE_OC2;
          tar_ang_calibrate = 0;
          tar_ang += (is_anticlockwise == true) ? -right_ang : right_ang;
          reset_encoder();
          break;
        } 
        else if (ultra1Dist < 20) {
          tar_ang_calibrate = 5;
        } else if (ultra2Dist < 20) {
          tar_ang_calibrate = -5;
        } else {
          steering_percentage = -(imu_yaw - (tar_ang + tar_ang_calibrate)) * imu_kp;
          break;
        }
        break;

      case BACKWARD_STATE_OC2:
        Serial.println("TURNING_STATE_OC2");
        if (abs(MOTOR_ENCODER_COUNT) < 50 * ENC_PER_CM) {
          motor_move(-tar_power);
          break;
        } else {
          motor_stop(BRAKE);
          prev_state = BACKWARD_STATE_OC2;
          state = TURNING_STATE_OC2;
          break;
        }

      case TURNING_STATE_OC2:
        Serial.println("TURNING_STATE_OC2");
        motor_move(tar_power);
        if (nearestPillarGlobal.colour != NO_COLOUR && nearestPillarGlobal.area >= 5){
          if (nearestPillarGlobal.colour == RED){
            if (nearestPillarGlobal.xpos >= min_xpos_Rcase(nearestPillarGlobal.area) - 5){
              pillars_array.push_back(nearestPillarGlobal);
              pillar_front = nearestPillarGlobal;
              prev_state = TURNING_STATE_OC2;
              state = CURVE_BLK_STATE_OC2;
              break;
            } else {
              prev_state = TURNING_STATE_OC2;
              state = SKIP_BLK_STATE_OC2;
              break;
            }
          } else {
            if (nearestPillarGlobal.xpos <= min_xpos_Gcase(nearestPillarGlobal.area) + 5){
              pillars_array.push_back(nearestPillarGlobal);
              pillar_front = nearestPillarGlobal;
              prev_state = TURNING_STATE_OC2;
              state = CURVE_BLK_STATE_OC2;
              break; 
            } else {
              prev_state = TURNING_STATE_OC2;
              state = SKIP_BLK_STATE_OC2;
              break;
            }
          }
        } else if ((abs(tar_ang - imu_yaw) > 25) && (abs(tar_ang) > abs(imu_yaw))){
          steering_percentage = (tar_ang > 0) ? 100 : -100;
        } else{
          steering_percentage = 0;
          prev_state = TURNING_STATE_OC2;
          state = DETECT_STATE_OC2;
          reset_encoder();
          break;
        }
        break;

      case CURVE_BLK_STATE_OC2:
        Serial.println("CURVE_BLK_STATE_OC2");
        motor_move(tar_power);
        if (pillar_front.colour == RED){
          if (nearestPillarGlobal.xpos >= min_xpos_Rcase(nearestPillarGlobal.area)) {
            steering_percentage = 100;
            break;
          }
          else {
            steering_percentage = 0;
            prev_state = CURVE_BLK_STATE_OC2;
            state = SKIP_BLK_STATE_OC2;
            break;
          }
        } else {
          if (nearestPillarGlobal.xpos <= min_xpos_Gcase(nearestPillarGlobal.area)) {
            steering_percentage = -100;
            break;
          } else {
            steering_percentage = 0;
            prev_state = CURVE_BLK_STATE_OC2;
            state = SKIP_BLK_STATE_OC2;
            break;
          }
        }
        break;

      case SKIP_BLK_STATE_OC2:
        Serial.println("SKIP_BLK_STATE_OC2");
        if (pillar_front.colour == RED){
          if (!motor_degree_accel(area2dist_red(pillar_front.area), area2dist_red(pillar_front.area) / 2, area2dist_red(pillar_front.area) / 2, 8, 12)) steering_percentage = 0;
          else {
            prev_state = SKIP_BLK_STATE_OC2;
            state = TURN_STRAIGHT_STATE_OC2;
            break;
          }
        } else {
          if (!motor_degree_accel(area2dist_green(pillar_front.area), area2dist_green(pillar_front.area) / 2, area2dist_green(pillar_front.area) / 2, 8, 13)) steering_percentage = 0;
          else {
            prev_state = SKIP_BLK_STATE_OC2;
            state = TURN_STRAIGHT_STATE_OC2;
            break;
          }
        }
        break;
      
      case TURN_STRAIGHT_STATE_OC2:
        Serial.println("TURN_STRAIGHT_STATE_OC2");
        motor_move(tar_power);
        if (abs(tar_ang - imu_yaw) > 20){
          steering_percentage = -(imu_yaw - tar_ang) * imu_kp;
        } else{
          prev_state = TURN_STRAIGHT_STATE_OC2;
          state = DETECT_STATE_OC2;
          break;
        }
        break;
      
      case ENDING_STATE_OC2:
        Serial.println("ENDING_STATE_OC2");
        // if (MOTOR_ENCODER_COUNT < abs(lane_length) / 2){
        //   steering_percentage = -(imu_yaw - tar_ang) * imu_kp;
        // } else {
          motor_stop(BRAKE);
          steering_percentage = 0;
          safeResume(blinkledThread);
          safeResume(OC1Thread);
          if (!is_anticlockwise) safeResume(Ultra1Thread);
          else safeResume(Ultra2Thread);
          end_game = true;
          OC2_endtime = internalClock.read();
        // }
        break;
    }
  }
  vTaskDelay(5 / portTICK_PERIOD_MS);
}

void OC2_slow(double right_ang, int dist_threshold, int power){
  safeSuspend(blinkledThread);
  safeSuspend(OC1Thread);

  float imu_kp = 4.5;
  static OC2_STATES state = INIT_STATE_OC2; // A variable storing the current state of OC2 run
  static OC2_STATES prev_state = INIT_STATE_OC2;
  static bool end_game = false;               // A flag determining whether the run has ended
  std::vector<COLOURED_OBJ> pillars_array;    // An array storing all the pillars detected
  static COLOURED_OBJ pillar_front;           // A struct storing all the info of the nearest pillar detected during DETECT_STATE
  static float inner_wall_dist = 0;           // A variable storing the distance between the car and the inner wall (measured using ultrasonic sensor)
  static int outer_wall_dist = 0;
  static double tar_ang = 0;                  // The target angle which the car should be facing
  static double tar_ang_calibrate = 0;
  static long dash_timeZero = 0;              // Variable to store the instant time from the internal clock for dashing
  static long dash_time = 0;                  // The time required for the car to run before using the ultrasonic sensors for detection again after turning
  static bool is_anticlockwise = true;          // A boolean storing whether the car is racing in clockwise or anti-clockwise direction
  static int num_turn = 0;                    // Variable storing the number of turns the car has made
  static long turn_waittimeZero = 0;          // Variable to store the instant time from the internal clock for waiting to turning
  static int turn_waittime = 150;             // Variable storing the time in ms that required to wait before the car turn 
  static bool blk_while_turning = false;      // A flag determining whether any pillars in close distance is detected during the turning
  static int change_state_time = 0;
  static double prev_inner_dist = 0;
  static double cur_inner_dist = 0;
  static int motor_last_counter = 0;
  static int tar_power = power;

  if (reset_OC2){
    tar_ang = 0;
    tar_ang_calibrate = 0;
    is_anticlockwise = false;
    num_turn = 0;
    steering_percentage = 0;
    end_game = false;
    state = INIT_STATE_OC2;
    gyro_ang_detect_phase = 0;
    pillars_array.clear();
    inner_wall_dist = 0;
    dash_timeZero = 0;
    dash_time = 0;
    num_turn = 0;
    turn_waittimeZero = 0;
    turn_waittime = 150;
    blk_while_turning = false;
    tar_power = power;
    OC2_starttime = internalClock.read();
    safeSuspend(blinkledThread);
    imu_resetYaw();
    reset_OC2 = false;
  }

  if (!end_game){
    OC2_endtime = internalClock.read();

    switch (state) {
      case INIT_STATE_OC2:
        Serial.println("INIT_STATE_OC2");
        motor_move(tar_power);
        tar_power = 0;
        if (ultra1Dist > ultra2Dist) is_anticlockwise = true;
        else is_anticlockwise = false;
        prev_state = INIT_STATE_OC2;
        state = DETECT_STATE_OC2;
        motor_last_counter = abs(MOTOR_ENCODER_COUNT);
        break;

      case DETECT_STATE_OC2:
        Serial.print("DETECT_STATE_OC2");
        motor_move(tar_power);
        tar_power = 10;
        if (!is_anticlockwise){
          if (imu_yaw >= 315){
            prev_state = DETECT_STATE_OC2;
            state = ENDING_STATE_OC2;
            tar_ang_calibrate = 0;
            break;
          }
        } else {
          if (imu_yaw <= -315){
            prev_state = DETECT_STATE_OC2;
            state = ENDING_STATE_OC2;
            tar_ang_calibrate = 0;
            break;
          }
        }
        if (ultra3Dist < 40 && (nearestPillarGlobal.colour != NO_COLOUR ||) {
          prev_state = DETECT_STATE_OC2;
          state = TURNING_P1_STATE_OC2;
          if (!is_anticlockwise) {
            outer_wall_dist = ultra1Dist;
          } else 
            outer_wall_dist = ultra2Dist;
          }
          tar_ang_calibrate = 0;
          tar_ang += (is_anticlockwise == true) ? -right_ang : right_ang;
          break;
        } else if (nearestPillarGlobal.colour != NO_COLOUR && nearestPillarGlobal.area >= 3){
          if (nearestPillarGlobal.colour == RED){
            if (nearestPillarGlobal.xpos >= min_xpos_Rcase(nearestPillarGlobal.area)){
              pillars_array.push_back(nearestPillarGlobal);
              pillar_front = nearestPillarGlobal;
              prev_state = DETECT_STATE_OC2;
              state = CURVE_BLK_STATE_OC2;
              tar_ang_calibrate = 0;
              break;
            } else {
              prev_state = DETECT_STATE_OC2;
              state = SKIP_BLK_STATE_OC2;
              tar_ang_calibrate = 0;
              break;
            }
          } else {
            if (nearestPillarGlobal.xpos <= min_xpos_Gcase(nearestPillarGlobal.area)){
              pillars_array.push_back(nearestPillarGlobal);
              pillar_front = nearestPillarGlobal;
              prev_state = DETECT_STATE_OC2;
              state = CURVE_BLK_STATE_OC2;
              tar_ang_calibrate = 0;
              break; 
            } else {
              prev_state = DETECT_STATE_OC2;
              state = SKIP_BLK_STATE_OC2;
              tar_ang_calibrate = 0;
              break;
            }
          }  
        } else if (ultra1Dist < 20) {
          tar_ang_calibrate = 5;
        } else if (ultra2Dist < 20) {
          tar_ang_calibrate = -5;
        } else {
          steering_percentage = -(imu_yaw - (tar_ang + tar_ang_calibrate)) * imu_kp;
          break;
        }
        break;

      case TURNING_P1_STATE_OC2:
        Serial.println("TURNING_P1_STATE_OC2");
        tar_power = 8;
        if (abs(imu_yaw - tar_ang) > 40){
          if(is_anticlockwise) steering_percentage = -100;
          else steering_percentage = 100;
          motor_move(tar_power);
          break;
        } else {
          motor_stop(BRAKE);
          prev_state = TURNING_P1_STATE_OC2;
          state = TURNING_P2_STATE_OC2;
          break;
        }
        break;

      case TURNING_P2_STATE_OC2:
        Serial.println("TURNING_P2_STATE_OC2");
        tar_power = -8;
        if (abs(imu_yaw - tar_ang) > 5){
          if(is_anticlockwise) steering_percentage = 100;
          else steering_percentage = -100;
          motor_move(tar_power);
          break;
        } else {
          motor_stop(BRAKE);
          tar_power = power;
          prev_state = TURNING_P2_STATE_OC2;
          state = DETECT_STATE_OC2;
          break;
        }
        break;
      // case BACKWARD_STATE_OC2:
      //   Serial.println("TURNING_STATE_OC2");
      //   if (abs(MOTOR_ENCODER_COUNT) < 50 * ENC_PER_CM) {
      //     motor_move(-tar_power);
      //     break;
      //   } else {
      //     motor_stop(BRAKE);
      //     prev_state = BACKWARD_STATE_OC2;
      //     state = TURNING_STATE_OC2;
      //     break;
      //   }

      case CURVE_BLK_STATE_OC2:
        Serial.println("CURVE_BLK_STATE_OC2");
        motor_move(tar_power);
        if (pillar_front.colour == RED){
          if (nearestPillarGlobal.xpos >= min_xpos_Rcase(nearestPillarGlobal.area)) {
            steering_percentage = 100;
            break;
          }
          else {
            steering_percentage = 0;
            prev_state = CURVE_BLK_STATE_OC2;
            state = SKIP_BLK_STATE_OC2;
            break;
          }
        } else {
          if (nearestPillarGlobal.xpos <= min_xpos_Gcase(nearestPillarGlobal.area)) {
            steering_percentage = -100;
            break;
          } else {
            steering_percentage = 0;
            prev_state = CURVE_BLK_STATE_OC2;
            state = SKIP_BLK_STATE_OC2;
            break;
          }
        }
        break;

      case SKIP_BLK_STATE_OC2:
        Serial.println("SKIP_BLK_STATE_OC2");
        if (pillar_front.colour == RED){
          if (!motor_degree_accel(area2dist_red(pillar_front.area), area2dist_red(pillar_front.area) / 2, area2dist_red(pillar_front.area) / 2, 8, 12)) steering_percentage = 0;
          else {
            prev_state = SKIP_BLK_STATE_OC2;
            state = TURN_STRAIGHT_STATE_OC2;
            break;
          }
        } else {
          if (!motor_degree_accel(area2dist_green(pillar_front.area), area2dist_green(pillar_front.area) / 2, area2dist_green(pillar_front.area) / 2, 8, 13)) steering_percentage = 0;
          else {
            prev_state = SKIP_BLK_STATE_OC2;
            state = TURN_STRAIGHT_STATE_OC2;
            break;
          }
        }
        break;
      
      case TURN_STRAIGHT_STATE_OC2:
        Serial.println("TURN_STRAIGHT_STATE_OC2");
        motor_move(tar_power);
        if (abs(tar_ang - imu_yaw) > 20){
          steering_percentage = -(imu_yaw - tar_ang) * imu_kp;
        } else{
          prev_state = TURN_STRAIGHT_STATE_OC2;
          state = DETECT_STATE_OC2;
          break;
        }
        break;
      
      case ENDING_STATE_OC2:
        Serial.println("ENDING_STATE_OC2");
        // if (MOTOR_ENCODER_COUNT < abs(lane_length) / 2){
        //   steering_percentage = -(imu_yaw - tar_ang) * imu_kp;
        // } else {
          motor_stop(BRAKE);
          steering_percentage = 0;
          safeResume(blinkledThread);
          safeResume(OC1Thread);
          if (!is_anticlockwise) safeResume(Ultra1Thread);
          else safeResume(Ultra2Thread);
          end_game = true;
          OC2_endtime = internalClock.read();
        // }
        break;
    }
  }
  vTaskDelay(5 / portTICK_PERIOD_MS);
}      
      // case OUT_PARKING_P1_STATE:
      //   Serial.println("OUT_PARKING_P1_STATE");
      //   tar_power = 10;
      //   if(is_anticlockwise){
      //     if (imu_yaw > -10) {
      //       steering_percentage = -100;
      //     } else {
      //       reset_encoder();
      //       prev_state = OUT_PARKING_P1_STATE;
      //       state = OUT_PARKING_P2_STATE;
      //       break;
      //     }
      //   } else {
      //     if (imu_yaw < 10) {
      //       steering_percentage = 100;
      //     } else {
      //       reset_encoder();
      //       prev_state = OUT_PARKING_P1_STATE;
      //       state = OUT_PARKING_P2_STATE;
      //       break;
      //     }
      //   }
      //   break;

      // case OUT_PARKING_P2_STATE:
      //   Serial.println("OUT_PARKING_P2_STATE");
      //   tar_power = -10;
      //   if(is_anticlockwise){
      //     if (imu_yaw > -45) {
      //       steering_percentage = 100;
      //     } else {
      //       reset_encoder();
      //       prev_state = OUT_PARKING_P2_STATE;
      //       state = OUT_PARKING_P3_STATE;
      //       break;
      //     }
      //   } else {
      //     if (imu_yaw < 45) {
      //       steering_percentage = -100;
      //     } else {
      //       reset_encoder();
      //       prev_state = OUT_PARKING_P2_STATE;
      //       state = OUT_PARKING_P3_STATE;
      //       break;
      //     }
      //   }
      //   break;

      // case OUT_PARKING_P3_STATE:
      //   Serial.println("OUT_PARKING_P3_STATE");
      //   tar_power = 10;
      //   if (is_anticlockwise) steering_percentage = -100;
      //   else steering_percentage = 100;
      //   if(abs(MOTOR_ENCODER_COUNT) < 150){
      //     tar_power = 50;
      //   } else {
      //     reset_encoder();
      //     prev_state = OUT_PARKING_P3_STATE;
      //     state = OUT_PARKING_P4_STATE;
      //     break;
      //   }
      //   break;
      
      // case OUT_PARKING_P4_STATE:
      //   Serial.println("OUT_PARKING_P4_STATE");
      //   tar_power = -10;
      //   if(is_anticlockwise){
      //     if (imu_yaw > -60) {
      //       steering_percentage = 100;
      //     } else {
      //       reset_encoder();
      //       prev_state = OUT_PARKING_P4_STATE;
      //       state = OUT_PARKING_P5_STATE;
      //       break;
      //     }
      //   } else {
      //     if (imu_yaw < 60) {
      //       steering_percentage = -100; 
      //     } else {
      //       reset_encoder();
      //       prev_state = OUT_PARKING_P4_STATE;
      //       state = OUT_PARKING_P5_STATE;
      //       break;
      //     }
      //   }
      //   break;

      // case OUT_PARKING_P5_STATE:
      //   Serial.println("OUT_PARKING_P5_STATE");
      //   tar_power = -10;
      //   if(is_anticlockwise){
      //     if (abs(MiniR4.M2.getCounter()) < 1800) {
      //       steering_percentage = (imu_yaw - -88) * 10;
      //     } else {
      //       reset_encoder();
      //       prev_state = OUT_PARKING_P5_STATE;
      //       state = OUT_PARKING_P6_STATE;
      //       huskylensThread.enabled = true;
      //       break;
      //     }
      //   } else {
      //     if (abs(MiniR4.M2.getCounter()) < 1800) {
      //       steering_percentage = (imu_yaw - 88) * 10; 
      //     } else {
      //       reset_encoder();
      //       prev_state = OUT_PARKING_P5_STATE;
      //       huskylensThread.enabled = true;
      //       state = OUT_PARKING_P6_STATE;
      //       break;
      //     }
      //   }
      //   break;

      // case OUT_PARKING_P6_STATE:
      //   Serial.println("OUT_PARKING_P6_STATE");
      //   tar_power = 10;
      //   if (is_anticlockwise){
      //     if (imu_yaw < 0) steering_percentage = 100;
      //     else {
      //       if (nearestPillarGlobal.colour != NO_COLOUR && nearestPillarGlobal.area > 13){
      //         reset_encoder();
      //         prev_state = OUT_PARKING_P6_STATE;
      //         state = OUT_PARKING_P7_STATE;
      //         break;
      //       } else {
      //         reset_encoder();
      //         prev_state = OUT_PARKING_P6_STATE;
      //         Laser1Thread.enabled = false;
      //         Ultra1Thread.enabled = false;
      //         Ultra2Thread.enabled = false;
      //         huskylensThread.enabled = false;
      //         state = DETECT_STATE;
      //         break;
      //       }
      //     }
      //   } else {
      //     if (imu_yaw > 15) steering_percentage = -100; // >0
      //     else {
      //         if (nearestPillarGlobal.colour != NO_COLOUR && nearestPillarGlobal.area > 13){
      //           reset_encoder();
      //           prev_state = OUT_PARKING_P6_STATE;
      //           state = OUT_PARKING_P7_STATE;
      //           break;
      //         } else {
      //           reset_encoder();
      //           prev_state = OUT_PARKING_P6_STATE;
      //           Laser1Thread.enabled = false;
      //           Ultra1Thread.enabled = false;
      //           Ultra2Thread.enabled = false;
      //           huskylensThread.enabled = false;
      //           state = DETECT_STATE;
      //           break;
      //         }
      //     }
      //   }
      //   break;
      
      // case OUT_PARKING_P7_STATE:
      //   Serial.println("OUT_PARKING_P7_STATE");
      //   tar_power = 10;
      //   if (abs(MiniR4.M2.getCounter()) < 1300){ //1600//1750
      //     steering_percentage = 0;
      //   } else {
      //     tar_power = -50;
      //     reset_encoder();
      //     Laser1Thread.enabled = true;
      //     prev_state = OUT_PARKING_P7_STATE;
      //     state = DETECT_STATE;
      //     break;
      //   }
      //   break;

      // case MID_P1_STATE:
      //   Serial.print("MID_P1_STATE ");
      //   Laser1Thread.enabled = false;
      //   Ultra1Thread.enabled = false;
      //   Ultra2Thread.enabled = false;
      //   if (sign(inner_wall_dist) < 0){
      //     Serial.println("Right");
      //   }
      //   else {
      //     Serial.println("Left");
      //   }
      //   motor_move(-60);
      //   if (nearestPillarGlobal.colour != NO_COLOUR && nearestPillarGlobal.area >= 11){
      //     if (nearestPillarGlobal.colour == RED && nearestPillarGlobal.xpos >= min_xpos_Rcase(nearestPillarGlobal.area) && getUltra2Dist() < dist_threshold){
      //       red_block_flash(is_anticlockwise);
      //       pillars_array.push_back({nearestPillarGlobal.colour, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area});
      //       pillar_front = {RED, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area};
      //       gyro_ang_detect_phase = imu_yaw;
      //       if (pillar_front.xpos > 230) curve_phase1_gyro_chkpt = 45; // For Pt A, B
      //       else curve_phase1_gyro_chkpt = pillar_front.xpos * 45 / 200; // For Pt C, D, E
      //       prev_state = MID_P1_STATE;
      //       steering_percentage = 0;
      //       state = CURVE_P1_STATE;
      //       break;
      //     } else if (nearestPillarGlobal.colour == GREEN && nearestPillarGlobal.xpos <= min_xpos_Gcase(nearestPillarGlobal.area) && getUltra1Dist() < dist_threshold){
      //       green_block_flash(is_anticlockwise);
      //       pillars_array.push_back({nearestPillarGlobal.colour, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area});
      //       pillar_front = {GREEN, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area};
      //       gyro_ang_detect_phase = imu_yaw;
      //       if (mirror(pillar_front.xpos) > 230 && pillar_front.area < 30) curve_phase1_gyro_chkpt = 40; // For Pt A
      //       else if (pillar_front.area >= 30) curve_phase1_gyro_chkpt = 40; // For Pt B
      //       else curve_phase1_gyro_chkpt = mirror(pillar_front.xpos) * 45 / 200; // For Pt C, D, E
      //       prev_state = MID_P1_STATE;
      //       steering_percentage = 0;
      //       state = CURVE_P1_STATE;
      //       break;
      //     }
      //   } else {
      //     if (abs(imu_yaw - gyro_ang_detect_phase) < mid_phase1_gyro_chkpt){
      //       steering_percentage = 100 * sign(inner_wall_dist);
      //       // Serial.println(inner_wall_dist);
      //       // if (inner_wall_dist < 0){
      //       //   motor_move(0);
      //       // }
      //     } else {
      //       // Serial.println(inner_wall_dist);
      //       mid_phase2_chk_t = abs(inner_wall_dist) * 2.4 * speed_amp_factor;
      //       Serial.println(mid_phase2_chk_t);
      //       mid_save_t = internalClock.read();
      //       prev_state = MID_P1_STATE;
      //       state = MID_P2_STATE;
      //       steering_percentage = 0;
      //       break;
      //     }
      //   }
      //   break;
        
      // case MID_P2_STATE:
      //   Serial.println("MID_P2_STATE");
      //   motor_move(-70);
      //   if (nearestPillarGlobal.colour != NO_COLOUR && nearestPillarGlobal.area >= 13){
      //     if (nearestPillarGlobal.colour == RED && nearestPillarGlobal.xpos >= min_xpos_Rcase(nearestPillarGlobal.area)){
      //       red_block_flash(is_anticlockwise);
      //       pillars_array.push_back({nearestPillarGlobal.colour, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area});
      //       pillar_front = {RED, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area};
      //       gyro_ang_detect_phase = imu_yaw;
      //       if (pillar_front.xpos > 230) curve_phase1_gyro_chkpt = 45; // For Pt A, B
      //       else curve_phase1_gyro_chkpt = pillar_front.xpos * 45 / 200; // For Pt C, D, E
      //       prev_state = MID_P2_STATE;
      //       state = CURVE_P1_STATE;
      //       steering_percentage = 0;
      //       break;
      //     } else if (nearestPillarGlobal.colour == GREEN && nearestPillarGlobal.xpos <= min_xpos_Gcase(nearestPillarGlobal.area)){
      //       green_block_flash(is_anticlockwise);
      //       pillars_array.push_back({nearestPillarGlobal.colour, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area});
      //       pillar_front = {GREEN, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area};
      //       gyro_ang_detect_phase = imu_yaw;
      //       if (mirror(pillar_front.xpos) > 230 && pillar_front.area < 30) curve_phase1_gyro_chkpt = 50; //40; // For Pt A
      //       else if (pillar_front.area >= 30) curve_phase1_gyro_chkpt = 40; // For Pt B
      //       else curve_phase1_gyro_chkpt = mirror(pillar_front.xpos) * 45 / 200; // For Pt C, D, E
      //       prev_state = MID_P2_STATE;
      //       state = CURVE_P1_STATE;
      //       steering_percentage = 0;
      //       // state = DEBUG_STATE;
      //       break;
      //     }
      //   } else {
      //     if (internalClock.read() - mid_save_t <= mid_phase2_chk_t){
      //       steering_percentage = 0;
      //     } else {
      //       gyro_ang_detect_phase = imu_yaw;
      //       prev_state = MID_P2_STATE;
      //       state = MID_P3_STATE;
      //       steering_percentage = 0;
      //       break;
      //     }
      //   }
      //   break;
    
      // case MID_P3_STATE:
      //   Serial.println("MID_P3_STATE");
      //   motor_move(-60);
      //   mid_already = true;
      //   if (nearestPillarGlobal.colour != NO_COLOUR && nearestPillarGlobal.area >= 13){
      //     if (nearestPillarGlobal.colour == RED && nearestPillarGlobal.xpos >= min_xpos_Rcase(nearestPillarGlobal.area)){
      //       red_block_flash(is_anticlockwise);
      //       pillars_array.push_back({nearestPillarGlobal.colour, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area});
      //       pillar_front = {RED, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area};
      //       gyro_ang_detect_phase = imu_yaw;
      //       // if (abs(imu_yaw - tar_ang) > 10){
      //         if (pillar_front.xpos > 230) curve_phase1_gyro_chkpt = 35; // For Pt A, B
      //         else curve_phase1_gyro_chkpt = pillar_front.xpos * 35 / 600; // For Pt C, D, E
      //       // } else {
      //       //   if (pillar_front.xpos > 230) curve_phase1_gyro_chkpt = 45; // For Pt A, B
      //       //   else curve_phase1_gyro_chkpt = pillar_front.xpos * 45 / 200; // For Pt C, D, E
      //       // }
      //       prev_state = MID_P3_STATE;
      //       state = CURVE_P1_STATE;
      //       steering_percentage = 0;
      //       break;
      //     } else if (nearestPillarGlobal.colour == GREEN && nearestPillarGlobal.xpos <= min_xpos_Gcase(nearestPillarGlobal.area)){
      //       green_block_flash(is_anticlockwise);
      //       pillars_array.push_back({nearestPillarGlobal.colour, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area});
      //       pillar_front = {GREEN, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area};
      //       gyro_ang_detect_phase = imu_yaw;
      //       // if (abs(imu_yaw - tar_ang) > 10){
      //         if (mirror(pillar_front.xpos) > 230 && pillar_front.area < 30) curve_phase1_gyro_chkpt = 35; // For Pt A
      //         else if (pillar_front.area >= 30) curve_phase1_gyro_chkpt = 35; // For Pt B
      //         else curve_phase1_gyro_chkpt = mirror(pillar_front.xpos) * 35 / 200; // For Pt C, D, E
      //       // } else {
      //       //   if (mirror(pillar_front.xpos) > 230 && pillar_front.area < 30) curve_phase1_gyro_chkpt = 40; // For Pt A
      //       //   else if (pillar_front.area >= 30) curve_phase1_gyro_chkpt = 40; // For Pt B
      //       //   else curve_phase1_gyro_chkpt = mirror(pillar_front.xpos) * 45 / 200; // For Pt C, D, E
      //       // }
      //       prev_state = MID_P3_STATE;
      //       state = CURVE_P1_STATE;
      //       steering_percentage = 0;
      //       break;
      //     }
      //   } else {
      //     if (abs(imu_yaw - tar_ang) > 12){
      //       steering_percentage = -100 * sign(inner_wall_dist);
      //     } else {
      //       motor_last_counter = MiniR4.M2.getCounter();
      //       prev_state = MID_P3_STATE;
      //       state = MID_DASH_STATE;
      //       steering_percentage = 0;
      //       mid_dash_save_t = internalClock.read();
      //       break;
      //     }
      //   }
      //   break;
        
      // case MID_DASH_STATE:
      //   Serial.println("MID_DASH_STATE");
      //   if (num_turn >= 12){
      //     state = STOP;
      //     break;
      //   } 
      //   else if (nearestPillarGlobal.colour != NO_COLOUR && nearestPillarGlobal.area >= 12){
      //     if (nearestPillarGlobal.colour == RED && nearestPillarGlobal.xpos >= min_xpos_Rcase(nearestPillarGlobal.area)){
      //       red_block_flash(is_anticlockwise);
      //       pillars_array.push_back({nearestPillarGlobal.colour, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area});
      //       pillar_front = {RED, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area};
      //       gyro_ang_detect_phase = imu_yaw;
      //       if (pillar_front.xpos > 230) curve_phase1_gyro_chkpt = 45; // For Pt A, B
      //       else curve_phase1_gyro_chkpt = pillar_front.xpos * 45 / 200; // For Pt C, D, E
      //       prev_state = MID_P3_STATE;
      //       state = CURVE_P1_STATE;
      //       steering_percentage = 0;
      //       break;
      //     } else if (nearestPillarGlobal.colour == GREEN && nearestPillarGlobal.xpos <= min_xpos_Gcase(nearestPillarGlobal.area)){
      //       green_block_flash(is_anticlockwise);
      //       pillars_array.push_back({nearestPillarGlobal.colour, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area});
      //       pillar_front = {GREEN, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area};
      //       gyro_ang_detect_phase = imu_yaw;
      //       if (mirror(pillar_front.xpos) > 230 && pillar_front.area < 30) curve_phase1_gyro_chkpt = 40; // For Pt A
      //       else if (pillar_front.area >= 30) curve_phase1_gyro_chkpt = 40; // For Pt B
      //       else curve_phase1_gyro_chkpt = mirror(pillar_front.xpos) * 45 / 200; // For Pt C, D, E
      //       prev_state = MID_P3_STATE;
      //       state = CURVE_P1_STATE;
      //       steering_percentage = 0;
      //       break;
      //     }
      //   } 
      //   else if (internalClock.read() - mid_dash_save_t <= 400 * speed_amp_factor){ // 400
      //     steering_percentage = (imu_yaw - tar_ang) * 10;
      //     motor_move(-50);
      //     prev_state = MID_DASH_STATE;
      //   } else state = DETECT_STATE;
      //   break;

      // case CURVE_P1_STATE:
      //   Serial.println("CURVE_P1_STATE");
      //   motor_move(-60);
      //   Serial.println(pillar_front.area);
      //   Serial.println(pillar_front.xpos);
      //   if (pillar_front.colour == RED){
      //     red_block_flash(is_anticlockwise);
      //     if (abs(imu_yaw - gyro_ang_detect_phase) + 10 < curve_phase1_gyro_chkpt){
      //       if (blk_while_turning) steering_percentage = -100; 
      //       else if (pillar_front.xpos > 230 || pillar_front.area < 13) steering_percentage = -(55 - (imu_yaw - gyro_ang_detect_phase)) * pillar_front.area * steering_amp_fact1 * 320 / (55 * area_dangerzone * pillar_front.xpos);
      //       else steering_percentage = -(60 - (imu_yaw - gyro_ang_detect_phase)) * pillar_front.area * steering_amp_fact1 * 320 / (50 * area_dangerzone * pillar_front.xpos);
      //       // Serial.print(steering_percentage);
      //       // Serial.print(" ");
      //     } else {
      //       if (blk_while_turning) {
      //         prev_state = CURVE_P1_STATE;
      //         state = CURVE_P2_5_STATE;
      //         curve_sphase3_chk_t = pillar_front.area * 15;
      //         Serial.println("RED blk_while_turning");
      //         pillar_avoid_save_t = internalClock.read();
      //         //break;
      //       } else if (pillar_front.xpos > 230 && pillar_front.area < 30){ // For Pt A
      //         curve_phase2_chk_t = 520 * (pillar_front.xpos - 20) * speed_amp_factor / 200; //500;
      //         prev_state = CURVE_P1_STATE;
      //         state = CURVE_P2_STATE;
      //         Serial.println("RED Pt A");
      //         //break;
      //       } else if (pillar_front.area >= 16){ // For Pt B
      //         curve_phase2_chk_t = 500 * (pillar_front.xpos - 20) * speed_amp_factor / 100; // 80*
      //         prev_state = CURVE_P1_STATE;
      //         state = CURVE_P2_STATE;
      //         Serial.println("RED Pt B");
      //         //break;
      //       } else if (pillar_front.area > 10){ // For Pt C, D
      //         curve_phase2_chk_t = 12 * (pillar_front.xpos - 50) * speed_amp_factor / 300; // 20*, 50*
      //         prev_state = CURVE_P1_STATE;
      //         state = CURVE_P2_STATE;
      //         Serial.println("RED Pt C, D");
      //         //break;
      //       } else { // For Pt E
      //         gyro_ang_detect_phase = imu_yaw;
      //         prev_state = CURVE_P1_STATE;
      //         state = CURVE_P3_STATE;
      //         curve_phase2_chk_t = 200;
      //         Serial.println("RED Pt E");
      //         //break;
      //       }
      //       pillar_avoid_save_t = internalClock.read();
      //       motor_last_counter = MiniR4.M2.getCounter();
      //       break;
      //     }
      //   } else {
      //     green_block_flash(is_anticlockwise);
      //     // if (prev_state == MID_P1_STATE || prev_state == MID_P2_STATE || prev_state == MID_P3_STATE) gyro_offset = 5;
      //     if (abs(imu_yaw - gyro_ang_detect_phase) + 2 + gyro_offset < curve_phase1_gyro_chkpt){
      //       if (blk_while_turning) steering_percentage = 100;
      //       if (mirror(pillar_front.xpos) > 230) steering_percentage = (50 - abs(imu_yaw - gyro_ang_detect_phase)) * pillar_front.area * steering_amp_fact1 * 320 / (60 * area_dangerzone * mirror(pillar_front.xpos)); // || pillar_front.area < 13
      //       else steering_percentage = abs((50 - abs(imu_yaw - gyro_ang_detect_phase)) * pillar_front.area * steering_amp_fact1 * 320 / (50 * area_dangerzone * mirror(pillar_front.xpos))); //320
      //       Serial.println(steering_percentage);
      //       // Serial.println(abs(imu_yaw - gyro_ang_detect_phase) + 10 + gyro_offset);
      //       // Serial.println(curve_phase1_gyro_chkpt);
      //       Serial.println(" ");
      //     } else {
      //       if (blk_while_turning) {
      //         prev_state = CURVE_P1_STATE;
      //         state = CURVE_P2_5_STATE;
      //         curve_sphase3_chk_t = pillar_front.area * 15;
      //         pillar_avoid_save_t = internalClock.read();
      //         Serial.println("GREEN blk_while_turning");
      //         //break;
      //       } else if (mirror(pillar_front.xpos) > 230 && pillar_front.area < 30){ // For Pt A
      //         curve_phase2_chk_t = 1200 * (mirror(pillar_front.xpos) - 50) * speed_amp_factor / 300; //900*
      //         prev_state = CURVE_P1_STATE;
      //         state = CURVE_P2_STATE;
      //         Serial.println("GREEN Pt A");
      //         //break;
      //       } else if (pillar_front.area >= 30 && pillar_front.xpos < 285){ // For Pt B
      //         curve_phase2_chk_t = 160 * (mirror(pillar_front.xpos) - 50) * speed_amp_factor / 300;
      //         prev_state = CURVE_P1_STATE;
      //         state = CURVE_P2_STATE;
      //         Serial.println("GREEN Pt B");
      //         //break;
      //       } else if (pillar_front.area > 13){ // For Pt C, D
      //         // curve_phase2_chk_t = 350 * (mirror(pillar_front.xpos) - 20) * speed_amp_factor / 250; // 30*
      //         curve_phase2_chk_t = 260 * 3;
      //         gyro_ang_detect_phase = imu_yaw;
      //         prev_state = CURVE_P1_STATE;
      //         state = CURVE_P2_STATE;
      //         Serial.println("GREEN Pt C, D");
      //         //break;
      //       } else { // For Pt E
      //         // curve_phase2_chk_t = 350 * (mirror(pillar_front.xpos) - 0) * speed_amp_factor / 300;
      //         curve_phase2_chk_t = 300 * 2.5;
      //         gyro_ang_detect_phase = imu_yaw;
      //         prev_state = CURVE_P1_STATE;
      //         state = CURVE_P2_STATE;
      //         Serial.println("GREEN Pt E");
      //         //break;
      //       }
      //       pillar_avoid_save_t = internalClock.read();
      //       motor_last_counter = MiniR4.M2.getCounter();
      //       //break;
      //     }
      //     break;
      //   }
      //   Serial.println(curve_phase2_chk_t);
      //   break;

      // case CURVE_P2_STATE:
      //   Serial.println("CURVE_P2_STATE");
      //   motor_move(-60);
      //   if (internalClock.read() - pillar_avoid_save_t <= curve_phase2_chk_t && abs(MiniR4.M2.getCounter() - motor_last_counter) < 500){ // - 20 // curve_phase2_chk_t
      //     steering_percentage = 0;
      //     Serial.print(internalClock.read() - pillar_avoid_save_t);
      //     Serial.print(" ");
      //     Serial.println(curve_phase2_chk_t);
      //   } else {
      //     gyro_ang_detect_phase = imu_yaw;
      //     prev_state = CURVE_P2_STATE;
      //     state = CURVE_P3_STATE;
      //     break;
      //   }
      //   break;

      // case CURVE_P2_5_STATE:
      //   Serial.println("CURVE_P2_5_STATE");
      //   motor_move(-60);
      //   if (internalClock.read() - pillar_avoid_save_t <= curve_sphase3_chk_t + 100){
      //     steering_percentage = 0;
      //   } else {
      //     prev_state = CURVE_P2_5_STATE;
      //     state = CURVE_P3_STATE;
      //     break;
      //   }
      //   break;

      // case CURVE_P3_STATE:
      //   Serial.println("CURVE_P3_STATE");
      //   motor_move(-60);
      //   // if (nearestPillarGlobal.colour != NO_COLOUR && nearestPillarGlobal.colour > 10){
      //   //    if (nearestPillarGlobal.colour == RED && nearestPillarGlobal.xpos >= min_xpos_Rcase(nearestPillarGlobal.area)){
      //   //     red_block_flash(is_anticlockwise);
      //   //     pillars_array.push_back({nearestPillarGlobal.colour, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area});
      //   //     pillar_front = {RED, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area};
      //   //     gyro_ang_detect_phase = imu_yaw;
      //   //     if (pillar_front.xpos > 230) curve_phase1_gyro_chkpt = 45; // For Pt A, B
      //   //     else curve_phase1_gyro_chkpt = pillar_front.xpos * 45 / 200; // For Pt C, D, E
      //   //     prev_state = CURVE_P3_STATE;
      //   //     state = CURVE_P1_STATE;
      //   //     break;
      //   //   } else if (nearestPillarGlobal.colour == GREEN && nearestPillarGlobal.xpos <= min_xpos_Gcase(nearestPillarGlobal.area)){
      //   //     green_block_flash(is_anticlockwise);
      //   //     pillars_array.push_back({nearestPillarGlobal.colour, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area});
      //   //     pillar_front = {GREEN, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area};
      //   //     gyro_ang_detect_phase = imu_yaw;
      //   //     if (mirror(pillar_front.xpos) > 230 && pillar_front.area < 30) curve_phase1_gyro_chkpt = 40; // For Pt A
      //   //     else if (pillar_front.area >= 30) curve_phase1_gyro_chkpt = 40; // For Pt B
      //   //     else curve_phase1_gyro_chkpt = mirror(pillar_front.xpos) * 45 / 200; // For Pt C, D, E
      //   //     prev_state = CURVE_P3_STATE;
      //   //     state = CURVE_P1_STATE;
      //   //     break;
      //   //   }
      //   // } else {
      //     if (abs(imu_yaw - tar_ang) > 12 && internalClock.read()){ // 20
      //       if (pillar_front.colour == RED){
      //         red_block_flash(is_anticlockwise);
      //         if (imu_yaw - tar_ang > 10){ //30
      //           if (blk_while_turning) steering_percentage = 100;
      //           else steering_percentage = (abs(tar_ang - imu_yaw) + 30) * steering_amp_fact2; // (abs(imu_yaw) + 25) * steering_amp_fact2;
      //         } else {
      //           prev_state = CURVE_P3_STATE;
      //           state = CURVE_P4_STATE;
      //           pillar_avoid_save_t = internalClock.read();
      //           break;
      //         }
      //       } else {
      //         green_block_flash(is_anticlockwise);
      //         if (tar_ang - imu_yaw > 30){ //20
      //           if (blk_while_turning) steering_percentage = -100;
      //           else steering_percentage = -(abs(tar_ang - imu_yaw) + 30) * steering_amp_fact2;
      //         } else {
      //           prev_state = CURVE_P3_STATE;
      //           state = CURVE_P4_STATE;
      //           pillar_avoid_save_t = internalClock.read();
      //           break;
      //         }
      //       }
      //     } else {
      //       prev_state = CURVE_P3_STATE;
      //       state = CURVE_P4_STATE;
      //       pillar_avoid_save_t = internalClock.read();
      //       break;
      //     }
      //   // }
      //   break;

      // case CURVE_P4_STATE:
      //   Serial.println("CURVE_P4_STATE");
      //   motor_move(-60);
      //   // if (nearestPillarGlobal.area >= 40) pillar_avoid_save_t = internalClock.read();
      //   CP4_t = (num_turn % 4 == 0 && num_turn < 12) ? 1000 : (blk_while_turning) ? 800 : 500;
      //   if (num_turn % 4 == 0 && num_turn < 12) CP4_t = 1000;
      //   // else if (blk_while_turning) CP4_t = 300;
      //   else if (pillar_front.colour == RED) {
      //     // if (blk_while_turning) CP4_t = 300;
      //     if (pillar_front.xpos > 230 && pillar_front.area < 30) CP4_t = 700; // For Pt A
      //     else if (pillar_front.area >= 16) CP4_t = 700; // For Pt B
      //     else if (pillar_front.area > 10) CP4_t = 600 + mirror(pillar_front.xpos) * 3 * 100 / 152; // For Pt C, D
      //     else CP4_t = 200; // For Pt E // 500
      //     // if (pillar_front.xpos > 230 && pillar_front.area < 30) CP4_t = 19000 / pillar_front.area - pillar_front.xpos * 1; // For Pt A
      //     // else if (pillar_front.area >= 16) CP4_t = 40000 / pillar_front.area - pillar_front.xpos * 10; // For Pt B
      //     // else if (pillar_front.area > 10) CP4_t = CP4_t = 43000 / pillar_front.area - pillar_front.xpos * 15; // For Pt C, D
      //     // else CP4_t = CP4_t = 40000 / pillar_front.area - pillar_front.xpos * 11.5; // For Pt E // 500
      //     if (blk_while_turning && !is_anticlockwise) CP4_t = 850 + mirror(pillar_front.xpos) * 3 * 100 / 152; //750
      //     // CP4_t = 40000 / pillar_front.area - pillar_front.xpos * 12;
      //   } else {
      //     if (mirror(pillar_front.xpos) > 230 && pillar_front.area < 30) CP4_t = 600; // For Pt A
      //     else if (pillar_front.area >= 30) CP4_t = 600; // For Pt B
      //     else curve_phase1_gyro_chkpt = CP4_t = 1000; // For Pt C, D, E
      //     // if (mirror(pillar_front.xpos) > 230 && pillar_front.area < 30) CP4_t = 40000 / pillar_front.area - pillar_front.xpos * 1; // For Pt A
      //     // else if (pillar_front.area >= 30) CP4_t = CP4_t = 40000 / pillar_front.area - pillar_front.xpos * 4; // For Pt B
      //     // else curve_phase1_gyro_chkpt = CP4_t = CP4_t = 43000 / pillar_front.area - pillar_front.xpos *2; // For Pt C, D, E
      //     // CP4_t = 40000 / pillar_front.area - mirror(pillar_front.xpos) * 11;
      //     if (blk_while_turning && is_anticlockwise) CP4_t = 850 + pillar_front.xpos * 3 * 100 / 152; //750
      //   }
      //   Serial.println(CP4_t);
      //   // if (nearestPillarGlobal.colour != NO_COLOUR && nearestPillarGlobal.area >= 14 && internalClock.read() - pillar_avoid_save_t > 1000){
      //   //   if (nearestPillarGlobal.colour == RED && nearestPillarGlobal.xpos >= min_xpos_Rcase(nearestPillarGlobal.area)){
      //   //     prev_state = CURVE_P4_STATE;
      //   //     state = DETECT_STATE;
      //   //     steering_percentage = 0;
      //   //     break;
      //   //   } else if (nearestPillarGlobal.colour == GREEN && nearestPillarGlobal.xpos <= min_xpos_Gcase(nearestPillarGlobal.area)){
      //   //     prev_state = CURVE_P4_STATE;
      //   //     state = DETECT_STATE;
      //   //     steering_percentage = 0;
      //   //     break;
      //   //   }
      //   // } else 
      //   if (internalClock.read() - pillar_avoid_save_t <= CP4_t){
      //     steering_percentage = (imu_yaw - tar_ang) * 10;
      //   } else {
      //     no_blk_flash(is_anticlockwise);
      //     prev_state = CURVE_P4_STATE;
      //     state = TESTING_OC2;
      //     change_state_time = internalClock.read();
      //     if (blk_while_turning) reset_encoder();
      //     blk_while_turning = false;
      //     motor_last_counter = MiniR4.M2.getCounter();
      //     steering_percentage = 0;
      //     break;
      //   }
      //   break;

      // case CHECK_MID_RACINGLN_STATE:
      //   Serial.println("CHECK_MID_RACINGLN_STATE");
      //   Laser1Thread.enabled = true;
      //   if (is_anticlockwise) Ultra1Thread.enabled = true;
      //   else Ultra2Thread.enabled = true;
      //   motor_move(-60);
      //   front_wall_dist = getLaser1DistMedian(7);
      //   if (!mid_already){
      //     if (is_anticlockwise){
      //       if (abs(ultra1Dist - MID_RACINGLN_POS) > 30 && ultra1Dist < MID_RACINGLN_POS){
      //         motor_last_counter = MiniR4.M2.getCounter();
      //         prev_state = CHECK_MID_RACINGLN_STATE;
      //         state = CHECK_MID_TWICE;
      //         break;
      //       }
      //       if (abs(ultra1Dist - MID_RACINGLN_POS) > 30 && ultra1Dist < dist_threshold && front_wall_dist > 2000){ // && MiniR4.M2.getCounter() < 1000
      //         inner_wall_dist = ultra1Dist - MID_RACINGLN_POS;
      //         gyro_ang_detect_phase = imu_yaw;
      //         prev_state = CHECK_MID_RACINGLN_STATE;
      //         state = MID_P1_STATE;
      //         // mid_already = true;
      //         break;
      //       } 
      //       else if (internalClock.read() - change_state_time > 200 * speed_amp_factor) {
      //         prev_state = CHECK_MID_RACINGLN_STATE;
      //         pillar_front = {pillar_front.colour, pillar_front.xpos, CAM_LOWEST_YPOS - pillar_front.ypos, pillar_front.height, pillar_front.width, pillar_front.area};
      //         state = DETECT_STATE;
      //         break;
      //       }
      //     } else {
      //       if (abs(ultra2Dist - MID_RACINGLN_POS) > 30 && ultra2Dist < MID_RACINGLN_POS){
      //         motor_last_counter = MiniR4.M2.getCounter();
      //         prev_state = CHECK_MID_RACINGLN_STATE;
      //         state = CHECK_MID_TWICE;
      //         break;
      //       }
      //       if (abs(ultra2Dist - MID_RACINGLN_POS) > 30 && ultra2Dist < dist_threshold && front_wall_dist > 2000){
      //         inner_wall_dist = -(ultra2Dist - MID_RACINGLN_POS); 
      //         gyro_ang_detect_phase = imu_yaw;
      //         prev_state = CHECK_MID_RACINGLN_STATE;
      //         state = MID_P1_STATE;
      //         // mid_already = true;
      //         break;
      //       } else if (internalClock.read() - change_state_time > 200 * speed_amp_factor) {
      //         prev_state = CHECK_MID_RACINGLN_STATE;
      //         state = DETECT_STATE;
      //         break;
      //       }
      //     }
      //   } else {
      //     prev_state = CHECK_MID_RACINGLN_STATE;
      //     state = DETECT_STATE;
      //     // mid_already = true;
      //     break;
      //   }
      //   break;

      // case CHECK_MID_TWICE:
      //   Serial.println("CHECK_MID_TWICE");
      //   if (is_anticlockwise) Ultra1Thread.enabled = true;
      //   else Ultra2Thread.enabled = true;
      //   if (abs(MiniR4.M2.getCounter() - motor_last_counter) < 500){ // 1000 degree = 254mm
      //     motor_move(-60);
      //   }
      //   else{
      //     if (is_anticlockwise && abs(ultra1Dist - MID_RACINGLN_POS) > 60){
      //       inner_wall_dist = ultra1Dist - MID_RACINGLN_POS;
      //       gyro_ang_detect_phase = imu_yaw;
      //       prev_state = CHECK_MID_TWICE;
      //       state = MID_P1_STATE;
      //       // mid_already = true;
      //       break;
      //     } else if (!is_anticlockwise && abs(ultra2Dist - MID_RACINGLN_POS) > 60){
      //       inner_wall_dist = ultra2Dist - MID_RACINGLN_POS;
      //       gyro_ang_detect_phase = imu_yaw;
      //       prev_state = CHECK_MID_TWICE;
      //       state = MID_P1_STATE;
      //       // mid_already = true;
      //       break;
      //     } else {
      //       prev_state = CHECK_MID_TWICE;
      //       state = DETECT_STATE;
      //       change_state_time = internalClock.read();
      //       break;
      //     }
      //   } 
      //   break;

      // case CHECK_B4_TURNING:
      //   Serial.println("CHECK_B4_TURNING");
      //   Laser1Thread.enabled = true;
      //   motor_move(0);
      //   MiniR4.M2.setBrake(true);
      //   front_wall_dist = getLaser1DistMedian(7);
      //   if (internalClock.read() - check_front_t > 100){
      //     if (front_wall_dist < 500){
      //       prev_state = CHECK_B4_TURNING;
      //       state = TURNING_P1;
      //     }
      //     else {
      //       prev_state = CHECK_B4_TURNING;
      //       state = DETECT_STATE;
      //     }
      //   }
      //   break;

      // case TURNING_P1:
      //   Serial.println("TURNING_P1");
      //   if (side_wall_dist >= 620){
      //     state = TURNING_P1_5;
      //     prev_state = TURNING_P1;
      //     break;
      //   }
      //   else if (front_wall_dist < 140){
      //     state = TURNING_P1_0;
      //     prev_state = TURNING_P1;
      //     break;
      //   }
      //   else if (abs(MiniR4.M2.getCounter() - motor_last_counter) < 850){ // || imu_yaw < tar_ang - 45 //is_anticlockwise
      //     motor_move(-50);
      //     // if (is_anticlockwise) steering_percentage = 100;
      //     // else steering_percentage = -100;
      //     steering_percentage = (is_anticlockwise) ? 100 : -100;
      //   }
      //   else {
      //     motor_last_counter = MiniR4.M2.getCounter();
      //     state = TURNING_P2;
      //     prev_state = TURNING_P1;
      //   }
      //   break;

      // case TURNING_P1_0:
      //   Serial.println("TURNING_P1_0");
      //   front_wall_dist = getLaser1DistMedian(7);
      //   if (front_wall_dist < 200){
      //     motor_move(50);
      //   }
      //   else {
      //     prev_state = TURNING_P1_0;
      //     state = TURNING_P1;
      //   }
      //   break;

      // case TURNING_P2:
      //   Serial.println("TURNING_P2");
      //   if (abs(MiniR4.M2.getCounter() - motor_last_counter) < 850){
      //     motor_move(50);
      //     // if (is_anticlockwise) steering_percentage = -100;
      //     // else steering_percentage = 100;
      //     steering_percentage = (is_anticlockwise) ? -100 : 100;
      //   }
      //   else {
      //     tar_ang += (is_anticlockwise == true) ? -right_ang : right_ang; // 88.58
      //     motor_last_counter = MiniR4.M2.getCounter();
      //     steering_percentage = 0;
      //     num_turn += 1;
      //     state = TURNING_P3;
      //     prev_state = TURNING_P2;
      //   }
      //   break;

      // case TURNING_P1_5:
      //   Serial.println("TURNING_P1_5");
      //   Laser1Thread.enabled = true;
      //   front_wall_dist = getLaser1DistMedian(7);
      //   if (abs(MiniR4.M2.getCounter() - motor_last_counter) < 1100 && front_wall_dist >= 120){
      //     motor_move(-60);
      //     steering_percentage = (is_anticlockwise) ? -10 : 10;
      //     led1_flashwhite();
      //     led2_flashwhite();
      //   }
      //   else {
      //     motor_last_counter = MiniR4.M2.getCounter();
      //     led1_off();
      //     led2_off();
      //     state = TURNING_P2_5;
      //     prev_state = TURNING_P1_5;
      //   }
      //   break;

      // case TURNING_P2_5:
      //   Serial.println("TURNING_P2_5");
      //   if (abs(MiniR4.M2.getCounter() - motor_last_counter) < 1600){ // 1600
      //     motor_move(50);
      //     // if (is_anticlockwise) steering_percentage = -100;
      //     // else steering_percentage = 100;
      //     steering_percentage = (is_anticlockwise) ? -100 : 100;
      //   }
      //   else {
      //     tar_ang += (is_anticlockwise == true) ? -right_ang : right_ang; // 88.58
      //     motor_last_counter = MiniR4.M2.getCounter();
      //     side_wall_dist -= 150;
      //     steering_percentage = 0;
      //     num_turn += 1;
      //     state = TURNING_P3;
      //     prev_state = TURNING_P2_5;
      //   }
      //   break;

      // case TURNING_P3:
      //   Serial.print("TURNING_P3 ");
      //   mid_already = false;
      //   mid_dist = abs(side_wall_dist - 450) * 3.89; //3.92
      //   Serial.println(mid_dist);
      //   if (side_wall_dist > 450){
      //     if (abs(MiniR4.M2.getCounter() - motor_last_counter) < mid_dist - 30){
      //       motor_move(60);
      //       Serial.println(">450");
      //     }
      //     else {
      //       if (pillar_front.area > 7) blk_while_turning = true;
      //       prev_state = TURNING_P3;
      //       state = DETECT_STATE;
      //       Laser1Thread.enabled = true;
      //       huskylensThread.enabled = true;
      //       reset_encoder();
      //     }
      //   }
      //   else if (side_wall_dist < 450){
      //     if (abs(MiniR4.M2.getCounter() - motor_last_counter) < mid_dist){
      //       motor_move(-60);
      //       Serial.println("<450");
      //     }
      //     else {
      //       if (pillar_front.area > 5) blk_while_turning = true;
      //       prev_state = TURNING_P3;
      //       state = DETECT_STATE;
      //       Laser1Thread.enabled = true;
      //       huskylensThread.enabled = true;
      //       reset_encoder();
      //     }
      //   }
      //   else {
      //     if (pillar_front.area > 5) blk_while_turning = true;
      //     prev_state = TURNING_P3;
      //     state = DETECT_STATE;
      //     Laser1Thread.enabled = true;
      //     huskylensThread.enabled = true;
      //     reset_encoder();
      //     }
      //   break;

      // case STOP:
      //   Serial.print("STOP ");
      //   Serial.println(imu_yaw);
      //   motor_move(0);
      //   MiniR4.M2.setBrake(true);
      //   break;
      
      // default:
      //   break;
      // }


void OC2main(void *){
  static bool run_OC2 = false;
  while (1){
    if (is_btn_bumped(TFT_BTN2)){
      run_OC2 = !run_OC2;
      reset_OC2 = true;
    }
    if (run_OC2){
      OC2_pixy2(90, 85, 8); //-80
    } else {
      // safeResume(displayThread);
      safeResume(Ultra1Thread);
      safeResume(Ultra2Thread);
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
  if (column == TFT_LEFT_CLN){
    tft.clearln(TFT_LEFT_CLN, line_number);
    tft.displayLeftln(line_number, text_size, OC2_time_text.c_str(), text_colour, false);
  } else {
    tft.clearln(TFT_RIGHT_CLN, line_number);
    tft.displayRightln(line_number, text_size, OC2_time_text.c_str(), text_colour, false);
  }
}