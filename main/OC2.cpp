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
float min_xpos_Rcase_detect(int area){
  return -0.0008 * pow(area, 3) + 0.0948 * area * area - 4.6858 * area + 134.52 - 70 * (1 - 1 / (area + 1));
}

float min_xpos_Rcase_skip(int area){
  return -0.0008 * pow(area, 3) + 0.0948 * area * area - 4.6858 * area + 134.52 - 70;
}

/**
 * @brief Algorithm to calculate the safe xpos of the Green pillar seen according to its area from the huskylens
 * @param area; int; the area of the pillar from the huskylens
 *
 * @return float; the safe xpos of the pillar seen
*/
float min_xpos_Gcase(int area){
  return 0.0004 * pow(area, 3) - 0.056 * area * area + 3.5432 * area + 175.27 + 45;
}

float area2dist_red(int area){
  return 169.46 * pow(area, -0.486) - 10;
}

float area2dist_green(int area){
  return 180.28 * pow(area, -0.474) - 10;
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
  static String OC2_text = "INIT_STATE_OC2";

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
    OC2_text = "INIT_STATE_OC2";
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
        OC2_text = "INIT_STATE_OC2";
        motor_move(tar_power);
        tar_power = 0;
        if (dist_t1 > dist_t2) is_anticlockwise = true; //dist_t1
        else is_anticlockwise = false;
        prev_state = INIT_STATE_OC2;
        state = DETECT_STATE_OC2;
        motor_last_counter = abs(MOTOR_ENCODER_COUNT);
        break;

      case DETECT_STATE_OC2:
        Serial.println("DETECT_STATE_OC2");
        OC2_text = "DETECT_STATE_OC2";
        motor_move(tar_power);
        tar_power = 10;
        if (!is_anticlockwise){
          if (imu_yaw >= 360 * 3 - 45){
            prev_state = DETECT_STATE_OC2;
            state = ENDING_STATE_OC2;
            tar_ang_calibrate = 0;
            break;
          }
        } else {
          if (imu_yaw <= -360 * 3 + 45){
            prev_state = DETECT_STATE_OC2;
            state = ENDING_STATE_OC2;
            tar_ang_calibrate = 0;
            break;
          }
        }
        if (((is_anticlockwise && dist_t1 > dist_threshold) || (!is_anticlockwise && dist_t2 > dist_threshold)) && (abs(MOTOR_ENCODER_COUNT) > 50 * ENC_PER_CM)){ //dist_t1
          prev_state = DETECT_STATE_OC2;
          state = TURNING_STATE_OC2;
          tar_ang_calibrate = 0;
          tar_ang += (is_anticlockwise == true) ? -right_ang : right_ang;
          break;
        } else if (nearestPillarGlobal.colour != NO_COLOUR && nearestPillarGlobal.area >= 3){
          if (nearestPillarGlobal.colour == RED){
            if (nearestPillarGlobal.xpos >= min_xpos_Rcase_detect(nearestPillarGlobal.area)){
              Serial.println(nearestPillarGlobal.xpos);
              Serial.println(min_xpos_Rcase_detect(nearestPillarGlobal.area));
              // motor_stop(BRAKE);
              // state = ENDING_STATE_OC2;
              // break;
              pillars_array.push_back(nearestPillarGlobal);
              pillar_front = nearestPillarGlobal;
              prev_state = DETECT_STATE_OC2;
              state = CURVE_BLK_STATE_OC2;
              tar_ang_calibrate = 0;
              break;
            } else {
              pillars_array.push_back(nearestPillarGlobal);
              pillar_front = nearestPillarGlobal;
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
              pillars_array.push_back(nearestPillarGlobal);
              pillar_front = nearestPillarGlobal;
              prev_state = DETECT_STATE_OC2;
              state = SKIP_BLK_STATE_OC2;
              tar_ang_calibrate = 0;
              break;
            }
          }
        } 
        // else if (ultra3Dist < 20 && nearestPillarGlobal.colour == NO_COLOUR){
        //   prev_state = DETECT_STATE_OC2;
        //   state = BACKWARD_STATE_OC2;
        //   tar_ang_calibrate = 0;
        //   tar_ang += (is_anticlockwise == true) ? -right_ang : right_ang;
        //   reset_encoder();
        //   break;
        // } 
        else if (dist_t1 < 10) { //dist_t1
          tar_ang_calibrate = 5;
        } else if (dist_t2 < 20) {
          tar_ang_calibrate = -5;
        } else {
          steering_percentage = -(imu_yaw - (tar_ang + tar_ang_calibrate)) * imu_kp;
          break;
        }
        break;

      case BACKWARD_STATE_OC2:
        Serial.println("BACKWARD_STATE_OC2");
        OC2_text = "BACKWARD_STATE_OC2";
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
        OC2_text = "TURNING_STATE_OC2";
        motor_move(tar_power);
        if (nearestPillarGlobal.colour != NO_COLOUR && nearestPillarGlobal.area >= 5){
          if (nearestPillarGlobal.colour == RED){
            if (nearestPillarGlobal.xpos >= min_xpos_Rcase_detect(nearestPillarGlobal.area) - 5){
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
        OC2_text = "CURVE_BLK_STATE_OC2";
        motor_move(tar_power);
        if (pillar_front.colour == RED){
          if (nearestPillarGlobal.xpos >= min_xpos_Rcase_detect(nearestPillarGlobal.area)) {
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
        OC2_text = "SKIP_BLK_STATE_OC2";
        motor_move(tar_power);
        if (nearestPillarGlobal.colour == RED){
          // if (!motor_degree_accel(area2dist_red(pillar_front.area), area2dist_red(pillar_front.area) / 2, area2dist_red(pillar_front.area) / 2, 8, 10)){
          //   steering_percentage = -(min_xpos_Rcase_detect(nearestPillarGlobal.area) - nearestPillarGlobal.xpos) * 1;
          // } 
          if (dist_t1> 30){ //dist_t1
            Serial.println(min_xpos_Rcase_skip(nearestPillarGlobal.area));
            steering_percentage = -(min_xpos_Rcase_skip(nearestPillarGlobal.area) - nearestPillarGlobal.xpos) * 0.8;
            break;
          } 
          else {
            if (prev_state == DETECT_STATE_OC2) {
              prev_state = SKIP_BLK_STATE_OC2;
              state = DETECT_STATE_OC2;
              break;
            } else {
              prev_state = SKIP_BLK_STATE_OC2;
              state = FW_AFTER_BLK_OC2;
              break;
            }
          }
        } else {
          // if (!motor_degree_accel(area2dist_green(pillar_front.area), area2dist_green(pillar_front.area) / 2, area2dist_green(pillar_front.area) / 2, 8, 10)){
          //   steering_percentage = -(min_xpos_Gcase(nearestPillarGlobal.area) - nearestPillarGlobal.xpos) * 1;
          // } 
          if (dist_t2 > 30){
            steering_percentage = -(min_xpos_Gcase(nearestPillarGlobal.area) - nearestPillarGlobal.xpos) * 0.8;
            break;
          } 
          else {
            if (prev_state == DETECT_STATE_OC2) {
              prev_state = SKIP_BLK_STATE_OC2;
              state = DETECT_STATE_OC2;
              break;
            } else {
              prev_state = SKIP_BLK_STATE_OC2;
              state = FW_AFTER_BLK_OC2;
              break;
            }
          }

        }
        break;

      case FW_AFTER_BLK_OC2:
        Serial.println("FW_AFTER_BLK_OC2");
        OC2_text = "FW_AFTER_BLK_OC2";
        if (!motor_degree_accel(25, 12, 12, 10, 12)){
          steering_percentage = 0;
        } else {
          prev_state = FW_AFTER_BLK_OC2;
          state = TURN_STRAIGHT_STATE_OC2;
          break;
        }
        break;
      
      case TURN_STRAIGHT_STATE_OC2:
        Serial.println("TURN_STRAIGHT_STATE_OC2");
        OC2_text = "TURN_STRAIGHT_STATE_OC2";
        motor_move(tar_power);
        if (abs(tar_ang - imu_yaw) > 15){
          steering_percentage = -(imu_yaw - tar_ang) * imu_kp;
        } else{
          prev_state = TURN_STRAIGHT_STATE_OC2;
          state = DETECT_STATE_OC2;
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
  static int side_wall_dist = 0;
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
  static int gyro_ang_detect_phase = 0;
  static int last_u3_error = 0;
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
        // if (dist_t1 > dist_t2) is_anticlockwise = true; //dist_t1
        // else is_anticlockwise = false;
        is_anticlockwise = true;
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
        if (ultra3Dist <= 35 && nearestPillarGlobal.colour == NO_COLOUR){//(is_anticlockwise && dist_t1 > dist_threshold) || (!is_anticlockwise && dist_t2 > dist_threshold)){ //dist_t1
          // if (ultra3Dist <= 30){
            motor_stop(BRAKE);
            prev_state = DETECT_STATE_OC2;
            tar_ang_calibrate = 0;
            tar_ang += (is_anticlockwise == true) ? -right_ang : right_ang;
            motor_last_counter = abs(MOTOR_ENCODER_COUNT);
            if ((is_anticlockwise && dist_t2 > 50) || (!is_anticlockwise && dist_t1 > 50)){
              side_wall_dist = (is_anticlockwise) ? dist_t2 : dist_t1;
              state = TURNING_P2_1_STATE;
            }
            else{
              side_wall_dist = (is_anticlockwise) ? dist_t2 : dist_t1;
              state = TURNING_P1_1_STATE;
            }
            break;
          // }
          // else {
          //   // tar_power = 8;
          // }
        }
        else if (nearestPillarGlobal.colour != NO_COLOUR && nearestPillarGlobal.area >= 3){
          if (nearestPillarGlobal.colour == RED){
            if (nearestPillarGlobal.xpos >= min_xpos_Rcase_detect(nearestPillarGlobal.area)){
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
        } else if (dist_t1 < 20) {
          tar_ang_calibrate = 5;
        } else if (dist_t2 < 20) {
          tar_ang_calibrate = -5;
        } else {
          steering_percentage = -(imu_yaw - (tar_ang + tar_ang_calibrate)) * imu_kp;
          break;
        }
        break;

      case BACKWARD_STATE_OC2:
        Serial.println("BACKWARD_STATE_OC2");
        if (abs(MOTOR_ENCODER_COUNT) - motor_last_counter < 20 * ENC_PER_CM) {
          steering_percentage = 0;
          motor_move(-10);
          break;
        } else {
          motor_stop(BRAKE);
          prev_state = BACKWARD_STATE_OC2;
          state = DETECT_STATE_OC2;
          break;
        }
        break;

      case TURNING_P1_1_STATE:
        Serial.println("TURNING_P1_1_STATE");
        tar_power = 10;
        if (abs(imu_yaw - tar_ang) > 40){
          if(is_anticlockwise) steering_percentage = -80;
          else steering_percentage = 80;
          motor_move(tar_power);
          break;
        } else {
          motor_stop(BRAKE);
          prev_state = TURNING_P1_1_STATE;
          state = TURNING_P1_2_STATE;
          break;
        }
        break;

      case TURNING_P1_2_STATE:
        Serial.println("TURNING_P1_2_STATE");
        tar_power = -10;
        if (abs(imu_yaw - tar_ang) > 15){
          if(is_anticlockwise) steering_percentage = 80;
          else steering_percentage = -80;
          motor_move(tar_power);
          break;
        } else {
          motor_stop(BRAKE);
          prev_state = TURNING_P1_2_STATE;
          state = TURNING_P_3_STATE;
          break;
        }
        break;

      case TURNING_P2_1_STATE:
        Serial.println("TURNING_P2_1_STATE");
        tar_power = 10;
        if (ultra3Dist > 15){
          steering_percentage = 0;
          motor_move(tar_power);
          break;
        } else {
          motor_stop(BRAKE);
          prev_state = TURNING_P2_1_STATE;
          state = TURNING_P2_2_STATE;
          break;
        }
        break;

      case TURNING_P2_2_STATE:
        Serial.println("TURNING_P2_2_STATE");
        tar_power = -10;
        if (abs(imu_yaw - tar_ang) > 20){
          if(is_anticlockwise) steering_percentage = 100;
          else steering_percentage = -100;
          motor_move(tar_power);
          break;
        } else {
          motor_stop(BRAKE);
          tar_power = power;
          prev_state = TURNING_P2_2_STATE;
          state = TURNING_P_3_STATE;
          break;
        }
        break;

      case TURNING_P_3_STATE:
        Serial.println("TURNING_P_3_STATE");
        prev_state = TURNING_P_3_STATE;
        state = STOP_OC2;
        break;

      case CURVE_BLK_STATE_OC2:
        Serial.println("CURVE_BLK_STATE_OC2");
        motor_move(tar_power);
        if (pillar_front.colour == RED){
          if (nearestPillarGlobal.xpos >= min_xpos_Rcase_detect(nearestPillarGlobal.area)) {
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
        if (abs(tar_ang - imu_yaw) > 15){
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
          if (!is_anticlockwise) safeResume(ToF1Thread);    //safeResume(ToF1Thread);
          else safeResume(ToF2Thread);
          end_game = true;
          OC2_endtime = internalClock.read();
        // }
        break;

        case STOP_OC2:
        Serial.print("STOP ");
        Serial.println(imu_yaw);
        motor_move(0);
        motor_stop(BRAKE);
        break;
    }
  }
  vTaskDelay(5 / portTICK_PERIOD_MS);
} 

bool run_OC2 = false;
void OC2main(void *){
  while (1){
    if (is_btn_bumped(TFT_BTN2)){
      Serial.println("Hello World");
      run_OC2 = !run_OC2;
      reset_OC2 = true;
    }
    if (run_OC2){
      OC2_pixy2(90, 85, 12);
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