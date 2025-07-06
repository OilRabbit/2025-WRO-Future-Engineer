#include "MatrixMiniR4.h"
#include <cmath>
#include "Arduino.h"
#include "OC2.h"

/**
 * @brief Algorithm to calculate the safe xpos of the Red pillar seen according to its area from the huskylens
 * @param area; int; the area of the pillar from the huskylens
 *
 * @return float; the safe xpos of the pillar seen
*/
float min_xpos_Rcase(int area){
  return -2.8164 * area + 114;
}

/**
 * @brief Algorithm to calculate the safe xpos of the Green pillar seen according to its area from the huskylens
 * @param area; int; the area of the pillar from the huskylens
 *
 * @return float; the safe xpos of the pillar seen
*/
float min_xpos_Gcase(int area){
  return 1.9722 * area + 210.81;
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
long OC2_starttime = 0; 
long OC2_endtime = 0; 

void lapDir_flash(bool anticlockwise){
  if (anticlockwise) led1_flashblue();
  else led2_flashblue();
}

void red_block_flash(bool anticlockwise){
  if (anticlockwise) led2_flashred();
  else led1_flashred();
}

void green_block_flash(bool anticlockwise){
  if (anticlockwise) led2_flashgreen();
  else led1_flashred();
}

void redblk_tna_flash(bool anticlockwise){
  if (anticlockwise) MiniR4.LED.setColor(2, 255, 0, 255);
  else MiniR4.LED.setColor(1, 255, 0, 255);
}

void greenblk_tna_flash(bool anticlockwise){
  if (anticlockwise) MiniR4.LED.setColor(2, 0, 255, 255);
  else MiniR4.LED.setColor(1, 0, 255, 255);
}

void no_blk_flash(bool anticlockwise){
  if (anticlockwise) led2_off();
  else led1_off();
}

void OC2HuskylensNColor(double right_ang, int dist_threshold, int power){
  // A flag to for resetting the IMU before starting the run.
  // Resetting IMU takes a few seconds, which means if we do so when the run starts, we will lack behind for a few seconds
  static bool first_run = true;

  // A flag to determine whether the run is started or not
  static bool start_game = false;

  // Variables and flag for checking the DOWN button state
  static bool prev_btn_state = false;
  bool curr_btn_state = MiniR4.BTN_DOWN.getState();
  
  if (curr_btn_state && !prev_btn_state){
    start_game = !start_game;
  }
  prev_btn_state = curr_btn_state;

  // Variables and flag for checking the UP button state
  static bool prev_imu_btn_state = false;
  bool curr_imu_btn_state = MiniR4.BTN_UP.getState();
  
  if (curr_imu_btn_state && !prev_imu_btn_state){
    MiniR4.Motion.resetIMUValues();
    resetIMU();
  }
  prev_imu_btn_state = curr_imu_btn_state;

  // Variables required for OC2
  static OC3_STATES state = INIT_CASE; // A variable storing the current state of OC2 run
  static OC3_STATES prev_state = INIT_CASE;
  static bool reset_OC2 = false;              // A flag determining whether the all the variables should be reseted
  static bool end_game = false;               // A flag determining whether the run has ended
  std::vector<COLOURED_OBJ> pillars_array;    // An array storing all the pillars detected
  static COLOURED_OBJ pillar_front;           // A struct storing all the info of the nearest pillar detected during DETECT_STATE
  // EDITABLE: Not recommended to edit this but, if you want to change the danger zone of area of pillar, edit the following line
  const int area_dangerzone = 40;             // A variable representing the area of a pillar which is identified as danger when its area is larger than this value
  // END OF EDITABLE
  // EDITABLE: Edit this if the car is turning too sharp/mild during CURVE_P1_STATE
  const float steering_amp_fact1 = 250; // 150       // An amplification factor for calculating the steering percentage during CURVE_P1_STATE
  // END OF EDITABLE
  // EDITABLE: Edit this if the car is turning too sharp/mild during CURVE_P3_STATE
  const float steering_amp_fact2 = 3;         // An amplification factor for calculating the steering percentage during CURVE_P3_STATE
  // END OF EDITABLE
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
  static long dash_timeZero = 0;              // Variable to store the instant time from the internal clock for dashing
  static long dash_time = 0;                  // The time required for the car to run before using the ultrasonic sensors for detection again after turning
  static bool anticlockwise = true;          // A boolean storing whether the car is racing in clockwise or anti-clockwise direction
  static int num_turn = 0;                    // Variable storing the number of turns the car has made
  static long turn_waittimeZero = 0;          // Variable to store the instant time from the internal clock for waiting to turning
  static int turn_waittime = 150;             // Variable storing the time in ms that required to wait before the car turn 
  static bool blk_while_turning = false;      // A flag determining whether any pillars in close distance is detected during the turning
  static int color_number = getColorType();
  static int power2 = power;
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

  if (!start_game){ 
    reset_OC2 = true;
    steering(0);
    MiniR4.M2.setPower(0);
    displayThread.enabled = true;
    Ultra1Thread.enabled = true;
    Ultra2Thread.enabled = true;
    OC2Thread.setInterval(10);
    Ultra1Thread.setInterval(10);
    Ultra2Thread.setInterval(10);
  } else {
    if (reset_OC2){
      OC2Thread.setInterval(0);
      Ultra1Thread.setInterval(0);
      Ultra2Thread.setInterval(0);
      huskylensThread.setInterval(0);

      // state = DETECT_STATE;
      reset_OC2 = false;
      end_game = false;
      gyro_ang_detect_phase = 0;
      pillar_avoid_save_t = 0;
      curve_phase2_chk_t = 300;
      curve_sphase3_chk_t = 300;
      curve_phase1_gyro_chkpt = 45;
      mid_phase1_gyro_chkpt = 35;
      inner_wall_dist = 0;
      mid_save_t = 0;
      mid_phase2_chk_t = 0;
      mid_dash_save_t = 0;
      tar_ang = 0;
      dash_timeZero = 0;
      dash_time = 0;
      anticlockwise = true;
      num_turn = 0;
      turn_waittimeZero = 0;
      turn_waittime = 150;
      blk_while_turning = false;

      OC2_starttime = internalClock.read();
      if (!first_run) resetIMU();
    }
    if (!end_game){
      displayThread.enabled = false;
      OC2_endtime = internalClock.read();
      steering_percentage = (steering_percentage > 100) ? 100 : steering_percentage;
      steering_percentage = (steering_percentage < -100) ? -100 : steering_percentage;
      steering(steering_percentage);
      //MiniR4.M2.setPower(power2);
      lapDir_flash(anticlockwise);



      switch (state) {
        case INIT_CASE:
          Serial.println("INIT_CASE");
          state = MOVE_FORWARD;
          motor_last_counter = MiniR4.M2.getCounter();
          prev_state = INIT_CASE;
          break;

        case TESTING_1:
          Serial.print("TESTING_1 ");
          Serial.println(getLaser1DistMedian(7));
          MiniR4.M2.setPower(-50);
          steering_percentage = (getIMU() - tar_ang) * 10;
          if (getLaser1DistMedian(7) < 500){ // || getColorType() == 2 || getColorType() == 4 // color bug bug dei
            state = TURNING_P1;
            motor_last_counter = MiniR4.M2.getCounter();
            front_wall_dist = getLaser1DistMedian(5);
            side_wall_dist = (anticlockwise == true) ? getUltra2Dist() : getUltra1Dist();
            Serial.println(side_wall_dist);
            MiniR4.M2.setPower(0);
            MiniR4.M2.setBrake(true);
            break;
          }
          break;

        case TESTING_2:
          Serial.println("TESTING_2");
          if (num_turn % 4 == 0 && blk_while_turning){
            prev_state = TESTING_2;
            state = MID_P1_STATE;
            blk_while_turning = false;
            Serial.println("Force back mid");
            break;
          }
          else {
            if (MiniR4.M2.getCounter() - motor_last_counter < 500){ // 1000 degree = 254mm
            MiniR4.M2.setPower(-60);
            }
            else{
              prev_state = TESTING_2;
              blk_while_turning = false;
              state = CHECK_MID_RACINGLN_STATE;
              change_state_time = internalClock.read();
            }
          }  
          break;

        case TEST_LINE_TRACK:
          Serial.println("TEST_LINE_TRACK");
          MiniR4.M2.setPower(-30);
          if (MiniR4.M2.getCounter() - motor_last_counter < 2000){
            steering_percentage = (getColorType() == 2) ? -30 : 30;
          }
          else {
            state = STOP;
          }
          break;

        case MOVE_FORWARD:
          Serial.print("MOVE_FORWARD tar_ang: ");
          Serial.print(tar_ang);
          Serial.print(" IMU: ");
          Serial.print(getIMU());
          front_wall_dist = getLaser1DistMedian(7);
          Serial.print(" Laser1: ");
          Serial.println(front_wall_dist);
          MiniR4.M2.setPower(-80);
          // if (num_turn >= 12){
          //   state = STOP;
          //   break;
          // }
          // else {
            if (nearestPillarGlobal.colour != NO_COLOUR && nearestPillarGlobal.area >= 10){
              if (nearestPillarGlobal.colour == RED && nearestPillarGlobal.xpos >= min_xpos_Rcase(nearestPillarGlobal.area)){
                pillars_array.push_back({nearestPillarGlobal.colour, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area});
                pillar_front = {RED, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area};
                gyro_ang_detect_phase = getIMU();
                if (ultra2Dist > dist_threshold){
                  prev_state = MOVE_FORWARD;
                  state = TURNING_N_AVOIDING_STATE;
                  break;
                }
                else {
                  if (pillar_front.xpos > 230) curve_phase1_gyro_chkpt = 40; // For Pt A, B
                  else curve_phase1_gyro_chkpt = pillar_front.xpos * 45 / 170; // For Pt C, D, E
                  state = CURVE_P1_STATE;
                  prev_state = MOVE_FORWARD;
                }
                break;
              }
              else if (nearestPillarGlobal.colour == GREEN && nearestPillarGlobal.xpos <= min_xpos_Gcase(nearestPillarGlobal.area)){
                pillars_array.push_back({nearestPillarGlobal.colour, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area});
                pillar_front = {GREEN, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area};
                gyro_ang_detect_phase = getIMU();
                if (ultra1Dist > dist_threshold){
                  prev_state = MOVE_FORWARD;
                  state = TURNING_N_AVOIDING_STATE;
                  break;
                }
                else {
                  if (mirror(pillar_front.xpos) > 230 && pillar_front.area < 30) curve_phase1_gyro_chkpt = 40; // For Pt A
                  else if (pillar_front.area >= 30) curve_phase1_gyro_chkpt = 40; // For Pt B
                  else curve_phase1_gyro_chkpt = mirror(pillar_front.xpos) * 45 / 200; // For Pt C, D, E
                  prev_state = MOVE_FORWARD;
                  state = CURVE_P1_STATE;
                }
                break;
              }
            }
            else if (front_wall_dist < 450 && nearestPillarGlobal.colour == NO_COLOUR){ // no colour at front
              if (front_wall_dist != 8191 && front_wall_dist > 1 && abs(MiniR4.M2.getCounter()) >= 3000){ // 
                Serial.print(MiniR4.M2.getCounter() - motor_last_counter);
                Serial.print(" ");
                Serial.println(front_wall_dist);
                MiniR4.M2.setPower(0);
                MiniR4.M2.setBrake(true);
                motor_last_counter = MiniR4.M2.getCounter();
                check_front_t = internalClock.read();
                side_wall_dist = (anticlockwise == true) ? getUltra2Dist() : getUltra1Dist();
                state = CHECK_B4_TURNING;
                prev_state = MOVE_FORWARD;
                break;
              }
            } else {
              steering_percentage = (getIMU() - tar_ang) * 10;
              MiniR4.M2.setPower(-60);
            }
          // }
          break;

        case MID_P1_STATE:
          Serial.print("MID_P1_STATE ");
          if (sign(inner_wall_dist) < 0){
            Serial.println("Right");
          }
          else {
            Serial.println("Left");
          }
          MiniR4.M2.setPower(-60);
          if (nearestPillarGlobal.colour != NO_COLOUR && nearestPillarGlobal.area >= 15){
            if (nearestPillarGlobal.colour == RED && nearestPillarGlobal.xpos >= min_xpos_Rcase(nearestPillarGlobal.area) && getUltra2Dist() < dist_threshold){
              red_block_flash(anticlockwise);
              pillars_array.push_back({nearestPillarGlobal.colour, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area});
              pillar_front = {RED, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area};
              gyro_ang_detect_phase = getIMU();
              if (pillar_front.xpos > 230) curve_phase1_gyro_chkpt = 45; // For Pt A, B
              else curve_phase1_gyro_chkpt = pillar_front.xpos * 45 / 200; // For Pt C, D, E
              prev_state = MID_P1_STATE;
              state = CURVE_P1_STATE;
              break;
            } else if (nearestPillarGlobal.colour == GREEN && nearestPillarGlobal.xpos <= min_xpos_Gcase(nearestPillarGlobal.area) && getUltra1Dist() < dist_threshold){
              green_block_flash(anticlockwise);
              pillars_array.push_back({nearestPillarGlobal.colour, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area});
              pillar_front = {GREEN, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area};
              gyro_ang_detect_phase = getIMU();
              if (mirror(pillar_front.xpos) > 230 && pillar_front.area < 30) curve_phase1_gyro_chkpt = 40; // For Pt A
              else if (pillar_front.area >= 30) curve_phase1_gyro_chkpt = 40; // For Pt B
              else curve_phase1_gyro_chkpt = mirror(pillar_front.xpos) * 45 / 200; // For Pt C, D, E
              prev_state = MID_P1_STATE;
              state = CURVE_P1_STATE;
              break;
            }
          } else {
            if (abs(getIMU() - gyro_ang_detect_phase) < mid_phase1_gyro_chkpt){
              steering_percentage = 100 * sign(inner_wall_dist);
              // Serial.println(inner_wall_dist);
              // if (inner_wall_dist < 0){
              //   MiniR4.M2.setPower(0);
              // }
            } else {
              // Serial.println(inner_wall_dist);
              mid_phase2_chk_t = abs(inner_wall_dist) * 2.1 * speed_amp_factor;
              Serial.println(mid_phase2_chk_t);
              mid_save_t = internalClock.read();
              prev_state = MID_P1_STATE;
              state = MID_P2_STATE;
              break;
            }
          }
          break;
          
        case MID_P2_STATE:
          Serial.println("MID_P2_STATE");
          MiniR4.M2.setPower(-70);
          if (nearestPillarGlobal.colour != NO_COLOUR && nearestPillarGlobal.area >= 15){
            if (nearestPillarGlobal.colour == RED && nearestPillarGlobal.xpos >= min_xpos_Rcase(nearestPillarGlobal.area)){
              red_block_flash(anticlockwise);
              pillars_array.push_back({nearestPillarGlobal.colour, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area});
              pillar_front = {RED, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area};
              gyro_ang_detect_phase = getIMU();
              if (pillar_front.xpos > 230) curve_phase1_gyro_chkpt = 45; // For Pt A, B
              else curve_phase1_gyro_chkpt = pillar_front.xpos * 45 / 200; // For Pt C, D, E
              prev_state = MID_P2_STATE;
              state = CURVE_P1_STATE;
              break;
            } else if (nearestPillarGlobal.colour == GREEN && nearestPillarGlobal.xpos <= min_xpos_Gcase(nearestPillarGlobal.area)){
              green_block_flash(anticlockwise);
              pillars_array.push_back({nearestPillarGlobal.colour, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area});
              pillar_front = {GREEN, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area};
              gyro_ang_detect_phase = getIMU();
              if (mirror(pillar_front.xpos) > 230 && pillar_front.area < 30) curve_phase1_gyro_chkpt = 40; // For Pt A
              else if (pillar_front.area >= 30) curve_phase1_gyro_chkpt = 40; // For Pt B
              else curve_phase1_gyro_chkpt = mirror(pillar_front.xpos) * 45 / 200; // For Pt C, D, E
              prev_state = MID_P2_STATE;
              state = CURVE_P1_STATE;
              // state = DEBUG_STATE;
              break;
            }
          } else {
            if (internalClock.read() - mid_save_t <= mid_phase2_chk_t){
              steering_percentage = 0;
            } else {
              gyro_ang_detect_phase = getIMU();
              prev_state = MID_P2_STATE;
              state = MID_P3_STATE;
              break;
            }
          }
          break;
      
        case MID_P3_STATE:
          Serial.println("MID_P3_STATE");
          MiniR4.M2.setPower(-60);
          if (nearestPillarGlobal.colour != NO_COLOUR && nearestPillarGlobal.area >= 18){
            if (nearestPillarGlobal.colour == RED && nearestPillarGlobal.xpos >= min_xpos_Rcase(nearestPillarGlobal.area)){
              red_block_flash(anticlockwise);
              pillars_array.push_back({nearestPillarGlobal.colour, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area});
              pillar_front = {RED, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area};
              gyro_ang_detect_phase = getIMU();
              if (pillar_front.xpos > 230) curve_phase1_gyro_chkpt = 45; // For Pt A, B
              else curve_phase1_gyro_chkpt = pillar_front.xpos * 45 / 200; // For Pt C, D, E
              prev_state = MID_P3_STATE;
              state = CURVE_P1_STATE;
              break;
            } else if (nearestPillarGlobal.colour == GREEN && nearestPillarGlobal.xpos <= min_xpos_Gcase(nearestPillarGlobal.area)){
              green_block_flash(anticlockwise);
              pillars_array.push_back({nearestPillarGlobal.colour, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area});
              pillar_front = {GREEN, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area};
              gyro_ang_detect_phase = getIMU();
              if (mirror(pillar_front.xpos) > 230 && pillar_front.area < 30) curve_phase1_gyro_chkpt = 40; // For Pt A
              else if (pillar_front.area >= 30) curve_phase1_gyro_chkpt = 40; // For Pt B
              else curve_phase1_gyro_chkpt = mirror(pillar_front.xpos) * 45 / 200; // For Pt C, D, E
              prev_state = MID_P3_STATE;
              state = CURVE_P1_STATE;
              break;
            }
          } else {
            if (abs(getIMU() - tar_ang) > 12){
              steering_percentage = -100 * sign(inner_wall_dist);
            } else {
              motor_last_counter = MiniR4.M2.getCounter();
              prev_state = MID_P3_STATE;
              state = MID_DASH_STATE;
              mid_dash_save_t = internalClock.read();
              break;
            }
          }
          break;
          
        case MID_DASH_STATE:
          Serial.println("MID_DASH_STATE");
          if (num_turn >= 12){
            state = STOP;
            break;
          }
          else if (internalClock.read() - mid_dash_save_t <= 300 * speed_amp_factor){ // 400
            steering_percentage = (getIMU() - tar_ang) * 10;
            MiniR4.M2.setPower(-50);
            prev_state = MID_DASH_STATE;
          } else state = MOVE_FORWARD;
          break;

        case CURVE_P1_STATE:
          Serial.println("CURVE_P1_STATE ");
          MiniR4.M2.setPower(-60);
          if (pillar_front.colour == RED){
            red_block_flash(anticlockwise);
            if (abs(getIMU() - gyro_ang_detect_phase) + 10 < curve_phase1_gyro_chkpt){
              if (blk_while_turning) steering_percentage = -100; 
              else if (pillar_front.xpos > 230 || pillar_front.area < 13) steering_percentage = -(55 - (getIMU() - gyro_ang_detect_phase)) * pillar_front.area * steering_amp_fact1 * 320 / (55 * area_dangerzone * pillar_front.xpos);
              else steering_percentage = -(60 - (getIMU() - gyro_ang_detect_phase)) * pillar_front.area * steering_amp_fact1 * 320 / (50 * area_dangerzone * pillar_front.xpos);
              Serial.print(steering_percentage);
              Serial.print(" ");
            } else {
              if (blk_while_turning) {
                prev_state = CURVE_P1_STATE;
                state = CURVE_P2_5_STATE;
                curve_sphase3_chk_t = pillar_front.area * 2 * speed_amp_factor;
                Serial.println("RED blk_while_turning");
                pillar_avoid_save_t = internalClock.read();
                //break;
              } else if (pillar_front.xpos > 230 && pillar_front.area < 30){ // For Pt A
                curve_phase2_chk_t = 520 * (pillar_front.xpos - 20) * speed_amp_factor / 200; //500;
                prev_state = CURVE_P1_STATE;
                state = CURVE_P2_STATE;
                Serial.println("RED Pt A");
                //break;
              } else if (pillar_front.area >= 16){ // For Pt B
                curve_phase2_chk_t = 100 * (pillar_front.xpos - 50) * speed_amp_factor / 200; // 80*
                prev_state = CURVE_P1_STATE;
                state = CURVE_P2_STATE;
                Serial.println("RED Pt B");
                //break;
              } else if (pillar_front.area > 10){ // For Pt C, D
                curve_phase2_chk_t = 20 * (pillar_front.xpos - 50) * speed_amp_factor / 300; // 20*, 50*
                prev_state = CURVE_P1_STATE;
                state = CURVE_P2_STATE;
                Serial.println("RED Pt C, D");
                //break;
              } else { // For Pt E
                gyro_ang_detect_phase = getIMU();
                prev_state = CURVE_P1_STATE;
                state = CURVE_P3_STATE;
                curve_phase2_chk_t = 200;
                Serial.println("RED Pt E");
                //break;
              }
              pillar_avoid_save_t = internalClock.read();
              motor_last_counter = MiniR4.M2.getCounter();
              break;
            }
          } else {
            green_block_flash(anticlockwise);
            if (abs(getIMU() - gyro_ang_detect_phase) + 20 < curve_phase1_gyro_chkpt){
              if (blk_while_turning) steering_percentage = 100;
              if (mirror(pillar_front.xpos) > 230) steering_percentage = (50 - abs(getIMU() - gyro_ang_detect_phase)) * pillar_front.area * steering_amp_fact1 * 320 / (60 * area_dangerzone * mirror(pillar_front.xpos)); // || pillar_front.area < 13
              else steering_percentage = (50 - abs(getIMU() - gyro_ang_detect_phase)) * pillar_front.area * steering_amp_fact1 * 320 / (50 * area_dangerzone * mirror(pillar_front.xpos)); //320
              Serial.print(steering_percentage);
              Serial.println(" ");
            } else {
              if (blk_while_turning) {
                prev_state = CURVE_P1_STATE;
                state = CURVE_P2_5_STATE;
                curve_sphase3_chk_t = pillar_front.area * 10 * speed_amp_factor;
                pillar_avoid_save_t = internalClock.read();
                Serial.println("GREEN blk_while_turning");
                //break;
              } else if (mirror(pillar_front.xpos) > 230 && pillar_front.area < 30){ // For Pt A
                curve_phase2_chk_t = 150 * (mirror(pillar_front.xpos) - 50) * speed_amp_factor / 300;
                prev_state = CURVE_P1_STATE;
                state = CURVE_P2_STATE;
                Serial.println("GREEN Pt A");
                //break;
              } else if (pillar_front.area >= 30 && pillar_front.xpos < 285){ // For Pt B
                curve_phase2_chk_t = 40 * (mirror(pillar_front.xpos) - 50) * speed_amp_factor / 300;
                prev_state = CURVE_P1_STATE;
                state = CURVE_P2_STATE;
                Serial.println("GREEN Pt B");
                //break;
              } else if (pillar_front.area > 13){ // For Pt C, D
                curve_phase2_chk_t = 20 * (mirror(pillar_front.xpos) - 50) * speed_amp_factor / 250; // 30*
                gyro_ang_detect_phase = getIMU();
                prev_state = CURVE_P1_STATE;
                state = CURVE_P2_STATE;
                Serial.println("GREEN Pt C, D");
                //break;
              } else { // For Pt E
                curve_phase2_chk_t = 100 * (mirror(pillar_front.xpos) - 50) * speed_amp_factor / 300;
                gyro_ang_detect_phase = getIMU();
                prev_state = CURVE_P1_STATE;
                state = CURVE_P2_STATE;
                Serial.println("GREEN Pt E");
                //break;
              }
              pillar_avoid_save_t = internalClock.read();
              motor_last_counter = MiniR4.M2.getCounter();
              //break;
            }
            break;
          }
          Serial.println(curve_phase2_chk_t);
          break;

        case CURVE_P2_STATE:
          Serial.println("CURVE_P2_STATE");
          MiniR4.M2.setPower(-60);
          if (internalClock.read() - pillar_avoid_save_t <= curve_phase2_chk_t + 20 || abs(MiniR4.M2.getCounter() - motor_last_counter) < 500){ // - 20 // curve_phase2_chk_t
            steering_percentage = 0;
            Serial.print(internalClock.read() - pillar_avoid_save_t);
            Serial.print(" ");
            Serial.println(abs(MiniR4.M2.getCounter() - motor_last_counter));
          } else {
            gyro_ang_detect_phase = getIMU();
            prev_state = CURVE_P2_STATE;
            state = CURVE_P3_STATE;
            break;
          }
          break;

        case CURVE_P2_5_STATE:
          Serial.println("CURVE_P2_5_STATE");
          MiniR4.M2.setPower(-60);
          if (internalClock.read() - pillar_avoid_save_t <= curve_sphase3_chk_t + 100){
            steering_percentage = 0;
          } else {
            prev_state = CURVE_P2_5_STATE;
            state = CURVE_P3_STATE;
            break;
          }
          break;

        case CURVE_P3_STATE:
          Serial.println("CURVE_P3_STATE");
          MiniR4.M2.setPower(-60);
          // if (nearestPillarGlobal.colour != NO_COLOUR && nearestPillarGlobal.colour > 10){
          //    if (nearestPillarGlobal.colour == RED && nearestPillarGlobal.xpos >= min_xpos_Rcase(nearestPillarGlobal.area)){
          //     red_block_flash(anticlockwise);
          //     pillars_array.push_back({nearestPillarGlobal.colour, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area});
          //     pillar_front = {RED, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area};
          //     gyro_ang_detect_phase = getIMU();
          //     if (pillar_front.xpos > 230) curve_phase1_gyro_chkpt = 45; // For Pt A, B
          //     else curve_phase1_gyro_chkpt = pillar_front.xpos * 45 / 200; // For Pt C, D, E
          //     prev_state = CURVE_P3_STATE;
          //     state = CURVE_P1_STATE;
          //     break;
          //   } else if (nearestPillarGlobal.colour == GREEN && nearestPillarGlobal.xpos <= min_xpos_Gcase(nearestPillarGlobal.area)){
          //     green_block_flash(anticlockwise);
          //     pillars_array.push_back({nearestPillarGlobal.colour, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area});
          //     pillar_front = {GREEN, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area};
          //     gyro_ang_detect_phase = getIMU();
          //     if (mirror(pillar_front.xpos) > 230 && pillar_front.area < 30) curve_phase1_gyro_chkpt = 40; // For Pt A
          //     else if (pillar_front.area >= 30) curve_phase1_gyro_chkpt = 40; // For Pt B
          //     else curve_phase1_gyro_chkpt = mirror(pillar_front.xpos) * 45 / 200; // For Pt C, D, E
          //     prev_state = CURVE_P3_STATE;
          //     state = CURVE_P1_STATE;
          //     break;
          //   }
          // } else {
            if (abs(getIMU() - tar_ang) > 12 && internalClock.read()){ // 20
              if (pillar_front.colour == RED){
                red_block_flash(anticlockwise);
                if (getIMU() - tar_ang > 10){ //30
                  if (blk_while_turning) steering_percentage = 100;
                  else steering_percentage = (abs(tar_ang - getIMU()) + 30) * steering_amp_fact2; // (abs(getIMU()) + 25) * steering_amp_fact2;
                } else {
                  prev_state = CURVE_P3_STATE;
                  state = CURVE_P4_STATE;
                  pillar_avoid_save_t = internalClock.read();
                  break;
                }
              } else {
                green_block_flash(anticlockwise);
                if (tar_ang - getIMU() > 30){ //20
                  if (blk_while_turning) steering_percentage = -100;
                  else steering_percentage = -(abs(tar_ang - getIMU()) + 30) * steering_amp_fact2;
                } else {
                  prev_state = CURVE_P3_STATE;
                  state = CURVE_P4_STATE;
                  pillar_avoid_save_t = internalClock.read();
                  break;
                }
              }
            } else {
              prev_state = CURVE_P3_STATE;
              state = CURVE_P4_STATE;
              pillar_avoid_save_t = internalClock.read();
              break;
            }
          // }
          break;

        case CURVE_P4_STATE:
          Serial.println("CURVE_P4_STATE");
          MiniR4.M2.setPower(-60);
          // if (nearestPillarGlobal.area >= 40) pillar_avoid_save_t = internalClock.read();
          CP4_t = (num_turn % 4 == 0 && num_turn < 12) ? 1000 : (blk_while_turning) ? 800 : 500;
          if (num_turn % 4 == 0 && num_turn < 12) CP4_t = 1000;
          else if (blk_while_turning) CP4_t = 300;
          else if (pillar_front.colour == RED) {
            if (pillar_front.xpos > 230 && pillar_front.area < 30) CP4_t = 600; // For Pt A
            else if (pillar_front.area >= 16) CP4_t = 300; // For Pt B
            else if (pillar_front.area > 10) CP4_t = 850; // For Pt C, D
            else CP4_t = 200; // For Pt E // 500
          } else {
            if (mirror(pillar_front.xpos) > 230 && pillar_front.area < 30) CP4_t = 500; // For Pt A
            else if (pillar_front.area >= 30) CP4_t = 300; // For Pt B
            else curve_phase1_gyro_chkpt = CP4_t = 500; // For Pt C, D, E
          }
          if (internalClock.read() - pillar_avoid_save_t <= CP4_t * speed_amp_factor){
            steering_percentage = (getIMU() - tar_ang) * 10;
          } else {
            no_blk_flash(anticlockwise);
            prev_state = CURVE_P4_STATE;
            state = TESTING_2;
            change_state_time = internalClock.read();
            if (blk_while_turning) MiniR4.M2.resetCounter();
            blk_while_turning = false;
            motor_last_counter = MiniR4.M2.getCounter();
            steering_percentage = 0;
            break;
          }
          break;

        case CHECK_MID_RACINGLN_STATE:
          Serial.println("CHECK_MID_RACINGLN_STATE");
          MiniR4.M2.setPower(-60);
          front_wall_dist = getLaser1DistMedian(7);
          if (anticlockwise){
            if (abs(ultra1Dist - MID_RACINGLN_POS) > 60 && ultra1Dist < MID_RACINGLN_POS){
              motor_last_counter = MiniR4.M2.getCounter();
              prev_state = CHECK_MID_RACINGLN_STATE;
              state = CHECK_MID_TWICE;
              break;
            }
            if (abs(ultra1Dist - MID_RACINGLN_POS) > 60 && ultra1Dist < dist_threshold && front_wall_dist > 2000){ // && MiniR4.M2.getCounter() < 1000
              inner_wall_dist = ultra1Dist - MID_RACINGLN_POS;
              gyro_ang_detect_phase = getIMU();
              prev_state = CHECK_MID_RACINGLN_STATE;
              state = MID_P1_STATE;
              break;
            } 
            else if (internalClock.read() - change_state_time > 120 * speed_amp_factor) {
              prev_state = CHECK_MID_RACINGLN_STATE;
              pillar_front = {pillar_front.colour, pillar_front.xpos, CAM_LOWEST_YPOS - pillar_front.ypos, pillar_front.height, pillar_front.width, pillar_front.area};
              state = MOVE_FORWARD;
              break;
            }
          } else {
            if (abs(ultra2Dist - MID_RACINGLN_POS) > 60 && ultra2Dist < MID_RACINGLN_POS){
              motor_last_counter = MiniR4.M2.getCounter();
              prev_state = CHECK_MID_RACINGLN_STATE;
              state = CHECK_MID_TWICE;
              break;
            }
            if (abs(ultra2Dist - MID_RACINGLN_POS) > 60 && ultra2Dist < dist_threshold && front_wall_dist > 2000){
              inner_wall_dist = -(ultra2Dist - MID_RACINGLN_POS); 
              gyro_ang_detect_phase = getIMU();
              prev_state = CHECK_MID_RACINGLN_STATE;
              state = MID_P1_STATE;
              break;
            } else if (internalClock.read() - change_state_time > 100 * speed_amp_factor) {
              prev_state = CHECK_MID_RACINGLN_STATE;
              state = MOVE_FORWARD;
              break;
            }
          }
          break;

        case CHECK_MID_TWICE:
          Serial.println("CHECK_MID_TWICE");
          if (MiniR4.M2.getCounter() - motor_last_counter < 400){ // 1000 degree = 254mm
            MiniR4.M2.setPower(-60);
          }
          else{
            prev_state = CHECK_MID_TWICE;
            state = CHECK_MID_RACINGLN_STATE;
            change_state_time = internalClock.read();
          } 
          break;

        case CHECK_B4_TURNING:
          Serial.println("CHECK_B4_TURNING");
          MiniR4.M2.setPower(0);
          MiniR4.M2.setBrake(true);
          front_wall_dist = getLaser1DistMedian(7);
          if (internalClock.read() - check_front_t > 100){
            if (front_wall_dist < 500){
              prev_state = CHECK_B4_TURNING;
              state = TURNING_P1;
            }
            else {
              prev_state = CHECK_B4_TURNING;
              state = MOVE_FORWARD;
            }
          }
          break;

        case TURNING_P1:
          Serial.println("TURNING_P1");
          if (side_wall_dist >= 620){
            state = TURNING_P1_5;
            prev_state = TURNING_P1;
            break;
          }
          else if (front_wall_dist < 140){
            state = TURNING_P1_0;
            prev_state = TURNING_P1;
            break;
          }
          else if (abs(MiniR4.M2.getCounter() - motor_last_counter) < 850){ // || getIMU() < tar_ang - 45 //anticlockwise
            MiniR4.M2.setPower(-50);
            // if (anticlockwise) steering_percentage = 100;
            // else steering_percentage = -100;
            steering_percentage = (anticlockwise) ? 100 : -100;
          }
          else {
            motor_last_counter = MiniR4.M2.getCounter();
            state = TURNING_P2;
            prev_state = TURNING_P1;
          }
          break;

        case TURNING_P1_0:
          Serial.println("TURNING_P1_0");
          if (front_wall_dist < 200){
            MiniR4.M2.setPower(50);
          }
          else {
            prev_state = TURNING_P1_0;
            state = TURNING_P1;
          }
          break;

        case TURNING_P2:
          Serial.println("TURNING_P2");
          if (abs(MiniR4.M2.getCounter() - motor_last_counter) < 850){
            MiniR4.M2.setPower(50);
            // if (anticlockwise) steering_percentage = -100;
            // else steering_percentage = 100;
            steering_percentage = (anticlockwise) ? -100 : 100;
          }
          else {
            tar_ang += (anticlockwise == true) ? -right_ang : right_ang; // 88.58
            motor_last_counter = MiniR4.M2.getCounter();
            steering_percentage = 0;
            num_turn += 1;
            state = TURNING_P3;
            prev_state = TURNING_P2;
          }
          break;

        case TURNING_P1_5:
          Serial.println("TURNING_P1_5");
          front_wall_dist = getLaser1DistMedian(7);
          if (abs(MiniR4.M2.getCounter() - motor_last_counter) < 1100 && front_wall_dist >= 120){
            MiniR4.M2.setPower(-60);
            steering_percentage = (anticlockwise) ? -10 : 10;
          }
          else {
            motor_last_counter = MiniR4.M2.getCounter();
            state = TURNING_P2_5;
            prev_state = TURNING_P1_5;
          }
          break;

        case TURNING_P2_5:
          Serial.println("TURNING_P2_5");
          if (abs(MiniR4.M2.getCounter() - motor_last_counter) < 1600){
            MiniR4.M2.setPower(50);
            // if (anticlockwise) steering_percentage = -100;
            // else steering_percentage = 100;
            steering_percentage = (anticlockwise) ? -100 : 100;
          }
          else {
            tar_ang += (anticlockwise == true) ? -right_ang : right_ang; // 88.58
            motor_last_counter = MiniR4.M2.getCounter();
            side_wall_dist -= 150;
            steering_percentage = 0;
            num_turn += 1;
            state = TURNING_P3;
            prev_state = TURNING_P2_5;
          }
          break;

        case TURNING_P3:
          Serial.print("TURNING_P3 ");
          mid_dist = abs(side_wall_dist - 450) * 3.92;
          Serial.println(mid_dist);
          if (side_wall_dist > 450){
            if (abs(MiniR4.M2.getCounter() - motor_last_counter) < mid_dist - 30){
              MiniR4.M2.setPower(60);
              Serial.println(">450");
            }
            else {
              if (pillar_front.area > 0) blk_while_turning = true;
              state = MOVE_FORWARD;
              MiniR4.M2.resetCounter();
            }
          }
          else if (side_wall_dist < 450){
            if (abs(MiniR4.M2.getCounter() - motor_last_counter) < mid_dist){
              MiniR4.M2.setPower(-60);
              Serial.println("<450");
            }
            else {
              if (pillar_front.area > 7) blk_while_turning = true;
              state = MOVE_FORWARD;
              MiniR4.M2.resetCounter();
            }
          }
          else {
            if (pillar_front.area > 7) blk_while_turning = true;
            state = MOVE_FORWARD;
            MiniR4.M2.resetCounter();
            }
          break;

          case TURNING_N_AVOIDING_STATE:
            Serial.println("TURNING_N_AVOIDING_STATE");
            from_tNa_state = true;
            if (pillar_front.colour == RED) {
              turn_waittime = pillar_front.area * 50; // 49.2; // 48.5
              redblk_tna_flash(anticlockwise);
              // red_block_flash(anticlockwise);
            } else {
              turn_waittime = pillar_front.area * 5;
              greenblk_tna_flash(anticlockwise);
              // green_block_flash(anticlockwise);
            }
            prev_state = TURNING_N_AVOIDING_STATE;
            state = WAIT_TURN_STATE;
            turn_waittimeZero = internalClock.read();
            break;

          case WAIT_TURN_STATE:
            Serial.println("WAIT_TURN_STATE");
            if ((internalClock.read() - turn_waittimeZero) < turn_waittime) {
              steering_percentage = (getIMU() - tar_ang) * 10;
              break;
            }
            else {
              if (anticlockwise){
                if (ultra1Dist < dist_threshold){
                  prev_state = WAIT_TURN_STATE;
                  state = MOVE_FORWARD;
                  break;
                } else {
                  prev_state = WAIT_TURN_STATE;
                  state = TURNING_STATE;
                  break;
                }
              } else {
                if (ultra2Dist < dist_threshold){
                  prev_state = WAIT_TURN_STATE;
                  state = MOVE_FORWARD;
                  break;
                } else {
                  prev_state = WAIT_TURN_STATE;
                  state = TURNING_STATE;
                  break;
                }
              }
            }
            break;

        case TURNING_STATE:
          Serial.println("TURNING_STATE");
          if (prev_state == WAIT_TURN_STATE){
            tar_ang += (anticlockwise == true) ? -right_ang : right_ang; // 88.58
            motor_last_counter = MiniR4.M2.getCounter();
            num_turn += 1;
            prev_state == TURNING_STATE;
          }
          if ((abs(tar_ang - getIMU()) > 20) && (abs(tar_ang) > abs(getIMU()))){
            steering_percentage = (tar_ang > 0) ? -100 : 100;
            MiniR4.M2.setPower(-50);
          }
          else {
            steering_percentage = 0;
            blk_while_turning = true;
            prev_state = TURNING_STATE;
            state = TESTING_2;
            break;
          }
          break;

        case OUT_PARKING_P1_STATE:
          Serial.println("OUT_PARKING_P1_STATE");
          MiniR4.M2.setPower(30);
          if(anticlockwise){
            if (getIMU() > -10) {
              steering_percentage = -100;
            } else {
              MiniR4.M2.resetCounter();
              prev_state = OUT_PARKING_P1_STATE;
              state = OUT_PARKING_P2_STATE;
              break;
            }
          } else {
            if (getIMU() < 10) {
              steering_percentage = 100;
            } else {
              MiniR4.M2.resetCounter();
              prev_state = OUT_PARKING_P1_STATE;
              state = OUT_PARKING_P2_STATE;
              break;
            }
          }
          break;

        case OUT_PARKING_P2_STATE:
          Serial.println("OUT_PARKING_P2_STATE");
          MiniR4.M2.setPower(-30);
          if(anticlockwise){
            if (getIMU() > -45) {
              steering_percentage = 100;
            } else {
              MiniR4.M2.resetCounter();
              prev_state = OUT_PARKING_P2_STATE;
              state = OUT_PARKING_P3_STATE;
              break;
            }
          } else {
            if (getIMU() < 45) {
              steering_percentage = -100;
            } else {
              MiniR4.M2.resetCounter();
              prev_state = OUT_PARKING_P2_STATE;
              state = OUT_PARKING_P3_STATE;
              break;
            }
          }
          break;

        case OUT_PARKING_P3_STATE:
          Serial.println("OUT_PARKING_P3_STATE");
          MiniR4.M2.setPower(30);
          if (anticlockwise) steering_percentage = -100;
          else steering_percentage = 100;
          if(abs(MiniR4.M2.getCounter()) < 300){
            power2 = 30;
          } else {
            MiniR4.M2.resetCounter();
            prev_state = OUT_PARKING_P3_STATE;
            state = OUT_PARKING_P4_STATE;
            break;
          }
          break;
        
        case OUT_PARKING_P4_STATE:
          Serial.println("OUT_PARKING_P4_STATE");
          MiniR4.M2.setPower(-30);
          if(anticlockwise){
            if (getIMU() > -85) {
              steering_percentage = 100;
            } else {
              MiniR4.M2.resetCounter();
              prev_state = OUT_PARKING_P4_STATE;
              state = OUT_PARKING_P5_STATE;
              break;
            }
          } else {
            if (getIMU() < 85) {
              steering_percentage = -100;
            } else {
              MiniR4.M2.resetCounter();
              prev_state = OUT_PARKING_P4_STATE;
              state = OUT_PARKING_P5_STATE;
              break;
            }
          }
          break;

        case OUT_PARKING_P5_STATE:
          Serial.println("OUT_PARKING_P5_STATE");
          MiniR4.M2.setPower(-50);
          if(anticlockwise){
            if (abs(MiniR4.M2.getCounter()) < 1750) {
              steering_percentage = (getIMU() - -90) * 10;
            } else {
              MiniR4.M2.resetCounter();
              prev_state = OUT_PARKING_P5_STATE;
              state = OUT_PARKING_P6_STATE;
              break;
            }
          } else {
            if (abs(MiniR4.M2.getCounter()) < 1750) {
              steering_percentage = (getIMU() - 90) * 10;
            } else {
              MiniR4.M2.resetCounter();
              prev_state = OUT_PARKING_P5_STATE;
              state = OUT_PARKING_P6_STATE;
              break;
            }
          }
          break;

        case OUT_PARKING_P6_STATE:
          Serial.println("OUT_PARKING_P6_STATE");
          MiniR4.M2.setPower(30);
          if (anticlockwise){
            if (getIMU() < 0) steering_percentage = 100;
            else {
              if (nearestPillarGlobal.colour != NO_COLOUR && nearestPillarGlobal.area > 13){
                MiniR4.M2.resetCounter();
                prev_state = OUT_PARKING_P6_STATE;
                state = OUT_PARKING_P7_STATE;
                break;
              } else {
                MiniR4.M2.resetCounter();
                prev_state = OUT_PARKING_P6_STATE;
                state = MOVE_FORWARD;
                break;
              }
            }
          } else {
            if (getIMU() > 0) steering_percentage = -100;
            else {
                if (nearestPillarGlobal.colour != NO_COLOUR && nearestPillarGlobal.area > 13){
                  MiniR4.M2.resetCounter();
                  prev_state = OUT_PARKING_P6_STATE;
                  state = OUT_PARKING_P7_STATE;
                  break;
                } else {
                  MiniR4.M2.resetCounter();
                  prev_state = OUT_PARKING_P6_STATE;
                  state = MOVE_FORWARD;
                  break;
                }
            }
          }
          break;
        
        case OUT_PARKING_P7_STATE:
          Serial.println("OUT_PARKING_P7_STATE");
          MiniR4.M2.setPower(50);
          if (abs(MiniR4.M2.getCounter()) < 1750){
            steering_percentage = 0;
          } else {
            power2 = -30;
            MiniR4.M2.resetCounter();
            prev_state = OUT_PARKING_P7_STATE;
            state = MOVE_FORWARD;
            break;
          }
          break;

        case STOP:
          Serial.print("STOP ");
          Serial.println(getIMU());
          MiniR4.M2.setPower(0);
          MiniR4.M2.setBrake(true);
          break;
        
        default:
          break;
        }
    } else {
      steering(0);
      MiniR4.M2.setBrake(true);
      MiniR4.M2.setPower(0);
      displayThread.enabled = true;
      Ultra1Thread.enabled = true;
      Ultra2Thread.enabled = true;
    }
  }
}

void OC2main(){
  // LaserMode = CISTERN_DIST;
  // LaserElements = 8;
  // OpenChallenge300(MEDIAN_DIST, 10, 88.58, 1500, 300, -100);
  // OpenChallengeLaserFilter(LaserMode, LaserElements, 90, 2500, -100);
  // OC2Huskylens(84, 1000, -70);
  OC2HuskylensNColor(88.1, 900, -80); //-80
}

void showOC2Time(COLUMN column, int line_number, int size, bool clearDisplay){
  static String curr_time_text;
  display.oledSetTextColour(BLACK);

  long total_ms = OC2_endtime - OC2_starttime;
  long seconds = total_ms / 1000;
  long milliseconds = total_ms % 1000;

  if (column == LEFT){
    display.oledDisplayLeftln(line_number, size, curr_time_text, clearDisplay);
    display.oledSetTextColour(WHITE);
    curr_time_text = String(seconds) + "." + String(milliseconds);
    display.oledDisplayLeftln(line_number, size, curr_time_text, clearDisplay);

  } else if (column == MID) {
    display.oledDisplayCenterln(line_number, size, curr_time_text, clearDisplay);
    display.oledSetTextColour(WHITE);
    curr_time_text = String(seconds) + "." + String(milliseconds);
    display.oledDisplayCenterln(line_number, size, curr_time_text, clearDisplay);
  } else {
    display.oledDisplayRightln(line_number, size, curr_time_text, clearDisplay);
    display.oledSetTextColour(WHITE);
    curr_time_text = String(seconds) + "." + String(milliseconds);
    display.oledDisplayRightln(line_number, size, curr_time_text, clearDisplay);
  }
}

