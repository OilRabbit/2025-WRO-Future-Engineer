#include "MatrixMiniR4.h"
#include <cmath>
#include "Arduino.h"
#include "OC2.h"

bool reset_OC2 = true;
long OC2_starttime = 0; 
long OC2_endtime = 0; 

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
void OC2_pixy2(double right_ang, int dist_threshold, int power){
  safeSuspend(blinkledThread);
  // safeSuspend(OC1Thread);

  static OC2_STATES state = DETECT_STATE;     // A variable storing the current state of OC2 run
  static OC2_STATES prev_state = DETECT_STATE;
  static bool end_game = false;           // A flag determining whether the run has ended
  static double tar_ang = 0;              // The target angle which the car should be facing
  static bool is_anticlockwise = false;      // A boolean storing whether the car is racing in clockwise or anti-clockwise direction
  static int num_turn = 0;                // Variable storing the number of turns the car has made
  float imu_kp = 2;
  std::vector<COLOURED_OBJ> pillars_array;    // An array storing all the pillars detected
  static COLOURED_OBJ pillar_front;           // A struct storing all the info of the nearest pillar detected during DETECT_STATE
  const int area_dangerzone = 40;             // A variable representing the area of a pillar which is identified as danger when its area is larger than this value
  const float steering_amp_fact1 = 200;       // An amplification factor for calculating the steering percentage during CURVE_P1_STATE
  const float steering_amp_fact2 = 2;         // An amplification factor for calculating the steering percentage during CURVE_P3_STATE
  static int gyro_ang_detect_phase = 0;       // A variable storing the current IMU angle
  static int pillar_avoid_save_t = 0;         // A variable storing the instant time for avoiding pillars
  static float curve_phase2_chk_t = 300;      // A variable storing the time required for the car to move straight forward during CURVE_P2_STATE. Non-editable as it will be recalculated during the run
  static float curve_sphase3_chk_t = 300;     // A variable storing the time required for the car to move straight forward during CURVE_P2_5_STATE. Non-editable as it will be recalculated during the run
  static int curve_phase1_gyro_chkpt = 45;    // A variable storing the IMU value for the car to turn to during CURVE_P1_STATE. Non-editable as it will be recalculated during the run
  static int mid_phase1_gyro_chkpt = 50;      // A variable storing the IMU value for the car to turn to during MID_P1_STATE. Non-editable as it will be recalculated during the run
  static float inner_wall_dist = 0;           // A variable storing the distance between the car and the inner wall (measured using ultrasonic sensor)
  static int mid_save_t = 0;                  // A variable storing the instant time for going back to mid racing line
  static float mid_phase2_chk_t = 0;          // A variable storing the time required for the car to move straight forward during MID_P2_STATE. Non-editable as it will be recalculated during the run
  static int mid_dash_save_t = 0;             // A variable storing the time required for the car to move straight forward during MID_DASH_STATE. Non-editable as it will be recalculated during the run
  static double tar_ang = 0;                  // The target angle which the car should be facing
  static long dash_timeZero = 0;              // Variable to store the instant time from the internal clock for dashing
  static long dash_time = 0;                  // The time required for the car to run before using the ultrasonic sensors for detection again after turning
  static bool anticlockwise = false;          // A boolean storing whether the car is racing in clockwise or anti-clockwise direction
  static int num_turn = 0;                    // Variable storing the number of turns the car has made
  static long turn_waittimeZero = 0;          // Variable to store the instant time from the internal clock for waiting to turning
  static int turn_waittime = 150;             // Variable storing the time in ms that required to wait before the car turn 
  static bool blk_while_turning = false;      // A flag determining whether any pillars in close distance is detected during the turning

  if (reset_OC2){
    tar_ang = 0;
    is_anticlockwise = false;
    num_turn = 0;
    steering_percentage = 0;
    end_game = false;
    state = DETECT_STATE;
    gyro_ang_detect_phase = 0;
    pillar_avoid_save_t = 0;
    curve_phase2_chk_t = 300;
    curve_sphase3_chk_t = 300;
    curve_phase1_gyro_chkpt = 45;
    mid_phase1_gyro_chkpt = 50;
    inner_wall_dist = 0;
    mid_save_t = 0;
    mid_phase2_chk_t = 0;
    mid_dash_save_t = 0;
    tar_ang = 0;
    dash_timeZero = 0;
    dash_time = 0;
    num_turn = 0;
    turn_waittimeZero = 0;
    turn_waittime = 150;
    blk_while_turning = false;
    OC2_starttime = internalClock.read();
    imu_resetYaw();
    reset_OC2 = false;
  }

  if (!end_game){
    displayThread.enabled = false;
    OC2_endtime = internalClock.read();
    steering(steering_percentage);
    MiniR4.M2.setPower(power);

    // The Finte State Machine for OC2 which is well organized
    switch (state) {
      // As the name implies, this state do the detections using ultrasonic sensors and huskylens when driving on a straight sector.
      case DETECT_STATE:
      Serial.println("DETECT_STATE");
        // Before doing any turning, both ultrasonic sensors are locked
        if (num_turn == 0) {
          if (ultra1Dist > dist_threshold){
            anticlockwise = true;
            Ultra2Thread.enabled = false;
            if (nearestPillarGlobal.colour != NO_COLOUR){
              if (nearestPillarGlobal.area >= 20){
                pillars_array.push_back({nearestPillarGlobal.colour, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area});
                pillar_front = {nearestPillarGlobal.colour, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area};
                state = TURNING_N_AVOIDING_STATE;
              }
            } else {
              state = WAIT_TURN_STATE;
              turn_waittime = 10;
              turn_waittimeZero = internalClock.read();
            }
          } else if (ultra2Dist > dist_threshold){
            anticlockwise = false;
            Ultra1Thread.enabled = false;
            if (nearestPillarGlobal.colour != NO_COLOUR){
              if (nearestPillarGlobal.area >= 20){
                pillars_array.push_back({nearestPillarGlobal.colour, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area});
                pillar_front = {nearestPillarGlobal.colour, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area};
                state = TURNING_N_AVOIDING_STATE;
              }
            } else {
              state = WAIT_TURN_STATE;
              turn_waittime = 10;
              turn_waittimeZero = internalClock.read();
            }
          } else if (nearestPillarGlobal.colour == RED){
              if (nearestPillarGlobal.area >= 10 && nearestPillarGlobal.xpos > min_xpos_Rcase(nearestPillarGlobal.area)){
                pillars_array.push_back({nearestPillarGlobal.colour, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area});
                pillar_front = {nearestPillarGlobal.colour, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area};
                gyro_ang_detect_phase = getIMU();
                if (pillar_front.xpos > 230) curve_phase1_gyro_chkpt = 45; // For Pt A, B
                else curve_phase1_gyro_chkpt = pillar_front.xpos * 45 / 200; // For Pt C, D, E
                state = CURVE_P1_STATE;
              }
          } else if (nearestPillarGlobal.colour == GREEN){
            if (nearestPillarGlobal.area >= 10 && nearestPillarGlobal.xpos < min_xpos_Gcase(nearestPillarGlobal.area)){
              pillars_array.push_back({nearestPillarGlobal.colour, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area});
              pillar_front = {nearestPillarGlobal.colour, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area};
              gyro_ang_detect_phase = getIMU();
              if (mirror(pillar_front.xpos) > 230 && pillar_front.area < 30) curve_phase1_gyro_chkpt = 40; // For Pt A
              else if (pillar_front.area >= 30) curve_phase1_gyro_chkpt = 40; // For Pt B
              else curve_phase1_gyro_chkpt = mirror(pillar_front.xpos) * 40 / 200; // For Pt C, D, E
              state = CURVE_P1_STATE;
            }
          } else {
            steering_percentage = (getIMU() - tar_ang) * 10; // TODO: Verify whether a minus sign is needed
          }
        } else {
          if (anticlockwise){
            if (ultra1Dist > dist_threshold){
              if (nearestPillarGlobal.colour != NO_COLOUR){
                if (nearestPillarGlobal.area >= 20){
                  pillars_array.push_back({nearestPillarGlobal.colour, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area});
                  pillar_front = {nearestPillarGlobal.colour, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area};
                  state = TURNING_N_AVOIDING_STATE;
                }
              } else {
                state = WAIT_TURN_STATE;
                turn_waittime = 10;
                turn_waittimeZero = internalClock.read();
              }
            } else if (nearestPillarGlobal.colour == RED){
              if (nearestPillarGlobal.area >= 10 && nearestPillarGlobal.xpos > min_xpos_Rcase(nearestPillarGlobal.area)){
                pillars_array.push_back({nearestPillarGlobal.colour, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area});
                pillar_front = {nearestPillarGlobal.colour, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area};
                gyro_ang_detect_phase = getIMU();
                if (pillar_front.xpos > 230) curve_phase1_gyro_chkpt = 45; // For Pt A, B
                else curve_phase1_gyro_chkpt = pillar_front.xpos * 45 / 200; // For Pt C, D, E
                state = CURVE_P1_STATE;
              }
            } else if (nearestPillarGlobal.colour == GREEN){
              if (nearestPillarGlobal.area >= 10 && nearestPillarGlobal.xpos < min_xpos_Gcase(nearestPillarGlobal.area)){
                pillars_array.push_back({nearestPillarGlobal.colour, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area});
                pillar_front = {nearestPillarGlobal.colour, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area};
                if (mirror(pillar_front.xpos) > 230 && pillar_front.area < 30) curve_phase1_gyro_chkpt = 40; // For Pt A
                else if (pillar_front.area >= 30) curve_phase1_gyro_chkpt = 40; // For Pt B
                else curve_phase1_gyro_chkpt = mirror(pillar_front.xpos) * 40 / 200; // For Pt C, D, E
                state = CURVE_P1_STATE;
              }
            } else {
              steering_percentage = (getIMU() - tar_ang) * 10; // TODO: Verify whether a minus sign is needed
            }
          } else {
            if (ultra2Dist > dist_threshold){
              if (nearestPillarGlobal.colour != NO_COLOUR){
                if (nearestPillarGlobal.area >= 20){
                  pillars_array.push_back({nearestPillarGlobal.colour, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area});
                  pillar_front = {nearestPillarGlobal.colour, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area};
                  state = TURNING_N_AVOIDING_STATE;
                }
              } else {
                state = WAIT_TURN_STATE;
                turn_waittime = 10;
                turn_waittimeZero = internalClock.read();
              }
            } else if (nearestPillarGlobal.colour == RED){
              if (nearestPillarGlobal.area >= 10 && nearestPillarGlobal.xpos > min_xpos_Rcase(nearestPillarGlobal.area)){
                pillars_array.push_back({nearestPillarGlobal.colour, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area});
                pillar_front = {nearestPillarGlobal.colour, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area};
                gyro_ang_detect_phase = getIMU();
                if (pillar_front.xpos > 230) curve_phase1_gyro_chkpt = 45; // For Pt A, B
                else curve_phase1_gyro_chkpt = pillar_front.xpos * 45 / 200; // For Pt C, D, E
                state = CURVE_P1_STATE;
              }
            } else if (nearestPillarGlobal.colour == GREEN){
              if (nearestPillarGlobal.area >= 10 && nearestPillarGlobal.xpos < min_xpos_Gcase(nearestPillarGlobal.area)){
                pillars_array.push_back({nearestPillarGlobal.colour, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area});
                pillar_front = {nearestPillarGlobal.colour, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area};
                if (mirror(pillar_front.xpos) > 230 && pillar_front.area < 30) curve_phase1_gyro_chkpt = 40; // For Pt A
                else if (pillar_front.area >= 30) curve_phase1_gyro_chkpt = 40; // For Pt B
                else curve_phase1_gyro_chkpt = mirror(pillar_front.xpos) * 40 / 200; // For Pt C, D, E
                state = CURVE_P1_STATE;
              }
            } else {
              steering_percentage = (getIMU() - tar_ang) * 10; // TODO: Verify whether a minus sign is needed
            }
          }
        }
        break;

        case TURNING_N_AVOIDING_STATE:
          Serial.println("TURNING_N_AVOIDING_STATE");
          if (pillar_front.colour == RED) turn_waittime = pillar_front.area * 48.5;
          else turn_waittime = pillar_front.area * 3;
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
            tar_ang += (anticlockwise == true) ? -right_ang : right_ang; // 88.58
            num_turn++;
            state = TURNING_STATE;
          }
          break;
        
        case TURNING_STATE:
          Serial.println("TURNING_STATE");
          if (nearestPillarGlobal.area > 10 && nearestPillarGlobal.colour == RED && nearestPillarGlobal.xpos <= min_xpos_Rcase(nearestPillarGlobal.area)) {
            pillars_array.push_back({nearestPillarGlobal.colour, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area});
            pillar_front = {nearestPillarGlobal.colour, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area};
            gyro_ang_detect_phase = getIMU();
            curve_phase1_gyro_chkpt = 50;
            blk_while_turning = true;
            state = CURVE_P1_STATE;
            break;
          } else if (nearestPillarGlobal.area > 10 && nearestPillarGlobal.colour == GREEN && nearestPillarGlobal.xpos >= min_xpos_Gcase(nearestPillarGlobal.area)) {
            pillars_array.push_back({nearestPillarGlobal.colour, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area});
            pillar_front = {nearestPillarGlobal.colour, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area};
            gyro_ang_detect_phase = getIMU();
            curve_phase1_gyro_chkpt = 50;
            blk_while_turning = true;
            state = CURVE_P1_STATE;
            break;
          } else if ((abs(tar_ang - getIMU()) > 15) && (abs(tar_ang) > abs(getIMU()))){
            steering_percentage = (tar_ang > 0) ? -100 : 100;
            dash_timeZero = internalClock.read();
          }
          else{
            steering_percentage = 0;
            state = DASH_AFTER_TURNING_STATE;
            dash_time = 600;
            dash_timeZero = internalClock.read();
            break;
          }
          break;
        
        case DASH_AFTER_TURNING_STATE:
          Serial.println("DASH_AFTER_TURNING_STATE");
          if (num_turn >= 12){
            state = PARKING_STATE;
            break;
          } else if (nearestPillarGlobal.colour != NO_COLOUR && nearestPillarGlobal.area >= 10){
            if (nearestPillarGlobal.colour == RED && nearestPillarGlobal.xpos <= min_xpos_Rcase(nearestPillarGlobal.area)){
              pillars_array.push_back({nearestPillarGlobal.colour, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area});
              pillar_front = {nearestPillarGlobal.colour, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area};
              gyro_ang_detect_phase = getIMU();
              if (pillar_front.xpos > 230) curve_phase1_gyro_chkpt = 45; // For Pt A, B
              else curve_phase1_gyro_chkpt = pillar_front.xpos * 45 / 200; // For Pt C, D, E
              state = CURVE_P1_STATE;
              break;
            } else if (nearestPillarGlobal.colour == GREEN && nearestPillarGlobal.xpos >= min_xpos_Gcase(nearestPillarGlobal.area)){
              pillars_array.push_back({nearestPillarGlobal.colour, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area});
              pillar_front = {nearestPillarGlobal.colour, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area};
              if (mirror(pillar_front.xpos) > 230 && pillar_front.area < 30) curve_phase1_gyro_chkpt = 40; // For Pt A
              else if (pillar_front.area >= 30) curve_phase1_gyro_chkpt = 40; // For Pt B
              else curve_phase1_gyro_chkpt = mirror(pillar_front.xpos) * 45 / 200; // For Pt C, D, E
              state = CURVE_P1_STATE;
              break;
            }
          } else {
            if (anticlockwise){
              if ((internalClock.read() - dash_timeZero) < dash_time){ // ultra1Dist > dist_threshold || && ultra2Dist > dist_threshold
                steering_percentage = (getIMU() - tar_ang) * 6; // TODO: Verify whether a minus sign is needed
                MiniR4.M2.setPower(power);
              } else {
                state = CHECK_FRONT_BLK_STATE;
                break;
              }
            } else {
              if ((internalClock.read() - dash_timeZero) < dash_time){ // ultra1Dist > dist_threshold || && ultra2Dist > dist_threshold
                steering_percentage = (getIMU() - tar_ang) * 6; // TODO: Verify whether a minus sign is needed
                MiniR4.M2.setPower(power);
              } else {
                state = CHECK_FRONT_BLK_STATE;
                break;
              }
            }
          }
          break;
        
        case CHECK_FRONT_BLK_STATE:
          Serial.println("CHECK_FRONT_BLK_STATE");
          if (nearestPillarGlobal.colour == RED && nearestPillarGlobal.area > 2 && nearestPillarGlobal.xpos < min_xpos_Rcase(nearestPillarGlobal.area)){
            Serial.println("Hi?");
            pillars_array.push_back({nearestPillarGlobal.colour, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area});
            pillar_front = {nearestPillarGlobal.colour, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area};
            state = ST_FW_WITH_BLK_STATE;
          } else if (nearestPillarGlobal.colour == GREEN && nearestPillarGlobal.area > 2 && nearestPillarGlobal.xpos > min_xpos_Gcase(nearestPillarGlobal.area)) {
            Serial.println("Hi2?");
            pillars_array.push_back({nearestPillarGlobal.colour, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area});
            pillar_front = {nearestPillarGlobal.colour, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area};
            state = ST_FW_WITH_BLK_STATE;
          } else {
            state = CHECK_MID_RACINGLN_STATE;
          }
          break;

        case CHECK_MID_RACINGLN_STATE:
          Serial.println("CHECK_MID_RACINGLN_STATE");
          if (num_turn == 0){
            state = DETECT_STATE;
          } else if (anticlockwise){
            if (abs(ultra1Dist - MID_RACINGLN_POS) > 150 && ultra1Dist < dist_threshold){
              inner_wall_dist = ultra1Dist - MID_RACINGLN_POS;
              gyro_ang_detect_phase = getIMU();
              state = MID_P1_STATE;
            } else state = DETECT_STATE;
          } else {
            if (abs(ultra2Dist - MID_RACINGLN_POS) > 150 && ultra2Dist < dist_threshold){
              inner_wall_dist = -(ultra2Dist - MID_RACINGLN_POS);
              gyro_ang_detect_phase = getIMU();
              state = MID_P1_STATE;
            } else state = DETECT_STATE;
          }
          break;
        
        case MID_P1_STATE:
          Serial.println("MID_P1_STATE");
          if (abs(getIMU() - gyro_ang_detect_phase) < mid_phase1_gyro_chkpt){
            steering_percentage = 100 * sign(inner_wall_dist);
          } else {
            Serial.println(inner_wall_dist);
            Serial.println(gyro_ang_detect_phase);
            mid_phase2_chk_t = abs(inner_wall_dist) * 0.001;
            mid_save_t = internalClock.read();
            state = MID_P2_STATE;
          }
          break;
        
        case MID_P2_STATE:
          Serial.println("MID_P2_STATE");
          if (internalClock.read() - mid_save_t <= mid_phase2_chk_t){
            steering_percentage = 0;
          } else {
            gyro_ang_detect_phase = getIMU();
            state = MID_P3_STATE;
          }
          break;
        
        case MID_P3_STATE:
          Serial.println("MID_P3_STATE");
          if (abs(getIMU() - tar_ang) > 20){
            steering_percentage = -100 * sign(inner_wall_dist);
          } else {
            state = MID_DASH_STATE;
            mid_dash_save_t = internalClock.read();
          }
          break;
        
        case MID_DASH_STATE:
          Serial.println("MID_DASH_STATE");
          if (internalClock.read() - mid_dash_save_t <= 100){
            steering_percentage = (getIMU() - tar_ang) * 8;
          } else state = DETECT_STATE;
          break;

        case CURVE_P1_STATE:
          Serial.println("CURVE_P1_STATE");
          if (pillar_front.colour == RED){
            if (abs(getIMU() - gyro_ang_detect_phase) < curve_phase1_gyro_chkpt){
              if (blk_while_turning) steering_percentage = -100;
              else if (pillar_front.xpos > 230 || pillar_front.area < 13) steering_percentage = -(55 - (getIMU() - gyro_ang_detect_phase)) * pillar_front.area * steering_amp_fact1 * 320 / (55 * area_dangerzone * pillar_front.xpos);
              else steering_percentage = -(50 - (getIMU() - gyro_ang_detect_phase)) * pillar_front.area * steering_amp_fact1 * 320 / (50 * area_dangerzone * pillar_front.xpos);
            } else {
              if (blk_while_turning) {
                state = CURVE_P2_5_STATE;
                curve_sphase3_chk_t = pillar_front.area * 1.5;
                Serial.println(curve_sphase3_chk_t);
                pillar_avoid_save_t = internalClock.read();
              } else if (pillar_front.xpos > 230 && pillar_front.area < 30){ // For Pt A
                curve_phase2_chk_t = 200 * (pillar_front.xpos - 50) / 300;
                state = CURVE_P2_STATE;
              } else if (pillar_front.area >= 30){ // For Pt B
                curve_phase2_chk_t = 150 * (pillar_front.xpos - 50) / 300;
                state = CURVE_P2_STATE;
              } else if (pillar_front.area > 13){ // For Pt C, D
                curve_phase2_chk_t = 50 * (pillar_front.xpos - 50) / 300;
                state = CURVE_P2_STATE;
              } else { // For Pt E
                gyro_ang_detect_phase = getIMU();
                state = CURVE_P3_STATE;
              }
              pillar_avoid_save_t = internalClock.read();
              break;
            }
          } else {
            if (abs(getIMU() - gyro_ang_detect_phase) < curve_phase1_gyro_chkpt){
              if (blk_while_turning) steering_percentage = 100;
              if (mirror(pillar_front.xpos) > 230) steering_percentage = (55 - (getIMU() - gyro_ang_detect_phase)) * pillar_front.area * steering_amp_fact1 * 320 / (60 * area_dangerzone * mirror(pillar_front.xpos)); // || pillar_front.area < 13
              else steering_percentage = (50 - (getIMU() - gyro_ang_detect_phase)) * pillar_front.area * steering_amp_fact1 * 320 / (50 * area_dangerzone * mirror(pillar_front.xpos));
            } else {
              if (blk_while_turning) {
                state = CURVE_P2_5_STATE;
                curve_sphase3_chk_t = pillar_front.area * 15;
                Serial.println(curve_sphase3_chk_t);
                pillar_avoid_save_t = internalClock.read();
              } else if (mirror(pillar_front.xpos) > 230 && pillar_front.area < 30){ // For Pt A
                curve_phase2_chk_t = 150 * (mirror(pillar_front.xpos) - 50) / 300;
                state = CURVE_P2_STATE;
              } else if (pillar_front.area >= 30){ // For Pt B
                curve_phase2_chk_t = 120 * (mirror(pillar_front.xpos) - 50) / 300;
                state = CURVE_P2_STATE;
              } else if (pillar_front.area > 10){ // 13 // For Pt C, D
              curve_phase2_chk_t = 20 * (mirror(pillar_front.xpos) - 50) / 300;
                gyro_ang_detect_phase = getIMU();
                state = CURVE_P2_STATE;
              } else { // For Pt E
                gyro_ang_detect_phase = getIMU();
                state = CURVE_P3_STATE;
              }
              pillar_avoid_save_t = internalClock.read();
              break;
            }
            break;
          }
          break;

        case CURVE_P2_STATE:
          Serial.println("CURVE_P2_STATE");
          if (internalClock.read() - pillar_avoid_save_t <= curve_phase2_chk_t){
            steering_percentage = 0;
          } else {
            gyro_ang_detect_phase = getIMU();
            state = CURVE_P3_STATE;
            break;
          }
          break;
        
        case CURVE_P2_5_STATE:
          Serial.println("CURVE_P2_5_STATE");
          if (internalClock.read() - pillar_avoid_save_t <= curve_sphase3_chk_t){
            steering_percentage = 0;
          } else {
            state = CURVE_P3_STATE;
            break;
          }
          break;
        
        case CURVE_P3_STATE:
          Serial.println("CURVE_P3_STATE");
          if (abs(getIMU() - tar_ang) > 20){
            if (pillar_front.colour == RED){
              if (blk_while_turning) steering_percentage = 100;
              else steering_percentage = (abs(getIMU()) + 25) * steering_amp_fact2;
            } else {
              if (blk_while_turning) steering_percentage = -100;
              else steering_percentage = -(abs(getIMU()) + 25) * steering_amp_fact2;
            }
          } else {
            state = CURVE_P4_STATE;
            blk_while_turning = false;
            pillar_avoid_save_t = internalClock.read();
            break;
          }
          Serial.println(steering_percentage);
          break;

        case CURVE_P4_STATE:
          Serial.println("CURVE_P4_STATE");
          if (internalClock.read() - pillar_avoid_save_t <= 1000){
            steering_percentage = (getIMU() - tar_ang) * 8;
          } else {
            state = CHECK_MID_RACINGLN_STATE;
            break;
          }
          break;

        case ST_FW_WITH_BLK_STATE:
          Serial.println("ST_FW_WITH_BLK_STATE");
          if (anticlockwise){
            if (ultra1Dist < dist_threshold){ // ultra1Dist > dist_threshold || && ultra2Dist > dist_threshold
              steering_percentage = (getIMU() - tar_ang) * 7;
              break;
            } else state = DETECT_STATE;
          } else {
            if (ultra2Dist < dist_threshold){ // ultra1Dist > dist_threshold || && ultra2Dist > dist_threshold
            steering_percentage = (getIMU() - tar_ang) * 7;
              break;
            } else state = CHECK_FRONT_BLK_STATE;
          }
          break;
        
        case PARKING_STATE:
          Serial.println("PARKING_STATE");
          motor_stop(BRAKE);
          steering_percentage = 0;
          safeResume(blinkledThread);
          if (!is_anticlockwise) safeResume(Ultra1Thread);
          else safeResume(Ultra2Thread);
          end_game = true;
          OC2_endtime = internalClock.read();
          break;

    }
  }
  vTaskDelay(5 / portTICK_PERIOD_MS);
}


void OC2main(){
  static bool run_OC2 = false;
  while (1){
    if (is_btn_bumped(TFT_BTN1)){
      run_OC2 = !run_OC2;
      reset_OC2 = true;
    }
    if (run_OC2){
      OC2_pixy2(90, 85, 15); //-80
    } else {
      // safeResume(displayThread);
      safeResume(Ultra1Thread);
      safeResume(Ultra2Thread);
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