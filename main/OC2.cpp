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
// void OC2Huskylens(double right_ang, int dist_threshold, int power){
//   // A flag to for resetting the IMU before starting the run.
//   // Resetting IMU takes a few seconds, which means if we do so when the run starts, we will lack behind for a few seconds
//   static bool first_run = true;

//   // A flag to determine whether the run is started or not
//   static bool start_game = false;

//   // Variables and flag for checking the DOWN button state
//   static bool prev_btn_state = false;
//   bool curr_btn_state = MiniR4.BTN_DOWN.getState();
  
//   if (curr_btn_state && !prev_btn_state){
//     start_game = !start_game;
//   }
//   prev_btn_state = curr_btn_state;

//   // Variables and flag for checking the UP button state
//   static bool prev_imu_btn_state = false;
//   bool curr_imu_btn_state = MiniR4.BTN_UP.getState();
  
//   if (curr_imu_btn_state && !prev_imu_btn_state){
//     MiniR4.Motion.resetIMUValues();
//     resetIMU();
//   }
//   prev_imu_btn_state = curr_imu_btn_state;

//   // Variables required for OC2
//   static OC2_STATES state = DETECT_STATE;     // A variable storing the current state of OC2 run
//   static bool reset_OC2 = false;              // A flag determining whether the all the variables should be reseted
//   static bool end_game = false;               // A flag determining whether the run has ended
//   std::vector<COLOURED_OBJ> pillars_array;    // An array storing all the pillars detected
//   static COLOURED_OBJ pillar_front;           // A struct storing all the info of the nearest pillar detected during DETECT_STATE
//   // EDITABLE: Not recommended to edit this but, if you want to change the danger zone of area of pillar, edit the following line
//   const int area_dangerzone = 40;             // A variable representing the area of a pillar which is identified as danger when its area is larger than this value
//   // END OF EDITABLE
//   // EDITABLE: Edit this if the car is turning too sharp/mild during CURVE_P1_STATE
//   const float steering_amp_fact1 = 200;       // An amplification factor for calculating the steering percentage during CURVE_P1_STATE
//   // END OF EDITABLE
//   // EDITABLE: Edit this if the car is turning too sharp/mild during CURVE_P3_STATE
//   const float steering_amp_fact2 = 2;         // An amplification factor for calculating the steering percentage during CURVE_P3_STATE
//   // END OF EDITABLE
//   static int gyro_ang_detect_phase = 0;       // A variable storing the current IMU angle
//   static int pillar_avoid_save_t = 0;         // A variable storing the instant time for avoiding pillars
//   static float curve_phase2_chk_t = 300;      // A variable storing the time required for the car to move straight forward during CURVE_P2_STATE. Non-editable as it will be recalculated during the run
//   static float curve_sphase3_chk_t = 300;     // A variable storing the time required for the car to move straight forward during CURVE_P2_5_STATE. Non-editable as it will be recalculated during the run
//   static int curve_phase1_gyro_chkpt = 45;    // A variable storing the IMU value for the car to turn to during CURVE_P1_STATE. Non-editable as it will be recalculated during the run
//   static int mid_phase1_gyro_chkpt = 50;      // A variable storing the IMU value for the car to turn to during MID_P1_STATE. Non-editable as it will be recalculated during the run
//   static float inner_wall_dist = 0;           // A variable storing the distance between the car and the inner wall (measured using ultrasonic sensor)
//   static int mid_save_t = 0;                  // A variable storing the instant time for going back to mid racing line
//   static float mid_phase2_chk_t = 0;          // A variable storing the time required for the car to move straight forward during MID_P2_STATE. Non-editable as it will be recalculated during the run
//   static int mid_dash_save_t = 0;             // A variable storing the time required for the car to move straight forward during MID_DASH_STATE. Non-editable as it will be recalculated during the run
//   static double tar_ang = 0;                  // The target angle which the car should be facing
//   static long dash_timeZero = 0;              // Variable to store the instant time from the internal clock for dashing
//   static long dash_time = 0;                  // The time required for the car to run before using the ultrasonic sensors for detection again after turning
//   static bool anticlockwise = false;          // A boolean storing whether the car is racing in clockwise or anti-clockwise direction
//   static int num_turn = 0;                    // Variable storing the number of turns the car has made
//   static long turn_waittimeZero = 0;          // Variable to store the instant time from the internal clock for waiting to turning
//   static int turn_waittime = 150;             // Variable storing the time in ms that required to wait before the car turn 
//   static bool blk_while_turning = false;      // A flag determining whether any pillars in close distance is detected during the turning
  
//   MiniR4.M2.setBrake(true);

//   if (!start_game){ 
//     reset_OC2 = true;
//     steering(0);
//     MiniR4.M2.setPower(0);
//     displayThread.enabled = true;
//     Ultra1Thread.enabled = true;
//     Ultra2Thread.enabled = true;
//     OC2Thread.setInterval(10);
//     Ultra1Thread.setInterval(10);
//     Ultra2Thread.setInterval(10);
//   } else {
//     if (reset_OC2){
//       OC2Thread.setInterval(0);
//       Ultra1Thread.setInterval(0);
//       Ultra2Thread.setInterval(0);
//       huskylensThread.setInterval(0);

//       state = DETECT_STATE;
//       reset_OC2 = false;
//       end_game = false;
//       gyro_ang_detect_phase = 0;
//       pillar_avoid_save_t = 0;
//       curve_phase2_chk_t = 300;
//       curve_sphase3_chk_t = 300;
//       curve_phase1_gyro_chkpt = 45;
//       mid_phase1_gyro_chkpt = 50;
//       inner_wall_dist = 0;
//       mid_save_t = 0;
//       mid_phase2_chk_t = 0;
//       mid_dash_save_t = 0;
//       tar_ang = 0;
//       dash_timeZero = 0;
//       dash_time = 0;
//       anticlockwise = false;
//       num_turn = 0;
//       turn_waittimeZero = 0;
//       turn_waittime = 150;
//       blk_while_turning = false;

//       OC2_starttime = internalClock.read();
//       if (!first_run) resetIMU();
//     }
//     if (!end_game){
//       displayThread.enabled = false;
//       OC2_endtime = internalClock.read();
//       steering(steering_percentage);
//       MiniR4.M2.setPower(power);

//       // The Finte State Machine for OC2 which is well organized
//       switch (state) {
//         // As the name implies, this state do the detections using ultrasonic sensors and huskylens when driving on a straight sector.
//         case DETECT_STATE:
//         Serial.println("DETECT_STATE");
//           // Before doing any turning, both ultrasonic sensors are locked
//           if (num_turn == 0) {
//             if (ultra1Dist > dist_threshold){
//               anticlockwise = true;
//               Ultra2Thread.enabled = false;
//               if (nearestPillarGlobal.colour != NO_COLOUR){
//                 if (nearestPillarGlobal.area >= 20){
//                   pillars_array.push_back({nearestPillarGlobal.colour, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area});
//                   pillar_front = {nearestPillarGlobal.colour, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area};
//                   state = TURNING_N_AVOIDING_STATE;
//                 }
//               } else {
//                 state = WAIT_TURN_STATE;
//                 turn_waittime = 10;
//                 turn_waittimeZero = internalClock.read();
//               }
//             } else if (ultra2Dist > dist_threshold){
//               anticlockwise = false;
//               Ultra1Thread.enabled = false;
//               if (nearestPillarGlobal.colour != NO_COLOUR){
//                 if (nearestPillarGlobal.area >= 20){
//                   pillars_array.push_back({nearestPillarGlobal.colour, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area});
//                   pillar_front = {nearestPillarGlobal.colour, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area};
//                   state = TURNING_N_AVOIDING_STATE;
//                 }
//               } else {
//                 state = WAIT_TURN_STATE;
//                 turn_waittime = 10;
//                 turn_waittimeZero = internalClock.read();
//               }
//             } else if (nearestPillarGlobal.colour == RED){
//                 if (nearestPillarGlobal.area >= 10 && nearestPillarGlobal.xpos > min_xpos_Rcase(nearestPillarGlobal.area)){
//                   pillars_array.push_back({nearestPillarGlobal.colour, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area});
//                   pillar_front = {nearestPillarGlobal.colour, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area};
//                   gyro_ang_detect_phase = getIMU();
//                   if (pillar_front.xpos > 230) curve_phase1_gyro_chkpt = 45; // For Pt A, B
//                   else curve_phase1_gyro_chkpt = pillar_front.xpos * 45 / 200; // For Pt C, D, E
//                   state = CURVE_P1_STATE;
//                 }
//             } else if (nearestPillarGlobal.colour == GREEN){
//               if (nearestPillarGlobal.area >= 10 && nearestPillarGlobal.xpos < min_xpos_Gcase(nearestPillarGlobal.area)){
//                 pillars_array.push_back({nearestPillarGlobal.colour, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area});
//                 pillar_front = {nearestPillarGlobal.colour, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area};
//                 gyro_ang_detect_phase = getIMU();
//                 if (mirror(pillar_front.xpos) > 230 && pillar_front.area < 30) curve_phase1_gyro_chkpt = 40; // For Pt A
//                 else if (pillar_front.area >= 30) curve_phase1_gyro_chkpt = 40; // For Pt B
//                 else curve_phase1_gyro_chkpt = mirror(pillar_front.xpos) * 40 / 200; // For Pt C, D, E
//                 state = CURVE_P1_STATE;
//               }
//             } else {
//               steering_percentage = (getIMU() - tar_ang) * 10; // TODO: Verify whether a minus sign is needed
//             }
//           } else {
//             if (anticlockwise){
//               if (ultra1Dist > dist_threshold){
//                 if (nearestPillarGlobal.colour != NO_COLOUR){
//                   if (nearestPillarGlobal.area >= 20){
//                     pillars_array.push_back({nearestPillarGlobal.colour, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area});
//                     pillar_front = {nearestPillarGlobal.colour, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area};
//                     state = TURNING_N_AVOIDING_STATE;
//                   }
//                 } else {
//                   state = WAIT_TURN_STATE;
//                   turn_waittime = 10;
//                   turn_waittimeZero = internalClock.read();
//                 }
//               } else if (nearestPillarGlobal.colour == RED){
//                 if (nearestPillarGlobal.area >= 10 && nearestPillarGlobal.xpos > min_xpos_Rcase(nearestPillarGlobal.area)){
//                   pillars_array.push_back({nearestPillarGlobal.colour, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area});
//                   pillar_front = {nearestPillarGlobal.colour, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area};
//                   gyro_ang_detect_phase = getIMU();
//                   if (pillar_front.xpos > 230) curve_phase1_gyro_chkpt = 45; // For Pt A, B
//                   else curve_phase1_gyro_chkpt = pillar_front.xpos * 45 / 200; // For Pt C, D, E
//                   state = CURVE_P1_STATE;
//                 }
//               } else if (nearestPillarGlobal.colour == GREEN){
//                 if (nearestPillarGlobal.area >= 10 && nearestPillarGlobal.xpos < min_xpos_Gcase(nearestPillarGlobal.area)){
//                   pillars_array.push_back({nearestPillarGlobal.colour, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area});
//                   pillar_front = {nearestPillarGlobal.colour, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area};
//                   if (mirror(pillar_front.xpos) > 230 && pillar_front.area < 30) curve_phase1_gyro_chkpt = 40; // For Pt A
//                   else if (pillar_front.area >= 30) curve_phase1_gyro_chkpt = 40; // For Pt B
//                   else curve_phase1_gyro_chkpt = mirror(pillar_front.xpos) * 40 / 200; // For Pt C, D, E
//                   state = CURVE_P1_STATE;
//                 }
//               } else {
//                 steering_percentage = (getIMU() - tar_ang) * 10; // TODO: Verify whether a minus sign is needed
//               }
//             } else {
//               if (ultra2Dist > dist_threshold){
//                 if (nearestPillarGlobal.colour != NO_COLOUR){
//                   if (nearestPillarGlobal.area >= 20){
//                     pillars_array.push_back({nearestPillarGlobal.colour, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area});
//                     pillar_front = {nearestPillarGlobal.colour, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area};
//                     state = TURNING_N_AVOIDING_STATE;
//                   }
//                 } else {
//                   state = WAIT_TURN_STATE;
//                   turn_waittime = 10;
//                   turn_waittimeZero = internalClock.read();
//                 }
//               } else if (nearestPillarGlobal.colour == RED){
//                 if (nearestPillarGlobal.area >= 10 && nearestPillarGlobal.xpos > min_xpos_Rcase(nearestPillarGlobal.area)){
//                   pillars_array.push_back({nearestPillarGlobal.colour, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area});
//                   pillar_front = {nearestPillarGlobal.colour, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area};
//                   gyro_ang_detect_phase = getIMU();
//                   if (pillar_front.xpos > 230) curve_phase1_gyro_chkpt = 45; // For Pt A, B
//                   else curve_phase1_gyro_chkpt = pillar_front.xpos * 45 / 200; // For Pt C, D, E
//                   state = CURVE_P1_STATE;
//                 }
//               } else if (nearestPillarGlobal.colour == GREEN){
//                 if (nearestPillarGlobal.area >= 10 && nearestPillarGlobal.xpos < min_xpos_Gcase(nearestPillarGlobal.area)){
//                   pillars_array.push_back({nearestPillarGlobal.colour, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area});
//                   pillar_front = {nearestPillarGlobal.colour, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area};
//                   if (mirror(pillar_front.xpos) > 230 && pillar_front.area < 30) curve_phase1_gyro_chkpt = 40; // For Pt A
//                   else if (pillar_front.area >= 30) curve_phase1_gyro_chkpt = 40; // For Pt B
//                   else curve_phase1_gyro_chkpt = mirror(pillar_front.xpos) * 40 / 200; // For Pt C, D, E
//                   state = CURVE_P1_STATE;
//                 }
//               } else {
//                 steering_percentage = (getIMU() - tar_ang) * 10; // TODO: Verify whether a minus sign is needed
//               }
//             }
//           }
//           break;

//           case TURNING_N_AVOIDING_STATE:
//             Serial.println("TURNING_N_AVOIDING_STATE");
//             if (pillar_front.colour == RED) turn_waittime = pillar_front.area * 48.5;
//             else turn_waittime = pillar_front.area * 3;
//             state = WAIT_TURN_STATE;
//             turn_waittimeZero = internalClock.read();
//             break;

//           case WAIT_TURN_STATE:
//             Serial.println("WAIT_TURN_STATE");
//             if ((internalClock.read() - turn_waittimeZero) < turn_waittime) {
//               steering_percentage = (getIMU() - tar_ang) * 10;
//               break;
//             }
//             else {
//               tar_ang += (anticlockwise == true) ? -right_ang : right_ang; // 88.58
//               num_turn++;
//               state = TURNING_STATE;
//             }
//             break;
          
//           case TURNING_STATE:
//             Serial.println("TURNING_STATE");
//             if (nearestPillarGlobal.area > 10 && nearestPillarGlobal.colour == RED && nearestPillarGlobal.xpos <= min_xpos_Rcase(nearestPillarGlobal.area)) {
//               pillars_array.push_back({nearestPillarGlobal.colour, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area});
//               pillar_front = {nearestPillarGlobal.colour, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area};
//               gyro_ang_detect_phase = getIMU();
//               curve_phase1_gyro_chkpt = 50;
//               blk_while_turning = true;
//               state = CURVE_P1_STATE;
//               break;
//             } else if (nearestPillarGlobal.area > 10 && nearestPillarGlobal.colour == GREEN && nearestPillarGlobal.xpos >= min_xpos_Gcase(nearestPillarGlobal.area)) {
//               pillars_array.push_back({nearestPillarGlobal.colour, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area});
//               pillar_front = {nearestPillarGlobal.colour, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area};
//               gyro_ang_detect_phase = getIMU();
//               curve_phase1_gyro_chkpt = 50;
//               blk_while_turning = true;
//               state = CURVE_P1_STATE;
//               break;
//             } else if ((abs(tar_ang - getIMU()) > 15) && (abs(tar_ang) > abs(getIMU()))){
//               steering_percentage = (tar_ang > 0) ? -100 : 100;
//               dash_timeZero = internalClock.read();
//             }
//             else{
//               steering_percentage = 0;
//               state = DASH_AFTER_TURNING_STATE;
//               dash_time = 600;
//               dash_timeZero = internalClock.read();
//               break;
//             }
//             break;
          
//           case DASH_AFTER_TURNING_STATE:
//             Serial.println("DASH_AFTER_TURNING_STATE");
//             if (num_turn >= 12){
//               state = PARKING_STATE;
//               break;
//             } else if (nearestPillarGlobal.colour != NO_COLOUR && nearestPillarGlobal.area >= 10){
//               if (nearestPillarGlobal.colour == RED && nearestPillarGlobal.xpos <= min_xpos_Rcase(nearestPillarGlobal.area)){
//                 pillars_array.push_back({nearestPillarGlobal.colour, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area});
//                 pillar_front = {nearestPillarGlobal.colour, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area};
//                 gyro_ang_detect_phase = getIMU();
//                 if (pillar_front.xpos > 230) curve_phase1_gyro_chkpt = 45; // For Pt A, B
//                 else curve_phase1_gyro_chkpt = pillar_front.xpos * 45 / 200; // For Pt C, D, E
//                 state = CURVE_P1_STATE;
//                 break;
//               } else if (nearestPillarGlobal.colour == GREEN && nearestPillarGlobal.xpos >= min_xpos_Gcase(nearestPillarGlobal.area)){
//                 pillars_array.push_back({nearestPillarGlobal.colour, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area});
//                 pillar_front = {nearestPillarGlobal.colour, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area};
//                 if (mirror(pillar_front.xpos) > 230 && pillar_front.area < 30) curve_phase1_gyro_chkpt = 40; // For Pt A
//                 else if (pillar_front.area >= 30) curve_phase1_gyro_chkpt = 40; // For Pt B
//                 else curve_phase1_gyro_chkpt = mirror(pillar_front.xpos) * 45 / 200; // For Pt C, D, E
//                 state = CURVE_P1_STATE;
//                 break;
//               }
//             } else {
//               if (anticlockwise){
//                 if ((internalClock.read() - dash_timeZero) < dash_time){ // ultra1Dist > dist_threshold || && ultra2Dist > dist_threshold
//                   steering_percentage = (getIMU() - tar_ang) * 6; // TODO: Verify whether a minus sign is needed
//                   MiniR4.M2.setPower(power);
//                 } else {
//                   state = CHECK_FRONT_BLK_STATE;
//                   break;
//                 }
//               } else {
//                 if ((internalClock.read() - dash_timeZero) < dash_time){ // ultra1Dist > dist_threshold || && ultra2Dist > dist_threshold
//                   steering_percentage = (getIMU() - tar_ang) * 6; // TODO: Verify whether a minus sign is needed
//                   MiniR4.M2.setPower(power);
//                 } else {
//                   state = CHECK_FRONT_BLK_STATE;
//                   break;
//                 }
//               }
//             }
//             break;
          
//           case CHECK_FRONT_BLK_STATE:
//             Serial.println("CHECK_FRONT_BLK_STATE");
//             if (nearestPillarGlobal.colour == RED && nearestPillarGlobal.area > 2 && nearestPillarGlobal.xpos < min_xpos_Rcase(nearestPillarGlobal.area)){
//               Serial.println("Hi?");
//               pillars_array.push_back({nearestPillarGlobal.colour, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area});
//               pillar_front = {nearestPillarGlobal.colour, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area};
//               state = ST_FW_WITH_BLK_STATE;
//             } else if (nearestPillarGlobal.colour == GREEN && nearestPillarGlobal.area > 2 && nearestPillarGlobal.xpos > min_xpos_Gcase(nearestPillarGlobal.area)) {
//               Serial.println("Hi2?");
//               pillars_array.push_back({nearestPillarGlobal.colour, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area});
//               pillar_front = {nearestPillarGlobal.colour, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area};
//               state = ST_FW_WITH_BLK_STATE;
//             } else {
//               state = CHECK_MID_RACINGLN_STATE;
//             }
//             break;

//           case CHECK_MID_RACINGLN_STATE:
//             Serial.println("CHECK_MID_RACINGLN_STATE");
//             if (num_turn == 0){
//               state = DETECT_STATE;
//             } else if (anticlockwise){
//               if (abs(ultra1Dist - MID_RACINGLN_POS) > 150 && ultra1Dist < dist_threshold){
//                 inner_wall_dist = ultra1Dist - MID_RACINGLN_POS;
//                 gyro_ang_detect_phase = getIMU();
//                 state = MID_P1_STATE;
//               } else state = DETECT_STATE;
//             } else {
//               if (abs(ultra2Dist - MID_RACINGLN_POS) > 150 && ultra2Dist < dist_threshold){
//                 inner_wall_dist = -(ultra2Dist - MID_RACINGLN_POS);
//                 gyro_ang_detect_phase = getIMU();
//                 state = MID_P1_STATE;
//               } else state = DETECT_STATE;
//             }
//             break;
          
//           case MID_P1_STATE:
//             Serial.println("MID_P1_STATE");
//             if (abs(getIMU() - gyro_ang_detect_phase) < mid_phase1_gyro_chkpt){
//               steering_percentage = 100 * sign(inner_wall_dist);
//             } else {
//               Serial.println(inner_wall_dist);
//               Serial.println(gyro_ang_detect_phase);
//               mid_phase2_chk_t = abs(inner_wall_dist) * 0.001;
//               mid_save_t = internalClock.read();
//               state = MID_P2_STATE;
//             }
//             break;
          
//           case MID_P2_STATE:
//             Serial.println("MID_P2_STATE");
//             if (internalClock.read() - mid_save_t <= mid_phase2_chk_t){
//               steering_percentage = 0;
//             } else {
//               gyro_ang_detect_phase = getIMU();
//               state = MID_P3_STATE;
//             }
//             break;
          
//           case MID_P3_STATE:
//             Serial.println("MID_P3_STATE");
//             if (abs(getIMU() - tar_ang) > 20){
//               steering_percentage = -100 * sign(inner_wall_dist);
//             } else {
//               state = MID_DASH_STATE;
//               mid_dash_save_t = internalClock.read();
//             }
//             break;
          
//           case MID_DASH_STATE:
//             Serial.println("MID_DASH_STATE");
//             if (internalClock.read() - mid_dash_save_t <= 100){
//               steering_percentage = (getIMU() - tar_ang) * 8;
//             } else state = DETECT_STATE;
//             break;

//           case CURVE_P1_STATE:
//             Serial.println("CURVE_P1_STATE");
//             if (pillar_front.colour == RED){
//               if (abs(getIMU() - gyro_ang_detect_phase) < curve_phase1_gyro_chkpt){
//                 if (blk_while_turning) steering_percentage = -100;
//                 else if (pillar_front.xpos > 230 || pillar_front.area < 13) steering_percentage = -(55 - (getIMU() - gyro_ang_detect_phase)) * pillar_front.area * steering_amp_fact1 * 320 / (55 * area_dangerzone * pillar_front.xpos);
//                 else steering_percentage = -(50 - (getIMU() - gyro_ang_detect_phase)) * pillar_front.area * steering_amp_fact1 * 320 / (50 * area_dangerzone * pillar_front.xpos);
//               } else {
//                 if (blk_while_turning) {
//                   state = CURVE_P2_5_STATE;
//                   curve_sphase3_chk_t = pillar_front.area * 1.5;
//                   Serial.println(curve_sphase3_chk_t);
//                   pillar_avoid_save_t = internalClock.read();
//                 } else if (pillar_front.xpos > 230 && pillar_front.area < 30){ // For Pt A
//                   curve_phase2_chk_t = 200 * (pillar_front.xpos - 50) / 300;
//                   state = CURVE_P2_STATE;
//                 } else if (pillar_front.area >= 30){ // For Pt B
//                   curve_phase2_chk_t = 150 * (pillar_front.xpos - 50) / 300;
//                   state = CURVE_P2_STATE;
//                 } else if (pillar_front.area > 13){ // For Pt C, D
//                   curve_phase2_chk_t = 50 * (pillar_front.xpos - 50) / 300;
//                   state = CURVE_P2_STATE;
//                 } else { // For Pt E
//                   gyro_ang_detect_phase = getIMU();
//                   state = CURVE_P3_STATE;
//                 }
//                 pillar_avoid_save_t = internalClock.read();
//                 break;
//               }
//             } else {
//               if (abs(getIMU() - gyro_ang_detect_phase) < curve_phase1_gyro_chkpt){
//                 if (blk_while_turning) steering_percentage = 100;
//                 if (mirror(pillar_front.xpos) > 230) steering_percentage = (55 - (getIMU() - gyro_ang_detect_phase)) * pillar_front.area * steering_amp_fact1 * 320 / (60 * area_dangerzone * mirror(pillar_front.xpos)); // || pillar_front.area < 13
//                 else steering_percentage = (50 - (getIMU() - gyro_ang_detect_phase)) * pillar_front.area * steering_amp_fact1 * 320 / (50 * area_dangerzone * mirror(pillar_front.xpos));
//               } else {
//                 if (blk_while_turning) {
//                   state = CURVE_P2_5_STATE;
//                   curve_sphase3_chk_t = pillar_front.area * 15;
//                   Serial.println(curve_sphase3_chk_t);
//                   pillar_avoid_save_t = internalClock.read();
//                 } else if (mirror(pillar_front.xpos) > 230 && pillar_front.area < 30){ // For Pt A
//                   curve_phase2_chk_t = 150 * (mirror(pillar_front.xpos) - 50) / 300;
//                   state = CURVE_P2_STATE;
//                 } else if (pillar_front.area >= 30){ // For Pt B
//                   curve_phase2_chk_t = 120 * (mirror(pillar_front.xpos) - 50) / 300;
//                   state = CURVE_P2_STATE;
//                 } else if (pillar_front.area > 10){ // 13 // For Pt C, D
//                 curve_phase2_chk_t = 20 * (mirror(pillar_front.xpos) - 50) / 300;
//                   gyro_ang_detect_phase = getIMU();
//                   state = CURVE_P2_STATE;
//                 } else { // For Pt E
//                   gyro_ang_detect_phase = getIMU();
//                   state = CURVE_P3_STATE;
//                 }
//                 pillar_avoid_save_t = internalClock.read();
//                 break;
//               }
//               break;
//             }
//             break;

//           case CURVE_P2_STATE:
//             Serial.println("CURVE_P2_STATE");
//             if (internalClock.read() - pillar_avoid_save_t <= curve_phase2_chk_t){
//               steering_percentage = 0;
//             } else {
//               gyro_ang_detect_phase = getIMU();
//               state = CURVE_P3_STATE;
//               break;
//             }
//             break;
          
//           case CURVE_P2_5_STATE:
//             Serial.println("CURVE_P2_5_STATE");
//             if (internalClock.read() - pillar_avoid_save_t <= curve_sphase3_chk_t){
//               steering_percentage = 0;
//             } else {
//               state = CURVE_P3_STATE;
//               break;
//             }
//             break;
          
//           case CURVE_P3_STATE:
//             Serial.println("CURVE_P3_STATE");
//             if (abs(getIMU() - tar_ang) > 20){
//               if (pillar_front.colour == RED){
//                 if (blk_while_turning) steering_percentage = 100;
//                 else steering_percentage = (abs(getIMU()) + 25) * steering_amp_fact2;
//               } else {
//                 if (blk_while_turning) steering_percentage = -100;
//                 else steering_percentage = -(abs(getIMU()) + 25) * steering_amp_fact2;
//               }
//             } else {
//               state = CURVE_P4_STATE;
//               blk_while_turning = false;
//               pillar_avoid_save_t = internalClock.read();
//               break;
//             }
//             Serial.println(steering_percentage);
//             break;

//           case CURVE_P4_STATE:
//             Serial.println("CURVE_P4_STATE");
//             if (internalClock.read() - pillar_avoid_save_t <= 1000){
//               steering_percentage = (getIMU() - tar_ang) * 8;
//             } else {
//               state = CHECK_MID_RACINGLN_STATE;
//               break;
//             }
//             break;

//           case ST_FW_WITH_BLK_STATE:
//             Serial.println("ST_FW_WITH_BLK_STATE");
//             if (anticlockwise){
//               if (ultra1Dist < dist_threshold){ // ultra1Dist > dist_threshold || && ultra2Dist > dist_threshold
//                 steering_percentage = (getIMU() - tar_ang) * 7;
//                 break;
//               } else state = DETECT_STATE;
//             } else {
//               if (ultra2Dist < dist_threshold){ // ultra1Dist > dist_threshold || && ultra2Dist > dist_threshold
//               steering_percentage = (getIMU() - tar_ang) * 7;
//                 break;
//               } else state = CHECK_FRONT_BLK_STATE;
//             }
//             break;

//         }
//     } else {
//       steering(0);
//       MiniR4.M2.setBrake(true);
//       MiniR4.M2.setPower(0);
//       displayThread.enabled = true;
//       Ultra1Thread.enabled = true;
//       Ultra2Thread.enabled = true;
//     }
//   }
// }

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

// /**
//  * @brief Function for OC2
//  * 
//  * @param right_ang; double; the value of an "right angle" for the IMU (as the value of IMU is not consistent)
//  * @param dist_threshold; int; the threshold that the car sees for turning (in mm)
//  * @param power; int; the power of the driving motor (0 ~ -100)
//  * 
//  */
// long OC2_starttime = 0; 
// long OC2_endtime = 0; 
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
  static OC2_STATES state = INIT_STATE;     // A variable storing the current state of OC2 run
  static bool reset_OC2 = false;              // A flag determining whether the all the variables should be reseted
  static bool end_game = false;               // A flag determining whether the run has ended
  std::vector<COLOURED_OBJ> pillars_array;    // An array storing all the pillars detected
  static COLOURED_OBJ pillar_front;           // A struct storing all the info of the nearest pillar detected during DETECT_STATE
  static COLOURED_OBJ prev_pillar;
  static OC2_STATES prev_state = INIT_STATE;
  // EDITABLE: Not recommended to edit this but, if you want to change the danger zone of area of pillar, edit the following line
  const int area_dangerzone = 40;             // A variable representing the area of a pillar which is identified as danger when its area is larger than this value
  // END OF EDITABLE
  // EDITABLE: Edit this if the car is turning too sharp/mild during CURVE_P1_STATE
  const float steering_amp_fact1 = 150;       // An amplification factor for calculating the steering percentage during CURVE_P1_STATE
  // END OF EDITABLE
  // EDITABLE: Edit this if the car is turning too sharp/mild during CURVE_P3_STATE
  const float steering_amp_fact2 = 2;         // An amplification factor for calculating the steering percentage during CURVE_P3_STATE
  // END OF EDITABLE
  static int gyro_ang_detect_phase = 0;       // A variable storing the current IMU angle
  static int pillar_avoid_save_t = 0;         // A variable storing the instant time for avoiding pillars
  static float curve_phase2_chk_t = 300;      // A variable storing the time required for the car to move straight forward during CURVE_P2_STATE. Non-editable as it will be recalculated during the run
  static float curve_sphase3_chk_t = 300;     // A variable storing the time required for the car to move straight forward during CURVE_P2_5_STATE. Non-editable as it will be recalculated during the run
  static int curve_phase1_gyro_chkpt = 45;    // A variable storing the IMU value for the car to turn to during CURVE_P1_STATE. Non-editable as it will be recalculated during the run
  static int mid_phase1_gyro_chkpt = 35;      // A variable storing the IMU value for the car to turn to during MID_P1_STATE. Non-editable as it will be recalculated during the run
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
  static int turn_waittime = 750;             // Variable storing the time in ms that required to wait before the car turn 
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
  static double encoder_counterzero = 0;
  static bool last_sector_no_blk = false;
  static int CP4_t = 0;
  static bool from_tNa_state = false;
  static int u1_ind = 0;
  static int u2_ind = 0;
  static int curve_p3_outloop_time = 0;
  static int dash_amp_fact = 1;
  static bool parking_colourblk = false;
  static bool blk_after_turn = false;
  static colour_check = true;

  MiniR4.M2.setBrake(true);

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

      state = INIT_STATE;   //DETECT_STATE;
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
      anticlockwise = false;
      num_turn = 0;
      turn_waittimeZero = 0;
      turn_waittime = 750 * 3;
      blk_while_turning = false;
      parking_colourblk = false;
      power2 = 0;
      blk_after_turn = 0;
      colour_checked = false;

      OC2_starttime = internalClock.read();
      if (!first_run) resetIMU();
    }
    if (!end_game){
      displayThread.enabled = false;
      OC2_endtime = internalClock.read();
      steering_percentage = (steering_percentage > 100) ? 100 : steering_percentage;
      steering_percentage = (steering_percentage < -100) ? -100 : steering_percentage;
      steering(steering_percentage);
      MiniR4.M2.setPower(power2);
      lapDir_flash(anticlockwise);

      // The Finte State Machine for OC2 which is well organized
      switch (state) {
        case INIT_STATE:
          Serial.println("INIT_STATE");
          power2 = 0;
          if (ultra1Dist > ultra2Dist) {
            anticlockwise = true;
            Ultra2Thread.enabled = false;
          }
          else {
            anticlockwise = false;
            Ultra1Thread.enabled = false;
          }
          power2 = 30;
          prev_state = INIT_STATE;
          state = OUT_PARKING_P1_STATE;
          break;

        case OUT_PARKING_P1_STATE:
          Serial.println("OUT_PARKING_P1_STATE");
          power2 = 30;
          if(anticlockwise){
            if (getIMU() > -15) {
              steering_percentage = -100;
            } else {
              MiniR4.M2.resetCounter();
              prev_state = OUT_PARKING_P1_STATE;
              state = OUT_PARKING_P2_STATE;
              break;
            }
          } else {
            if (getIMU() < 15) {
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
          power2 = -30;
          if(anticlockwise){
            if (getIMU() > -85) {
              steering_percentage = 100;
            } else {
              MiniR4.M2.resetCounter();
              prev_state = OUT_PARKING_P2_STATE;
              state = OUT_PARKING_P3_STATE;
              break;
            }
          } else {
            if (getIMU() < 85) {
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
          power2 = -50;
          if(anticlockwise){
            if (abs(MiniR4.M2.getCounter()) < 1500) {
              steering_percentage = (getIMU() - -90) * 10;
            } else {
              MiniR4.M2.resetCounter();
              prev_state = OUT_PARKING_P3_STATE;
              state = OUT_PARKING_P4_STATE;
              break;
            }
          } else {
            if (abs(MiniR4.M2.getCounter()) < 1500) {
              steering_percentage = (getIMU() - 90) * 10;
            } else {
              MiniR4.M2.resetCounter();
              prev_state = OUT_PARKING_P3_STATE;
              state = OUT_PARKING_P4_STATE;
              break;
            }
          }
          break;

        case OUT_PARKING_P4_STATE:
          Serial.println("OUT_PARKING_P4_STATE");
          power2 = 30;
          if (anticlockwise){
            if (getIMU() < 0) steering_percentage = 100;
            else {
              if (nearestPillarGlobal.colour != NO_COLOUR && nearestPillarGlobal.area > 13){
                MiniR4.M2.resetCounter();
                prev_state = OUT_PARKING_P4_STATE;
                state = OUT_PARKING_P5_STATE;
                break;
              } else {
                MiniR4.M2.resetCounter();
                prev_state = OUT_PARKING_P4_STATE;
                state = DETECT_STATE;
                break;
              }
            }
          } else {
            if (getIMU() > 0) steering_percentage = 100;
            else {
                if (nearestPillarGlobal.colour != NO_COLOUR && nearestPillarGlobal.area > 13){
                  MiniR4.M2.resetCounter();
                  prev_state = OUT_PARKING_P4_STATE;
                  state = OUT_PARKING_P5_STATE;
                  break;
                } else {
                  MiniR4.M2.resetCounter();
                  prev_state = OUT_PARKING_P4_STATE;
                  state = DETECT_STATE;
                  break;
                }
            }
          }
          break;
        
        case OUT_PARKING_P5_STATE:
          Serial.println("OUT_PARKING_P5_STATE");
          power2 = 50;
          if (abs(MiniR4.M2.getCounter()) < 1750){
            steering_percentage = 0;
          } else {
            power2 = -30;
            MiniR4.M2.resetCounter();
            prev_state = OUT_PARKING_P5_STATE;
            state = DETECT_STATE;
            break;
          }
          break;

        case DETECT_DRIFTING_STATE:
          Serial.println("DETECT_DRIFTING_STATE");
          power2 = -20;
          if ((internalClock.read() - detect_drifting_t) < 2000){ // ultra1Dist > dist_threshold || && ultra2Dist > dist_threshold
            if (initial_drift_detect) {
              MiniR4.M2.resetCounter();
              steering_percentage = (anticlockwise) ? -30 : 30;
              initial_drift_detect = false;
            }
            else steering_percentage = 0;
            // steering(0);
            if (anticlockwise) cur_inner_dist = ultra1Dist;
            else cur_inner_dist = ultra2Dist;
          } else {
            if (abs(cur_inner_dist - prev_inner_dist) < 5) {
              steering_percentage = 0;
              power2 = 0;
              prev_state = DETECT_DRIFTING_STATE;
              state = RESET_IMU_STATE;
              reset_imu_t = internalClock.read();
              break;
            } else {
              // Serial.println(cur_inner_dist - prev_inner_dist);
              if (cur_inner_dist > prev_inner_dist) drift_ang = asin((cur_inner_dist - prev_inner_dist) / 25) * 180 / PI;
              else drift_ang = -asin((prev_inner_dist - cur_inner_dist) / 25) * 180 / PI;
              steering_percentage = drift_ang * 100 * 5.0 / 180;
              if (steering_percentage < 6.5 && steering_percentage > -6.5) steering_percentage = 14;
              // Serial.println(steering_percentage);
              // steering_percentage = steering_for_drift;
              drift_tuning_t = internalClock.read();
              prev_state = DETECT_DRIFTING_STATE;
              state = DRIFTING_TUNING_STATE;
              break;
            }
          }
          break;
        
        case DRIFTING_TUNING_STATE:
          Serial.println("DRIFTING_TUNING_STATE");
          if ((internalClock.read() - drift_tuning_t) < 500){ // ultra1Dist > dist_threshold || && ultra2Dist > dist_threshold
            steering(steering_percentage);
          } else {
            prev_state = DRIFTING_TUNING_STATE;
            state = DETECT_DRIFTING_STATE;
            detect_drifting_t = internalClock.read();
            prev_inner_dist = ultra1Dist;
            break;
          }
          break;

        case RESET_IMU_STATE:
          if ((internalClock.read() - reset_imu_t) < 1000){
            MiniR4.Motion.resetIMUValues();
            resetIMU();
            tar_ang = 0;
            steering_percentage = 0;
            initial_drift_detect = true;
            power2 = 0;
            break;
          } else {
            num_reset_imu++;
            encoder_counter = MiniR4.M2.getCounter();
            MiniR4.M2.resetCounter();
            prev_state = RESET_IMU_STATE;
            state = BW_AFTER_RESET_IMU_STATE;
            // state = DETECT_STATE;
            break;
          }
          break;
        
        case BW_AFTER_RESET_IMU_STATE:
          if (abs(MiniR4.M2.getCounter()) < abs(encoder_counter) - 250) {
            if (last_sector_no_blk) steering_percentage = (anticlockwise) ? abs(encoder_counter) / 150 : -abs(encoder_counter) / 150;
            if (steering_percentage > 20 || steering_percentage < -20) steering_percentage = (anticlockwise) ? 20 : -20; 
            else steering_percentage = 0;
            power2 = 50;
            break;
          } else {
            MiniR4.M2.resetCounter();
            prev_state = BW_AFTER_RESET_IMU_STATE;
            state = FW_AFTER_RESET_IMU_STATE;
            break;
          }
          break;
        
        case FW_AFTER_RESET_IMU_STATE:
          steering_percentage = 0;
          if (abs(MiniR4.M2.getCounter()) < 500) {
            power2 = -50;
            break;
          } else {
            MiniR4.M2.resetCounter();
            prev_state = FW_AFTER_RESET_IMU_STATE;
            state = DETECT_STATE;
            break;
          }
          break;

        // As the name implies, this state do the detections using ultrasonic sensors and huskylens when driving on a straight sector.
        case DETECT_STATE:
          Serial.println("DETECT_STATE");
          power2 = -30;
          if (anticlockwise){
            if (ultra1Dist > dist_threshold && abs(MiniR4.M2.getCounter() > 1500) || (getColorType() == 2 || getColorType() == 4)){
              if (getColorType() == 2 || getColorType() == 4) colour_check = true;
              if (nearestPillarGlobal.colour != NO_COLOUR){
                if (nearestPillarGlobal.area >= 13){
                  pillars_array.push_back({nearestPillarGlobal.colour, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area});
                  pillar_front = {nearestPillarGlobal.colour, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area};
                  prev_state = DETECT_STATE;
                  state = TURNING_N_AVOIDING_STATE;
                  power2 = power;
                  break;
                }
              } else {
                prev_state = DETECT_STATE;
                state = WAIT_TURN_STATE;
                turn_waittime = ((num_turn + 1) % 4 == 0) ? 750 * 3 : 750 * speed_amp_factor * 3;
                last_sector_no_blk = true;
                turn_waittimeZero = internalClock.read();
                power2 = power;
                break;
              }
            } else if (nearestPillarGlobal.colour == RED){
              if (ultra1Dist < dist_threshold && u1_ind < ultra1Dist) u1_ind = ultra1Dist;
              // Serial.println(u1_ind);
              if (nearestPillarGlobal.area >= 13 && nearestPillarGlobal.xpos > min_xpos_Rcase(nearestPillarGlobal.area)){
                red_block_flash(anticlockwise);
                pillars_array.push_back({nearestPillarGlobal.colour, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area});
                pillar_front = {RED, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area};
                gyro_ang_detect_phase = getIMU();
                if (pillar_front.xpos > 230) curve_phase1_gyro_chkpt = 45; // For Pt A, B
                else curve_phase1_gyro_chkpt = pillar_front.xpos * 45 / 200; // For Pt C, D, E
                prev_state = DETECT_STATE;
                state = CURVE_P1_STATE;
                power2 = power;
                break;
              }
            } else if (nearestPillarGlobal.colour == GREEN){
              if (ultra1Dist < dist_threshold && u1_ind < ultra1Dist) u1_ind = ultra1Dist;
              // Serial.println(u1_ind);
              if (nearestPillarGlobal.area >= 13 && nearestPillarGlobal.xpos < min_xpos_Gcase(nearestPillarGlobal.area)){
                green_block_flash(anticlockwise);
                pillars_array.push_back({nearestPillarGlobal.colour, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area});
                pillar_front = {GREEN, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area};
                gyro_ang_detect_phase = getIMU();
                if (mirror(pillar_front.xpos) > 230 && pillar_front.area < 30) curve_phase1_gyro_chkpt = 45; // For Pt A
                else if (pillar_front.area >= 30) curve_phase1_gyro_chkpt = 45; // For Pt B
                else curve_phase1_gyro_chkpt = mirror(pillar_front.xpos) * 45 / 200; // For Pt C, D, E
                prev_state = DETECT_STATE;
                state = CURVE_P1_STATE;
                // state = DEBUG_STATE;
                power2 = power;
                break;
              }
            } else {
              if (ultra1Dist < dist_threshold && u1_ind < ultra1Dist) u1_ind = ultra1Dist;
              // Serial.println(u1_ind);
              steering_percentage = (getIMU() - tar_ang) * 10; // TODO: Verify whether a minus sign is needed
            }
          } else {
            if (ultra2Dist > dist_threshold && abs(MiniR4.M2.getCounter() > 1500) || (getColorType() == 2 || getColorType() == 4)){
              if (getColorType() == 2 || getColorType() == 4) colour_check = true;
              if (nearestPillarGlobal.colour != NO_COLOUR){
                if (nearestPillarGlobal.area >= 13){
                  pillars_array.push_back({nearestPillarGlobal.colour, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area});
                  pillar_front = {nearestPillarGlobal.colour, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area};
                  prev_state = DETECT_STATE;
                  state = TURNING_N_AVOIDING_STATE;
                  power2 = power;
                  break;
                }
              } else {
                if (prev_pillar.colour == RED && prev_state == CHECK_MID_RACINGLN_STATE && prev_pillar.xpos > min_xpos_Rcase(nearestPillarGlobal.area)) dash_amp_fact = prev_pillar.xpos / CAM_MID_XPOS;
                else dash_amp_fact = 1;
                prev_state = DETECT_STATE;
                state = WAIT_TURN_STATE;
                turn_waittime = 750 * 3;
                turn_waittimeZero = internalClock.read();
                power2 = power;
                break;
              }
            } else if (nearestPillarGlobal.colour == RED){
              if (ultra2Dist < dist_threshold && u2_ind < ultra2Dist) u2_ind = ultra2Dist;
              if (nearestPillarGlobal.area >= 13 && nearestPillarGlobal.xpos > min_xpos_Rcase(nearestPillarGlobal.area)){
                red_block_flash(anticlockwise);
                pillars_array.push_back({nearestPillarGlobal.colour, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area});
                pillar_front = {RED, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area};
                gyro_ang_detect_phase = getIMU();
                if (pillar_front.xpos > 230) curve_phase1_gyro_chkpt = 45; // For Pt A, B
                else curve_phase1_gyro_chkpt = pillar_front.xpos * 45 / 200; // For Pt C, D, E
                prev_state = DETECT_STATE;
                state = CURVE_P1_STATE;
                power2 = power;
                break;
              }
            } else if (nearestPillarGlobal.colour == GREEN){
              if (ultra2Dist < dist_threshold && u2_ind < ultra2Dist) u2_ind = ultra2Dist;
              if (nearestPillarGlobal.area >= 13 && nearestPillarGlobal.xpos < min_xpos_Gcase(nearestPillarGlobal.area)){
                green_block_flash(anticlockwise);
                pillars_array.push_back({nearestPillarGlobal.colour, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area});
                pillar_front = {GREEN, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area};
                gyro_ang_detect_phase = getIMU();
                if (mirror(pillar_front.xpos) > 230 && pillar_front.area < 30) curve_phase1_gyro_chkpt = 40; // For Pt A
                else if (pillar_front.area >= 30) curve_phase1_gyro_chkpt = 40; // For Pt B
                else curve_phase1_gyro_chkpt = mirror(pillar_front.xpos) * 40 / 200; // For Pt C, D, E
                prev_state = DETECT_STATE;
                state = CURVE_P1_STATE;
                // state = DEBUG_STATE;…
                power2 = power;
                break;
              }
            } else {
              if (ultra2Dist < dist_threshold && u2_ind < ultra2Dist) u2_ind = ultra2Dist;
              steering_percentage = (getIMU() - tar_ang) * 10; // TODO: Verify whether a minus sign is needed
            }
          }
          break;

          case TURNING_N_AVOIDING_STATE:
            Serial.println("TURNING_N_AVOIDING_STATE");
            from_tNa_state = true;
            if (pillar_front.colour == RED) {
              turn_waittime = pillar_front.area * 50 * 3; // 49.2; // 48.5
              redblk_tna_flash(anticlockwise);
              // red_block_flash(anticlockwise);
            } else {
              turn_waittime = pillar_front.area * 5 * 3;
              greenblk_tna_flash(anticlockwise);
              // green_block_flash(anticlockwise);
            }
            prev_state = TURNING_N_AVOIDING_STATE;
            state = WAIT_TURN_STATE;
            turn_waittimeZero = internalClock.read();
            break;

          case WAIT_TURN_STATE:
            Serial.println("WAIT_TURN_STATE");
            power2 = -30;
            if ((internalClock.read() - turn_waittimeZero) < turn_waittime) {
              // MiniR4.M2.setPower(0);
              steering_percentage = (getIMU() - tar_ang) * 10;
              break;
            }
            else {
              if (!colour_check){
                if (anticlockwise){
                  if (ultra1Dist < dist_threshold && (internalClock.read() - turn_waittimeZero) > turn_waittime * 0.3){
                    prev_state = WAIT_TURN_STATE;
                    if (from_tNa_state) state = CURVE_P1_STATE;
                    else state = DETECT_STATE;
                    from_tNa_state = false;
                    break;
                  } else {
                    tar_ang += (anticlockwise == true) ? -right_ang : right_ang; // 88.58
                    num_turn++;
                    from_tNa_state = false;
                    prev_state = WAIT_TURN_STATE;
                    state = TURNING_STATE;
                    power2 = power;
                    break;
                  }
                } else {
                  if (ultra2Dist < dist_threshold && (internalClock.read() - turn_waittimeZero) > turn_waittime * 0.95){
                    prev_state = WAIT_TURN_STATE;
                    if (from_tNa_state) state = CURVE_P1_STATE;
                    else state = DETECT_STATE;
                    from_tNa_state = false;
                    break;
                  } else {
                    tar_ang += (anticlockwise == true) ? -right_ang : right_ang; // 88.58
                    num_turn++;
                    from_tNa_state = false;
                    prev_state = WAIT_TURN_STATE;
                    state = TURNING_STATE;
                    power2 = power;
                    break;
                  }
                }
              } else {
                tar_ang += (anticlockwise == true) ? -right_ang : right_ang; // 88.58
                num_turn++;
                from_tNa_state = false;
                prev_state = WAIT_TURN_STATE;
                state = TURNING_STATE;
                power2 = power;
                colour_check = false;
                break;
              }
            }
            break;
          
          case TURNING_STATE:
            Serial.println("TURNING_STATE");
            if (num_turn % 4 == 0 && num_turn < 12) {
              if (nearestPillarGlobal.area > 10 && nearestPillarGlobal.colour == RED && nearestPillarGlobal.xpos >= min_xpos_Rcase(nearestPillarGlobal.area)) {
                red_block_flash(anticlockwise);
                last_sector_no_blk = false;
                pillars_array.push_back({nearestPillarGlobal.colour, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area});
                pillar_front = {RED, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area};
                gyro_ang_detect_phase = getIMU();
                curve_phase1_gyro_chkpt = 20;
                blk_while_turning = true;
                prev_state = TURNING_STATE;
                state = CURVE_P1_STATE;
                break;
              } else if (nearestPillarGlobal.area > 10 && nearestPillarGlobal.colour == GREEN && nearestPillarGlobal.xpos <= min_xpos_Gcase(nearestPillarGlobal.area)) {
                green_block_flash(anticlockwise);
                last_sector_no_blk = false;
                pillars_array.push_back({nearestPillarGlobal.colour, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area});
                pillar_front = {GREEN, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area};
                gyro_ang_detect_phase = getIMU();
                Serial.println(gyro_ang_detect_phase);
                curve_phase1_gyro_chkpt = 35; // 60; // 70;
                blk_while_turning = true;
                prev_state = TURNING_STATE;
                state = CURVE_P1_STATE;
                // state = DEBUG_STATE;
                break;
              } else if ((abs(tar_ang - getIMU()) > 20) && (abs(tar_ang) > abs(getIMU()))){
                steering_percentage = (tar_ang > 0) ? -100 : 100;
                dash_timeZero = internalClock.read();
              } else{
                steering_percentage = 0;
                prev_state = TURNING_STATE;
                state = DASH_AFTER_TURNING_STATE;
                dash_time = (anticlockwise) ? u1_ind * 1.5 * dash_amp_fact : u2_ind * 1.5 * dash_amp_fact;
                dash_timeZero = internalClock.read();
                break;
                // steering_percentage = 0;
                // prev_state = TURNING_STATE;
                // state = LAST_SECTOR_DASH;
                // power2 = -30;
                // dash_time = (anticlockwise) ? u1_ind * 1.5 : u2_ind * 1.5;
                // // Serial.println(u1_ind);
                // // state = DEBUG_STATE;
                // // dash_time = 1200 * speed_amp_factor;
                // dash_timeZero = internalClock.read();
                // break;
              }
            } else if (nearestPillarGlobal.area > 10 && nearestPillarGlobal.colour == RED && nearestPillarGlobal.xpos >= min_xpos_Rcase(nearestPillarGlobal.area)) {
              red_block_flash(anticlockwise);
              pillars_array.push_back({nearestPillarGlobal.colour, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area});
              pillar_front = {RED, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area};
              gyro_ang_detect_phase = getIMU();
              curve_phase1_gyro_chkpt = 45;
              blk_while_turning = true;
              prev_state = TURNING_STATE;
              state = CURVE_P1_STATE;
              break;
            } else if (nearestPillarGlobal.area > 10 && nearestPillarGlobal.colour == GREEN && nearestPillarGlobal.xpos <= min_xpos_Gcase(nearestPillarGlobal.area)) {
              green_block_flash(anticlockwise);
              pillars_array.push_back({nearestPillarGlobal.colour, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area});
              pillar_front = {GREEN, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area};
              gyro_ang_detect_phase = getIMU();
              Serial.println(gyro_ang_detect_phase);
              curve_phase1_gyro_chkpt = 30; // 60; // 70;
              blk_while_turning = true;
              prev_state = TURNING_STATE;
              state = CURVE_P1_STATE;
              // state = DEBUG_STATE;
              break;
            } else if ((abs(tar_ang - getIMU()) > 20) && (abs(tar_ang) > abs(getIMU()))){
              steering_percentage = (tar_ang > 0) ? -100 : 100;
              dash_timeZero = internalClock.read();
            } else{
              steering_percentage = 0;
              prev_state = WAIT_TURN_STATE;
              state = DASH_AFTER_TURNING_STATE;
              // dash_time = 1200 * speed_amp_factor;
              dash_time = (anticlockwise) ? u1_ind * 1.5 * dash_amp_fact : u2_ind * 1.5 * dash_amp_fact;
              // Serial.println(u1_ind);
              // state = DEBUG_STATE;
              dash_timeZero = internalClock.read();
              break;
            }
            break;
          
          case LAST_SECTOR_DASH:
            Serial.println("LAST_SECTOR_DASH");
            if (anticlockwise){
              if (ultra1Dist > dist_threshold){
                steering_percentage = 0;
                power2 = -30;
              } else {
                prev_state = LAST_SECTOR_DASH;
                state = DETECT_DRIFTING_STATE;
                detect_drifting_t = internalClock.read();
                MiniR4.M2.resetCounter();
                break;
              }
            } else {
              if (ultra2Dist > dist_threshold){
                steering_percentage = 0;
                power2 = -30;
              } else {
                prev_state = LAST_SECTOR_DASH;
                state = DETECT_DRIFTING_STATE;
                detect_drifting_t = internalClock.read();
                MiniR4.M2.resetCounter();
                break;
              }
            }
            break;

          case DASH_AFTER_TURNING_STATE:
            Serial.println("DASH_AFTER_TURNING_STATE");
            if (anticlockwise) MiniR4.LED.setColor(2, 255, 255, 255);
            else MiniR4.LED.setColor(1, 255, 255, 255);
            // if()
            if (num_turn >= 12){
              if (anticlockwise) MiniR4.LED.setColor(2, 0, 0, 0);
              else MiniR4.LED.setColor(1, 0, 0, 0);
              prev_state = DASH_AFTER_TURNING_STATE;
              state = PARKING_STATE;
              break;
            } else if (nearestPillarGlobal.colour != NO_COLOUR && nearestPillarGlobal.area >= 13){
              blk_after_turn = true;
              if (nearestPillarGlobal.colour == RED && nearestPillarGlobal.xpos >= min_xpos_Rcase(nearestPillarGlobal.area)){
                red_block_flash(anticlockwise);
                pillars_array.push_back({nearestPillarGlobal.colour, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area});
                pillar_front = {RED, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area};
                gyro_ang_detect_phase = getIMU();
                Serial.println(gyro_ang_detect_phase);
                if (pillar_front.xpos > 230) curve_phase1_gyro_chkpt = 45; // For Pt A, B
                else curve_phase1_gyro_chkpt = pillar_front.xpos * 45 / 170; // For Pt C, D, E
                if (anticlockwise) MiniR4.LED.setColor(2, 0, 0, 0);
                else MiniR4.LED.setColor(1, 0, 0, 0);
                prev_state = DASH_AFTER_TURNING_STATE;
                state = CURVE_P1_STATE;
                power2 = power;
                break;
              } else if (nearestPillarGlobal.colour == GREEN && nearestPillarGlobal.xpos <= min_xpos_Gcase(nearestPillarGlobal.area)){
                green_block_flash(anticlockwise);
                pillars_array.push_back({nearestPillarGlobal.colour, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area});
                pillar_front = {GREEN, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area};
                gyro_ang_detect_phase = getIMU();
                if (mirror(pillar_front.xpos) > 230 && pillar_front.area < 30) curve_phase1_gyro_chkpt = 40; // For Pt A
                else if (pillar_front.area >= 30) curve_phase1_gyro_chkpt = 40; // For Pt B
                else curve_phase1_gyro_chkpt = mirror(pillar_front.xpos) * 45 / 200; // For Pt C, D, E
                prev_state = DASH_AFTER_TURNING_STATE;
                state = CURVE_P1_STATE;
                if (anticlockwise) MiniR4.LED.setColor(2, 0, 0, 0);
                else MiniR4.LED.setColor(1, 0, 0, 0);
                // state = DEBUG_STATE;
                power2 = power;
                break;
              }
            } else {
              if (anticlockwise){
                if ((internalClock.read() - dash_timeZero) < dash_time){ // ultra1Dist > dist_threshold || && ultra2Dist > dist_threshold
                  steering_percentage = (getIMU() - tar_ang) * 10; // TODO: Verify whether a minus sign is needed
                  MiniR4.M2.setPower(power2);
                } else {
                  if (num_reset_imu < num_turn / 4) {
                    initial_drift_detect = true;
                    if (anticlockwise) MiniR4.LED.setColor(2, 0, 0, 0);
                    else MiniR4.LED.setColor(1, 0, 0, 0);
                    prev_state = DASH_AFTER_TURNING_STATE;
                    state = DETECT_DRIFTING_STATE;
                    MiniR4.M2.resetCounter();
                    detect_drifting_t = internalClock.read();
                    prev_inner_dist = ultra1Dist;
                    break;
                  } else { 
                    power2 = power;
                    if (anticlockwise) MiniR4.LED.setColor(2, 0, 0, 0);
                    else MiniR4.LED.setColor(1, 0, 0, 0);
                    prev_state = DASH_AFTER_TURNING_STATE;
                    state = CHECK_FRONT_BLK_STATE;
                    MiniR4.M2.resetCounter();
                    break;
                  }
                }
              } else {
                if ((internalClock.read() - dash_timeZero) < dash_time){ // ultra1Dist > dist_threshold || && ultra2Dist > dist_threshold
                  steering_percentage = (getIMU() - tar_ang) * 10; // TODO: Verify whether a minus sign is needed
                  MiniR4.M2.setPower(power2);
                } else {
                  if (num_reset_imu < num_turn / 4){
                    initial_drift_detect = true;
                    if (anticlockwise) MiniR4.LED.setColor(2, 0, 0, 0);
                    else MiniR4.LED.setColor(1, 0, 0, 0);
                    prev_state = DASH_AFTER_TURNING_STATE;
                    state = DETECT_DRIFTING_STATE;
                    MiniR4.M2.resetCounter();
                    detect_drifting_t = internalClock.read();
                    prev_inner_dist = (anticlockwise) ? ultra1Dist : ultra2Dist;
                    break;
                  } else {
                    power2 = power;
                    if (anticlockwise) MiniR4.LED.setColor(2, 0, 0, 0);
                    else MiniR4.LED.setColor(1, 0, 0, 0);
                    prev_state = DASH_AFTER_TURNING_STATE;
                    state = CHECK_FRONT_BLK_STATE;
                    MiniR4.M2.resetCounter();
                    break;
                  }
                  break;
                }
              }
            }
            break;
          
          case CHECK_FRONT_BLK_STATE:
            Serial.println("CHECK_FRONT_BLK_STATE");
            if (nearestPillarGlobal.colour == RED && nearestPillarGlobal.area >= 7){
              // Serial.println("Hi?");
              pillars_array.push_back({nearestPillarGlobal.colour, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area});
              pillar_front = {RED, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area};
              prev_state = CHECK_FRONT_BLK_STATE;
              if (nearestPillarGlobal.xpos > min_xpos_Rcase(nearestPillarGlobal.area)) state = ST_FW_WITH_BLK_STATE;
              else {
                red_block_flash(anticlockwise);
                state = CURVE_P1_STATE;
                gyro_ang_detect_phase = getIMU();
                if (pillar_front.xpos > 230) curve_phase1_gyro_chkpt = 45; // For Pt A, B
                else curve_phase1_gyro_chkpt = pillar_front.xpos * 45 / 200; // For Pt C, D, E
                break;
              }
              break;
            } else if (nearestPillarGlobal.colour == GREEN && nearestPillarGlobal.area >= 7) {
              // Serial.println("Hi2?");
              pillars_array.push_back({nearestPillarGlobal.colour, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area});
              pillar_front = {GREEN, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area};
              gyro_ang_detect_phase = getIMU();
              prev_state = CHECK_FRONT_BLK_STATE;
              if (nearestPillarGlobal.xpos < min_xpos_Gcase(nearestPillarGlobal.area)) state = ST_FW_WITH_BLK_STATE;
              else {
                green_block_flash(anticlockwise);
                state = CURVE_P1_STATE;
                // state = DEBUG_STATE;
                gyro_ang_detect_phase = getIMU();
                if (mirror(pillar_front.xpos) > 230 && pillar_front.area < 30) curve_phase1_gyro_chkpt = 40; // For Pt A
                else if (pillar_front.area >= 30) curve_phase1_gyro_chkpt = 40; // For Pt B
                else curve_phase1_gyro_chkpt = mirror(pillar_front.xpos) * 45 / 200; // For Pt C, D, E
                break;
              }
              break;
            } else {
              prev_state = CHECK_FRONT_BLK_STATE;
              MiniR4.M2.resetCounter();
              state = CHECK_MID_RACINGLN_STATE;
              break;
            }
            break;

          case CHECK_MID_RACINGLN_STATE:
            Serial.println("CHECK_MID_RACINGLN_STATE");
            if (num_turn == 0){
              prev_state = CHECK_MID_RACINGLN_STATE;
              state = DETECT_STATE;
            } else if (anticlockwise){
              if (abs(ultra1Dist - MID_RACINGLN_POS) > 60 && ultra1Dist < dist_threshold && abs(MiniR4.M2.getCounter()) < 1500){
                inner_wall_dist = ultra1Dist - MID_RACINGLN_POS;
                gyro_ang_detect_phase = getIMU();
                prev_state = CHECK_MID_RACINGLN_STATE;
                state = MID_P1_STATE;
                break;
              } 
              else if (internalClock.read() - change_state_time > 50 * speed_amp_factor) {
                prev_state = CHECK_MID_RACINGLN_STATE;
                prev_pillar = {pillar_front.colour, pillar_front.xpos, CAM_LOWEST_YPOS - pillar_front.ypos, pillar_front.height, pillar_front.width, pillar_front.area};
                state = DETECT_STATE;
                break;
              }
            } else {
              if (abs(ultra2Dist - MID_RACINGLN_POS) > 60 && ultra2Dist < dist_threshold && abs(MiniR4.M2.getCounter()) < 1500){
                inner_wall_dist = -(ultra2Dist - MID_RACINGLN_POS);
                gyro_ang_detect_phase = getIMU();
                prev_state = CHECK_MID_RACINGLN_STATE;
                state = MID_P1_STATE;
                break;
              }  else if (internalClock.read() - change_state_time > 50 * speed_amp_factor) {
                prev_state = CHECK_MID_RACINGLN_STATE;
                state = DETECT_STATE;
                break;
              }
            }
            break;
          
          case MID_P1_STATE:
            Serial.println("MID_P1_STATE");
            if (nearestPillarGlobal.colour != NO_COLOUR && nearestPillarGlobal.area >= 15){
              if (nearestPillarGlobal.colour == RED && nearestPillarGlobal.xpos >= min_xpos_Rcase(nearestPillarGlobal.area)){
                red_block_flash(anticlockwise);
                pillars_array.push_back({nearestPillarGlobal.colour, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area});
                pillar_front = {RED, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area};
                gyro_ang_detect_phase = getIMU();
                if (pillar_front.xpos > 230) curve_phase1_gyro_chkpt = 45; // For Pt A, B
                else curve_phase1_gyro_chkpt = pillar_front.xpos * 45 / 200; // For Pt C, D, E
                prev_state = MID_P1_STATE;
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
                prev_state = MID_P1_STATE;
                state = CURVE_P1_STATE;
                // state = DEBUG_STATE;
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
                mid_phase2_chk_t = abs(inner_wall_dist) * 0.70 * speed_amp_factor;
                mid_save_t = internalClock.read();
                prev_state = MID_P1_STATE;
                state = MID_P2_STATE;
                break;
              }
            }
            break;
          
          case MID_P2_STATE:
            Serial.println("MID_P2_STATE");
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
            if (nearestPillarGlobal.colour != NO_COLOUR && nearestPillarGlobal.area >= 15){
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
                // state = DEBUG_STATE;
                break;
              }
            } else {
              if (abs(getIMU() - tar_ang) > 15){
                steering_percentage = -100 * sign(inner_wall_dist);
              } else {
                prev_state = MID_P3_STATE;
                state = MID_DASH_STATE;
                mid_dash_save_t = internalClock.read();
                break;
              }
            }
            break;
          
          case MID_DASH_STATE:
            Serial.println("MID_DASH_STATE");
            if (internalClock.read() - mid_dash_save_t <= 200 * speed_amp_factor){
              steering_percentage = (getIMU() - tar_ang) * 10;
              prev_state = MID_DASH_STATE;
            } else state = DETECT_STATE;
            break;

          case CURVE_P1_STATE:
            Serial.println("CURVE_P1_STATE");
            curve_p3_outloop_time = internalClock.read();
            if (pillar_front.colour == RED){
              red_block_flash(anticlockwise);
              if (getIMU() - gyro_ang_detect_phase + 30 < curve_phase1_gyro_chkpt - 5){
                if (blk_while_turning) steering_percentage = -100; 
                else if (pillar_front.xpos > 230 || pillar_front.area < 13) steering_percentage = -(68 - (getIMU() - gyro_ang_detect_phase)) * pillar_front.area * steering_amp_fact1 * 320 / (55 * area_dangerzone * pillar_front.xpos);
                else steering_percentage = -(60 - (getIMU() - gyro_ang_detect_phase)) * pillar_front.area * steering_amp_fact1 * 320 / (50 * area_dangerzone * pillar_front.xpos);
              } else {
                if (blk_while_turning) {
                  steering_percentage = 0;
                  prev_state = CURVE_P1_STATE;
                  state = CURVE_P2_5_STATE;
                  curve_sphase3_chk_t = pillar_front.area * 0.2 * speed_amp_factor;
                  Serial.println(curve_sphase3_chk_t);
                  pillar_avoid_save_t = internalClock.read();
                  break;
                } else if (pillar_front.xpos > 230 && pillar_front.area < 30){ // For Pt A
                  curve_phase2_chk_t = 200 * (pillar_front.xpos - 50) * speed_amp_factor / 300;
                  prev_state = CURVE_P1_STATE;
                  state = CURVE_P2_STATE;
                  break;
                } else if (pillar_front.area >= 16){ // For Pt B
                  curve_phase2_chk_t = 80 * (pillar_front.xpos - 50) * speed_amp_factor / 300;
                  prev_state = CURVE_P1_STATE;
                  state = CURVE_P2_STATE;
                  break;
                } else if (pillar_front.area > 10){ // For Pt C, D
                  curve_phase2_chk_t = 10 * (pillar_front.xpos - 50) * speed_amp_factor / 350;
                  prev_state = CURVE_P1_STATE;
                  state = CURVE_P2_STATE;
                  break;
                } else { // For Pt E
                  gyro_ang_detect_phase = getIMU();
                  prev_state = CURVE_P1_STATE;
                  state = CURVE_P3_STATE;
                  break;
                }
                pillar_avoid_save_t = internalClock.read();
                break;
              }
            } else {
              green_block_flash(anticlockwise);
              if (abs(getIMU() - gyro_ang_detect_phase) < curve_phase1_gyro_chkpt){
                if (blk_while_turning) steering_percentage = 100;
                if (mirror(pillar_front.xpos) > 230) steering_percentage = (55 - abs(getIMU() - gyro_ang_detect_phase)) * pillar_front.area * steering_amp_fact1 * 320 / (60 * area_dangerzone * mirror(pillar_front.xpos)); // || pillar_front.area < 13
                else steering_percentage = (50 - abs(getIMU() - gyro_ang_detect_phase)) * pillar_front.area * steering_amp_fact1 * 320 / (50 * area_dangerzone * mirror(pillar_front.xpos));
              } else {
                if (blk_while_turning) {
                  prev_state = CURVE_P1_STATE;
                  state = CURVE_P2_5_STATE;
                  curve_sphase3_chk_t = pillar_front.area * 10 * speed_amp_factor;
                  // Serial.println(curve_sphase3_chk_t);
                  pillar_avoid_save_t = internalClock.read();
                  break;
                } else if (mirror(pillar_front.xpos) > 230 && pillar_front.area < 30){ // For Pt A
                  curve_phase2_chk_t = 150 * (mirror(pillar_front.xpos) - 50) * speed_amp_factor / 300;
                  prev_state = CURVE_P1_STATE;
                  state = CURVE_P2_STATE;
                  break;
                } else if (pillar_front.area >= 30){ // For Pt B
                  curve_phase2_chk_t = 120 * (mirror(pillar_front.xpos) - 50) * speed_amp_factor / 300;
                  prev_state = CURVE_P1_STATE;
                  state = CURVE_P2_STATE;
                  break;
                } else if (pillar_front.area > 14){ // For Pt C, D
                  curve_phase2_chk_t = 100 * (mirror(pillar_front.xpos) - 50) * speed_amp_factor / 300;
                  gyro_ang_detect_phase = getIMU();
                  prev_state = CURVE_P1_STATE;
                  state = CURVE_P2_STATE;
                  break;
                } else { // For Pt E
                  curve_phase2_chk_t = 100 * (mirror(pillar_front.xpos) - 50) * speed_amp_factor / 300;
                  gyro_ang_detect_phase = getIMU();
                  prev_state = CURVE_P1_STATE;
                  state = CURVE_P3_STATE;
                  break;
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
              prev_state = CURVE_P2_STATE;
              state = CURVE_P3_STATE;
              break;
            }
            break;
          
          case CURVE_P2_5_STATE:
            Serial.println("CURVE_P2_5_STATE");
            if (internalClock.read() - pillar_avoid_save_t <= curve_sphase3_chk_t){
              steering_percentage = 0;
            } else {
              prev_state = CURVE_P2_5_STATE;
              state = CURVE_P3_STATE;
              break;
            }
            break;
          
          case CURVE_P3_STATE:
            Serial.println("CURVE_P3_STATE");
            if (nearestPillarGlobal.colour != NO_COLOUR && nearestPillarGlobal.colour > 10){
               if (nearestPillarGlobal.colour == RED && nearestPillarGlobal.xpos >= min_xpos_Rcase(nearestPillarGlobal.area)){
                red_block_flash(anticlockwise);
                pillars_array.push_back({nearestPillarGlobal.colour, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area});
                pillar_front = {RED, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area};
                gyro_ang_detect_phase = getIMU();
                if (pillar_front.xpos > 230) curve_phase1_gyro_chkpt = 45; // For Pt A, B
                else curve_phase1_gyro_chkpt = pillar_front.xpos * 45 / 200; // For Pt C, D, E
                prev_state = CURVE_P3_STATE;
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
                prev_state = CURVE_P3_STATE;
                state = CURVE_P1_STATE;
                // state = DEBUG_STATE;
                break;
              }
            } else {
              // Serial.println(abs(getIMU() - tar_ang));
              if (pillar_front.colour == RED){
                red_block_flash(anticlockwise);
                if (getIMU() - tar_ang > 20){
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
                if (tar_ang - getIMU() > 20){
                  if (blk_while_turning) steering_percentage = -100;
                  else steering_percentage = -(abs(tar_ang - getIMU()) + 30) * steering_amp_fact2;
                } else {
                  prev_state = CURVE_P3_STATE;
                  state = CURVE_P4_STATE;
                  pillar_avoid_save_t = internalClock.read();
                  break;
                }
              }
            }
            Serial.println(steering_percentage);
            break;

          case CURVE_P4_STATE:
            Serial.println("CURVE_P4_STATE");
            // if (nearestPillarGlobal.area >= 40) pillar_avoid_save_t = internalClock.read();
            CP4_t = (num_turn % 4 == 0 && num_turn < 12) ? 1000 : (blk_while_turning) ? 500 : 500;
            if (num_turn % 4 == 0 && num_turn < 12) CP4_t = 1000;
            else if (blk_while_turning) CP4_t = 100;
            else if (pillar_front.colour == RED) {
              if (pillar_front.xpos > 230 && pillar_front.area < 30) CP4_t = 600; // For Pt A
              else if (pillar_front.area >= 16) CP4_t = 300; // For Pt B
              else if (pillar_front.area > 10) CP4_t = 600; // For Pt C, D
              else CP4_t = 500; // For Pt E
            } else {
              if (mirror(pillar_front.xpos) > 230 && pillar_front.area < 30) CP4_t = 500; // For Pt A
              else if (pillar_front.area >= 30) CP4_t = 200; // For Pt B
              else curve_phase1_gyro_chkpt = CP4_t = 500; // For Pt C, D, E
            }
            if (internalClock.read() - pillar_avoid_save_t <= CP4_t * speed_amp_factor){
              if (num_reset_imu < num_turn / 4){
                initial_drift_detect = true;
                no_blk_flash(anticlockwise);
                prev_state = CURVE_P4_STATE;
                state = DETECT_DRIFTING_STATE;
                if (blk_while_turning || blk_after_turn) MiniR4.M2.resetCounter();
                detect_drifting_t = internalClock.read();
                prev_inner_dist = ultra1Dist;
                blk_while_turning = false;
                break;
              } else if (anticlockwise && ultra1Dist > dist_threshold || (getColorType() == 2 || getColorType() == 4)) {
                prev_state = CURVE_P4_STATE;
                state = DETECT_STATE;
                break;
              } else if (!anticlockwise && (ultra2Dist > dist_threshold || (getColorType() == 2 || getColorType() == 4)) {
                prev_state = CURVE_P4_STATE;
                state = DETECT_STATE;
                break;
              } else 
              steering_percentage = (getIMU() - tar_ang) * 10;
            } else {
              no_blk_flash(anticlockwise);
              prev_state = CURVE_P4_STATE;
              state = CHECK_MID_RACINGLN_STATE;
              change_state_time = internalClock.read();
              if (blk_while_turning || blk_after_turn) MiniR4.M2.resetCounter();
              blk_while_turning = false;
              break;
            }
            break;

          case ST_FW_WITH_BLK_STATE:
            Serial.println("ST_FW_WITH_BLK_STATE");
            if (anticlockwise){
              if (ultra1Dist < dist_threshold){ // ultra1Dist > dist_threshold || && ultra2Dist > dist_threshold
                steering_percentage = (getIMU() - tar_ang) * 10;
                break;
              } else{
                prev_state = ST_FW_WITH_BLK_STATE;
                state = DETECT_STATE;
              }

            } else {
              if (ultra2Dist < dist_threshold){ // ultra1Dist > dist_threshold || && ultra2Dist > dist_threshold
              steering_percentage = (getIMU() - tar_ang) * 10;
                break;
              } else{
                prev_state = ST_FW_WITH_BLK_STATE;
                state = CHECK_FRONT_BLK_STATE;
              }
            }
            break;

          case PARKING_STATE:
            end_game = true;

          case DEBUG_STATE:
            power2 = 0;
            MiniR4.M2.setPower(power2);
            // Serial.println("Colour:");
            // Serial.println(pillar_front.colour);
            // Serial.println("Area:");
            // Serial.println(pillar_front.area);
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
  OC2HuskylensNColor(86, 800, -80); //-80
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

