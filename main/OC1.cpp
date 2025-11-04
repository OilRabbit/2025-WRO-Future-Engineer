#include "OC1.h"

static inline void safeSuspend(TaskHandle_t h){ if (h) vTaskSuspend(h); }
static inline void safeResume(TaskHandle_t h){ if (h) vTaskResume(h); }

/**
 * @brief To get the sign of a variable
 * @param num; float; the variable to get the sign
 *
 * @return int; 1, -1 or 0
*/
int sign(float num){
  if (num > 0) return 1;
  else if (num < 0) return -1;
  else return 0;
}

/**
 * @brief Function for OC1 using Ultrasonic sensors
 * 
 * @param right_ang; double; the value of an "right angle" for the IMU (as the value of IMU is not consistent)
 * @param dist_threshold; int; the threshold that the car sees for turning (in mm)
 * @param power; int; the power of the driving motor (0 ~ -100)
 * 
 */
bool reset_OC1 = true;
long OC1_starttime = 0; 
long OC1_endtime = 0; 
void OC1(double right_ang, int dist_threshold, int power){
  // Disable the huskylens thread to boost the efficiency of this thread
  safeSuspend(blinkledThread);
  safeSuspend(Pixy2Thread);
  safeSuspend(OC2Thread);

  // Variables required for OC1
  static OC1_STATES state = DETECT_STATE;     // A variable storing the current state of OC2 run
  static OC1_STATES prev_state = DETECT_STATE;
  static bool end_game = false;           // A flag determining whether the run has ended
  static double tar_ang = 0;              // The target angle which the car should be facing
  static bool is_anticlockwise = false;      // A boolean storing whether the car is racing in clockwise or anti-clockwise direction
  static int num_turn = 0;                // Variable storing the number of turns the car has made
  static int innerWall_dist = 11;        // The distance with the inner wall in mm that the car should keep
  static int turn_waittime = 150;
  static long turn_waittimeZero = 0;
  static long dash_time = 0;
  static long dash_timeZero = 0;
  static int lane_length = 0;
  String OC1_text = String("oc1:");

  if (reset_OC1){
    tar_ang = 0;
    is_anticlockwise = false;
    num_turn = 0;
    innerWall_dist = 11;
    steering_percentage = 0;
    turn_waittime = 150;
    turn_waittimeZero = 0;
    dash_time = 0;
    dash_timeZero = 0;
    lane_length = 0;
    end_game = false;
    OC1_starttime = internalClock.read();
    imu_resetYaw();
    reset_OC1 = false;
  }

  if (!end_game){
    // safeSuspend(displayThread);
    motor_move(power);

    switch(state){
      case DETECT_STATE:
        Serial.println("DETECT_STATE");
        OC1_text = String("oc1: ") + String("detect");
        Serial.print("num_turn = ");
        Serial.println(num_turn);
        if (num_turn == 0) {
          if (ultra1Dist > dist_threshold){
            is_anticlockwise = true;
            safeSuspend(Ultra2Thread);
            prev_state = DETECT_STATE;
            state = WAIT_TURN_STATE;
            turn_waittime = 150;
            turn_waittimeZero = internalClock.read();
            break;
          } else if (ultra2Dist > dist_threshold){
            is_anticlockwise = false;
            safeSuspend(Ultra1Thread);
            prev_state = DETECT_STATE;
            state = WAIT_TURN_STATE;
            turn_waittime = 150;
            turn_waittimeZero = internalClock.read();
            break;
          } else {
            steering_percentage = -(imu_yaw - tar_ang) * 10; // TODO: Verify whether a minus sign is needed
          }
        } else {
          if (is_anticlockwise){
            if (ultra1Dist > dist_threshold){
              prev_state = DETECT_STATE;
              state = WAIT_TURN_STATE;
              turn_waittime = 150;
              turn_waittimeZero = internalClock.read();
              if (num_turn == 4) lane_length = MOTOR_ENCODER_VALUE;
              break;
            } else {
              steering_percentage = -(ultra1Dist - innerWall_dist) * 5; // TODO: Verify whether a minus sign is needed
            }
          } else {
            if (ultra2Dist > dist_threshold){
              prev_state = DETECT_STATE;
              state = WAIT_TURN_STATE;
              turn_waittime = 150;
              turn_waittimeZero = internalClock.read();
              if (num_turn == 4) lane_length = MOTOR_ENCODER_VALUE;
              break;
            } else {
              steering_percentage = (ultra2Dist - innerWall_dist) * 5; // TODO: Verify whether a minus sign is needed
            }
          }
        }
        break;

      case WAIT_TURN_STATE:
        OC1_text = String("oc1: ") + String("wait");
        Serial.println("WAIT_TURN_STATE");
        Serial.println(is_anticlockwise ? "Anti-C" : "C");
        if ((internalClock.read() - turn_waittimeZero) < turn_waittime) {
          steering_percentage = -(imu_yaw - tar_ang) * 10;
          break;
        }
        else {
          if (is_anticlockwise){
            if (ultra1Dist < dist_threshold){
              prev_state = WAIT_TURN_STATE;
              state = DETECT_STATE;
              break;
            } else {
              tar_ang += (is_anticlockwise == true) ? -right_ang : right_ang;
              num_turn++;
              prev_state = WAIT_TURN_STATE;
              state = TURNING_STATE;
              break;
            }
          } else {
            if (ultra2Dist < dist_threshold){
              prev_state = WAIT_TURN_STATE;
              state = DETECT_STATE;
              break;
            } else {
              tar_ang += (is_anticlockwise == true) ? -right_ang : right_ang;
              num_turn++;
              prev_state = WAIT_TURN_STATE;
              state = TURNING_STATE;
              break;
            }
          }
        }
        break;
      
      case TURNING_STATE:
        OC1_text = String("oc1: ") + String("turn");
        Serial.println("TURNING_STATE");
        if ((abs(tar_ang - imu_yaw) > 10) && (abs(tar_ang) > abs(imu_yaw))){
          steering_percentage = (tar_ang > 0) ? 100 : -100;
          dash_timeZero = internalClock.read();
        } else{
          steering_percentage = 0;
          prev_state = TURNING_STATE;
          state = DASH_AFTER_TURNING_STATE;
          dash_time = 500;
          dash_timeZero = internalClock.read();
          if (num_turn == 4) reset_encoder();
          break;
        }
        break;
      
      case DASH_AFTER_TURNING_STATE:
        OC1_text = String("oc1: ") + String("dash");
        Serial.println("DASH_AFTER_TURNING_STATE");
        if (num_turn >= 12){
          prev_state = DASH_AFTER_TURNING_STATE;
          reset_encoder();
          state = ENDING_STATE;
          break;
        } else {
          if (is_anticlockwise){
            if ((internalClock.read() - dash_timeZero) < dash_time){ // ultra1Dist > dist_threshold || && ultra2Dist > dist_threshold
              steering_percentage = -(imu_yaw - tar_ang) * 10; // TODO: Verify whether a minus sign is needed
            } else {
              prev_state = DASH_AFTER_TURNING_STATE;
              state = DETECT_STATE;
              break;
            }
          } else {
            if ((internalClock.read() - dash_timeZero) < dash_time){ // ultra1Dist > dist_threshold || && ultra2Dist > dist_threshold
              steering_percentage = -(imu_yaw - tar_ang) * 10; // TODO: Verify whether a minus sign is needed
            } else {
              prev_state = DASH_AFTER_TURNING_STATE;
              state = DETECT_STATE;
              break;
            }
          }
        }
        break;

      case ENDING_STATE:
        OC1_text = String("oc1: ") + String("end");
        Serial.println("ENDING_STATE");
        if (MOTOR_ENCODER_VALUE < lane_length){
          steering_percentage = -(imu_yaw - tar_ang) * 10;
        } else {
          motor_stop(BRAKE);
          steering_percentage = 0;
          end_game = true;
          OC1_endtime = internalClock.read();
        }
        break;
    }
  }
  // tft.clearln(TFT_LEFT_CLN, 11);
  // tft.clearln(TFT_RIGHT_CLN, 11);
  // tft.displayLeftln(11, 2, OC1_text.c_str(), TFT_WHITE, false);
  vTaskDelay(5 / portTICK_PERIOD_MS);
}

/**
 * @brief The main thread function for OC1
 * 
 */
void OC1main(void *){
  static bool run_OC1 = false;
  while (1){
    if (is_btn_bumped(TFT_BTN1)){
      run_OC1 = !run_OC1;
      reset_OC1 = true;
    }
    if (run_OC1){
      OC1(90, 100, 20);
    } else {
      // safeResume(displayThread);
      motor_stop(COAST);
      steering_percentage = 0;
    }
    vTaskDelay(5 / portTICK_PERIOD_MS);
  }
}

// /**
//  * @brief The display function for OC1 showing the time required to finish a race
//  * 
//  */
// void showOC1Time(COLUMN column, int line_number, int size, bool clearDisplay){
//   static String curr_time_text;
//   display.oledSetTextColour(BLACK);

//   long total_ms = OC1_endtime - OC1_starttime;
//   long seconds = total_ms / 1000;
//   long milliseconds = total_ms % 1000;

//   if (column == LEFT){
//     display.oledDisplayLeftln(line_number, size, curr_time_text, clearDisplay);
//     display.oledSetTextColour(WHITE);
//     curr_time_text = String(seconds) + "." + String(milliseconds);
//     display.oledDisplayLeftln(line_number, size, curr_time_text, clearDisplay);

//   } else if (column == MID) {
//     display.oledDisplayCenterln(line_number, size, curr_time_text, clearDisplay);
//     display.oledSetTextColour(WHITE);
//     curr_time_text = String(seconds) + "." + String(milliseconds);
//     display.oledDisplayCenterln(line_number, size, curr_time_text, clearDisplay);
//   } else {
//     display.oledDisplayRightln(line_number, size, curr_time_text, clearDisplay);
//     display.oledSetTextColour(WHITE);
//     curr_time_text = String(seconds) + "." + String(milliseconds);
//     display.oledDisplayRightln(line_number, size, curr_time_text, clearDisplay);
//   }
// }