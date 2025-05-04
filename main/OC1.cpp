#include "Modules/MiniR4Buzzer.h"
#include "Arduino.h"
#include "OC1.h"

/* Global variable for storing the steering angle percentage (-100% ~ 100%) */
float steering_percentage;

/**
 * @brief The main function for OC1
 * If this function is being used, the smart car will turn to the direction after one of the laser sensor detects a distance > 1700mm for 300ms
 * 
 */
void OpenChallenge300(LASERMODE laserMode, int laserMaxElements, double right_ang, int dist_threshold, int turn_time, int power){
  LaserMode = laserMode;
  LaserElements = laserMaxElements;
  static bool start_game = false;
  static bool prev_btn_state = false;
  bool curr_btn_state = MiniR4.BTN_DOWN.getState();
  if (curr_btn_state && !prev_btn_state){
    start_game = !start_game;
  }
  prev_btn_state = curr_btn_state;
  static bool end_game = false;
  static bool anticlockwise = false;
  static long milliseconds = 0;
  static long dash_timeZero = 0;
  static long dash_time = 0;
  static bool is_left = false;
  static bool reset_timer = true;
  static double tar_ang = 0;
  static bool cal_tar_ang = false;
  static bool need_turn = false;
  static bool dash_forward = false;
  static int num_turn = 0;
  if (!start_game){
    milliseconds = 0;
    dash_timeZero = 0;
    dash_time = 0;
    reset_timer = true;
    tar_ang = 0;
    is_left = false;
    cal_tar_ang = false;
    need_turn = false;
    dash_forward = false;
    num_turn = 0;
    steering(0);
    MiniR4.M2.setPower(0);
  } else {
    if (!end_game){
      if (num_turn == 0){
        if (laser1Dist > dist_threshold && !dash_forward){
          anticlockwise = true;
          is_left = true;
          cal_tar_ang = true;
        } else if (laser2Dist > dist_threshold && !dash_forward){
          anticlockwise = false;
          is_left = false;
          cal_tar_ang = true;
        } else {
          cal_tar_ang = false;
        }
      } else {
        if (anticlockwise){
          if (laser1Dist > dist_threshold && !dash_forward){
            is_left = true;
            cal_tar_ang = true;
          } else {
            cal_tar_ang = false;
          }
        } else {
          if (laser2Dist > dist_threshold && !dash_forward){
            anticlockwise = false;
            is_left = false;
            cal_tar_ang = true;
          } else {
            cal_tar_ang = false;
          }
        }
      }
      if (((internalClock.read() - milliseconds) > turn_time) && cal_tar_ang){
        // MiniR4.Buzzer.Tone(1000, 100);
        tar_ang += (is_left == true) ? -right_ang : right_ang; // 88.58
        cal_tar_ang = false;
        need_turn = true;
        num_turn++;
      }
      if ((abs(tar_ang - getIMU()) > 5) && (abs(tar_ang) > abs(getIMU())) && need_turn){
        steering_percentage = (tar_ang > 0) ? -100 : 100;
        steering(steering_percentage);
        dash_forward = true;
        dash_timeZero = internalClock.read();
      } else {
        need_turn = false;
        if (dash_forward){
          if (num_turn >= 12) dash_time = 1000;
          else dash_time = 2500;
          if ((internalClock.read() - dash_timeZero) < dash_time){
            steering_percentage = (getIMU() - tar_ang) * 6;
            steering(steering_percentage);
            MiniR4.M2.setPower(power);
          } else {
            dash_forward = false;
          }
        } else {
          // MiniR4.Buzzer.Tone(1000, 100);
          steering_percentage = (getIMU() - tar_ang) * 6;
          steering(steering_percentage);
          cal_tar_ang = true;
          need_turn = false;
        }
      }
      if (num_turn >= 12 && !dash_forward){
        // MiniR4.Buzzer.Tone(1000, 100);
        steering(0);
        MiniR4.M2.setPower(0);
        end_game = true;
      } else {
        MiniR4.M2.setPower(power);
      }
    } else {
      steering(0);
      MiniR4.M2.setPower(0);
    }
  }
}

/**
 * @brief The main function for OC1
 * If this function is being used, the smart car will turn to the direction immediately after one of the laser sensor detects a MEAN/MEDIAN distance > 1700mm
 * 
 */
void OpenChallengeLaserFilter(LASERMODE laserMode, int laserMaxElements, double right_ang, int dist_threshold, int power){
  LaserMode = laserMode;
  LaserElements = laserMaxElements;
  static bool start_game = false;
  static bool prev_btn_state = false;
  bool curr_btn_state = MiniR4.BTN_DOWN.getState();
  if (curr_btn_state && !prev_btn_state){
    start_game = !start_game;
  }
  prev_btn_state = curr_btn_state;

  static bool end_game = false;
  static long dash_timeZero = 0;
  static long dash_timeZero2 = 0;
  static long dash_time = 0;
  static double tar_ang = 0;
  static bool is_left = false;
  static bool anticlockwise = false;
  static bool cal_tar_ang = false;
  static bool need_turn = false;
  static bool dash_forward = false;
  static int num_turn = 0;
  static bool close_wall = false;
  static long milliseconds = 0;
  if (!start_game){ 
    end_game = false;
    dash_timeZero = 0;
    dash_timeZero2 = 0;
    dash_time = 0;
    tar_ang = 0;
    is_left = false;
    cal_tar_ang = false;
    need_turn = false;
    dash_forward = false;
    num_turn = 0;
    close_wall = false;
    steering(0);
    MiniR4.M2.setPower(0);
  } else {
    if (!end_game){
      if (num_turn == 0){
        if (laser1Dist > dist_threshold && !dash_forward){
          anticlockwise = true;
          is_left = true;
          cal_tar_ang = true;
        } else if (laser2Dist > dist_threshold && !dash_forward){
          anticlockwise = false;
          is_left = false;
          cal_tar_ang = true;
        } else {
          cal_tar_ang = false;
          milliseconds = internalClock.read();
        }
      } else {
        if (anticlockwise){
          if (laser1Dist > dist_threshold && !dash_forward){
            is_left = true;
            cal_tar_ang = true;
          } else {
            cal_tar_ang = false;
          }
        } else {
          if (laser2Dist > dist_threshold && !dash_forward){
            anticlockwise = false;
            is_left = false;
            cal_tar_ang = true;
          } else {
            cal_tar_ang = false;
            milliseconds = internalClock.read();
          }
        }
      }
      if (((internalClock.read() - milliseconds) > 200) && cal_tar_ang){
        // MiniR4.Buzzer.Tone(1000, 100);
        tar_ang += (is_left == true) ? -right_ang : right_ang; // 88.58
        cal_tar_ang = false;
        need_turn = true;
        close_wall = false;
        num_turn++;
        // dash_timeZero2 = internalClock.read();
      }

      if ((abs(tar_ang - getIMU()) > 5) && (abs(tar_ang) > abs(getIMU())) && need_turn){
        steering_percentage = (tar_ang > 0) ? -100 : 100;
        steering(steering_percentage);
        dash_forward = true;
        dash_timeZero = internalClock.read();
      } else {
        need_turn = false;
        if (dash_forward){
          if (num_turn >= 12) dash_time = 1000;
          else if (num_turn == 1) dash_time = 1500;
          else dash_time = 1500;
          if ((internalClock.read() - dash_timeZero) < dash_time){
            steering_percentage = (getIMU() - tar_ang) * 3;
            steering(steering_percentage);
            MiniR4.M2.setPower(power);
          } else {
            dash_forward = false;
          }
        } else {
          // MiniR4.Buzzer.Tone(1000, 100);
          if (anticlockwise){
            if (laser1Dist > 150 && num_turn > 0 && !close_wall){
              steering_percentage = (laser1Dist - 150) * 0.1;
              if (steering_percentage > 20) steering_percentage = 20;
              steering(steering_percentage);
            } else {
              steering_percentage = (getIMU() - tar_ang) * 2;
              steering(steering_percentage);
              cal_tar_ang = true;
              need_turn = false;
              close_wall = true;
            }
          } else {
            if (laser2Dist > 150 && num_turn > 0 && !close_wall){
              steering_percentage = -(laser2Dist - 150) * 0.1;
              if (steering_percentage < -30) steering_percentage = -30;
              steering(steering_percentage);
            } else {
              steering_percentage = (getIMU() - tar_ang) * 2;
              steering(steering_percentage);
              cal_tar_ang = true;
              need_turn = false;
              close_wall = true;
            }
          }
        }
      }
      if (num_turn >= 12 && !dash_forward){
        // MiniR4.Buzzer.Tone(1000, 100);
        steering(0);
        MiniR4.M2.setPower(0);
        end_game = true;
      } else {
        MiniR4.M2.setPower(power);
      }
    } else {
      steering(0);
      MiniR4.M2.setPower(0);
    }
  }
}

void OC1main(){
  LaserMode = CISTERN_DIST;
  LaserElements = 8;
  // OpenChallenge300(MEDIAN_DIST, 10, 88.58, 1500, 300, -100);
  OpenChallengeLaserFilter(LaserMode, LaserElements, 90, 2500, -100);
}

/**
 * @brief A function to put into display thread for showing the steering angle percentage
 * 
 * @param column; (enum) COLUMN; A enum defined in oled.h indicating which column the data should be displaced at
 * @param line_number; int; The line number where the data should be displaced at (0 ~ 3)
 * @param size; int; The size of the text being displaced (1 ~ 2)
 * @param clearDisplay; bool; Set true to clear the whole OLED display everytime before displaying the battery percentage
 */
void showSteeringOC1(COLUMN column, int line_number, int size, bool clearDisplay){
  static String steering_text = "St: " + String(steering_percentage) + "%";
  display.oledSetTextColour(BLACK);
  if (column == LEFT){
    display.oledDisplayLeftln(line_number, size, steering_text, clearDisplay);
    display.oledSetTextColour(WHITE);
    steering_text = "St: " + String(steering_percentage) + "%";
    display.oledDisplayLeftln(line_number, size, steering_text, clearDisplay);
  } else if (column == MID) {
    display.oledDisplayCenterln(line_number, size, steering_text, clearDisplay);
    display.oledSetTextColour(WHITE);
    steering_text = "St: " + String(steering_percentage) + "%";
    display.oledDisplayCenterln(line_number, size, steering_text, clearDisplay);
  } else {
    display.oledDisplayRightln(line_number, size, steering_text, clearDisplay);
    display.oledSetTextColour(WHITE);
    steering_text = "St: " + String(steering_percentage) + "%";
    display.oledDisplayRightln(line_number, size, steering_text, clearDisplay);
  }
}
