#include "Modules/MiniR4Buzzer.h"
#include "Arduino.h"
#include "OC1.h"

float steering_percentage;

void OpenChallenge300(){
  static bool start_game = false;
  static bool prev_btn_state = false;
  bool curr_btn_state = MiniR4.BTN_DOWN.getState();
  if (curr_btn_state && !prev_btn_state){
    start_game = !start_game;
  }
  prev_btn_state = curr_btn_state;

  static bool end_game = false;
  double right_ang = 88.58;
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
  static int speed = 0;
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
    speed = 0;
    MiniR4.M2.setPower(speed);
  } else {
    speed = -100;
    if (!end_game){
      if (num_turn == 0){
        if (laser1Mean > 1700 && !dash_forward){
          anticlockwise = true;
          is_left = true;
          cal_tar_ang = true;
        } else if (laser2Mean > 1700 && !dash_forward){
          anticlockwise = false;
          is_left = false;
          cal_tar_ang = true;
        } else {
          cal_tar_ang = false;
        }
      } else {
        if (anticlockwise){
          if (laser1Mean > 1700 && !dash_forward){
            is_left = true;
            cal_tar_ang = true;
          } else {
            cal_tar_ang = false;
          }
        } else {
          if (laser2Mean > 1700 && !dash_forward){
            anticlockwise = false;
            is_left = false;
            cal_tar_ang = true;
          } else {
            cal_tar_ang = false;
          }
        }
      }
      if (((internalClock.read() - milliseconds) > 300) && cal_tar_ang){
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
            MiniR4.M2.setPower(speed);
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
        MiniR4.M2.setPower(0);
        end_game = true;
      } else {
        MiniR4.M2.setPower(speed);
      }
    } else {
      MiniR4.M2.setPower(0);
    }
  }
}

void OpenChallengeMeanDist(){
  static bool start_game = false;
  static bool prev_btn_state = false;
  bool curr_btn_state = MiniR4.BTN_DOWN.getState();
  if (curr_btn_state && !prev_btn_state){
    start_game = !start_game;
  }
  prev_btn_state = curr_btn_state;

  static bool end_game = false;
  double right_ang = 88.58;
  static long dash_timeZero = 0;
  static long dash_time = 0;
  static double tar_ang = 0;
  static bool is_left = false;
  static bool anticlockwise = false;
  static bool cal_tar_ang = false;
  static bool need_turn = false;
  static bool dash_forward = false;
  static int num_turn = 0;
  static int speed = 0;
  if (!start_game){ 
    end_game = false;
    dash_timeZero = 0;
    dash_time = 0;
    tar_ang = 0;
    is_left = false;
    cal_tar_ang = false;
    need_turn = false;
    dash_forward = false;
    num_turn = 0;
    speed = 0;
    MiniR4.M2.setPower(speed);
  } else {
    speed = -100;
    if (!end_game){
      if (num_turn == 0){
        if (laser1Mean > 1700 && !dash_forward){
          anticlockwise = true;
          is_left = true;
          cal_tar_ang = true;
        } else if (laser2Mean > 1700 && !dash_forward){
          anticlockwise = false;
          is_left = false;
          cal_tar_ang = true;
        } else {
          cal_tar_ang = false;
        }
      } else {
        if (anticlockwise){
          if (laser1Mean > 1700 && !dash_forward){
            is_left = true;
            cal_tar_ang = true;
          } else {
            cal_tar_ang = false;
          }
        } else {
          if (laser2Mean > 1700 && !dash_forward){
            anticlockwise = false;
            is_left = false;
            cal_tar_ang = true;
          } else {
            cal_tar_ang = false;
          }
        }
      }
      if (cal_tar_ang){
        // MiniR4.Buzzer.Tone(1000, 100);
        tar_ang += (is_left == true) ? -right_ang : right_ang; // 88.58
        cal_tar_ang = false;
        need_turn = true;
        num_turn++;
      }
      // if (num_turn == 4){
      //   speed = 0;
      //   MiniR4.M2.setPower(speed);
      // }
      if ((abs(tar_ang - getIMU()) > 5) && (abs(tar_ang) > abs(getIMU())) && need_turn){
        steering_percentage = (tar_ang > 0) ? -100 : 100;
        steering(steering_percentage);
        dash_forward = true;
        dash_timeZero = internalClock.read();
      } else {
        need_turn = false;
        if (dash_forward){
          if (num_turn >= 12) dash_time = 1000;
          else dash_time = 1500;
          if ((internalClock.read() - dash_timeZero) < dash_time){
            steering_percentage = (getIMU() - tar_ang) * 3;
            steering(steering_percentage);
            MiniR4.M2.setPower(speed);
          } else {
            dash_forward = false;
          }
        } else {
          // MiniR4.Buzzer.Tone(1000, 100);
          steering_percentage = (getIMU() - tar_ang) * 2;
          steering(steering_percentage);
          cal_tar_ang = true;
          need_turn = false;
        }
      }
      if (num_turn >= 12 && !dash_forward){
        // MiniR4.Buzzer.Tone(1000, 100);
        MiniR4.M2.setPower(0);
        end_game = true;
      } else {
        MiniR4.M2.setPower(speed);
      }
    } else {
      MiniR4.M2.setPower(0);
    }
  }
}

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
