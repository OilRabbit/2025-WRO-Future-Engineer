#include "Arduino.h"
#include "OC1.h"

float steering_percentage;

void OpenChallenge(){
  static long milliseconds = 0;
  static long dash_timeZero = 0;
  static long dash_time = 0;
  static bool reset_timer = true;
  static double tar_ang = 0;
  static bool turn_left = false;
  static bool cal_tar_ang = false;
  static bool need_turn = false;
  static bool dash_forward = false;
  static int num_turn = 0;
  int speed = 100;
  if (getLaserDist(I2CPORT1) > 1200 && !dash_forward){
    turn_left = true;
    if (reset_timer){
      milliseconds = internalClock.read();
      reset_timer = false;
    }
  } else if (getLaserDist(I2CPORT2) > 1200 && !dash_forward){
    turn_left = false;
    if (reset_timer){
      milliseconds = internalClock.read();
      reset_timer = false;
    }
  } else {
    milliseconds = internalClock.read();
    reset_timer = true;
  }
  if (((internalClock.read() - milliseconds) > 300) && cal_tar_ang){
    tar_ang += (turn_left == true) ? -88.0 : 88.0; // 88.58
    cal_tar_ang = false;
    need_turn = true;
    num_turn++;
  }
  if ((abs(tar_ang - getIMU()) > 7) && (abs(tar_ang) > abs(getIMU())) && need_turn){
    // display.oledDisplayRightln(1, 1, String(tar_ang) + String('\0'), true);
    // steering_percentage = abs(tar_ang - getIMU()) / 90 * 100;
    // if (steering_percentage < 37.5){
    //   steering_percentage = 37.5;
    // }
    steering_percentage = (tar_ang > 0) ? -100 : 100;
    steering(steering_percentage);
    dash_forward = true;
    dash_timeZero = internalClock.read();
  } else {
    need_turn = false;
    if (dash_forward){
      if (num_turn >= 12) dash_time = 1000;
      else dash_time = 2000;
      if ((internalClock.read() - dash_timeZero) < dash_time){
        steering_percentage = (getIMU() - tar_ang) * 6;
        steering(steering_percentage);
        MiniR4.M2.setPower(speed);
      } else {
        dash_forward = false;
      }
    } else {
      // steering_percentage = (getLaserDist(I2CPORT1) - getLaserDist(I2CPORT2)) * 100 / (getLaserDist(I2CPORT1) + getLaserDist(I2CPORT2));
      // if (abs(steering_percentage) > 12.5){
      //   steering_percentage = (steering_percentage > 0) ? 1 : -1;
      //   steering_percentage *= 12.5;
      // }
      steering_percentage = (getIMU() - tar_ang) * 6;
      steering(steering_percentage);
      cal_tar_ang = true;
      need_turn = false;
    }
  }
  // display.oledDisplayRightln(1, 1, String(need_turn) + String('\0'), true);
  if (num_turn >= 12 && !dash_forward){
    MiniR4.M2.setPower(0);
  } else{
    MiniR4.M2.setPower(speed);
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