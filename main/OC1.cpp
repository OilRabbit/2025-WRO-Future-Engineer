#include "Arduino.h"
#include "OC1.h"

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
 * @brief UNUSED, function for OC1
 * If this function is being used, the smart car will turn to the direction after one of the laser sensor detects a distance > 1700mm for 300ms
 * 
 */
void OpenChallengeLaser300(LASERMODE laserMode, int laserMaxElements, double right_ang, int dist_threshold, int turn_time, int power){
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
 * @brief UNUSED, function for OC1
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

/**
 * @brief Function for OC1 using Ultrasonic sensors
 * 
 * @param right_ang; double; the value of an "right angle" for the IMU (as the value of IMU is not consistent)
 * @param dist_threshold; int; the threshold that the car sees for turning (in mm)
 * @param power; int; the power of the driving motor (0 ~ -100)
 * 
 */
long OC1_starttime = 0; 
long OC1_endtime = 0; 
void OpenChallengeUltraFilter(double right_ang, int dist_threshold, int power){
  // Disable the huskylens thread to boost the efficiency of this thread
  huskylensThread.enabled = false;

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

  // Variables required for OC1
  static bool end_game = false;           // A flag determining whether the run has ended
  static long dash_timeZero = 0;          // Variable to store the instant time from the internal clock for dashing
  static long dash_time = 0;              // The time required for the car to run before using the ultrasonic sensors for detection again after turning
  static double tar_ang = 0;              // The target angle which the car should be facing
  static bool anticlockwise = false;      // A boolean storing whether the car is racing in clockwise or anti-clockwise direction
  static bool cal_tar_ang = false;        // A state-like boolean determining whether the car should calculate a new target angle or not
  static bool need_turn = false;          // A state-like boolean determining whether the car should make a turning or not
  static bool dash_forward = false;       // A state-like boolean determining whether the car should dash forward without doing any detection with the ultrasonic sensors or not
  static bool wait_turn = false;          // A state-like boolean determining whether the car should wait for turning or not
  static bool last_turn_cali = true;      // A state-like boolean determining whether the car should calibrate the distance to drive before stopping
  static int num_turn = 0;                // Variable storing the number of turns the car has made
  static long turn_waittimeZero = 0;      // Variable to store the instant time from the internal clock for waiting to turning
  static int turn_waittime = 0;           // Variable storing the time in ms that required to wait before the car turn 
  static int innerWall_dist = 110;        // The distance with the inner wall in mm that the car should keep
  static int start_sector_starttime = 0;  // Variable storing the instant time from the internal clock when the run starts 
  static int start_sector_endtime = 0;    // Variable storing the instant time from the internal clock when the run ends
  static bool reset_OC1 = false;          // A flag determining whether the all the variables should be reseted
  static int imu_threshold = 15;
  int motor_power = power;

  // MiniR4.M2.setBrake(true);

  if (!start_game){ 
    reset_OC1 = true;
    steering(0);
    MiniR4.M2.setPower(0);
    displayThread.enabled = true;
    Ultra1Thread.enabled = true;
    Ultra2Thread.enabled = true;
    OC1Thread.setInterval(10);
    Ultra1Thread.setInterval(10);
    Ultra2Thread.setInterval(10);
  } else {
    if (reset_OC1){
      OC1Thread.setInterval(0);
      Ultra1Thread.setInterval(0);
      Ultra2Thread.setInterval(0);
      end_game = false;
      dash_timeZero = 0;
      dash_time = 0;
      tar_ang = 0;
      anticlockwise = false;
      cal_tar_ang = false;
      need_turn = false;
      dash_forward = false;
      wait_turn = false;
      last_turn_cali = true;
      num_turn = 0;
      turn_waittimeZero = 0;
      innerWall_dist = 110;
      start_sector_starttime = 0;
      start_sector_endtime = 0;
      steering_percentage = 0;
      OC1_starttime = internalClock.read();
      if (!first_run) resetIMU();
      reset_OC1 = false;
    }
    if (!end_game){
      MiniR4.M2.setBrake(false);
      // Disable the display thread before for the rest of the run until it ends
      displayThread.enabled = false;

      // Before the first turn, the car does not know which direction it is racing, so both ultrasonic sensors should be unlocked
      if (num_turn == 0){
        // Unlock one of the sensor after ensuring the direction of the race to boost sensitivity of the lock one
        if (ultra1Dist > dist_threshold && !dash_forward){
          anticlockwise = true;
          Ultra2Thread.enabled = false;
          wait_turn = true;
        } else if (ultra2Dist > dist_threshold && !dash_forward){
          anticlockwise = false;
          Ultra1Thread.enabled = false;
          wait_turn = true;
        } else {
          wait_turn = false;
          turn_waittimeZero = internalClock.read();
        }
      } else {
        if (anticlockwise){
          if (ultra1Dist > dist_threshold && !dash_forward){
            imu_threshold = 20;
            wait_turn = true;
          } else {
            wait_turn = false;
            turn_waittimeZero = internalClock.read();
          }
        } else {
          if (ultra2Dist > dist_threshold && !dash_forward){
            imu_threshold = 10;
            anticlockwise = false;
            wait_turn = true;
          } else {
            wait_turn = false;
            turn_waittimeZero = internalClock.read();
          }
        }
      }

      // EDITABLE: To control the time (in ms) for the car to dash forward before turning, edit the value of turn_waittime in follow two line
      if (num_turn == 0) turn_waittime = 0;
      else turn_waittime = 100;
      // END OF EDITABLE

      if (wait_turn) {
        if ((internalClock.read() - turn_waittimeZero) < turn_waittime) {
        } else {
          wait_turn = false;
          cal_tar_ang = true;
        }
      }
      // Calculate the target angle
      if (cal_tar_ang){ // && ((internalClock.read() - turn_waittimeZero) > 200)
        tar_ang += (anticlockwise) ? -right_ang : right_ang;
        cal_tar_ang = false;
        need_turn = true;
        // After turn 4, calculate the total time required to do the next turn, getting prepared for the last sector
        if (num_turn == 4) start_sector_endtime = internalClock.read();
        num_turn++;
      }

      // Turning
      // EDITABLE: If the car is turning too less/much, edit the value "15" in the following line
      if ((abs(tar_ang) - abs(getIMU()) > imu_threshold) && (abs(tar_ang) > abs(getIMU())) && need_turn){
        if (!anticlockwise){
          motor_power = -100;
          MiniR4.M2.setPower(motor_power);
        }
        
        // Serial.println(abs(tar_ang) > abs(getIMU()));
      // END OF EDITABLE
        steering_percentage = (tar_ang > 0) ? -100 : 100;
        steering(steering_percentage);
        dash_forward = true;
        dash_timeZero = internalClock.read();
        if (num_turn == 4) start_sector_starttime = internalClock.read();
      } else {
        motor_power = -100;
        need_turn = false;
        // Dash forward for a while after turning to avoid re-activate the turning state
        if (dash_forward){
          // end_game = true;
          if (num_turn >= 12) dash_time = (start_sector_endtime - start_sector_starttime) / 2;
          // EDITABLE: If the dash time after turning is too short/long, edit the value of dash_time in the following two lines
          else if (num_turn == 1) dash_time = 1200;
          else dash_time = 150;
          // END OF EDITABLE

          if (num_turn >= 12 && last_turn_cali) {
            tar_ang += (anticlockwise) ? 5 : -5;
            last_turn_cali = false;
          }
          if (anticlockwise){
            if ((internalClock.read() - dash_timeZero) < dash_time){ // ultra1Dist > dist_threshold || && ultra2Dist > dist_threshold
              // EDITABLEL: If the car is not moving straigtly, edit the kp value in the following line
              steering_percentage = (getIMU() - tar_ang) * 3;
              // END OF EDITABLE
              steering(steering_percentage);
              MiniR4.M2.setPower(motor_power);
            } else {
              dash_forward = false;
            }
          } else {
            if ((internalClock.read() - dash_timeZero) < dash_time){ // ultra2Dist > dist_threshold || && ultra1Dist > dist_threshold
              // EDITABLEL: If the car is not moving straigtly, edit the kp value in the following line
              steering_percentage = (getIMU() - tar_ang) * 3;
              // END OF EDITABLE
              steering(steering_percentage);
              MiniR4.M2.setPower(motor_power);
            } else {
              dash_forward = false;
            }
          }
        } else {
          // Keep the car in a safe distance with the inner wall using PID with values from the active ultrasonic sensor
          // EDITABLE: If the car is too close/far from the inner wall, edit the value of innerWall_dist in the following two lines
          if (anticlockwise){
            if (num_turn == 1) innerWall_dist = 110;
            else innerWall_dist = 120;
          } else {
            if (num_turn == 1) innerWall_dist = 140;
            else innerWall_dist = 90;
          }

          // END OF EDITABLE
          if (anticlockwise){
            // EDITABLE: If the car is not moving straigtly, edit the kp value in the following lines
            if (ultra1Dist < (innerWall_dist + 50)) steering_percentage = (ultra1Dist - innerWall_dist) * 0.65;
            else if (num_turn == 0) steering_percentage = getIMU() * 3; // (ultra1Dist - ultra2Dist) * 0.2;
            else if (num_turn < 2 && num_turn != 0) steering_percentage = (ultra1Dist - innerWall_dist) * 0.08;
            else steering_percentage = (ultra1Dist - innerWall_dist) * 0.5;
            // END OF EDITABLE
            if (steering_percentage > 30 && num_turn != 0) steering_percentage = 30;
            steering(steering_percentage);
          } else {
            // EDITABLE: If the car is not moving straigtly, edit the kp value in the following lines
            if (ultra2Dist < innerWall_dist) steering_percentage = -(ultra2Dist - innerWall_dist) * 1.2;
            else if (num_turn == 0) steering_percentage = getIMU() * 3; // (ultra1Dist - ultra2Dist) * 0.2;
            else if (num_turn < 2 && num_turn != 0) steering_percentage = -(ultra2Dist - innerWall_dist) * 0.3;
            else steering_percentage = -(ultra2Dist - innerWall_dist) * 0.50;
            // END OF EDITABLE
            if (steering_percentage < -30 && num_turn != 0) steering_percentage = -30;
            steering(steering_percentage);
          }
        }
      }
      // End of race case
      if (num_turn >= 12 && !dash_forward){
        MiniR4.M2.setBrake(true);
        steering(0);
        MiniR4.M2.setPower(0);
        end_game = true;
        first_run = false;
        OC1_endtime = internalClock.read();
      } else {
        MiniR4.M2.setPower(motor_power);
      }
      OC1_endtime = internalClock.read();
    } else {
      // Stop the car after the end of race
      steering(0);
      MiniR4.M2.setPower(0);
      MiniR4.M2.setBrake(true);
      displayThread.enabled = true;
      Ultra1Thread.enabled = true;
      Ultra2Thread.enabled = true;
    }
  }
}

/**
 * @brief The main thread function for OC1
 * 
 */
void OC1main(){
  OpenChallengeUltraFilter(84, 1000, -100);
  // LaserMode = CISTERN_DIST;
  // LaserElements = 8;
  // OpenChallenge300(MEDIAN_DIST, 10, 88.58, 1500, 300, -100);
  // OpenChallengeLaserFilter(LaserMode, LaserElements, 90, 2500, -100);
}

/**
 * @brief The display function for OC1 showing the time required to finish a race
 * 
 */
void showOC1Time(COLUMN column, int line_number, int size, bool clearDisplay){
  static String curr_time_text;
  display.oledSetTextColour(BLACK);

  long total_ms = OC1_endtime - OC1_starttime;
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


// void OpenChallengeCSMoving(double right_ang, int dist_threshold, int power){
//   static bool first_run = true;
//   static bool start_game = false;
//   static bool prev_btn_state = false;
//   bool curr_btn_state = MiniR4.BTN_DOWN.getState();

//   if (curr_btn_state && !prev_btn_state){
//     start_game = !start_game;
//   }

//   static bool end_game = false;
//   static long dash_timeZero = 0;
//   static long dash_timeZero2 = 0;
//   static long dash_time = 0;
//   static double tar_ang = 0;
//   static bool is_left = false;
//   static bool anticlockwise = false;
//   static bool cal_tar_ang = false;
//   static bool need_turn = false;
//   static bool dash_forward = false;
//   static int num_turn = 0;
//   static long milliseconds = 0;
//   static bool reset_OC1 = false;
//   static int turn_waittime = 0;
//   // static int innerWall_dist = 75;
//   static bool wait_turn = false;
//   static int start_sector_starttime = 0;
//   static int start_sector_endtime = 0;
//   static bool last_turn_cali = true;
//   static bool forward_b4_turn = false;

//   MiniR4.M2.setPower(0);

//   if (!start_game){ 
//     reset_OC1 = true;
//     steering(0);
//     MiniR4.M2.setPower(0);
//     displayThread.enabled = true;
//     Ultra1Thread.enabled = true;
//     Ultra2Thread.enabled = true;
//     ColorThread.enabled = true;
//     OC1Thread.setInterval(10);
//     Ultra1Thread.setInterval(10);
//     Ultra2Thread.setInterval(10);
//     ColorThread.setInterval(10);
//   } else {
//     if(reset_OC1){
//       OC1Thread.setInterval(0);
//       Ultra1Thread.setInterval(0);
//       Ultra2Thread.setInterval(0);
//       ColorThread.setInterval(0);
//       end_game = false;
//       dash_timeZero = 0;
//       dash_timeZero2 = 0;
//       dash_time = 0;
//       tar_ang = 0;
//       is_left = false;
//       anticlockwise = false;
//       cal_tar_ang = false;
//       need_turn = false;
//       dash_forward = false;
//       num_turn = 0;
//       steering_percentage = 0;
//       milliseconds = 0;
//       // innerWall_dist = 75;
//       wait_turn = false;
//       start_sector_starttime = 0;
//       start_sector_endtime = 0;
//       last_turn_cali = true;
//       OC1_starttime = internalClock.read();
//       if (!first_run) resetIMU();
//       reset_OC1 = false;
//     }
//     if (!end_game){
//       displayThread.enabled = false;
//       if (num_turn == 0){
//         if (getColorType() == 4 && !dash_forward){
//           anticlockwise = true;
//           is_left = true;
//           wait_turn = true;
//         } else if (getColorType() == 9 && !dash_forward){
//           anticlockwise = false;
//           is_left = false;
//           wait_turn = true;
//         } else{
//           wait_turn = false ;
//           milliseconds = internalClock.read();       
//         }
//       } else{
//         if(anticlockwise){
//           if (getColorType() == 4 && !dash_forward){
//             is_left = true;
//             wait_turn = true;
//           } else {
//             wait_turn = false;
//             milliseconds = internalClock.read();
//           }
//         } else {
//           if (getColorType() == 9 && !dash_forward){
//             anticlockwise = false;
//             is_left = false;
//             wait_turn = true;
//           } else {
//             wait_turn = false;
//             milliseconds = internalClock.read();
//           }
//         }
//       }
//       if (anticlockwise){
//         if (ultra1Dist >= 700) turn_waittime = 0;
//         else turn_waittime = (((390 - 0.58 * ultra1Dist))*1000) / 730;
//       } else {
//         if (ultra2Dist >= 700) turn_waittime = 0;
//         else turn_waittime = (((390 - 0.58 * ultra1Dist))*1000) / 730;
//       }
//     }
//   }
// }