#include "MatrixMiniR4.h"
#include <cmath>
#include "Arduino.h"
#include "OC2.h"

int min_xpos_Rcase(int area){
  return -2.8164 * area + 114;
}

int min_xpos_Gcase(int area){
  return 1.9722 * area + 210.81;
}

float xpos2steeringPer(float xpos){
  return 1 / (pow((159 - xpos), 2) + 1);
}

POINT bezier(float t, POINT P0, POINT P1, POINT P2) {
  POINT pt;
  pt.x = (1 - t) * (1 - t) * P0.x + 2 * (1 - t) * t * P1.x + t * t * P2.x;
  pt.y = (1 - t) * (1 - t) * P0.y + 2 * (1 - t) * t * P1.y + t * t * P2.y;
  return pt;
}

float normalizeAngle(float angle) {
  while (angle > PI) angle -= 2 * PI;
  while (angle < -PI) angle += 2 * PI;
  return angle;
}

int computeSteeringAngle(POINT prev, POINT curr, POINT next) {
  float headingCurr = atan2(curr.y - prev.y, curr.x - prev.x);
  float headingNext = atan2(next.y - curr.y, next.x - curr.x);
  float deltaHeading = normalizeAngle(headingNext - headingCurr);
  Serial.println("deltaHeading");
  Serial.println(deltaHeading);

  float angleDeg = -deltaHeading * 180.0 / PI;
  angleDeg = constrain(angleDeg, -33, 33);
  angleDeg /= 33 / 100;

  return angleDeg;
}

/**
 * @brief The main function for OC2
 * If this function is being used, the smart car will turn to the direction immediately after one of the laser sensor detects a MEAN/MEDIAN distance > 1700mm
 * 
 */
long OC2_starttime = 0; 
long OC2_endtime = 0; 
void OC2Huskylens(double right_ang, int dist_threshold, int power){
  static bool first_run = true;
  static bool start_game = false;
  static bool prev_btn_state = false;
  bool curr_btn_state = MiniR4.BTN_DOWN.getState();
  if (curr_btn_state && !prev_btn_state){
    start_game = !start_game;
  }
  prev_btn_state = curr_btn_state;

  static bool reset_OC2 = false;
  static bool end_game = false;
  std::vector<COLOURED_OBJ> pillars_array;
  static bool pillar_detect_phase = true;
  static COLOURED_OBJ pillar_front;
  const int area_dangerzone = 40;
  bool stopping_phase = false;
  const float steering_amp_fact1 = 200;
  const float steering_amp_fact2 = 2;
  
  const float bezier_t_amp_fact = 5.1;
  static float max_steering = 0;
  static int gyro_ang_detect_phase = 0;


  static bool cal_point = false;
  static POINT current_pt = {CAM_MID_XPOS, CAM_LOWEST_YPOS};
  static bool curve_phase1 = false;
  static bool curve_phase2 = false;
  static bool curve_phase3 = false;
  static float t_bezier = 0;
  static int bezier_iter_time = 0;
  static float bezier_checktime = 300;
  static int curve_phase1_gyro_chkpt = 45;

  // static bool pillarPassDash_phase = false;
  // int pillarPass_dashDist = 800;
  // static long dash_timeZero = 0;
  // static long dash_timeZero2 = 0;
  // static long dash_time = 0;
  // static double tar_ang = 0;
  // static bool is_left = false;
  // static bool anticlockwise = false;
  // static bool cal_tar_ang = false;
  // static bool need_turn = false;
  // static bool dash_forward = false;
  // static int num_turn = 0;
  // static bool close_wall = false;
  // static long milliseconds = 0;
  // static int turn_waittime = 150;
  // static int innerWall_dist = 75;
  // static bool wait_turn = false;
  if (!start_game){ 
    // Serial.begin(115200);
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
      end_game = false;
      steering_percentage = 0;
      pillar_detect_phase = true;
      cal_point = false;

      curve_phase1 = false;
      curve_phase2 = false;
      curve_phase3 = false;
      bezier_iter_time = 0;
      t_bezier = 0;
      bezier_checktime = 300;
      curve_phase1_gyro_chkpt = 45;
      max_steering = 0;
      gyro_ang_detect_phase = 0;

      // pillarPass_checkTime = 0;
      // pillarPassDash_phase = false;
      // dash_timeZero = 0;
      // dash_timeZero2 = 0;
      // dash_time = 0;
      // tar_ang = 0;
      // is_left = false;
      // anticlockwise = false;
      // cal_tar_ang = false;
      // need_turn = false;
      // dash_forward = false;
      // num_turn = 0;
      // close_wall = false;
      // milliseconds = 0;
      // innerWall_dist = 75;
      // wait_turn = false;
      OC2_starttime = internalClock.read();
      if (!first_run) resetIMU();
      reset_OC2 = false;
    }
    if (!end_game){
      displayThread.enabled = false;
      OC2_endtime = internalClock.read();
      if (pillar_detect_phase){
        if (nearestPillarGlobal.colour == RED){
          if (nearestPillarGlobal.area >= 10 && nearestPillarGlobal.xpos > min_xpos_Rcase(nearestPillarGlobal.area)){
            pillar_detect_phase = false;
            pillars_array.push_back({nearestPillarGlobal.colour, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area});
            pillar_front = {nearestPillarGlobal.colour, nearestPillarGlobal.xpos, CAM_LOWEST_YPOS - nearestPillarGlobal.ypos, nearestPillarGlobal.height, nearestPillarGlobal.width, nearestPillarGlobal.area};
            gyro_ang_detect_phase = getIMU();
            if (pillar_front.xpos > 230) curve_phase1_gyro_chkpt = 45; // || pillar_front.area < 13
            else curve_phase1_gyro_chkpt = pillar_front.xpos * 45 / 200;
            curve_phase1 = true;
          }
        } else if (nearestPillarGlobal.colour == GREEN){
          if (nearestPillarGlobal.area >= 10 && nearestPillarGlobal.xpos < min_xpos_Gcase(nearestPillarGlobal.area)){
          }
        }
      } else if (curve_phase1){
        if (pillar_front.colour == RED){
          if (abs(getIMU() - gyro_ang_detect_phase) < curve_phase1_gyro_chkpt){
            if (pillar_front.colour == RED){
              if (pillar_front.xpos > 230 || pillar_front.area < 13) steering_percentage = -(60 - (getIMU() - gyro_ang_detect_phase)) * pillar_front.area * steering_amp_fact1 * 320 / (60 * area_dangerzone * pillar_front.xpos);
              else steering_percentage = -(50 - (getIMU() - gyro_ang_detect_phase)) * pillar_front.area * steering_amp_fact1 * 320 / (50 * area_dangerzone * pillar_front.xpos);
            } else {
            }
          } else {
            curve_phase1 = false;
            if (pillar_front.xpos > 230 && pillar_front.area < 30){
              bezier_checktime = 200 * (pillar_front.xpos - 50) / 300;
              curve_phase2 = true;
            } else if (pillar_front.area >= 30){
              bezier_checktime = 150 * (pillar_front.xpos - 50) / 300;
              curve_phase2 = true;
            } else if (pillar_front.area > 13){
              bezier_checktime = 50 * (pillar_front.xpos - 50) / 300;
              curve_phase2 = true;
            } else {
              gyro_ang_detect_phase = getIMU();
              curve_phase3 = true;
            }
            bezier_iter_time = internalClock.read();
          }
        } else {}
      } else if (curve_phase2) {
        if (internalClock.read() - bezier_iter_time <= bezier_checktime){
          steering_percentage = 0;
        } else {
          gyro_ang_detect_phase = getIMU();
          curve_phase2 = false;
          curve_phase3 = true;
        }
      } else if (curve_phase3){
        if (abs(getIMU() - gyro_ang_detect_phase) < curve_phase1_gyro_chkpt){
          if (pillar_front.colour == RED){
            steering_percentage = (abs(getIMU()) + 25) * steering_amp_fact2;
          } else {}
        } else {
          curve_phase3 = false;
          end_game = true;
        }
      }
      
      // else if (dodge_phase){
      //   // Serial.print("dodge_phase\n");
      //   if (nearestPillarGlobal.area >= 10) pillarPass_checkTime = internalClock.read();
      //   if (nearestPillarGlobal.area < 10 && internalClock.read() - pillarPass_checkTime > pillarPass_firmTime) {
      //     steering_percentage = 0;
      //     dodge_phase = false;
      //     pillarPassDash_phase = true;
      //     MiniR4.M2.resetCounter();
      //   } else {
      //     steering_percentage = ceil(xpos2steeringPer(nearestPillarGlobal.xpos) * 100) * (nearestPillarGlobal.area - 10);
      //     Serial.print(steering_percentage);
      //     Serial.print("\n");
      //     steering_percentage = ceil(steering_percentage);
      //     steering_percentage *=  (nearestPillarGlobal.colour == GREEN) ? 1 : -1;
      //   }
      // } else if (pillarPassDash_phase){
      //   // Serial.print("pillarPassDash_phase\n");
      //   if (MiniR4.M2.getDegrees() > pillarPass_dashDist){
      //     pillarPass_dashDist = false;
      //     pillar_detect_phase = true;
      //   }
      // } else pillar_detect_phase = true;


      // if (num_turn == 0){
      //   if (ultra1Dist > dist_threshold && !dash_forward){
      //     anticlockwise = true;
      //     Ultra2Thread.enabled = false;
      //     is_left = true;
      //     wait_turn = true;
      //   } else if (ultra2Dist > dist_threshold && !dash_forward){
      //     anticlockwise = false;
      //     Ultra1Thread.enabled = false;
      //     is_left = false;
      //     wait_turn = true;
      //   } else {
      //     wait_turn = false;
      //     milliseconds = internalClock.read();
      //   }
      // } else {
      //   if (anticlockwise){
      //     if (ultra1Dist > dist_threshold && !dash_forward){
      //       is_left = true;
      //       wait_turn = true;
      //     } else {
      //       wait_turn = false;
      //     }
      //   } else {
      //     if (ultra2Dist > dist_threshold && !dash_forward){
      //       anticlockwise = false;
      //       is_left = false;
      //       wait_turn = true;
      //     } else {
      //       wait_turn = false;
      //       milliseconds = internalClock.read();
      //     }
      //   }
      // }
      // if (num_turn == 0) turn_waittime = 0;
      // else turn_waittime = 150;
      // if (wait_turn) {
      //   if ((internalClock.read() - milliseconds) < turn_waittime) ;
      //   else {
      //     wait_turn = false;
      //     cal_tar_ang = true;
      //   }
      // }
      // if (cal_tar_ang){ // && ((internalClock.read() - milliseconds) > 200)
      //   // MiniR4.Buzzer.Tone(1000, 100);
      //   tar_ang += (is_left == true) ? -right_ang : right_ang; // 88.58
      //   cal_tar_ang = false;
      //   need_turn = true;
      //   close_wall = false;
      //   num_turn++;
      //   // dash_timeZero2 = internalClock.read();
      // }

      // if ((abs(tar_ang - getIMU()) > 15) && (abs(tar_ang) > abs(getIMU())) && need_turn){
      //   steering_percentage = (tar_ang > 0) ? -100 : 100;
      //   steering(steering_percentage);
      //   dash_forward = true;
      //   dash_timeZero = internalClock.read();
      // } else {
      //   need_turn = false;
      //   if (dash_forward){
      //     if (num_turn >= 12) dash_time = 200;
      //     else if (num_turn == 1) dash_time = 1200;
      //     else dash_time = 50;

      //     if (num_turn >= 12) tar_ang += (is_left == true) ? 5 : -5;
      //     if (anticlockwise){
      //       if ((internalClock.read() - dash_timeZero) < dash_time){ // ultra1Dist > dist_threshold || && ultra2Dist > dist_threshold
      //         steering_percentage = (getIMU() - tar_ang) * 3;
      //         steering(steering_percentage);
      //         MiniR4.M2.setPower(power);
      //       } else {
      //         dash_forward = false;
      //       }
      //     } else {
      //       if ((internalClock.read() - dash_timeZero) < dash_time){ // ultra2Dist > dist_threshold || && ultra1Dist > dist_threshold
      //         steering_percentage = (getIMU() - tar_ang) * 3;
      //         steering(steering_percentage);
      //         MiniR4.M2.setPower(power);
      //       } else {
      //         dash_forward = false;
      //       }
      //     }
      //   } else {
      //     // MiniR4.Buzzer.Tone(1000, 100);
      //     if (num_turn == 1) innerWall_dist = 110;
      //     else innerWall_dist = 90;
      //     if (anticlockwise){
      //       if (ultra1Dist < innerWall_dist) steering_percentage = (ultra1Dist - innerWall_dist) * 0.75;
      //       else if (num_turn == 0) steering_percentage = 0; // (ultra1Dist - ultra2Dist) * 0.2;
      //       else if (num_turn < 2 && num_turn != 0) steering_percentage = (ultra1Dist - innerWall_dist) * 0.08;
      //       else steering_percentage = (ultra1Dist - innerWall_dist) * 0.25;
      //       if (steering_percentage > 30 && num_turn != 0) steering_percentage = 30;
      //       steering(steering_percentage);
      //     } else {
      //       if (ultra2Dist < innerWall_dist) steering_percentage = -(ultra2Dist - innerWall_dist) * 0.75;
      //       else if (num_turn == 0) steering_percentage = 0; // (ultra1Dist - ultra2Dist) * 0.2;
      //       else if (num_turn < 2 && num_turn != 0) steering_percentage = -(ultra2Dist - innerWall_dist) * 0.08;
      //       else steering_percentage = -(ultra2Dist - innerWall_dist) * 0.25;
      //       if (steering_percentage < -30 && num_turn != 0) steering_percentage = -30;
      //       steering(steering_percentage);
      //     }
      //   }
      // }
      // if (num_turn >= 12 && !dash_forward){
      //   // MiniR4.Buzzer.Tone(1000, 100);
      //   steering(0);
      //   MiniR4.M2.setPower(0);
      //   end_game = true;
      //   OC2_endtime = internalClock.read();
      // } else {
        // Serial.println(steering_percentage);
        steering(steering_percentage);
        MiniR4.M2.setPower(power);
      // }
    } else {
      steering(0);
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
  OC2Huskylens(85, 1000, -100);
}

/**
 * @brief A function to put into display thread for showing the steering angle percentage
 * 
 * @param column; (enum) COLUMN; A enum defined in oled.h indicating which column the data should be displaced at
 * @param line_number; int; The line number where the data should be displaced at (0 ~ 3)
 * @param size; int; The size of the text being displaced (1 ~ 2)
 * @param clearDisplay; bool; Set true to clear the whole OLED display everytime before displaying the battery percentage
 */
void showSteeringOC2(COLUMN column, int line_number, int size, bool clearDisplay){
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

