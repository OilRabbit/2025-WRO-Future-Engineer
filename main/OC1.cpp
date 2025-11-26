#include "OC1.h"

// A global variables for OC1
bool run_OC1 = false;
bool reset_OC1 = true;
long OC1_starttime = 0; 
long OC1_endtime = 0; 

// Functions for safely suspend and resume threads
static inline void safeSuspend(TaskHandle_t h){ if (h) vTaskSuspend(h); }
static inline void safeResume(TaskHandle_t h){ if (h) vTaskResume(h); }

/**
 * @brief Function for OC1 using ToF sensors over the run
 * 
 * @param right_ang; double; the value of an "right angle" for the IMU (if the value returned by the IMU is not consistent)
 * @param dist_threshold; int; the threshold that the ToFs consider as a turnable corner (in mm)
 * @param power; int; the power of the driving motor (0 ~ -100)
 * 
 */
void OC1_tof(double right_ang, int dist_threshold, int power){
  // Disable unecessary threads for efficiency
  // Pixy thread cannot be disabled as it is sharing the same I2C address with the IMU
  safeSuspend(blinkledThread);
  safeSuspend(OC2Thread);

  // Variables required for OC1
  static OC1_STATES state = DETECT_STATE;     
  static OC1_STATES prev_state = DETECT_STATE;
  static bool end_game = false;           
  static double tar_ang = 0;              
  static bool is_anticlockwise = false;   
  static int num_turn = 0;                
  static int innerWall_dist = 15;        
  static long dash_time = 0;
  static long dash_timeZero = 0;
  float imu_kp = 2;
  String OC1_text = String("oc1:");
  float start_sector_dist = 0;

  // Initializing variables for this function
  if (reset_OC1){
    state = DETECT_STATE;
    prev_state = DETECT_STATE;
    tar_ang = 0;
    is_anticlockwise = false;
    num_turn = 0;
    innerWall_dist = 15;
    steering_percentage = 0;
    dash_time = 0;
    dash_timeZero = 0;
    start_sector_dist = 0;
    end_game = false;
    OC1_starttime = internalClock.read();
    imu_resetYaw();
    reset_OC1 = false;
  }

  if (!end_game){
    // Constant speed for the whole run
    motor_move(power);

    // Finite State Machine of this funciton
    switch(state){
      // State to detect for a turnable corner
      case DETECT_STATE:
        Serial.println("DETECT_STATE");
        Serial.print("num_turn = ");
        Serial.println(num_turn);
        // The direction of the run will be determined in the first sector
        if (num_turn == 0) {
          if (dist_t1 > dist_threshold){
            is_anticlockwise = true;
            safeSuspend(ToF2Thread);
            prev_state = DETECT_STATE;
            state = WAIT_TURN_STATE;
            break;
          } else if (dist_t2 > dist_threshold){
            is_anticlockwise = false;
            safeSuspend(ToF1Thread);
            prev_state = DETECT_STATE;
            state = WAIT_TURN_STATE;
            break;
          } else {
            steering_percentage = -(imu_yaw - tar_ang) * imu_kp;
          }
        } else {
          if (is_anticlockwise){
            if (dist_t1 > dist_threshold){
              prev_state = DETECT_STATE;
              state = WAIT_TURN_STATE;
              break;
            } else {
              steering_percentage = -(imu_yaw - tar_ang) * imu_kp;
            }
          } else {
            if (dist_t2 > dist_threshold){
              prev_state = DETECT_STATE;
              state = WAIT_TURN_STATE;
              break;
            } else {
              steering_percentage = -(imu_yaw - tar_ang) * imu_kp;
            }
          }
        }
        break;

      // Calculate the target yaw angle for the IMU
      case WAIT_TURN_STATE:
        Serial.println("WAIT_TURN_STATE");
        tar_ang += (is_anticlockwise == true) ? -right_ang : right_ang;
        num_turn++;
        if (num_turn == 5) start_sector_dist = abs(MOTOR_ENCODER_COUNT);
        prev_state = WAIT_TURN_STATE;
        state = TURNING_STATE;
        break;
      
      // Turn to the target angle
      case TURNING_STATE:
        Serial.println("TURNING_STATE");
        if (num_turn == 4) reset_encoder();
        if ((abs(tar_ang - imu_yaw) > 16) && (abs(tar_ang) > abs(imu_yaw))){
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
      
      // Dash forward for a certain amount of time to pervent false detection by the ToF
      case DASH_AFTER_TURNING_STATE:
        Serial.println("DASH_AFTER_TURNING_STATE");
        if (num_turn >= 12){
          prev_state = DASH_AFTER_TURNING_STATE;
          reset_encoder();
          state = ENDING_STATE;
          break;
        } else {
          if (is_anticlockwise){
            if ((internalClock.read() - dash_timeZero) < dash_time){
              steering_percentage = -(imu_yaw - tar_ang) * imu_kp;
            } else {
              prev_state = DASH_AFTER_TURNING_STATE;
              state = DETECT_STATE;
              break;
            }
          } else {
            if ((internalClock.read() - dash_timeZero) < dash_time){
              steering_percentage = -(imu_yaw - tar_ang) * imu_kp;
            } else {
              prev_state = DASH_AFTER_TURNING_STATE;
              state = DETECT_STATE;
              break;
            }
          }
        }
        break;

      // End the run by braking the car
      case ENDING_STATE:
        Serial.println("ENDING_STATE");
        if (abs(MOTOR_ENCODER_COUNT) < start_sector_dist / 2){
          steering_percentage = -(imu_yaw - tar_ang) * imu_kp;
        } else {
          motor_stop(BRAKE);
          steering_percentage = 0;
          safeResume(blinkledThread);
          end_game = true;
          OC1_endtime = internalClock.read();
        }
        break;
    }
  }
  vTaskDelay(5 / portTICK_PERIOD_MS);
}

/**
 * @brief An agressive function for OC1. Fast but not stable
 * 
 * @param right_ang; double; the value of an "right angle" for the IMU (if the value returned by the IMU is not consistent)
 * @param dist_threshold; int; the threshold that the ToFs consider as a turnable corner (in mm)
 * @param power; int; the power of the driving motor (0 ~ -100)
 * 
 */
void OC1_fixed(double right_ang, int dist_threshold, int power, int hypower){
  // Disable unecessary threads for efficiency
  // Pixy thread cannot be disabled as it is sharing the same I2C address with the IMU
  safeSuspend(blinkledThread);
  safeSuspend(OC2Thread);

  // Variables required for OC1
  static OC1_STATES state = DETECT_STATE;    
  static OC1_STATES prev_state = DETECT_STATE;
  static bool end_game = false;           
  static double tar_ang = 0;             
  static bool is_anticlockwise = false;  
  static int num_turn = 0;               
  static long sector1_length = 0;
  static long sector2_length = 0;
  static bool left_init_close_wall = false;
  static bool right_init_close_wall = false;
  float imu_kp = 2;
  float imu_kp_fast = 1.6;
  String OC1_text = String("oc1:");

  // Initializing variables for this function
  if (reset_OC1){
    left_init_close_wall = false;
    right_init_close_wall = false;
    tar_ang = 0;
    is_anticlockwise = false;
    num_turn = 0;
    sector1_length = 0;
    sector2_length = 0;
    steering_percentage = 0;
    if (dist_t1 < 15) left_init_close_wall = true;
    if (dist_t2 < 15) right_init_close_wall = true;
    end_game = false;
    OC1_starttime = internalClock.read();
    imu_resetYaw();
    reset_OC1 = false;
  }

  if (!end_game){
    // Finite State Machine of this funciton
    switch(state){
      // Detect whether there is a turnable corner, and record the encoder value for the sectors (for the first three sectors only)
      case DETECT_STATE:
        Serial.println("DETECT_STATE");
        // The direction of the run will be determined in the first sector
        if (num_turn == 0) {
          motor_move(power);
          if (dist_t1 > dist_threshold){
            is_anticlockwise = true;
            safeSuspend(ToF2Thread);
            prev_state = DETECT_STATE;
            state = WAIT_TURN_STATE;
            break;
          } else if (dist_t2 > dist_threshold){
            is_anticlockwise = false;
            safeSuspend(ToF1Thread);
            prev_state = DETECT_STATE;
            state = WAIT_TURN_STATE;
            break;
          } else {
            steering_percentage = -(imu_yaw - tar_ang) * imu_kp; // TODO: Verify whether a minus sign is needed
          }
        } else {
          // The length and the width of the circuit will be determined in the first three sectors
          if (is_anticlockwise){
            if (dist_t1 > dist_threshold){
              if (num_turn == 1) sector1_length += abs(MOTOR_ENCODER_COUNT);
              if (num_turn == 2) sector2_length += abs(MOTOR_ENCODER_COUNT);
              prev_state = DETECT_STATE;
              state = WAIT_TURN_STATE;
              break;
            } else {
              steering_percentage = -(imu_yaw - tar_ang) * imu_kp;
            }
          } else {
            if (dist_t2 > dist_threshold){
              if (num_turn == 1) sector1_length += abs(MOTOR_ENCODER_COUNT);
              if (num_turn == 2) sector2_length += abs(MOTOR_ENCODER_COUNT);
              prev_state = DETECT_STATE;
              state = WAIT_TURN_STATE;
              break;
            } else {
              steering_percentage = -(imu_yaw - tar_ang) * imu_kp;
            }
          }
        }
        break;

      // Move with high speed using acceleration and deceleration for the remaining sectors before the end of the run
      case RUN_SECTOR_STATE:
        if (num_turn % 2 == 1) {
          if (!motor_degree_accel(sector1_length, sector1_length / 2, sector1_length / 2, 10, hypower)) steering_percentage = -(imu_yaw - tar_ang) * imu_kp_fast;
          else {
            state = WAIT_TURN_STATE;
            break;
          }
        } else{
          if (!motor_degree_accel(sector2_length, sector2_length / 2, sector2_length / 2, 10, hypower)) steering_percentage = -(imu_yaw - tar_ang) * imu_kp_fast;
          else {
            state = WAIT_TURN_STATE;
            break;
          }
        }
        break;

      // Adjust the distance requierd to travel for each sector, and calculate the target angle for the IMU
      case WAIT_TURN_STATE:
        Serial.println("WAIT_TURN_STATE");
        motor_move(power);
        tar_ang += (is_anticlockwise == true) ? -right_ang : right_ang;
        num_turn++;
        // if (is_anticlockwise){
        //   if (num_turn == 3) sector2_length -= 40;
        //   if (num_turn == 4) sector2_length -= 25;
        //   if (num_turn == 5) sector1_length -= 35;
        //   if (num_turn == 6) sector2_length -= 15;
        //   // if (num_turn == 7) sector1_length -= 15;
        // }
        prev_state = WAIT_TURN_STATE;
        state = TURNING_STATE;
        break;

      case INTO_SECTOR:
        Serial.println(prev_state);
        Serial.println(" INTO_SECTOR");
        OC1_text = "INTO_SECTOR";
        motor_move(power);
        if (dist_t1 > dist_threshold || dist_t2 > dist_threshold) {
          steering_percentage = -(imu_yaw - (tar_ang)) * imu_kp;
          reset_encoder();
        } else {
          prev_state = INTO_SECTOR;
          state = DETECT_STATE;
          break;
        }
        break;
      
      // Turn to the target angle 
      case TURNING_STATE:
        Serial.println("TURNING_STATE");
        motor_move(power);
        if ((abs(tar_ang - imu_yaw) > 35) && (abs(tar_ang) > abs(imu_yaw))){
          steering_percentage = (tar_ang > 0) ? 100 : -100;
        } else{
          steering_percentage = 0;
          prev_state = TURNING_STATE;
          if (num_turn >= 12) {
            prev_state = TURNING_STATE;
            state = ENDING_STATE;
          } else if (num_turn > 2){
            prev_state = TURNING_STATE;
            state = RUN_SECTOR_STATE;
          } else {
            if (num_turn <= 2) reset_encoder();
            state = INTO_SECTOR;
            break;
          }
          break;
        }
        break;

      // End the run by braking the car
      case ENDING_STATE:
        Serial.println("ENDING_STATE");
        if (!motor_degree_accel(65, 30, 30, 10, 20)) steering_percentage = -(imu_yaw - tar_ang) * imu_kp_fast;
        else {
          motor_stop(BRAKE);
          steering_percentage = 0;
          if (!is_anticlockwise) safeResume(ToF1Thread);
          else safeResume(ToF2Thread);
          end_game = true;
          OC1_endtime = internalClock.read();
        }
        break;

      // A state for us to debug. This will stop the car until the button is being pressed.
      case DEBUG_STATE:
        Serial.println("ENDING_STATE");
        if (is_btn_bumped(TFT_BTN3)){
          state = DETECT_STATE;
          break;
        } else {
          motor_stop(BRAKE);
          break;
        }
        break;
    }
  }
  vTaskDelay(5 / portTICK_PERIOD_MS);
}

/**
 * @brief The main thread function for OC1
 */
void OC1main(void *){
  while (1){
    if (is_btn_bumped(TFT_BTN1)){
      run_OC1 = !run_OC1;
      reset_OC1 = true;
    }
    if (run_OC1){
      // OC1_fixed(90, 85, 10, 11);
      OC1_tof(90, 85, 13);
    } else {
      safeResume(ToF1Thread);
      safeResume(ToF2Thread);
      safeResume(OC2Thread);
      motor_stop(BRAKE);
      steering_percentage = 0;
    }
    vTaskDelay(5 / portTICK_PERIOD_MS);
  }
}

/**
 * @brief Function to print the time and status of OC1 run on the LCD monitor
 * 
 * @param column; TFT_COLUMN; the column on the LCD where the info is being printed
 * @param line_number; int; the line number on the LCD where the info is being printed
 * @param text_size; int; text size of the info on the LCD (1 or 2)
 * @param text_colour; uint16_t; text colour printed on the LCD
 * @param clearDisplay; bool; (UNUSED) whether the display will be cleared before running
 * 
 */
void showOC1Time(TFT_COLUMN column, int line_number, int text_size, uint16_t text_colour = TFT_WHITE, bool clearDisplay = false){
  static long total_ms = 0;
  static long seconds = 0;
  static long milliseconds = 0;
  total_ms = OC1_endtime - OC1_starttime;
  seconds = total_ms / 1000;
  milliseconds = total_ms % 1000;
  String OC1_time_text = String(seconds) + "." + String(milliseconds);
  String OC1_onoff_text = run_OC1 ? "OC1 ON" : "OC1 OFF";
  tft.clearln(TFT_LEFT_CLN, line_number);
  tft.clearln(TFT_RIGHT_CLN, line_number);
  tft.displayLeftln(line_number, text_size, OC1_time_text.c_str(), text_colour, false);
  tft.displayRightln(line_number, text_size, OC1_onoff_text.c_str(), run_OC1 ? TFT_GREEN : TFT_RED, false);
}