#include "steering.h"

/* Global variable for storing the steering angle percentage (-100% ~ 100%) */
float steering_percentage;

/**
 * @brief Initialize the servo motor
 * 
 */
void steeringInit(){
  MiniR4.RC3.begin();
}

/**
 * @brief Resetting the servo motor to "zero-pos"
 * 
 */
void reset_steering(){
  MiniR4.RC3.setAngle(90);
}

/**
 * @brief A function which control the steering angle of the servo motor by a given steering angle position
 * 
 * @param steering_percentage; int; Steering angle percentage (-100% ~ 100%)
 */
void steering(int steering_percentage){
  MiniR4.RC3.setAngle(int(90 + MAX_STEERING_ANGLE * -steering_percentage / 100));
}

/**
 * @brief A function to put into display thread for showing the steering angle percentage
 * 
 * @param column; (enum) COLUMN; A enum defined in oled.h indicating which column the data should be displaced at
 * @param line_number; int; The line number where the data should be displaced at (0 ~ 3)
 * @param size; int; The size of the text being displaced (1 ~ 2)
 * @param clearDisplay; bool; Set true to clear the whole OLED display everytime before displaying the battery percentage
 */
void showSteering(COLUMN column, int line_number, int size, bool clearDisplay){
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