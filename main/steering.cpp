#include "steering.h"

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