#include "steering.h"

void steeringInit(){
  MiniR4.RC3.begin();
}

void reset_steering(){
  MiniR4.RC3.setAngle(90);
}

void steering(int steering_percentage){
  MiniR4.RC3.setAngle(int(90 + MAX_STEERING_ANGLE * -steering_percentage / 100));
}