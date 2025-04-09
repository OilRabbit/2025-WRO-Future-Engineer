#include "OC1.h"

float steering_percentage;

void OpenChallenge(){
  int diff = getLaserDist(I2CPORT1) - getLaserDist(I2CPORT2);
  if (diff > 300) steering_percentage = (diff > 0) ? 100 : -100;
  else steering_percentage = (getLaserDist(I2CPORT1) - getLaserDist(I2CPORT2)) * 100 / (getLaserDist(I2CPORT1) + getLaserDist(I2CPORT2));
  steering(steering_percentage);
  // MiniR4.M2.setPower(100);
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