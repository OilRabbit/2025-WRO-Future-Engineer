#include "display.h"

/**
 * @brief A threading function for displaying all the necessary data from different sensors.
 * 
 */
void displayData(){
  // Put your display functions here. The first one MUST clear display, while those after that MUST NOT clear the display
  showBattPercentage(false);
  showInternalClock(false);
  showUltra1Dist(LEFT, 1, 1, false);
  showUltra2Dist(LEFT, 2, 1, false);
  showIMU(IMU_ORIGIN, LEFT, 3, 1, false);
  // showNearestColour(RIGHT, 1, 1, false);
  // showSteeringOC1(RIGHT, 3, 1, false);
  showSteeringOC2(RIGHT, 3, 1, false);
  showOC1Time(MID, 0, 1, false);
  showColorType(RIGHT, 1, 1, false);
}