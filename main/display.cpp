#include "display.h"

void displayData(){
  // Put your display functions here. The first one MUST clear display, while those after that MUST NOT clear the display
  showBattPercentage(false);
  showInternalClock(false);
  showLaser1Dist(LEFT, 1, 1, false);
  showLaser2Dist(LEFT, 2, 1, false);
  showIMU(IMU_ORIGIN, LEFT, 3, 1, false);
  // showNearestColour(RIGHT, 1, 1, false);
  showSteeringOC1(RIGHT, 3, 1, false);
}