#include "display.h"

void displayData(){
  // Put your display functions here. The first one MUST clear display, while those after that MUST NOT clear the display
  showBattPercentage(true);
  showInternalClock(false);
  showLaserDist(I2CPORT1, RIGHT, 2, 1, false);
  showLaserDist(I2CPORT2, RIGHT, 3, 1, false);
}