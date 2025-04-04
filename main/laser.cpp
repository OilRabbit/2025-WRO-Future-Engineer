#include "laser.h"

void laserInit(LASERPORT port){
  if (port == I2CPORT1) MiniR4.I2C1.MXLaser.begin();
  else MiniR4.I2C2.MXLaser.begin();
}

int getLaserDist(LASERPORT port){
  if (port == I2CPORT1){
    if (MiniR4.I2C1.MXLaser.getDistance() == 8191){
      return 8191;
    } else {
      return MiniR4.I2C1.MXLaser.getDistance();
    }
  } else {
    if (MiniR4.I2C2.MXLaser.getDistance() == 8191){
      return 8191;
    } else {
      return MiniR4.I2C2.MXLaser.getDistance();
    }
  }
}

void showLaserDist(LASERPORT port, COLUMN column, int line_number, int size, bool clearDisplay){
  if (column == LEFT){
    display.oledDisplayLeftln(line_number, size, "L" + String(port + 1) + ":" + String(getLaserDist(port)) + "mm", clearDisplay);
  } else if (column == MID) {
    display.oledDisplayCenterln(line_number, size, "L" + String(port + 1) + ":" + String(getLaserDist(port)) + "mm", clearDisplay);
  } else {
    display.oledDisplayRightln(line_number, size, "L" + String(port + 1) + ":" + String(getLaserDist(port)) + "mm", clearDisplay);
  }
}