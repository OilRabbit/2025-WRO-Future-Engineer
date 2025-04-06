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

void showLaser1Dist(COLUMN column, int line_number, int size, bool clearDisplay){
  static String laser_text = "L" + String(I2CPORT1 + 1) + ":" + String(getLaserDist(I2CPORT1)) + "mm";
  display.oledSetTextColour(BLACK);
  if (column == LEFT){
    display.oledDisplayLeftln(line_number, size, laser_text, clearDisplay);
    display.oledSetTextColour(WHITE);
    laser_text = "L" + String(I2CPORT1 + 1) + ":" + String(getLaserDist(I2CPORT1)) + "mm";
    display.oledDisplayLeftln(line_number, size, laser_text, clearDisplay);
  } else if (column == MID) {
    display.oledDisplayCenterln(line_number, size, laser_text, clearDisplay);
    display.oledSetTextColour(WHITE);
    laser_text = "L" + String(I2CPORT1 + 1) + ":" + String(getLaserDist(I2CPORT1)) + "mm";
    display.oledDisplayCenterln(line_number, size, laser_text, clearDisplay);
  } else {
    display.oledDisplayRightln(line_number, size, laser_text, clearDisplay);
    display.oledSetTextColour(WHITE);
    laser_text = "L" + String(I2CPORT1 + 1) + ":" + String(getLaserDist(I2CPORT1)) + "mm";
    display.oledDisplayRightln(line_number, size, laser_text, clearDisplay);
  }
}

void showLaser2Dist(COLUMN column, int line_number, int size, bool clearDisplay){
  static String laser_text = "L" + String(I2CPORT2 + 1) + ":" + String(getLaserDist(I2CPORT2)) + "mm";
  display.oledSetTextColour(BLACK);
  if (column == LEFT){
    display.oledDisplayLeftln(line_number, size, laser_text, clearDisplay);
    display.oledSetTextColour(WHITE);
    laser_text = "L" + String(I2CPORT2 + 1) + ":" + String(getLaserDist(I2CPORT2)) + "mm";
    display.oledDisplayLeftln(line_number, size, laser_text, clearDisplay);
  } else if (column == MID) {
    display.oledDisplayCenterln(line_number, size, laser_text, clearDisplay);
    display.oledSetTextColour(WHITE);
    laser_text = "L" + String(I2CPORT2 + 1) + ":" + String(getLaserDist(I2CPORT2)) + "mm";
    display.oledDisplayCenterln(line_number, size, laser_text, clearDisplay);
  } else {
    display.oledDisplayRightln(line_number, size, laser_text, clearDisplay);
    display.oledSetTextColour(WHITE);
    laser_text = "L" + String(I2CPORT2 + 1) + ":" + String(getLaserDist(I2CPORT2)) + "mm";
    display.oledDisplayRightln(line_number, size, laser_text, clearDisplay);
  }
}