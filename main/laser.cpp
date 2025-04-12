#include <vector>
#include "laser.h"

double laser1Mean;
double laser2Mean;

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

int getLaser1DistMean(int max_element){
  static int sum = 0;
  static std::vector<int> dist_arr;
  int curr_dist = 0;
  while (dist_arr.size() < max_element){
    curr_dist = getLaserDist(I2CPORT1);
    dist_arr.push_back(curr_dist);
    sum += curr_dist;
  }
  sum -= dist_arr.at(0);
  dist_arr.erase(dist_arr.begin());
  curr_dist = getLaserDist(I2CPORT1);
  sum += curr_dist;
  dist_arr.push_back(getLaserDist(I2CPORT1));
  return sum / max_element;
}

int getLaser2DistMean(int max_element){
  static int sum = 0;
  static std::vector<int> dist_arr;
  int curr_dist = 0;
  while (dist_arr.size() < max_element){
    curr_dist = getLaserDist(I2CPORT2);
    dist_arr.push_back(curr_dist);
    sum += curr_dist;
  }
  sum -= dist_arr.at(0);
  dist_arr.erase(dist_arr.begin());
  curr_dist = getLaserDist(I2CPORT2);
  sum += curr_dist;
  dist_arr.push_back(getLaserDist(I2CPORT2));
  return sum / max_element;
}

void getLasersDistMean(){
  laser1Mean = getLaser1DistMean(20);
  laser2Mean = getLaser2DistMean(20);
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