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
  dist_arr.push_back(curr_dist);
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
  dist_arr.push_back(curr_dist);
  return sum / max_element;
}

int getLaser1DistMedian(int max_element) {
  static std::vector<int> dist_arr;
  int curr_dist = getLaserDist(I2CPORT1);

  if (dist_arr.size() < max_element) {
    dist_arr.push_back(curr_dist);
  } else {
    dist_arr.erase(dist_arr.begin());  // Remove oldest
    dist_arr.push_back(curr_dist);     // Add newest
  }

  // Make a copy to sort for median
  std::vector<int> sorted_arr = dist_arr;
  std::sort(sorted_arr.begin(), sorted_arr.end());

  int mid = sorted_arr.size() / 2;
  if (sorted_arr.size() % 2 == 0) {
    // even number of elements, return average of middle two
    return (sorted_arr[mid - 1] + sorted_arr[mid]) / 2;
  } else {
    // odd number of elements, return middle
    return sorted_arr[mid];
  }
}

int getLaser2DistMedian(int max_element) {
  static std::vector<int> dist_arr;
  int curr_dist = getLaserDist(I2CPORT2);

  if (dist_arr.size() < max_element) {
    dist_arr.push_back(curr_dist);
  } else {
    dist_arr.erase(dist_arr.begin());  // Remove oldest
    dist_arr.push_back(curr_dist);     // Add newest
  }

  // Make a copy to sort for median
  std::vector<int> sorted_arr = dist_arr;
  std::sort(sorted_arr.begin(), sorted_arr.end());

  int mid = sorted_arr.size() / 2;
  if (sorted_arr.size() % 2 == 0) {
    // even number of elements, return average of middle two
    return (sorted_arr[mid - 1] + sorted_arr[mid]) / 2;
  } else {
    // odd number of elements, return middle
    return sorted_arr[mid];
  }
}

void getLaser1Distloop(){
  // laser1Mean = getLaser1DistMean(5);
  laser1Mean = getLaser1DistMedian(5);
}

void getLaser2Distloop(){
  // laser2Mean = getLaser2DistMean(5);
  laser2Mean = getLaser2DistMedian(5);
}

void showLaser1Dist(COLUMN column, int line_number, int size, bool clearDisplay){
  // static String laser_text = "L" + String(I2CPORT1 + 1) + ":" + String(getLaserDist(I2CPORT1)) + "mm";
  static String laser_text = "L" + String(I2CPORT1 + 1) + "M:" + String(laser1Mean) + "mm";
  display.oledSetTextColour(BLACK);
  if (column == LEFT){
    display.oledDisplayLeftln(line_number, size, laser_text, clearDisplay);
    display.oledSetTextColour(WHITE);
    // laser_text = "L" + String(I2CPORT1 + 1) + ":" + String(getLaserDist(I2CPORT1)) + "mm";
    laser_text = "L" + String(I2CPORT1 + 1) + "M:" + String(laser1Mean) + "mm";
    display.oledDisplayLeftln(line_number, size, laser_text, clearDisplay);
  } else if (column == MID) {
    display.oledDisplayCenterln(line_number, size, laser_text, clearDisplay);
    display.oledSetTextColour(WHITE);
    // laser_text = "L" + String(I2CPORT1 + 1) + ":" + String(getLaserDist(I2CPORT1)) + "mm";
    laser_text = "L" + String(I2CPORT1 + 1) + "M:" + String(laser1Mean) + "mm";
    display.oledDisplayCenterln(line_number, size, laser_text, clearDisplay);
  } else {
    display.oledDisplayRightln(line_number, size, laser_text, clearDisplay);
    display.oledSetTextColour(WHITE);
    // laser_text = "L" + String(I2CPORT1 + 1) + ":" + String(getLaserDist(I2CPORT1)) + "mm";
    laser_text = "L" + String(I2CPORT1 + 1) + "M:" + String(laser1Mean) + "mm";
    display.oledDisplayRightln(line_number, size, laser_text, clearDisplay);
  }
}

void showLaser2Dist(COLUMN column, int line_number, int size, bool clearDisplay){
  // static String laser_text = "L" + String(I2CPORT2 + 1) + ":" + String(getLaserDist(I2CPORT2)) + "mm";
  static String laser_text = "L" + String(I2CPORT2 + 1) + "M:" + String(laser2Mean) + "mm";
  display.oledSetTextColour(BLACK);
  if (column == LEFT){
    display.oledDisplayLeftln(line_number, size, laser_text, clearDisplay);
    display.oledSetTextColour(WHITE);
    // laser_text = "L" + String(I2CPORT2 + 1) + ":" + String(getLaserDist(I2CPORT2)) + "mm";
    laser_text = "L" + String(I2CPORT2 + 1) + "M:" + String(laser2Mean) + "mm";
    display.oledDisplayLeftln(line_number, size, laser_text, clearDisplay);
  } else if (column == MID) {
    display.oledDisplayCenterln(line_number, size, laser_text, clearDisplay);
    display.oledSetTextColour(WHITE);
    // laser_text = "L" + String(I2CPORT2 + 1) + ":" + String(getLaserDist(I2CPORT2)) + "mm";
    laser_text = "L" + String(I2CPORT2 + 1) + "M:" + String(laser2Mean) + "mm";
    display.oledDisplayCenterln(line_number, size, laser_text, clearDisplay);
  } else {
    display.oledDisplayRightln(line_number, size, laser_text, clearDisplay);
    display.oledSetTextColour(WHITE);
    // laser_text = "L" + String(I2CPORT2 + 1) + ":" + String(getLaserDist(I2CPORT2)) + "mm";
    laser_text = "L" + String(I2CPORT2 + 1) + "M:" + String(laser2Mean) + "mm";
    display.oledDisplayRightln(line_number, size, laser_text, clearDisplay);
  }
}