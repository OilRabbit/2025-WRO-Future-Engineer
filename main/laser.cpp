#include <vector>
#include "laser.h"

/* Global variables for storing the laser values */
double laser1Mean;
double laser2Mean;

/**
 * @brief Initialize the laser(s)
 * 
 * @param port; (enum) LASERPORT; The port of the laser
 */
void laserInit(LASERPORT port){
  if (port == I2CPORT1) MiniR4.I2C1.MXLaser.begin();
  else MiniR4.I2C2.MXLaser.begin();
}

/**
 * @brief Get the distance measured by a specific laser
 * 
 * @param port; (enum) LASERPORT; The port of the laser
 * @return int; The distance in mm measured by the laser
 */
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

/**
 * @brief Get the MEAN distance measured by Laser 1 (the one on the LHS)
 * 
 * @param max_element; Number of data taken everytime for calculating the mean distance value
 * @return int; The mean distance in mm measured by the laser
 */
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

/**
 * @brief Get the MEAN distance measured by Laser 2 (the one on the RHS)
 * 
 * @param max_element; Number of data taken everytime for calculating the mean distance value
 * @return int; The mean distance in mm measured by the laser
 */
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

/**
 * @brief Get the MEDIAN distance measured by Laser 1 (the one on the LHS)
 * 
 * @param max_element; Number of data taken everytime for getting the median distance value
 * @return int; The median distance in mm measured by the laser
 */
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

/**
 * @brief Get the MEDIAN distance measured by Laser 2 (the one on the RHS)
 * 
 * @param max_element; Number of data taken everytime for getting the median distance value
 * @return int; The median distance in mm measured by the laser
 */
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

/**
 * @brief A threading function for getting the distance measured by Laser 1 (the one on the LHS)
 * 
 */
void getLaser1Distloop(){
  // laser1Mean = getLaser1DistMean(5);
  laser1Mean = getLaser1DistMedian(5);
}

/**
 * @brief A threading function for getting the distance measured by Laser 2 (the one on the RHS)
 * 
 */
void getLaser2Distloop(){
  // laser2Mean = getLaser2DistMean(5);
  laser2Mean = getLaser2DistMedian(5);
}

 /**
  * @brief A function to put into display thread for showing the distance measured by Laser 1 (the one on the LHS)
  * 
 * @param column; (enum) COLUMN; A enum defined in oled.h indicating which column the data should be displaced at
 * @param line_number; int; The line number where the data should be displaced at (0 ~ 3)
 * @param size; int; The size of the text being displaced (1 ~ 2)
 * @param clearDisplay; bool; Set true to clear the whole OLED display everytime before displaying the battery percentage
  */
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

 /**
  * @brief A function to put into display thread for showing the distance measured by Laser 1 (the one on the RHS)
  * 
 * @param column; (enum) COLUMN; A enum defined in oled.h indicating which column the data should be displaced at
 * @param line_number; int; The line number where the data should be displaced at (0 ~ 3)
 * @param size; int; The size of the text being displaced (1 ~ 2)
 * @param clearDisplay; bool; Set true to clear the whole OLED display everytime before displaying the battery percentage
  */
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