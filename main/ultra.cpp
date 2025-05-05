#include "ultra.h"

/* Global variables for storing the ultra distances */
int ultra1Dist;
int ultra2Dist;

// /* Global variables for storing the laser mode and number of elements */
// LASERMODE LaserMode = RAW_DIST;
// int LaserElements = 5;

/**
 * @brief Initialize the laser(s)
 * 
 * @param port; (enum) LASERPORT; The port of the laser
 */
void ultraInit(ULTRA ultra){
  pinMode(ultra.trig, OUTPUT);
  pinMode(ultra.echo, INPUT);
}

// Write two functions to prevent data crashing
/**
 * @brief Get the distance measured by a specific laser
 * 
 * @param port; (enum) LASERPORT; The port of the laser
 * @return int; The distance in mm measured by the laser
 */
int getUltra1Dist(){
  float duration = 0;
  digitalWrite(ultra1.trig, LOW);
  delayMicroseconds(2);
  digitalWrite(ultra1.trig, HIGH);
  delayMicroseconds(2);
  digitalWrite(ultra1.trig, LOW);

  duration = pulseIn(ultra1.echo, HIGH);
  return (duration * 0.343) / 2;
  delay(10);
}

int getUltra2Dist(){
  float duration = 0;
  digitalWrite(ultra2.trig, LOW);
  delayMicroseconds(2);
  digitalWrite(ultra2.trig, HIGH);
  delayMicroseconds(2);
  digitalWrite(ultra2.trig, LOW);

  duration = pulseIn(ultra2.echo, HIGH);
  return (duration * 0.343) / 2;
  delay(10);
}

/**
 * @brief Get the MEAN distance measured by Laser 1 (the one on the LHS)
 * 
 * @param max_element; Number of data taken everytime for calculating the mean distance value
 * @return int; The mean distance in mm measured by the laser
 */
// int getultra1DistMean(int max_element){
//   int sum = 0;
//   std::vector<int> dist_arr;

//   // Collect a fresh dataset each call
//   for (int i = 0; i < max_element; ++i){
//     int curr_dist = getLaserDist(I2CPORT1);
//     dist_arr.push_back(curr_dist);
//     sum += curr_dist;
//   }

//   // Compute and return mean
//   return sum / max_element;
// }
// int getultra1DistMean(int max_element){
//   static int sum = 0;
//   static std::vector<int> dist_arr;
//   int curr_dist = 0;
//   while (dist_arr.size() < max_element){
//     curr_dist = getLaserDist(I2CPORT1);
//     dist_arr.push_back(curr_dist);
//     sum += curr_dist;
//   }
//   sum -= dist_arr.at(0);
//   dist_arr.erase(dist_arr.begin());
//   curr_dist = getLaserDist(I2CPORT1);
//   sum += curr_dist;
//   dist_arr.push_back(curr_dist);
//   return sum / max_element;
// }

/**
 * @brief Get the MEAN distance measured by Laser 2 (the one on the RHS)
 * 
 * @param max_element; Number of data taken everytime for calculating the mean distance value
 * @return int; The mean distance in mm measured by the laser
 */
// int getultra2DistMean(int max_element){
//   int sum = 0;
//   std::vector<int> dist_arr;

//   // Collect a fresh dataset each call
//   for (int i = 0; i < max_element; ++i){
//     int curr_dist = getLaserDist(I2CPORT2);
//     dist_arr.push_back(curr_dist);
//     sum += curr_dist;
//   }

//   // Compute and return mean
//   return sum / max_element;
// }

// int getultra2DistMean(int max_element){
//   static int sum = 0;
//   static std::vector<int> dist_arr;
//   int curr_dist = 0;
//   while (dist_arr.size() < max_element){
//     curr_dist = getLaserDist(I2CPORT2);
//     dist_arr.push_back(curr_dist);
//     sum += curr_dist;
//   }
//   sum -= dist_arr.at(0);
//   dist_arr.erase(dist_arr.begin());
//   curr_dist = getLaserDist(I2CPORT2);
//   sum += curr_dist;
//   dist_arr.push_back(curr_dist);
//   return sum / max_element;
// }

/**
 * @brief Get the MEDIAN distance measured by Laser 1 (the one on the LHS)
 * 
 * @param max_element; Number of data taken everytime for getting the median distance value
 * @return int; The median distance in mm measured by the laser
 */
// int getultra1DistMedian(int max_element){
//   std::vector<int> dist_arr;

//   // Collect fresh data each time
//   for (int i = 0; i < max_element; ++i) {
//     dist_arr.push_back(getLaserDist(I2CPORT1));
//   }

//   // Sort to find the median
//   std::sort(dist_arr.begin(), dist_arr.end());

//   int mid = max_element / 2;
//   if (max_element % 2 == 0) {
//     // Even number of elements: average middle two
//     return (dist_arr[mid - 1] + dist_arr[mid]) / 2;
//   } else {
//     // Odd number of elements: return middle
//     return dist_arr[mid];
//   }
// }
// int getultra1DistMedian(int max_element) {
//   static std::vector<int> dist_arr;
//   int curr_dist = getLaserDist(I2CPORT1);

//   if (dist_arr.size() < max_element) {
//     dist_arr.push_back(curr_dist);
//   } else {
//     dist_arr.erase(dist_arr.begin());  // Remove oldest
//     dist_arr.push_back(curr_dist);     // Add newest
//   }

//   // Make a copy to sort for median
//   std::vector<int> sorted_arr = dist_arr;
//   std::sort(sorted_arr.begin(), sorted_arr.end());

//   int mid = sorted_arr.size() / 2;
//   if (sorted_arr.size() % 2 == 0) {
//     // even number of elements, return average of middle two
//     return (sorted_arr[mid - 1] + sorted_arr[mid]) / 2;
//   } else {
//     // odd number of elements, return middle
//     return sorted_arr[mid];
//   }
// }

/**
 * @brief Get the MEDIAN distance measured by Laser 2 (the one on the RHS)
 * 
 * @param max_element; Number of data taken everytime for getting the median distance value
 * @return int; The median distance in mm measured by the laser
 */
// int getultra2DistMedian(int max_element) {
//   std::vector<int> dist_arr;

//   // Collect fresh data each time
//   for (int i = 0; i < max_element; ++i) {
//     dist_arr.push_back(getLaserDist(I2CPORT2));
//   }

//   // Sort to find the median
//   std::sort(dist_arr.begin(), dist_arr.end());

//   int mid = max_element / 2;
//   if (max_element % 2 == 0) {
//     // Even number of elements: average middle two
//     return (dist_arr[mid - 1] + dist_arr[mid]) / 2;
//   } else {
//     // Odd number of elements: return middle
//     return dist_arr[mid];
//   }
// }
// int getultra2DistMedian(int max_element) {
//   static std::vector<int> dist_arr;
//   int curr_dist = getLaserDist(I2CPORT2);

//   if (dist_arr.size() < max_element) {
//     dist_arr.push_back(curr_dist);
//   } else {
//     dist_arr.erase(dist_arr.begin());  // Remove oldest
//     dist_arr.push_back(curr_dist);     // Add newest
//   }

//   // Make a copy to sort for median
//   std::vector<int> sorted_arr = dist_arr;
//   std::sort(sorted_arr.begin(), sorted_arr.end());

//   int mid = sorted_arr.size() / 2;
//   if (sorted_arr.size() % 2 == 0) {
//     // even number of elements, return average of middle two
//     return (sorted_arr[mid - 1] + sorted_arr[mid]) / 2;
//   } else {
//     // odd number of elements, return middle
//     return sorted_arr[mid];
//   }
// }

// int getultra1DistCistern(int max_element, int dropCount = 1){
//   static std::vector<int> dist_arr;

//   int curr_dist = getLaserDist(I2CPORT1);
  
//   // Maintain sliding window
//   if (dist_arr.size() < max_element) {
//     dist_arr.push_back(curr_dist);
//   } else {
//     std::rotate(dist_arr.begin(), dist_arr.begin() + 1, dist_arr.end());
//     dist_arr.back() = curr_dist;
//   }

//   // Not enough data yet
//   if (dist_arr.size() <= 2 * dropCount) {
//     return curr_dist; 
//   }

//   // Copy data for processing
//   std::vector<int> sorted_arr = dist_arr;
  
//   // Partially sort just enough to remove extremes
//   std::nth_element(sorted_arr.begin(), sorted_arr.begin() + dropCount, sorted_arr.end());
//   std::nth_element(sorted_arr.begin() + dropCount, sorted_arr.end() - dropCount, sorted_arr.end());

//   // Compute mean without extremes
//   int sum = 0;
//   int valid_count = 0;
//   for (int i = dropCount; i < sorted_arr.size() - dropCount; ++i) {
//     sum += sorted_arr[i];
//     valid_count++;
//   }

//   return sum / valid_count;
// }

// Fast cistern filter implementation for Laser sensor readings
// int getultra2DistCistern(int max_element, int dropCount = 1) {
//   static std::vector<int> dist_arr;

//   int curr_dist = getLaserDist(I2CPORT2);
  
//   // Maintain sliding window
//   if (dist_arr.size() < max_element) {
//     dist_arr.push_back(curr_dist);
//   } else {
//     std::rotate(dist_arr.begin(), dist_arr.begin() + 1, dist_arr.end());
//     dist_arr.back() = curr_dist;
//   }

//   // Not enough data yet
//   if (dist_arr.size() <= 2 * dropCount) {
//     return curr_dist; 
//   }

//   // Copy data for processing
//   std::vector<int> sorted_arr = dist_arr;
  
//   // Partially sort just enough to remove extremes
//   std::nth_element(sorted_arr.begin(), sorted_arr.begin() + dropCount, sorted_arr.end());
//   std::nth_element(sorted_arr.begin() + dropCount, sorted_arr.end() - dropCount, sorted_arr.end());

//   // Compute mean without extremes
//   int sum = 0;
//   int valid_count = 0;
//   for (int i = dropCount; i < sorted_arr.size() - dropCount; ++i) {
//     sum += sorted_arr[i];
//     valid_count++;
//   }

//   return sum / valid_count;
// }

// int getFilteredultra1Dist(int threshold){
//   static std::vector<int> dist_arr;
//   int curr_dist = getLaserDist(I2CPORT1);

//   // Maintain last 3 measurements
//   if (dist_arr.size() < 3) {
//     dist_arr.push_back(curr_dist);
//     return curr_dist;  // Not enough data yet, directly return current
//   } else {
//     dist_arr.erase(dist_arr.begin());  
//     dist_arr.push_back(curr_dist);
//   }

//   // Calculate slope based on last two distances
//   int y2 = dist_arr[2];  // latest measurement
//   int y1 = dist_arr[1];  // previous measurement
//   int slope = y2 - y1;   // simple difference as slope (since time interval is constant)

//   // Predict current distance
//   int predicted_dist = y2 + slope;

//   // Check if current distance deviates too much from prediction
//   if (abs(curr_dist - predicted_dist) > threshold) {
//     // Current measurement seems like an outlier; return predicted distance
//     return predicted_dist;
//   } else {
//     // Measurement is acceptable
//     return curr_dist;
//   }
// }

// int getFilteredultra2Dist(int threshold){
//   static std::vector<int> dist_arr;
//   int curr_dist = getLaserDist(I2CPORT2);

//   // Maintain last 3 measurements
//   if (dist_arr.size() < 3) {
//     dist_arr.push_back(curr_dist);
//     return curr_dist;  // Not enough data yet, directly return current
//   } else {
//     dist_arr.erase(dist_arr.begin());  
//     dist_arr.push_back(curr_dist);
//   }

//   // Calculate slope based on last two distances
//   int y2 = dist_arr[2];  // latest measurement
//   int y1 = dist_arr[1];  // previous measurement
//   int slope = y2 - y1;   // simple difference as slope (since time interval is constant)

//   // Predict current distance
//   int predicted_dist = y2 + slope;

//   // Check if current distance deviates too much from prediction
//   if (abs(curr_dist - predicted_dist) > threshold) {
//     // Current measurement seems like an outlier; return predicted distance
//     return predicted_dist;
//   } else {
//     // Measurement is acceptable
//     return curr_dist;
//   }
// }

// int getultra1DistEMA(int alpha_percent = 20) {
//   static int filtered_dist = 0;
//   int curr_dist = getLaserDist(I2CPORT1);

//   // If first time running, initialize
//   if (filtered_dist == 0 && curr_dist != 8191) {
//     filtered_dist = curr_dist;
//   }

//   // If current reading is 8191 → treat as out of range → return 8191 immediately
//   // if (curr_dist == 8191) {
//   //   return 8191;
//   // }

//   if (curr_dist == 8191) curr_dist = 3000;

//   // Update EMA only when reading is valid
//   filtered_dist = (alpha_percent * curr_dist + (100 - alpha_percent) * filtered_dist) / 100;

//   return filtered_dist;
// }


// int getultra2DistEMA(int alpha_percent = 20) {
//   static int filtered_dist = 0;
//   int curr_dist = getLaserDist(I2CPORT2);

//   // If first time running, initialize
//   if (filtered_dist == 0 && curr_dist != 8191) {
//     filtered_dist = curr_dist;
//   }

//   // If current reading is 8191 → treat as out of range → return 8191 immediately
//   // if (curr_dist == 8191) {
//   //   return 8191;
//   // }

//   if (curr_dist == 8191) curr_dist = 5000;

//   // Update EMA only when reading is valid
//   filtered_dist = (alpha_percent * curr_dist + (100 - alpha_percent) * filtered_dist) / 100;

//   return filtered_dist;
// }


/**
 * @brief A threading function for getting the distance measured by Laser 1 (the one on the LHS).
 * 
 * Uses global variables:
 * - LaserMode: (enum) indicating mode of distance calculation (MEANDIST, MEDIANDIST, DEFAULT)
 * - LaserElements: (int) number of data points used for calculation
 * - ultra1Dist: (int) stores measured distance (global output)
 */
void getultra1Distloop() {
  ultra1Dist = getUltra1Dist();
  // int curr_laser = 0;
  // switch (LaserMode) {
  //   case MEAN_DIST:
  //     curr_laser = getultra1DistMean(LaserElements);
  //     break;

  //   case MEDIAN_DIST:
  //     curr_laser = getultra1DistMedian(LaserElements);
  //     break;
    
  //   case CISTERN_DIST:
  //     curr_laser = getultra1DistCistern(LaserElements, 1);
  //     break;
    
  //   case FILTER_DIST:
  //     curr_laser = getFilteredultra1Dist(100);
  //     break;

  //   case EMA_DIST:
  //     curr_laser = getultra1DistEMA(40);
  //     break;

  //   default:
  //     curr_laser = getLaserDist(I2CPORT1);
  //     break;
  // }
  // // ultra1Dist = (curr_laser >= 8190) ? 21 : curr_laser;
  // ultra1Dist = curr_laser;
}


/**
 * @brief A threading function for getting the distance measured by Laser 2 (the one on the RHS).
 * 
 * Uses global variables:
 * - LaserMode: (enum) indicating mode of distance calculation (MEANDIST, MEDIANDIST, DEFAULT)
 * - LaserElements: (int) number of data points used for calculation
 * - ultra2Dist: (int) stores measured distance (global output)
 */
void getultra2Distloop() {
  ultra2Dist = getUltra2Dist();
  // int curr_laser = 0;
  // switch (LaserMode) {
  //   case MEAN_DIST:
  //     curr_laser = getultra2DistMean(LaserElements);
  //     break;

  //   case MEDIAN_DIST:
  //     curr_laser = getultra2DistMedian(LaserElements);
  //     break;

  //   case CISTERN_DIST:
  //     curr_laser = getultra2DistCistern(LaserElements, 1);
  //     break;

  //   case FILTER_DIST:
  //     curr_laser = getFilteredultra2Dist(100);
  //     break;

  //   case EMA_DIST:
  //     curr_laser = getultra2DistEMA(60);
  //     break;

  //   default:
  //     curr_laser = getLaserDist(I2CPORT2);
  //     break;
  // }
  // // ultra2Dist = (curr_laser >= 8190) ? 21 : curr_laser;
  // ultra2Dist = curr_laser;
}


/**
  * @brief A function to put into display thread for showing the distance measured by Laser 1 (the one on the LHS)
  * 
 * @param column; (enum) COLUMN; A enum defined in oled.h indicating which column the data should be displaced at
 * @param line_number; int; The line number where the data should be displaced at (0 ~ 3)
 * @param size; int; The size of the text being displaced (1 ~ 2)
 * @param clearDisplay; bool; Set true to clear the whole OLED display everytime before displaying the battery percentage
  */
void showUltra1Dist(COLUMN column, int line_number, int size, bool clearDisplay){
  static String ultra_text = String("U1:") + String(ultra1Dist) + "mm";
  display.oledSetTextColour(BLACK);
  if (column == LEFT){
    display.oledDisplayLeftln(line_number, size, ultra_text, clearDisplay);
    display.oledSetTextColour(WHITE);
    ultra_text = String("U1:") + String(ultra1Dist) + "mm";
    display.oledDisplayLeftln(line_number, size, ultra_text, clearDisplay);
  } else if (column == MID) {
    display.oledDisplayCenterln(line_number, size, ultra_text, clearDisplay);
    display.oledSetTextColour(WHITE);
    ultra_text =String("U1:") + String(ultra1Dist) + "mm";
    display.oledDisplayCenterln(line_number, size, ultra_text, clearDisplay);
  } else {
    display.oledDisplayRightln(line_number, size, ultra_text, clearDisplay);
    display.oledSetTextColour(WHITE);
    ultra_text = String("U1:") + String(ultra1Dist) + "mm";
    display.oledDisplayRightln(line_number, size, ultra_text, clearDisplay);
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
void showUltra2Dist(COLUMN column, int line_number, int size, bool clearDisplay){
  static String ultra_text = String("U2:") + String(ultra2Dist) + "mm";
  display.oledSetTextColour(BLACK);
  if (column == LEFT){
    display.oledDisplayLeftln(line_number, size, ultra_text, clearDisplay);
    display.oledSetTextColour(WHITE);
    ultra_text = String("U2:") + String(ultra2Dist) + "mm";
    display.oledDisplayLeftln(line_number, size, ultra_text, clearDisplay);
  } else if (column == MID) {
    display.oledDisplayCenterln(line_number, size, ultra_text, clearDisplay);
    display.oledSetTextColour(WHITE);
    ultra_text = String("U2:") + String(ultra2Dist) + "mm";
    display.oledDisplayCenterln(line_number, size, ultra_text, clearDisplay);
  } else {
    display.oledDisplayRightln(line_number, size, ultra_text, clearDisplay);
    display.oledSetTextColour(WHITE);
    ultra_text = String("U2:") + String(ultra2Dist) + "mm";
    display.oledDisplayRightln(line_number, size, ultra_text, clearDisplay);
  }
}

ULTRA ultra1 = {1, 3, 2};
ULTRA ultra2 = {2, 5, 4};
