#include "ultra.h"

#define U1_TRIG 10
#define U1_ECHO 11

#define U2_TRIG 47
#define U2_ECHO 48

/* Global variables for storing the ultra distances */
int ultra1Dist;
int ultra2Dist;

NewPing ultra1(U1_TRIG, U1_ECHO, 250);
NewPing ultra2(U2_TRIG, U2_ECHO, 250);

/**
 * @brief Initialize the laser(s)
 */
void ultraInit(){
  // Basically do nothing since we are using the NewPing libraries
}

// Write two functions to prevent data crashing
/**
 * @brief Get the distance measured by a specific laser
 * 
 * @param port; (enum) LASERPORT; The port of the laser
 * @return int; The distance in mm measured by the laser
 */
int getUltra1Dist(int max_dist_cm = 300){
  return ultra1.ping_cm(max_dist_cm);
}

int getUltra1DistMedian(int iter = 10, int max_dist_cm = 300){
  float temp = 25.0; // Temperature in Celsius (this value would probably come from a temperature sensor).
  float factor = sqrt(1 + temp / 273.15) / 60.368; // Speed of sound calculation based on temperature.
  return (float)ultra1.ping_median(iter, max_dist_cm) * factor;
}

/**
 * @brief A threading function for getting the distance measured by Laser 1 (the one on the LHS).
 * 
 * Uses global variables:
 * - LaserMode: (enum) indicating mode of distance calculation (MEANDIST, MEDIANDIST, DEFAULT)
 * - LaserElements: (int) number of data points used for calculation
 * - ultra1Dist: (int) stores measured distance (global output)
 */
void getultra1Distloop(void *parameters){
  while(1){
    ultra1Dist = getUltra1Dist(300);

    // If you choose to use Median Filter, 5 iteration and 5ms after each detection is the best
    ultra1Dist = getUltra1DistMedian(5, 300);
    vTaskDelay(5 / portTICK_PERIOD_MS);
  }
}

/**
  * @brief A function to put into display thread for showing the distance measured by Laser 1 (the one on the LHS)
  * 
 * @param column; (enum) COLUMN; A enum defined in oled.h indicating which column the data should be displaced at
 * @param line_number; int; The line number where the data should be displaced at (0 ~ 3)
 * @param size; int; The size of the text being displaced (1 ~ 2)
 * @param clearDisplay; bool; Set true to clear the whole OLED display everytime before displaying the battery percentage
  */
void showUltra1Dist(TFT_COLUMN column, int line_number, int text_size, uint16_t text_colour = TFT_WHITE, bool clearDisplay = false){
  String ultra_text = String("U1:") + String(ultra1Dist) + "cm";
  if (column == TFT_LEFT_CLN){
    tft.clearln(TFT_LEFT_CLN, line_number);
    tft.displayLeftln(line_number, text_size, ultra_text.c_str(), text_colour, false);
  } else {
    tft.clearln(TFT_RIGHT_CLN, line_number);
    tft.displayRightln(line_number, text_size, ultra_text.c_str(), text_colour, false);
  }
}

// Write two functions to prevent data crashing
/**
 * @brief Get the distance measured by a specific laser
 * 
 * @param port; (enum) LASERPORT; The port of the laser
 * @return int; The distance in mm measured by the laser
 */
int getUltra2Dist(int max_dist_cm = 300){
  return ultra2.ping_cm(max_dist_cm);
}

int getUltra2DistMedian(int iter = 10, int max_dist_cm = 300){
  float temp = 25.0; // Temperature in Celsius (this value would probably come from a temperature sensor).
  float factor = sqrt(1 + temp / 273.15) / 60.368; // Speed of sound calculation based on temperature.
  return (float)ultra2.ping_median(iter, max_dist_cm) * factor;
}

/**
 * @brief A threading function for getting the distance measured by Laser 1 (the one on the LHS).
 * 
 * Uses global variables:
 * - LaserMode: (enum) indicating mode of distance calculation (MEANDIST, MEDIANDIST, DEFAULT)
 * - LaserElements: (int) number of data points used for calculation
 * - ultra1Dist: (int) stores measured distance (global output)
 */
void getultra2Distloop(void *parameters){
  while(1){
    // ultra1Dist = getUltra2Dist(300);

    // If you choose to use Median Filter, 5 iteration and 5ms after each detection is the best
    ultra2Dist = getUltra2DistMedian(5, 300);
    vTaskDelay(5 / portTICK_PERIOD_MS);
  }
}

/**
  * @brief A function to put into display thread for showing the distance measured by Laser 1 (the one on the LHS)
  * 
 * @param column; (enum) COLUMN; A enum defined in oled.h indicating which column the data should be displaced at
 * @param line_number; int; The line number where the data should be displaced at (0 ~ 3)
 * @param size; int; The size of the text being displaced (1 ~ 2)
 * @param clearDisplay; bool; Set true to clear the whole OLED display everytime before displaying the battery percentage
  */
void showUltra2Dist(TFT_COLUMN column, int line_number, int text_size, uint16_t text_colour = TFT_WHITE, bool clearDisplay = false){
  String ultra_text = String("U2:") + String(ultra2Dist) + "cm";
  if (column == TFT_LEFT_CLN){
    tft.clearln(TFT_LEFT_CLN, line_number);
    tft.displayLeftln(line_number, text_size, ultra_text.c_str(), text_colour, false);
  } else {
    tft.clearln(TFT_RIGHT_CLN, line_number);
    tft.displayRightln(line_number, text_size, ultra_text.c_str(), text_colour, false);
  }
}

