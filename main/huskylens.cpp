#include "huskylens.h"

HUSKYLENS huskylens;
movingAvg xCenter(10);
COLOURED_OBJ nearestPillarGlobal;

/**
 * @brief Initialize the huskylens
 * 
 */
void huskylensInit() {
  Wire.begin();
  huskylens.begin(Wire);
  xCenter.begin();
  huskylens.writeAlgorithm(ALGORITHM_COLOR_RECOGNITION);
}

/**
 * @brief A testing function for colour block detection and data displacement for the huskylens
 * 
 * @param column; (enum) COLUMN; A enum defined in oled.h indicating which column the data should be displaced at
 * @param line_number; int; The line number where the data should be displaced at (0 ~ 3)
 * @param size; int; The size of the text being displaced (1 ~ 2)
 * @param clearDisplay; bool; Set true to clear the whole OLED display everytime before displaying the battery percentage
 */
void huskylensColorRegTest(COLUMN column, int line_number, int size, bool clearDisplay) {
  static String huskylens_textR = "\0";

  display.oledSetTextColour(BLACK);
  if (column == LEFT) display.oledDisplayLeftln(line_number, 1, huskylens_textR, clearDisplay);
  else if (column == MID) display.oledDisplayCenterln(line_number, 1, huskylens_textR, clearDisplay);
  else display.oledDisplayRightln(line_number, 1, huskylens_textR, clearDisplay);
  display.oledSetTextColour(WHITE);

  if (!huskylens.request()) huskylens_textR = "Failed to request";
  else if (!huskylens.isLearned()) huskylens_textR = "Not learned";
  else if (!huskylens.available()) huskylens_textR = "No obj";
  else {
    HUSKYLENSResult result = huskylens.read();
    String block = (result.ID == RED) ? "R" : ((result.ID == GREEN) ? "G" : "M");
    huskylens_textR = block + "x: " + String(result.xCenter) + " h:" + String(result.height);
  }

  if (column == LEFT) display.oledDisplayLeftln(line_number, 1, huskylens_textR, clearDisplay);
  else if (column == MID) display.oledDisplayCenterln(line_number, 1, huskylens_textR, clearDisplay);
  else display.oledDisplayRightln(line_number, 1, huskylens_textR, clearDisplay);
}

/**
 * @brief Get the ID of the object from the Huskylens
 * 
 * @return int; the ID of the object
 */
int getHuskyResultID() {
  huskylens.request();
  HUSKYLENSResult result = huskylens.read();
  return result.ID;
}

/**
 * @brief Get the x-pos of the objected detected from the Huskylens
 * 
 * @param colour; (enum) COLOUR_BLOCK; A enum defined in oled.h, indicating the colour which the huskylens may detect
 * @return int; The x-pos of the object detected. Return -1 if nothing is detected
 */
int getHuskyXPos(COLOUR_BLOCK colour) {
  huskylens.request();
  HUSKYLENSResult result = huskylens.read();
  if (result.ID == colour) return result.xCenter;
  else return -1;
}

/**
 * @brief Get the height of the object detected from the Huskylens
 * 
 * @param colour; (enum) COLOUR_BLOCK; A enum defined in oled.h, indicating the colour which the huskylens may detect
 * @return int; The height of the object detected. Return -1 if nothing is detected
 */
int getHuskyHeight(COLOUR_BLOCK colour) {
  huskylens.request();
  HUSKYLENSResult result = huskylens.read();
  if (result.ID == colour) return result.height;
  else return -1;
}

/**
 * @brief A function to put into display thread for showing the info of the CLOSEST RED object detetced (if any)
 * 
 * @param column; (enum) COLUMN; A enum defined in oled.h indicating which column the data should be displaced at
 * @param line_number; int; The line number where the data should be displaced at (0 ~ 3)
 * @param size; int; The size of the text being displaced (1 ~ 2)
 * @param clearDisplay; bool; Set true to clear the whole OLED display everytime before displaying the battery percentage
 */
void showHuskyRed(COLUMN column, int line_number, int size, bool clearDisplay) {
  static String huskylens_textR = "\0";

  display.oledSetTextColour(BLACK);
  if (column == LEFT) display.oledDisplayLeftln(line_number, 1, huskylens_textR, clearDisplay);
  else if (column == MID) display.oledDisplayCenterln(line_number, 1, huskylens_textR, clearDisplay);
  else display.oledDisplayRightln(line_number, 1, huskylens_textR, clearDisplay);

  display.oledSetTextColour(WHITE);
  if (getHuskyResultID() != RED) huskylens_textR = "R: NA";
  else huskylens_textR = "R: " + String(getHuskyXPos(RED)) + ", " + String(getHuskyHeight(RED));
  if (column == LEFT) display.oledDisplayLeftln(line_number, 1, huskylens_textR, clearDisplay);
  else if (column == MID) display.oledDisplayCenterln(line_number, 1, huskylens_textR, clearDisplay);
  else display.oledDisplayRightln(line_number, 1, huskylens_textR, clearDisplay);
}

/**
 * @brief A function to put into display thread for showing the info of the CLOSEST GREEN object detetced (if any)
 * 
 * @param column; (enum) COLUMN; A enum defined in oled.h indicating which column the data should be displaced at
 * @param line_number; int; The line number where the data should be displaced at (0 ~ 3)
 * @param size; int; The size of the text being displaced (1 ~ 2)
 * @param clearDisplay; bool; Set true to clear the whole OLED display everytime before displaying the battery percentage
 */
void showHuskyGreen(COLUMN column, int line_number, int size, bool clearDisplay) {
  static String huskylens_textR = "\0";

  display.oledSetTextColour(BLACK);
  if (column == LEFT) display.oledDisplayLeftln(line_number, 1, huskylens_textR, clearDisplay);
  else if (column == MID) display.oledDisplayCenterln(line_number, 1, huskylens_textR, clearDisplay);
  else display.oledDisplayRightln(line_number, 1, huskylens_textR, clearDisplay);

  display.oledSetTextColour(WHITE);
  if (getHuskyResultID() != GREEN) huskylens_textR = "G: NA";
  else huskylens_textR = "G: " + String(getHuskyXPos(GREEN)) + ", " + String(getHuskyHeight(GREEN));
  if (column == LEFT) display.oledDisplayLeftln(line_number, 1, huskylens_textR, clearDisplay);
  else if (column == MID) display.oledDisplayCenterln(line_number, 1, huskylens_textR, clearDisplay);
  else display.oledDisplayRightln(line_number, 1, huskylens_textR, clearDisplay);
}

/**
 * @brief A function to put into display thread for showing the info of the CLOSEST MAGENTA object detetced (if any)
 * 
 * @param column; (enum) COLUMN; A enum defined in oled.h indicating which column the data should be displaced at
 * @param line_number; int; The line number where the data should be displaced at (0 ~ 3)
 * @param size; int; The size of the text being displaced (1 ~ 2)
 * @param clearDisplay; bool; Set true to clear the whole OLED display everytime before displaying the battery percentage
 */
void showHuskyMagenta(COLUMN column, int line_number, int size, bool clearDisplay) {
  static String huskylens_textR = "\0";

  display.oledSetTextColour(BLACK);
  if (column == LEFT) display.oledDisplayLeftln(line_number, 1, huskylens_textR, clearDisplay);
  else if (column == MID) display.oledDisplayCenterln(line_number, 1, huskylens_textR, clearDisplay);
  else display.oledDisplayRightln(line_number, 1, huskylens_textR, clearDisplay);

  display.oledSetTextColour(WHITE);
  if (getHuskyResultID() != MAGENTA) huskylens_textR = "M: NA";
  else huskylens_textR = "M: " + String(getHuskyXPos(MAGENTA)) + ", " + String(getHuskyHeight(MAGENTA));
  if (column == LEFT) display.oledDisplayLeftln(line_number, 1, huskylens_textR, clearDisplay);
  else if (column == MID) display.oledDisplayCenterln(line_number, 1, huskylens_textR, clearDisplay);
  else display.oledDisplayRightln(line_number, 1, huskylens_textR, clearDisplay);
}

/**
 * @brief Get the CLOSEST object detected
 * 
 * @return COLOURED_OBJ; struct; A struct defined at huskylens.h which store the colour, x-pos and the height of the object
 */
COLOURED_OBJ getNearestPillar() {
  COLOURED_OBJ nearest_pillar;
  huskylens.request();
  int count = huskylens.count();
  int numColours = huskylens.count();
  int largestArea = 0;

  if (count == 0) {
    nearest_pillar.colour = NO_COLOUR;
    return nearest_pillar;
  } else {
    for (int i = 0; i < count; i++) {
      HUSKYLENSResult result = huskylens.get(i);
      int id = result.ID;
      if (result.height * result.width > largestArea) {
        if (result.ID != MAGENTA || result.ID != NO_COLOUR) {
          largestArea = result.height * result.width;
          nearest_pillar.height = result.height;
          nearest_pillar.width = result.width;
          nearest_pillar.area = largestArea / 100;
          nearest_pillar.xpos = result.xCenter;
          nearest_pillar.colour = (result.ID == RED) ? RED : (result.ID == GREEN) ? GREEN
                                                                                  : NO_COLOUR;
        }
      }
    }
  }
  return nearest_pillar;
}

/**
 * @brief A function to put into display thread for showing the info of the CLOSEST object detetced (if any)
 * 
 * @param column; (enum) COLUMN; A enum defined in oled.h indicating which column the data should be displaced at
 * @param line_number; int; The line number where the data should be displaced at (0 ~ 3)
 * @param size; int; The size of the text being displaced (1 ~ 2)
 * @param clearDisplay; bool; Set true to clear the whole OLED display everytime before displaying the battery percentage
 */
void showNearestPillar(COLUMN column, int line_number, int size, bool clearDisplay) {
  static String huskylens_textR = "\0";

  display.oledSetTextColour(BLACK);
  if (column == LEFT) display.oledDisplayLeftln(line_number, 1, huskylens_textR, clearDisplay);
  else if (column == MID) display.oledDisplayCenterln(line_number, 1, huskylens_textR, clearDisplay);
  else display.oledDisplayRightln(line_number, 1, huskylens_textR, clearDisplay);

  display.oledSetTextColour(WHITE);
  COLOURED_OBJ nearest_pillar = getNearestPillar();
  if (nearest_pillar.colour != NO_COLOUR) huskylens_textR = ((nearest_pillar.colour == RED) ? "R: " : "G: ") + String(nearest_pillar.xpos) + ", " + String(nearest_pillar.area);
  else huskylens_textR = "None";
  if (column == LEFT) display.oledDisplayLeftln(line_number, 1, huskylens_textR, clearDisplay);
  else if (column == MID) display.oledDisplayCenterln(line_number, 1, huskylens_textR, clearDisplay);
  else display.oledDisplayRightln(line_number, 1, huskylens_textR, clearDisplay);
}

/**
 * @brief Get the colour of the nearest object detected
 * 
 * @return COLOURED_OBJ; struct; A struct defined at huskylens.h which store the colour, x-pos and the height of the object
 */
COLOURED_OBJ getNearestColour() {
  COLOURED_OBJ nearest_colour;
  huskylens.request();
  int count = huskylens.count();
  int numColours = huskylens.count();
  int largestHeight = 0;

  if (count == 0) {
    nearest_colour.colour = NO_COLOUR;
    return nearest_colour;
  } else {
    for (int i = 0; i < count; i++) {
      HUSKYLENSResult result = huskylens.get(i);
      int id = result.ID;
      if (result.height > largestHeight) {
        largestHeight = result.height;
        nearest_colour.height = result.height;
        nearest_colour.xpos = result.xCenter;
        nearest_colour.colour = (result.ID == RED) ? RED : GREEN;
      }
    }
  }
  return nearest_colour;
}

void getNearestPillarGlobal(){
  nearestPillarGlobal = getNearestPillar();
}

/**
 * @brief A function to put into display thread for showing the colour of the CLOSEST object detetced (if any)
 * 
 * @param column; (enum) COLUMN; A enum defined in oled.h indicating which column the data should be displaced at
 * @param line_number; int; The line number where the data should be displaced at (0 ~ 3)
 * @param size; int; The size of the text being displaced (1 ~ 2)
 * @param clearDisplay; bool; Set true to clear the whole OLED display everytime before displaying the battery percentage
 */
void showNearestColour(COLUMN column, int line_number, int size, bool clearDisplay) {
  static String huskylens_textR = "\0";
  static String huskylens_textG = "\0";

  display.oledSetTextColour(BLACK);
  if (column == LEFT) {
    display.oledDisplayLeftln(line_number, 1, huskylens_textR, clearDisplay);
    display.oledDisplayLeftln(line_number + 1, 1, huskylens_textG, clearDisplay);
  } else if (column == MID) {
    display.oledDisplayCenterln(line_number, 1, huskylens_textR, clearDisplay);
    display.oledDisplayCenterln(line_number + 1, 1, huskylens_textG, clearDisplay);
  } else {
    display.oledDisplayRightln(line_number, 1, huskylens_textR, clearDisplay);
    display.oledDisplayRightln(line_number + 1, 1, huskylens_textG, clearDisplay);
  }
  display.oledSetTextColour(WHITE);
  COLOURED_OBJ nearest_colour = getNearestPillar();
  if (nearest_colour.colour == RED) {
    huskylens_textR = "R: " + String(nearest_colour.xpos) + ", " + String(nearest_colour.area);
    huskylens_textG = "G: NA";
  } else if (nearest_colour.colour == GREEN) {
    huskylens_textG = "G: " + String(nearest_colour.xpos) + ", " + String(nearest_colour.area);
    huskylens_textR = "R: NA";
  } else {
    huskylens_textR = "R: NA";
    huskylens_textG = "G: NA";
  }
  if (column == LEFT) {
    display.oledDisplayLeftln(line_number, 1, huskylens_textR, clearDisplay);
    display.oledDisplayLeftln(line_number + 1, 1, huskylens_textG, clearDisplay);
  } else if (column == MID) {
    display.oledDisplayCenterln(line_number, 1, huskylens_textR, clearDisplay);
    display.oledDisplayCenterln(line_number + 1, 1, huskylens_textG, clearDisplay);
  } else {
    display.oledDisplayRightln(line_number, 1, huskylens_textR, clearDisplay);
    display.oledDisplayRightln(line_number + 1, 1, huskylens_textG, clearDisplay);
  }
}