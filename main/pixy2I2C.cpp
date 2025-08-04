#include "pixy2I2C.h"

Pixy2UART pixy;
COLOURED_OBJ nearestBlockGlobal;

/**
 * @brief Initialize the pixy2
 * 
 */
void pixy2Init() {
   pixy.init();
}

/**
 * @brief A testing function for colour block detection and data displacement for the pixy2
 * 
 * @param column; (enum) COLUMN; A enum defined in oled.h indicating which column the data should be displaced at
 * @param line_number; int; The line number where the data should be displaced at (0 ~ 3)
 * @param size; int; The size of the text being displaced (1 ~ 2)
 * @param clearDisplay; bool; Set true to clear the whole OLED display everytime before displaying the battery percentage
 */
void pixy2ColorRegTest(COLUMN column, int line_number, int size, bool clearDisplay) {
  static String pixy2_textR = "\0";

  display.oledSetTextColour(BLACK);
  if (column == LEFT) display.oledDisplayLeftln(line_number, 1, pixy2_textR, clearDisplay);
  else if (column == MID) display.oledDisplayCenterln(line_number, 1, pixy2_textR, clearDisplay);
  else display.oledDisplayRightln(line_number, 1, pixy2_textR, clearDisplay);
  display.oledSetTextColour(WHITE);

  pixy.ccc.getBlocks();
  if (!pixy.ccc.numBlocks) {
    pixy2_textR = "No OBJ";
    Serial.println("No OBJ");
  } 
  else {
    Serial.println("Detected");
    Serial.println(pixy.ccc.numBlocks);
    int i;
    for(i = 0; i<pixy.ccc.numBlocks; i++){
      if (pixy.ccc.blocks[i].m_signature = RED){
        pixy2_textR = "Red Detected";
      }
      if(pixy.ccc.blocks[i].m_signature = GREEN){
        pixy2_textR = "Green";
      }
      Serial.println(pixy.ccc.blocks[0].m_signature);
    }
  }
  if (column == LEFT) display.oledDisplayLeftln(line_number, 1, pixy2_textR, clearDisplay);
  else if (column == MID) display.oledDisplayCenterln(line_number, 1, pixy2_textR, clearDisplay);
  else display.oledDisplayRightln(line_number, 1, pixy2_textR, clearDisplay);
}

/**
 * @brief Get the signature of the object from the pixy2
 * 
 * @return int; the signature of the object
 */
int getPixy2Signature() {
  pixy.ccc.getBlocks();
  return pixy.ccc.blocks[0].m_signature;
}

/**
 * @brief Get the x-pos of the objected detected from the Pixy2
 * 
 * @param colour; (enum) COLOUR_BLOCK; A enum defined in oled.h, indicating the colour which the Pixy2 may detect
 * @return int; The x-pos of the object detected. Return -1 if nothing is detected
 */
int getPixy2XPos(COLOUR_BLOCK colour) {
  if (pixy.ccc.blocks[0].m_signature == colour) return pixy.ccc.blocks[0].m_x;
  else return -1;
}

/**
 * @brief Get the height of the object detected from the Pixy2
 * 
 * @param colour; (enum) COLOUR_BLOCK; A enum defined in oled.h, indicating the colour which the pixy2 may detect
 * @return int; The height of the object detected. Return -1 if nothing is detected
 */
int getPixy2Height(COLOUR_BLOCK colour) {
  pixy.ccc.getBlocks();
  if (pixy.ccc.blocks[0].m_signature == colour) return pixy.ccc.blocks[0].m_height;
  else return -1;
}

/**
 * @brief Get the colour of the nearest object detected
 * 
 * @return COLOURED_OBJ; struct; A struct defined at huskylens.h which store the colour, x-pos and the height of the object by using pixy2
 */
COLOURED_OBJ getPixy2NearestColour() {
  COLOURED_OBJ nearest_colour;;
  pixy.ccc.getBlocks();
  int count = pixy.ccc.numBlocks;
  int numColours = pixy.ccc.numBlocks;
  int largestHeight = 0;

  if (count == 0) {
    nearest_colour.colour = NO_COLOUR;
    return nearest_colour;
  } else {
    for (int i = 0; i < count; i++) {
      int signature = pixy.ccc.blocks[i].m_signature;
      if (pixy.ccc.blocks[i].m_height > largestHeight) {
        largestHeight = pixy.ccc.blocks[i].m_height;
        nearest_colour.width = pixy.ccc.blocks[i].m_width;
        nearest_colour.height = pixy.ccc.blocks[i].m_height;
        nearest_colour.area = nearest_colour.width * nearest_colour.height;
        nearest_colour.xpos = pixy.ccc.blocks[i].m_x;
        nearest_colour.colour = (pixy.ccc.blocks[i].m_signature == RED) ? RED : GREEN;
      }
    }
  }
  return nearest_colour;
}

void getPixy2NearestPillarGlobal(){
  nearestPillarGlobal = getPixy2NearestColour();
}

/**
 * @brief A function to put into display thread for showing the colour of the CLOSEST object detetced with using pixy2(if any)
 * 
 * @param column; (enum) COLUMN; A enum defined in oled.h indicating which column the data should be displaced at
 * @param line_number; int; The line number where the data should be displaced at (0 ~ 3)
 * @param size; int; The size of the text being displaced (1 ~ 2)
 * @param clearDisplay; bool; Set true to clear the whole OLED display everytime before displaying the battery percentage
 */
void showPixy2NearestColour(COLUMN column, int line_number, int size, bool clearDisplay) {
  static String pixy2_textR = "\0";
  static String pixy2_textG = "\0";

  display.oledSetTextColour(BLACK);
  if (column == LEFT) {
    display.oledDisplayLeftln(line_number, 1, pixy2_textR, clearDisplay);
    display.oledDisplayLeftln(line_number + 1, 1, pixy2_textG, clearDisplay);
  } else if (column == MID) {
    display.oledDisplayCenterln(line_number, 1, pixy2_textR, clearDisplay);
    display.oledDisplayCenterln(line_number + 1, 1, pixy2_textG, clearDisplay);
  } else {
    display.oledDisplayRightln(line_number, 1, pixy2_textR, clearDisplay);
    display.oledDisplayRightln(line_number + 1, 1, pixy2_textG, clearDisplay);
  }
  display.oledSetTextColour(WHITE);
  COLOURED_OBJ nearest_colour = getNearestPillar();
  if (nearest_colour.colour == RED) {
    pixy2_textR = "R: " + String(nearest_colour.xpos) + ", " + String(nearest_colour.area);
    pixy2_textG = "G: NA";
  } else if (nearest_colour.colour == GREEN) {
    pixy2_textG = "G: " + String(nearest_colour.xpos) + ", " + String(nearest_colour.area);
    pixy2_textR = "R: NA";
  } else {
    pixy2_textR = "R: NA";
    pixy2_textG = "G: NA";
  }
  if (column == LEFT) {
    display.oledDisplayLeftln(line_number, 1, pixy2_textR, clearDisplay);
    display.oledDisplayLeftln(line_number + 1, 1, pixy2_textG, clearDisplay);
  } else if (column == MID) {
    display.oledDisplayCenterln(line_number, 1, pixy2_textR, clearDisplay);
    display.oledDisplayCenterln(line_number + 1, 1, pixy2_textG, clearDisplay);
  } else {
    display.oledDisplayRightln(line_number, 1, pixy2_textR, clearDisplay);
    display.oledDisplayRightln(line_number + 1, 1, pixy2_textG, clearDisplay);
  }
}