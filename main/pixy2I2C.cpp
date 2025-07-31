#define UART
#include "pixy2I2C.h"

Pixy2UART pixy;
COLOURED_OBJ nearestBlockGlobal;

// /**
//  * @brief Initialize the pixy2
//  * 
//  */
// void pixy2Init() {
//   pixy.init();
// }

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
    Serial.print("No OBJ");
  } 
  else {
    pixy2_textR = "Detected";
    Serial.print("Detected");
    Serial.println(pixy.ccc.numBlocks);
    int i;
    for(i = 0; i<pixy.ccc.numBlocks; i++){
      Serial.print("  block ");
      Serial.print(i);
      Serial.print(": ");
      pixy.ccc.blocks[i].print();
    }
  }
  if (column == LEFT) display.oledDisplayLeftln(line_number, 1, pixy2_textR, clearDisplay);
  else if (column == MID) display.oledDisplayCenterln(line_number, 1, pixy2_textR, clearDisplay);
  else display.oledDisplayRightln(line_number, 1, pixy2_textR, clearDisplay);
}
