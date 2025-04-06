#include "huskylens.h"

HUSKYLENS huskylens;
movingAvg xCenter(10);

void huskylensInit(){
  Wire.begin();
  huskylens.begin(Wire);
  xCenter.begin();
  huskylens.writeAlgorithm(ALGORITHM_COLOR_RECOGNITION);
}

void huskylensColorRegTest(COLUMN column, int line_number, int size, bool clearDisplay){
  static String huskylens_text = "\0";

  display.oledSetTextColour(BLACK);
  if (column == LEFT) display.oledDisplayLeftln(line_number, 1, huskylens_text, clearDisplay);
  else if (column == MID) display.oledDisplayCenterln(line_number, 1, huskylens_text, clearDisplay);
  else display.oledDisplayRightln(line_number, 1, huskylens_text, clearDisplay);
  display.oledSetTextColour(WHITE);

  if (!huskylens.request()) huskylens_text = "Failed to request";
  else if (!huskylens.isLearned()) huskylens_text = "Not learned";
  else if (!huskylens.available()) huskylens_text = "No obj";
  else{
    HUSKYLENSResult result = huskylens.read();
    String block = (result.ID == RED) ? "R" : ((result.ID == GREEN) ? "G" : "M");
    huskylens_text = block + "x: " + String(result.xCenter) + " h:" + String(result.height);
  }

  if (column == LEFT) display.oledDisplayLeftln(line_number, 1, huskylens_text, clearDisplay);
  else if (column == MID) display.oledDisplayCenterln(line_number, 1, huskylens_text, clearDisplay);
  else display.oledDisplayRightln(line_number, 1, huskylens_text, clearDisplay);
}

int getHuskyResultID(){
  huskylens.request();
  HUSKYLENSResult result = huskylens.read();
  return result.ID;
}

int getHuskyXPos(COLOUR_BLOCK colour){
  huskylens.request();
  HUSKYLENSResult result = huskylens.read();
  if (result.ID == colour) return result.xCenter;
  else return -1;
}

int getHuskyHeight(COLOUR_BLOCK colour){
  huskylens.request();
  HUSKYLENSResult result = huskylens.read();
  if (result.ID == colour) return result.height;
  else return -1;
}

void showHuskyRed(COLUMN column, int line_number, int size, bool clearDisplay){
  static String huskylens_text = "\0";

  display.oledSetTextColour(BLACK);
  if (column == LEFT) display.oledDisplayLeftln(line_number, 1, huskylens_text, clearDisplay);
  else if (column == MID) display.oledDisplayCenterln(line_number, 1, huskylens_text, clearDisplay);
  else display.oledDisplayRightln(line_number, 1, huskylens_text, clearDisplay);

  display.oledSetTextColour(WHITE);
  if (getHuskyResultID() != RED) huskylens_text = "R: NA";
  else huskylens_text = "R: " + String(getHuskyXPos(RED)) + ", " + String(getHuskyHeight(RED));
  if (column == LEFT) display.oledDisplayLeftln(line_number, 1, huskylens_text, clearDisplay);
  else if (column == MID) display.oledDisplayCenterln(line_number, 1, huskylens_text, clearDisplay);
  else display.oledDisplayRightln(line_number, 1, huskylens_text, clearDisplay);
}

void showHuskyGreen(COLUMN column, int line_number, int size, bool clearDisplay){
  static String huskylens_text = "\0";

  display.oledSetTextColour(BLACK);
  if (column == LEFT) display.oledDisplayLeftln(line_number, 1, huskylens_text, clearDisplay);
  else if (column == MID) display.oledDisplayCenterln(line_number, 1, huskylens_text, clearDisplay);
  else display.oledDisplayRightln(line_number, 1, huskylens_text, clearDisplay);

  display.oledSetTextColour(WHITE);
  if (getHuskyResultID() != GREEN) huskylens_text = "G: NA";
  else huskylens_text = "G: " + String(getHuskyXPos(GREEN)) + ", " + String(getHuskyHeight(GREEN));
  if (column == LEFT) display.oledDisplayLeftln(line_number, 1, huskylens_text, clearDisplay);
  else if (column == MID) display.oledDisplayCenterln(line_number, 1, huskylens_text, clearDisplay);
  else display.oledDisplayRightln(line_number, 1, huskylens_text, clearDisplay);
}

void showHuskyMagenta(COLUMN column, int line_number, int size, bool clearDisplay){
  static String huskylens_text = "\0";

  display.oledSetTextColour(BLACK);
  if (column == LEFT) display.oledDisplayLeftln(line_number, 1, huskylens_text, clearDisplay);
  else if (column == MID) display.oledDisplayCenterln(line_number, 1, huskylens_text, clearDisplay);
  else display.oledDisplayRightln(line_number, 1, huskylens_text, clearDisplay);

  display.oledSetTextColour(WHITE);
  if (getHuskyResultID() != MAGENTA) huskylens_text = "M: NA";
  else huskylens_text = "M: " + String(getHuskyXPos(MAGENTA)) + ", " + String(getHuskyHeight(MAGENTA));
  if (column == LEFT) display.oledDisplayLeftln(line_number, 1, huskylens_text, clearDisplay);
  else if (column == MID) display.oledDisplayCenterln(line_number, 1, huskylens_text, clearDisplay);
  else display.oledDisplayRightln(line_number, 1, huskylens_text, clearDisplay);
}

COLOURED_OBJ getNearestPillar(){
  COLOURED_OBJ nearest_pillar;
  huskylens.request();
  int count = huskylens.count();
  int numColours = huskylens.count();
  int largestHeight = 0;

  if (count == 0){
    nearest_pillar.colour = NO_COLOUR;
    return nearest_pillar;
  } else {
    for (int i = 0; i < count; i++) {
      HUSKYLENSResult result = huskylens.get(i);
      int id = result.ID;
      if (result.height > largestHeight){
        if (result.ID != MAGENTA){
          largestHeight = result.height;
          nearest_pillar.height = result.height;
          nearest_pillar.xpos = result.xCenter;
          nearest_pillar.colour = (result.ID == RED) ? RED : GREEN;
        }
      }
    }
  }
  return nearest_pillar;
}

void showNearestPillar(COLUMN column, int line_number, int size, bool clearDisplay){
  static String huskylens_text = "\0";

  display.oledSetTextColour(BLACK);
  if (column == LEFT) display.oledDisplayLeftln(line_number, 1, huskylens_text, clearDisplay);
  else if (column == MID) display.oledDisplayCenterln(line_number, 1, huskylens_text, clearDisplay);
  else display.oledDisplayRightln(line_number, 1, huskylens_text, clearDisplay);

  display.oledSetTextColour(WHITE);
  COLOURED_OBJ nearest_pillar = getNearestPillar();
  if (nearest_pillar.colour != NO_COLOUR) huskylens_text = ((nearest_pillar.colour == RED) ? "R: " : "G: ") + String(nearest_pillar.xpos) + ", " + String(nearest_pillar.height);
  else huskylens_text = "None";
  if (column == LEFT) display.oledDisplayLeftln(line_number, 1, huskylens_text, clearDisplay);
  else if (column == MID) display.oledDisplayCenterln(line_number, 1, huskylens_text, clearDisplay);
  else display.oledDisplayRightln(line_number, 1, huskylens_text, clearDisplay);
}

COLOURED_OBJ getNearestColour(){
  COLOURED_OBJ nearest_colour;
  huskylens.request();
  int count = huskylens.count();
  int numColours = huskylens.count();
  int largestHeight = 0;
  
  if (count == 0){
    nearest_colour.colour = NO_COLOUR;
    return nearest_colour;
  } else {
    for (int i = 0; i < count; i++) {
      HUSKYLENSResult result = huskylens.get(i);
      int id = result.ID;
      if (result.height > largestHeight){
        largestHeight = result.height;
        nearest_colour.height = result.height;
        nearest_colour.xpos = result.xCenter;
        nearest_colour.colour = (result.ID == RED) ? RED : GREEN;
      }
    }
  }
  return nearest_colour;
}

void showNearestColour(COLUMN column, int line_number, int size, bool clearDisplay){
  static String huskylens_text = "\0";

  display.oledSetTextColour(BLACK);
  if (column == LEFT) display.oledDisplayLeftln(line_number, 1, huskylens_text, clearDisplay);
  else if (column == MID) display.oledDisplayCenterln(line_number, 1, huskylens_text, clearDisplay);
  else display.oledDisplayRightln(line_number, 1, huskylens_text, clearDisplay);

  display.oledSetTextColour(WHITE);
  COLOURED_OBJ nearest_colour = getNearestPillar();
  if (nearest_colour.colour != NO_COLOUR) huskylens_text = ((nearest_colour.colour == RED) ? "R: " : ((nearest_colour.colour == GREEN) ? "G: ": "M: ")) + String(nearest_colour.xpos) + ", " + String(nearest_colour.height);
  else huskylens_text = "None";
  if (column == LEFT) display.oledDisplayLeftln(line_number, 1, huskylens_text, clearDisplay);
  else if (column == MID) display.oledDisplayCenterln(line_number, 1, huskylens_text, clearDisplay);
  else display.oledDisplayRightln(line_number, 1, huskylens_text, clearDisplay);
}