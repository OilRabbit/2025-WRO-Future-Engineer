#include "api/Common.h"
#include "ultra_newping.h"

NewPing sonar[SONAR_NUM] = {   // Sensor object array.
  NewPing(3, 2, MAX_DISTANCE), // Ultra 1 trigger pin, echo pin, and max distance to ping. 
  NewPing(5, 4, MAX_DISTANCE)  // Ultra 2 trigger pin, echo pin, and max distance to ping.
};

int ultra1Dist_new;
int ultra2Dist_new;

void ultra_testing(){
    delay(50); // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
    Serial.print("ultra1");
    Serial.print("=");
    Serial.print(sonar[0].ping_cm());
    Serial.println("cm ");
}

int getultra1Dist_cm(){
  delay(30);
  sonar[0].ping_cm();
}

int getultra2Dist_cm(){
  delay(30);
  sonar[1].ping_cm();
}

void ultra1Dist_cmloop(){
  ultra1Dist_new = getultra1Dist_cm() * 10;
}

void ultra2Dist_cmloop(){
  ultra2Dist_new = getultra2Dist_cm() * 10;
}

void showUltra1Dist_new(COLUMN column, int line_number, int size, bool clearDisplay){
  static String ultra_text = String("U1:") + String(ultra1Dist_new) + "mm";
  display.oledSetTextColour(BLACK);
  if (column == LEFT){
    display.oledDisplayLeftln(line_number, size, ultra_text, clearDisplay);
    display.oledSetTextColour(WHITE);
    ultra_text = String("U1:") + String(ultra1Dist_new) + "mm";
    display.oledDisplayLeftln(line_number, size, ultra_text, clearDisplay);
  } else if (column == MID) {
    display.oledDisplayCenterln(line_number, size, ultra_text, clearDisplay);
    display.oledSetTextColour(WHITE);
    ultra_text =String("U1:") + String(ultra1Dist_new) + "mm";
    display.oledDisplayCenterln(line_number, size, ultra_text, clearDisplay);
  } else {
    display.oledDisplayRightln(line_number, size, ultra_text, clearDisplay);
    display.oledSetTextColour(WHITE);
    ultra_text = String("U1:") + String(ultra1Dist_new) + "mm";
    display.oledDisplayRightln(line_number, size, ultra_text, clearDisplay);
  }
}

void showUltra2Dist_new(COLUMN column, int line_number, int size, bool clearDisplay){
  static String ultra_text = String("U2:") + String(ultra2Dist_new) + "mm";
  display.oledSetTextColour(BLACK);
  if (column == LEFT){
    display.oledDisplayLeftln(line_number, size, ultra_text, clearDisplay);
    display.oledSetTextColour(WHITE);
    ultra_text = String("U2:") + String(ultra2Dist_new) + "mm";
    display.oledDisplayLeftln(line_number, size, ultra_text, clearDisplay);
  } else if (column == MID) {
    display.oledDisplayCenterln(line_number, size, ultra_text, clearDisplay);
    display.oledSetTextColour(WHITE);
    ultra_text = String("U2:") + String(ultra2Dist_new) + "mm";
    display.oledDisplayCenterln(line_number, size, ultra_text, clearDisplay);
  } else {
    display.oledDisplayRightln(line_number, size, ultra_text, clearDisplay);
    display.oledSetTextColour(WHITE);
    ultra_text = String("U2:") + String(ultra2Dist_new) + "mm";
    display.oledDisplayRightln(line_number, size, ultra_text, clearDisplay);
  }
}