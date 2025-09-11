// #include "battery.h"

// /**
//  * @brief Get the battery percentage. It is being offset.
//  *
//  * @return int; the battery percentage
//  */
// int getBatteryPercentage(){ return MiniR4.PWR.getBattPercentage() + 20; }

// /**
//  * @brief A function to put into display thread for showing the battery percentage on the OLED display.
//  * 
//  * @param clearDisplay; bool; Set true to clear the whole OLED display everytime before displaying the battery percentage
//  */
// void showBattPercentage(bool clearDisplay){
//   static String batt_text = String(getBatteryPercentage()) + "%";
//   display.oledSetTextColour(BLACK);
//   display.oledDisplayRightln(0, 1, batt_text, clearDisplay);
//   display.oledSetTextColour(WHITE);
//   batt_text = String(getBatteryPercentage()) + "%";
//   display.oledDisplayRightln(0, 1, batt_text, clearDisplay);
// }