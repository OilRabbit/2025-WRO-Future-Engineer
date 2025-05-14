// #include "color.h"

// /* Global variables for storing the color number values */
// int ColorNo;

// /**
//  * @brief Initialize the laser(s)
//  * 
//  * @param port; (enum) LASERPORT; The port of the laser
//  */
// void colorInit(){
//    MiniR4.I2C3.MXColor.begin();
//    MiniR4.I2C3.MXColor.setGamma(true);
//    MiniR4.I2C3.MXColor.setlight(true ,false ,200);
// }
// uint8_t getColorRGB(ColorType color){
//    return MiniR4.I2C3.MXColor.getColor(ColorType color);
// }

// uint8_t getColorType(){
//    return MiniR4.I2C3.MXColor.getColorNumber();
// }

// uint8_t getGreyScale(){
//    return MiniR4.I2C3.MXColor.getGrayScale();
// }

// void getColorTypeloop(){
//    int current_col_type = 20;
//    switch (MXColor1.getColorNumber())
//   {
//   case 0:
//     current_col_type = 0;
//     //black
//     break;
//   case 1:
//     current_col_type = 1;
//     //white
//     break;
//   case 2:
//     current_col_type = 2;
//     //cyan
//     break;
//   case 3:
//     current_col_type = 3;
//     //ocean
//     break;
//   case 4:
//     current_col_type = 4;
//     //blue
//     break;
//   case 5:
//     current_col_type = 5;
//     //violet
//     break;
//   case 6:
//     current_col_type = 6;
//     //Magenta
//     break;
//   case 7:
//     current_col_type = 7;
//     //Raspberry
//     break;
//   case 8:
//     current_col_type = 8;
//     //Red
//     break;
//   case 9:
//     current_col_type = 9;
//     //Orange
//     break;
//   case 10:
//     current_col_type = 10;
//     //Yellow
//     break;
//   case 11:
//     current_col_type = 11;
//     //Spring Green
//     break;
//   case 12:
//     current_col_type = 12;
//     //Green
//     break;
//   case 13:
//     current_col_type = 13);
//     //Turquoise
//     break;
//   }
//   ColorNo = cuurent_col_type;
// }

// void showColorType (COLUMN column, int line_number, int size, bool clearDisplay){
//   static String laser_text = "Color" + ":" + String(ColorNo) ;
//   display.oledSetTextColour(BLACK);
//   if (column == LEFT){
//     display.oledDisplayLeftln(line_number, size, laser_text, clearDisplay);
//     display.oledSetTextColour(WHITE);
//     laser_text = "Color" + ":" + String(ColorNo);
//     display.oledDisplayLeftln(line_number, size, laser_text, clearDisplay);
//   } else if (column == MID) {
//     display.oledDisplayCenterln(line_number, size, laser_text, clearDisplay);
//     display.oledSetTextColour(WHITE);
//     laser_text = "Color" + ":" + String(ColorNo);
//     display.oledDisplayCenterln(line_number, size, laser_text, clearDisplay);
//   } else {
//     display.oledDisplayRightln(line_number, size, laser_text, clearDisplay);
//     display.oledSetTextColour(WHITE);
//     laser_text = "Color" +  ":" + String(ColorNo);
//     display.oledDisplayRightln(line_number, size, laser_text, clearDisplay);
//   }
// }
