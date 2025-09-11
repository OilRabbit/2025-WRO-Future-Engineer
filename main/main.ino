#include "main.h"

TaskHandle_t blinkledThread = NULL;
TaskHandle_t displayThread = NULL;
TaskHandle_t Ultra1Thread = NULL;
TaskHandle_t Ultra2Thread = NULL;
TaskHandle_t OC1Thread = NULL;
TaskHandle_t OC2Thread = NULL;
TaskHandle_t huskylensThread = NULL;
TaskHandle_t colorThread = NULL;

void setup() {
  tft.init();
  tft.displayln(30, 0, 2, "Initializing...", TFT_WHITE, true);
  tft.displayLeftln(1, 2, "LED Init:", TFT_WHITE, false);
  tft.displayLeftln(2, 2, "Clk Init:", TFT_WHITE, false);
  tft.displayLeftln(3, 2, "Ser Init:", TFT_WHITE, false);
  tft.displayLeftln(4, 2, "Ut1 Init:", TFT_WHITE, false);
  
  led_init();
  delay(1000);
  tft.displayRightln(1, 2, "Done", TFT_GREEN, false);
  internalClock.start();
  delay(1000);
  tft.displayRightln(2, 2, "Done", TFT_GREEN, false);
  Serial.begin(115200);
  delay(1000);
  tft.displayRightln(3, 2, "Done", TFT_GREEN, false);
  ultraInit();
  delay(1000);
  tft.displayRightln(4, 2, "Done", TFT_GREEN, false);

  xTaskCreatePinnedToCore(
    blinkledLED_test,         // Task function
    "Blink LED",       // Task name
    10000,             // Stack size (bytes)
    NULL,              // Parameters
    1,                 // Priority
    &blinkledThread,  // Task handle
    1                  // Core 1
  );

  xTaskCreatePinnedToCore(
    getultra1Distloop,         // Task function
    "Get Ultra 1 Distance",       // Task name
    10000,             // Stack size (bytes)
    NULL,              // Parameters
    1,                 // Priority
    &Ultra1Thread,  // Task handle
    1                  // Core 1
  );

  delay(1000);
  tft.clear();

  xTaskCreatePinnedToCore(
    displayData,         // Task function
    "Display info",       // Task name
    10000,             // Stack size (bytes)
    NULL,              // Parameters
    1,                 // Priority
    &displayThread,  // Task handle
    1                  // Core 1
  );
}

void loop() {
  Serial.println(ultra1Dist);
  // led_blue(32);
  // Serial.println("On");
  // delay(500);
  // led_off();
  // Serial.println("Off");
  // delay(500);
}



// void setup() {
//   // Init
//   Serial.begin(115200);
//   internalClock.start();
//   MiniR4.begin();
//   display.oledDisplayCenterln(1, 1, "Initializing...", true);
//   MiniR4.PWR.setBattCell(2);  // 18650x2, two-cell (2S)
//   MiniR4.M2.begin();
//   MiniR4.M2.setBrake(true);
//   steeringInit();
//   reset_steering();
//   ultraInit(ultra1);
//   ultraInit(ultra2);
//   colorInit();
//   laserInit(I2CPORT1);
//   // laserInit(I2CPORT2);
//   huskylensInit();
//   display.oledDisplayCenterln(1, 1, "Initializing IMU", true);
//   imuInit();
//   display.oledClear();

//   // Follow this to create your own thread
//   displayThread.onRun(displayData);
//   displayThread.setInterval(0);
//   controller.add(&displayThread);

//   // OC1Thread.onRun(OC1main);
//   // OC1Thread.setInterval(10);
//   // controller.add(&OC1Thread);

//   OC2Thread.onRun(OC2main);
//   OC2Thread.setInterval(15);
//   controller.add(&OC2Thread);

//   Ultra1Thread.onRun(getultra1Distloop);
//   Ultra1Thread.setInterval(10);
//   controller.add(&Ultra1Thread);
//   Ultra2Thread.onRun(getultra2Distloop);
//   Ultra2Thread.setInterval(10);
//   controller.add(&Ultra2Thread);

//   huskylensThread.onRun(getNearestPillarGlobal);
//   huskylensThread.setInterval(10);
//   controller.add(&huskylensThread);

//   colorThread.onRun(getColorTypeloop);
//   colorThread.setInterval(10);
//   controller.add(&colorThread);

//   Laser1Thread.onRun(getLaser1Distloop);
//   Laser1Thread.setInterval(0);
//   controller.add(&Laser1Thread);
//   // Laser2Thread.onRun(getLaser2Distloop);
//   // Laser2Thread.setInterval(0);
//   // controller.add(&Laser2Thread);
// }

// void loop() {
//   controller.run();
// }