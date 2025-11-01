#include "main.h"

TaskHandle_t blinkledThread = NULL;
TaskHandle_t displayThread = NULL;
TaskHandle_t Ultra1Thread = NULL;
TaskHandle_t Ultra2Thread = NULL;
TaskHandle_t IMUThread = NULL;
TaskHandle_t SteeringThread = NULL;
TaskHandle_t MotorEncThread = NULL;
TaskHandle_t Pixy2Thread = NULL;
TaskHandle_t OC1Thread = NULL;
TaskHandle_t OC2Thread = NULL;
TaskHandle_t colorThread = NULL;

void setup() {
  Serial.begin(115200);
  tft.init();
  tft.displayln(30, 0, 2, "Initializing...", TFT_WHITE, true);

  #ifdef FENZY_MODE
    tft.displayLeftln(2, 2, "LED Init:", TFT_WHITE, false);
    tft.displayLeftln(3, 2, "Clk Init:", TFT_WHITE, false);
    tft.displayLeftln(4, 2, "Ser Init:", TFT_WHITE, false);
    tft.displayLeftln(5, 2, "Ut1 Init:", TFT_WHITE, false);
    tft.displayLeftln(6, 2, "Ut1 Init:", TFT_WHITE, false);
    tft.displayLeftln(7, 2, "Btn Init:", TFT_WHITE, false);
    tft.displayLeftln(8, 2, "Imu Init:", TFT_WHITE, false);
    tft.displayLeftln(9, 2, "Str Init:", TFT_WHITE, false);
    // tft.displayLeftln(10, 2, "Mtr Init:", TFT_WHITE, false);
    tft.displayLeftln(11, 2, "Pxy Init:", TFT_WHITE, false);
  #endif
  
  led_init();
  #ifdef FENZY_MODE
    delay(500);
    tft.displayRightln(2, 2, "Done", TFT_GREEN, false);
  #endif
  internalClock.start();

  #ifdef FENZY_MODE
    delay(500);
    tft.displayRightln(3, 2, "Done", TFT_GREEN, false);
  #endif

  Serial.begin(115200);
  #ifdef FENZY_MODE
    delay(500);
    tft.displayRightln(4, 2, "Done", TFT_GREEN, false);
  #endif

  ultraInit();
  #ifdef FENZY_MODE
    delay(500);
    tft.displayRightln(5, 2, "Done", TFT_GREEN, false);
    tft.displayRightln(6, 2, "Done", TFT_GREEN, false);
  #endif

  btn_init();
  #ifdef FENZY_MODE
    delay(500);
    tft.displayRightln(7, 2, "Done", TFT_GREEN, false);
  #endif

  imu_init();
  #ifdef FENZY_MODE
    delay(500);
    tft.displayRightln(8, 2, "Done", TFT_GREEN, false);
  #endif

  steeringInit();
  #ifdef FENZY_MODE
    delay(500);
    tft.displayRightln(9, 2, "Done", TFT_GREEN, false);
  #endif

  // motor_init();
  // #ifdef FENZY_MODE
  //   delay(500);
  //   tft.displayRightln(10, 2, "Done", TFT_GREEN, false);
  // #endif

  Pixy2I2C pixy;

  Serial.println("Init Pixy2...");
  // assume Wire.begin() was already called in setup()
  Wire.begin(PIXY2_SDA, PIXY2_SCL, 100000);
  Wire.setClock(400000);
  Serial.println("Init Pixy2...");
  pixy.init();
  #ifdef FENZY_MODE
    delay(500);
    tft.displayRightln(11, 2, "Done", TFT_GREEN, false);
  #endif

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
    0                  // Core 1
  );

  xTaskCreatePinnedToCore(
    getultra2Distloop,         // Task function
    "Get Ultra 2 Distance",       // Task name
    10000,             // Stack size (bytes)
    NULL,              // Parameters
    1,                 // Priority
    &Ultra2Thread,  // Task handle
    0                  // Core 1
  );

  xTaskCreatePinnedToCore(
    getYPRloop,         // Task function
    "Get IMU Data",       // Task name
    10000,             // Stack size (bytes)
    NULL,              // Parameters
    1,                 // Priority
    &IMUThread,  // Task handle
    1                  // Core 1
  );

  xTaskCreatePinnedToCore(
    steeringloop,         // Task function
    "Steering",       // Task name
    10000,             // Stack size (bytes)
    NULL,              // Parameters
    1,                 // Priority
    &SteeringThread,  // Task handle
    1                  // Core 1
  );

  xTaskCreatePinnedToCore(
    getNearestBlkloop,         // Task function
    "Pixy2",       // Task name
    10000,             // Stack size (bytes)
    NULL,              // Parameters
    1,                 // Priority
    &Pixy2Thread,  // Task handle
    1                  // Core 1
  );

  // xTaskCreatePinnedToCore(
  //   motor_encloop,         // Task function
  //   "Pixy2",       // Task name
  //   10000,             // Stack size (bytes)
  //   NULL,              // Parameters
  //   1,                 // Priority
  //   &Pixy2Thread,  // Task handle
  //   1                  // Core 1
  // );

  // xTaskCreatePinnedToCore(
  //   serialprint,         // Task function
  //   "Print on the Serial Monitor",       // Task name
  //   10000,             // Stack size (bytes)
  //   NULL,              // Parameters
  //   1,                 // Priority
  //   &SerialThread,  // Task handle
  //   1                  // Core 1
  // );

  #ifdef FENZY_MODE
    delay(500);
  #else
    delay(300);
  #endif

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