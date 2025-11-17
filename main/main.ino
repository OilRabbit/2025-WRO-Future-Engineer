#include "main.h"

TaskHandle_t blinkledThread = NULL;
TaskHandle_t displayThread = NULL;
TaskHandle_t ToF1Thread = NULL;
TaskHandle_t ToF2Thread = NULL;
// TaskHandle_t Ultra1Thread = NULL;
// TaskHandle_t Ultra2Thread = NULL;
TaskHandle_t Ultra3Thread = NULL;
TaskHandle_t IMUThread = NULL;
TaskHandle_t SteeringThread = NULL;
TaskHandle_t MotorEncThread = NULL;
TaskHandle_t Pixy2Thread = NULL;
TaskHandle_t OC1Thread = NULL;
TaskHandle_t OC2Thread = NULL;

void setup() {
  motor_preinit_safe();
  Serial.begin(115200);
  tft.init();
  tft.displayln(30, 0, 2, "Initializing...", TFT_WHITE, true);
  
  int line_iter = 2;
  #ifdef FENZY_MODE
    tft.displayLeftln(line_iter++, 2, "LED Init:", TFT_WHITE, false);
    tft.displayLeftln(line_iter++, 2, "Clk Init:", TFT_WHITE, false);
    tft.displayLeftln(line_iter++, 2, "Ser Init:", TFT_WHITE, false);
    tft.displayLeftln(line_iter++, 2, "Tf1 Init:", TFT_WHITE, false);
    tft.displayLeftln(line_iter++, 2, "Tf2 Init:", TFT_WHITE, false);
    // tft.displayLeftln(line_iter++, 2, "Ut3 Init:", TFT_WHITE, false);
    tft.displayLeftln(line_iter++, 2, "Btn Init:", TFT_WHITE, false);
    tft.displayLeftln(line_iter++, 2, "Imu Init:", TFT_WHITE, false);
    tft.displayLeftln(line_iter++, 2, "Str Init:", TFT_WHITE, false);
    tft.displayLeftln(line_iter++, 2, "Pxy Init:", TFT_WHITE, false);
    tft.displayLeftln(line_iter++, 2, "Mtr Init:", TFT_WHITE, false);
  #endif
  
  line_iter = 2;

  led_init();
  #ifdef FENZY_MODE
    delay(100);
    tft.displayRightln(line_iter++, 2, "Done", TFT_GREEN, false);
  #endif
  internalClock.start();

  #ifdef FENZY_MODE
    delay(100);
    tft.displayRightln(line_iter++, 2, "Done", TFT_GREEN, false);
  #endif

  Serial.begin(115200);
  #ifdef FENZY_MODE
    delay(100);
    tft.displayRightln(line_iter++, 2, "Done", TFT_GREEN, false);
  #endif

  tofInit();
  #ifdef FENZY_MODE
    delay(100);
    tft.displayRightln(line_iter++, 2, "Done", TFT_GREEN, false);
    tft.displayRightln(line_iter++, 2, "Done", TFT_GREEN, false);
  #endif

  // ultraInit();
  // #ifdef FENZY_MODE
  //   delay(100);
  //   tft.displayRightln(line_iter++, 2, "Done", TFT_GREEN, false);
  // #endif

  btn_init();
  #ifdef FENZY_MODE
    delay(100);
    tft.displayRightln(line_iter++, 2, "Done", TFT_GREEN, false);
  #endif

  imu_init();
  #ifdef FENZY_MODE
    delay(100);
    tft.displayRightln(line_iter++, 2, "Done", TFT_GREEN, false);
  #endif

  steeringInit();
  #ifdef FENZY_MODE
    delay(100);
    tft.displayRightln(line_iter++, 2, "Done", TFT_GREEN, false);
  #endif

  motor_init();
  #ifdef FENZY_MODE
    delay(100);
    tft.displayRightln(line_iter++, 2, "Done", TFT_GREEN, false);
  #endif

  Serial.println("Init Pixy2...");
  // assume Wire.begin() was already called in setup()
  Wire.begin(PIXY2_SDA, PIXY2_SCL, 100000);
  Wire.setClock(400000);
  Serial.println("Init Pixy2...");
  pixy.init();
  #ifdef FENZY_MODE
    delay(100);
    tft.displayRightln(line_iter++, 2, "Done", TFT_GREEN, false);
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
    getToF1Distloop,         // Task function
    "Get ToF 1 Distance",       // Task name
    10000,             // Stack size (bytes)
    NULL,              // Parameters
    1,                 // Priority
    &ToF1Thread,  // Task handle
    0                  // Core 0
  );

  xTaskCreatePinnedToCore(
    getToF2Distloop,         // Task function
    "Get ToF 2 Distance",       // Task name
    10000,             // Stack size (bytes)
    NULL,              // Parameters
    1,                 // Priority
    &ToF2Thread,  // Task handle
    0                  // Core 0
  );

  // xTaskCreatePinnedToCore(
  //   getultra1Distloop,         // Task function
  //   "Get Ultra 1 Distance",       // Task name
  //   10000,             // Stack size (bytes)
  //   NULL,              // Parameters
  //   1,                 // Priority
  //   &Ultra1Thread,  // Task handle
  //   0                  // Core 0
  // );

  // xTaskCreatePinnedToCore(
  //   getultra2Distloop,         // Task function
  //   "Get Ultra 2 Distance",       // Task name
  //   10000,             // Stack size (bytes)
  //   NULL,              // Parameters
  //   1,                 // Priority
  //   &Ultra2Thread,  // Task handle
  //   0                  // Core 0
  // );

  // xTaskCreatePinnedToCore(
  //   getultra3Distloop,         // Task function
  //   "Get Ultra 3 Distance",       // Task name
  //   10000,             // Stack size (bytes)
  //   NULL,              // Parameters
  //   1,                 // Priority
  //   &Ultra3Thread,  // Task handle
  //   0                  // Core 0
  // );

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
    motor_encloop,         // Task function
    "Motor Encoder",       // Task name
    10000,             // Stack size (bytes)
    NULL,              // Parameters
    1,                 // Priority
    &MotorEncThread,  // Task handle
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

  // xTaskCreatePinnedToCore(
  //   OC1main,         // Task function
  //   "OC1",       // Task name
  //   10000,             // Stack size (bytes)
  //   NULL,              // Parameters
  //   1,                 // Priority
  //   &OC1Thread,  // Task handle
  //   1                  // Core 1
  // );

  xTaskCreatePinnedToCore(
    OC2main,         // Task function
    "OC2",       // Task name
    10000,             // Stack size (bytes)
    NULL,              // Parameters
    1,                 // Priority
    &OC2Thread,  // Task handle
    1                  // Core 1
  );
}

void loop() {
  while (Serial2.available()) Serial.write(Serial2.read());

}