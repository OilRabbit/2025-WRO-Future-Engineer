#include "main.h"
// #include <NewPing.h>
#define I2C

// Define your threads here
// Thread displayThread = Thread();
// Thread OC1Thread = Thread();
// Thread OC2Thread = Thread();
// Thread Ultra1Thread = Thread();
// Thread Ultra2Thread = Thread();
// Thread pixy2Thread = Thread();
// Thread huskylensThread = Thread();
// Thread colorThread = Thread();
// Thread Laser1Thread = Thread();
// Thread Laser2Thread = Thread();

// Controller to manage all threads
// ThreadController controller = ThreadController();

// #define SONAR_NUM 2      // Number of sensors.
// #define MAX_DISTANCE 300 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm. 

// NewPing sonar[SONAR_NUM] = {   // Sensor object array.
//   NewPing(3, 2, MAX_DISTANCE), // Ultra 1 trigger pin, echo pin, and max distance to ping. 
//   NewPing(5, 4, MAX_DISTANCE)  // Ultra 2 trigger pin, echo pin, and max distance to ping.
// };

void setup() {
  // Init
  Serial.begin(115200);
  internalClock.start();
  MiniR4.begin();
  display.oledDisplayCenterln(1, 1, "Initializing...", true);
  MiniR4.PWR.setBattCell(2);  // 18650x2, two-cell (2S)
  MiniR4.M2.begin();
  MiniR4.M2.setBrake(true);
  steeringInit();
  reset_steering();
  ultraInit(ultra1);
  ultraInit(ultra2);
  // colorInit();
  // laserInit(I2CPORT1);
  // laserInit(I2CPORT2);
  // huskylensInit();
  display.oledDisplayCenterln(1, 1, "Initializing IMU", true);
  imuInit();
  // pixy2Init();
  // display.oledDisplayCenterln(1, 1, "Initializing pixy2", true);
  // pixy.init();
  display.oledClear();

  // Follow this to create your own thread
  // displayThread.onRun(displayData);
  // displayThread.setInterval(0);
  // controller.add(&displayThread);

  // OC1Thread.onRun(OC1main);
  // OC1Thread.setInterval(10);
  // controller.add(&OC1Thread);

  // // OC2Thread.onRun(OC2main);
  // // OC2Thread.setInterval(15);
  // // controller.add(&OC2Thread);

  // Ultra1Thread.onRun(ultra1Dist_cmloop);
  // Ultra1Thread.setInterval(10);
  // controller.add(&Ultra1Thread);
  // Ultra2Thread.onRun(ultra2Dist_cmloop);
  // Ultra2Thread.setInterval(10);
  // controller.add(&Ultra2Thread);

  // huskylensThread.onRun(getNearestPillarGlobal);
  // huskylensThread.setInterval(10);
  // controller.add(&huskylensThread);

  // colorThread.onRun(getColorTypeloop);
  // colorThread.setInterval(10);
  // controller.add(&colorThread);

  // Laser1Thread.onRun(getLaser1Distloop);
  // Laser1Thread.setInterval(0);
  // controller.add(&Laser1Thread);
  // Laser2Thread.onRun(getLaser2Distloop);
  // Laser2Thread.setInterval(0);
  // controller.add(&Laser2Thread);

  // pixy2Thread.onRun(getPixy2NearestPillarGlobal);
  // pixy2Thread.setInterval(10);
  // controller.add(&pixy2Thread);
}

void loop() {
  // controller.run();
  // OC1main();
  ultra_testing();
  // ultra1_event_timer_sketch();
}
