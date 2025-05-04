#include <DFRobot_HuskyLens.h>
#include <HUSKYLENS.h>
#include <HUSKYLENSMindPlus.h>
#include <HuskyLensProtocolCore.h>
#include <MatrixMiniR4.h>
#include <Thread.h>
#include <ThreadController.h>
#include <string.h>
#include "Timer.h"
#include "oled.h"
#include "display.h"
#include "battery.h"
#include "timestamp.h"
#include "steering.h"
#include "laser.h"
#include "OC1.h"
#include "huskylens.h"
#include "imu.h"

// Define your threads here
Thread displayThread = Thread();
Thread OC1Thread = Thread();
Thread Laser1Thread = Thread();
Thread Laser2Thread = Thread();

// Controller to manage all threads
ThreadController controller = ThreadController();

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
  laserInit(I2CPORT1);
  laserInit(I2CPORT2);
  huskylensInit();
  display.oledDisplayCenterln(1, 1, "Initializing IMU", true);
  delay(1000);
  imuInit();
  display.oledClear();

  // Follow this to create your own thread
  displayThread.onRun(displayData);
  displayThread.setInterval(1);

  OC1Thread.onRun(OpenChallenge300);
  // OC1Thread.onRun(OpenChallengeMeanDist);
  OC1Thread.setInterval(1);

  Laser1Thread.onRun(getLaser1Distloop);
  Laser1Thread.setInterval(0);

  Laser2Thread.onRun(getLaser2Distloop);
  Laser2Thread.setInterval(0);

  // Add the threads to the controller
  controller.add(&displayThread);
  controller.add(&OC1Thread);
  controller.add(&Laser1Thread);
  controller.add(&Laser2Thread);

  display.oledDisplayCenterln(1, 1, "Starting...", true);
  display.oledClear();
}

void loop() {
  // Start all the threads in this controller
  controller.run();
  
  // huskylensColorRegTest();
  // steering(50);
  // MiniR4.M2.setPower(100);
}