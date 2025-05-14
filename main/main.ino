#include "main.h"

// Define your threads here
Thread displayThread = Thread();
Thread OC1Thread = Thread();
Thread OC2Thread = Thread();
Thread Ultra1Thread = Thread();
Thread Ultra2Thread = Thread();
// Thread Laser1Thread = Thread();
// Thread Laser2Thread = Thread();

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
  ultraInit(ultra1);
  ultraInit(ultra2);
  // laserInit(I2CPORT1);
  // laserInit(I2CPORT2);
  huskylensInit();
  display.oledDisplayCenterln(1, 1, "Initializing IMU", true);
  delay(1000);
  imuInit();
  display.oledClear();

  // Follow this to create your own thread
  displayThread.onRun(displayData);
  displayThread.setInterval(0);

  // OC1Thread.onRun(OC1main);
  // OC1Thread.setInterval(10);

  OC2Thread.onRun(OC2main);
  OC2Thread.setInterval(10);

  Ultra1Thread.onRun(getultra1Distloop);
  Ultra1Thread.setInterval(10);
  Ultra2Thread.onRun(getultra2Distloop);
  Ultra2Thread.setInterval(10);

  // Laser1Thread.onRun(getLaser1Distloop);
  // Laser1Thread.setInterval(0);
  // Laser2Thread.onRun(getLaser2Distloop);
  // Laser2Thread.setInterval(0);

  // Add the threads to the controller
  controller.add(&displayThread);
  // controller.add(&OC1Thread);
  controller.add(&OC2Thread);
  controller.add(&Ultra1Thread);
  controller.add(&Ultra2Thread);
  // controller.add(&Laser1Thread);
  // controller.add(&Laser2Thread);

  display.oledDisplayCenterln(1, 1, "Starting...", true);
  display.oledClear();
}

float duration, distance;
void loop() {
  // Start all the threads in this controller
  controller.run();
  // Serial.println(laser1Dist);
  // huskylensColorRegTest();
  // steering(50);
  // MiniR4.M2.setPower(100);
}