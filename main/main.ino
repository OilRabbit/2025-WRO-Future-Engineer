#include <MatrixMiniR4.h>
#include "Timer.h"
#include <string.h>
#include "displayhelper.h"
#include "display.h"
#include "battery.h"
#include "timestamp.h"
#include <Thread.h>
#include <ThreadController.h>

// Define your threads here
Thread displayThread = Thread();

// Controller to manage all threads
ThreadController controller = ThreadController();

void setup() {
  // Init
  Serial.begin(115200);
  MiniR4.begin();
  internalClock.start();
  display.oledDisplayCenterln(1, 1, "Initializing...", true);
  MiniR4.PWR.setBattCell(2);  // 18650x2, two-cell (2S)
  MiniR4.M2.setBrake(true);
  display.oledClear();

  // Follow this to create your own thread
  displayThread.onRun(displayData);
  displayThread.setInterval(1);

  // Add the threads to the controller
  controller.add(&displayThread);

  display.oledDisplayCenterln(1, 1, "Starting...", true);
  MiniR4.OLED.clearDisplay();
}

void loop() {
  // Start all the threads in this controller
  controller.run();
}