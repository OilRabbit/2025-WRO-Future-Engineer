#include "HardwareSerial.h"
#include "buttons.h"
#include "imu.h"
#include "steering.h"
#include "motor.h"

static inline uint8_t pinOf(TFT_BTNS b) { return static_cast<uint8_t>(b); }

void btn_init(){
  // pinMode(pinOf(TFT_JS_BTN), INPUT_PULLUP);
  pinMode(pinOf(TFT_BTN1), INPUT_PULLUP);
  pinMode(pinOf(TFT_BTN2), INPUT_PULLUP);
  pinMode(pinOf(TFT_BTN3), INPUT_PULLUP);
  // pinMode(pinOf(TFT_JS_UP), INPUT_PULLUP);
  // pinMode(pinOf(TFT_JS_DOWN), INPUT_PULLUP);
  // pinMode(pinOf(TFT_JS_LEFT), INPUT_PULLUP);
  // pinMode(pinOf(TFT_JS_RIGHT), INPUT_PULLUP);
}

bool is_btn_pressed(TFT_BTNS btn){
  return (digitalRead(pinOf(btn)) == LOW);
}

bool is_btn_bumped(TFT_BTNS btn){
  const uint8_t pin = pinOf(btn);
  static uint64_t wasPressedMask = 0;
  const uint64_t bit = 1ULL << pin;
  const bool nowPressed = (digitalRead(pin) == LOW);
  const bool edge = nowPressed && !(wasPressedMask & bit);

  if (nowPressed) wasPressedMask |= bit;
  else            wasPressedMask &= ~bit;

  return edge;
}

// For testing only
void showbtnState(TFT_COLUMN column, int line_number, int text_size, uint16_t text_colour = TFT_WHITE, bool clearDisplay = false){
  String btn_text = String("Bt1:") + (is_btn_pressed(TFT_BTN1) ? "Pre" : "Rel");
  static bool run = false;
  static bool finished = false;
  if (is_btn_pressed(TFT_BTN2)) imu_resetYaw();
  if (is_btn_pressed(TFT_BTN3)) tft.clear();
  if (is_btn_bumped(TFT_BTN1)){
    finished = false;
    run = true;
  }
  if (run && !finished){
    // Serial.println("hi");
    finished = motor_degree_accel(1000, 300, 300, -20);
    if (finished) {
      run = false;
      motor_stop(BRAKE);
    }
  }
  // static int nump = 0;
  // nump += is_btn_bumped(TFT_BTN1);
  // String btn_text = String("NumP:") + String(nump);
  if (column == TFT_LEFT_CLN){
    tft.clearln(TFT_LEFT_CLN, line_number);
    tft.displayLeftln(line_number, text_size, btn_text.c_str(), text_colour, false);
  } else {
    tft.clearln(TFT_RIGHT_CLN, line_number);
    tft.displayRightln(line_number, text_size, btn_text.c_str(), text_colour, false);
  }
}