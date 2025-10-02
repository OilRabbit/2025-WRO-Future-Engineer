#include "esp32-hal-gpio.h"
#include "motor.h"

volatile long enc_value = 0;

void read_encoder(){
  if (digitalRead(EncoderPinA) == 0){
    enc_value += 1;
  } else {
    enc_value -= 1;
  }
}

void reset_encoder(){
  enc_value = 0;
}

void motor_init(){
  // Set all the motor control pins to outputs
  pinMode(EnablePin, OUTPUT);
  pinMode(ForwardPin, OUTPUT);
  pinMode(BackwardPin, OUTPUT);
  pinMode(EncoderPinA, INPUT);
  pinMode(EncoderPinB, INPUT);

  // Turn off motors - Initial state
  digitalWrite(ForwardPin, LOW);
  digitalWrite(BackwardPin, LOW);
  attachInterrupt(digitalPinToInterrupt(EncoderPinB), read_encoder, RISING);
}

void motor_move(int speed_percentage){
  // if(speed_percentage > 0){
    digitalWrite(ForwardPin, HIGH);
    digitalWrite(BackwardPin, LOW);
    analogWrite(EnablePin, speed_percentage * 255 / 100);
  // } else if (speed_percentage < 0){
  //   digitalWrite(ForwardPin, HIGH);
  //   digitalWrite(BackwardPin, LOW);
  //   analogWrite(EnablePin, speed_percentage);
  // }
}

void motor_backward(int speed_percentage){
  digitalWrite(ForwardPin, LOW);
  digitalWrite(BackwardPin, HIGH);
  analogWrite(EnablePin, speed_percentage * 255 / 100);
}

void motor_brake(){
  digitalWrite(ForwardPin, LOW);
  digitalWrite(BackwardPin, LOW);
  analogWrite(EnablePin, 0);
}

// void motor_encloop(void* parameters){
//   while(1){
//     read_encoder();
//     vTaskDelay(10 / portTICK_PERIOD_MS);
//   }
// }

void showEncoder(TFT_COLUMN column, int line_number, int text_size, uint16_t text_colour = TFT_WHITE, bool clearDisplay = false){
  String encoder_text = "Enc:" + String(int(enc_value));
  if (column == TFT_LEFT_CLN){
    tft.clearln(TFT_LEFT_CLN, line_number);
    tft.displayLeftln(line_number, text_size, encoder_text.c_str(), text_colour, false);
  } else {
    tft.clearln(TFT_RIGHT_CLN, line_number);
    tft.displayRightln(line_number, text_size, encoder_text.c_str(), text_colour, false);
  }
}