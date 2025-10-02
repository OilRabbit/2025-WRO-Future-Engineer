#pragma once
#include <Arduino.h>
#include "tft.h"

// Pick safe, broken-out pins on your board:
#define EnablePin    5    // PWM to L298N ENA
#define ForwardPin   6   // L298N IN1
#define BackwardPin  7   // L298N IN2
#define EncoderPinA  38   // OK if actually wired and available on your board
#define EncoderPinB  46

extern volatile long total_pulse;

void read_encoder();
void reset_pulse();
void motor_init();
void motor_move(int speed_percentage);    // + = fwd, - = back
void motor_backward(int speed_percentage);
void motor_brake();
// void motor_encloop(void* parameters);
void showEncoder(TFT_COLUMN column, int line_number, int text_size, uint16_t text_colour, bool clearDisplay);