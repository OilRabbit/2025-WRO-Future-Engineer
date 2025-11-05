#pragma once
#include <Arduino.h>
#include <Ticker.h>
#include <math.h>
#include "tft.h"

// =================== Pins (edit as needed) ===================
#define PIN_M_BRAKE   5    // LOW = brake, HIGH = run
#define PIN_M_PWM     6    // LEDC PWM pin
#define PIN_M_DIR     7    // DIR pin
#define PIN_ENC_A     39   // encoder A
#define PIN_ENC_B     17   // encoder B

// =================== Encoder settings ===================
#ifndef ENCODER_CPR
#define ENCODER_CPR   600   // counts per mechanical revolution (after gearbox if you want shaft degrees)
#endif

// =================== PWM settings ===================
#define PWM_FREQ_HZ   20000     // 20 kHz
#define PWM_BITS      8
#define PWM_MAX_DUTY  ((1u << PWM_BITS) - 1)
extern const bool PWM_ACTIVE_LOW;   // defined in motor.cpp

// Stop modes
typedef enum{
  COAST = 0, 
  BRAKE = 1 
} BRAKE_TYPE;

// -------------------- Globals you asked for --------------------
extern volatile long MOTOR_ENCODER_COUNT;   // live value (PCNT reading)
extern long          MOTOR_ENCODER_VALUE;   // last snapshotted value by read_encoder()

// -------------------- Safe pre-init (call first!) --------------------
static inline void motor_preinit_safe() {
  pinMode(PIN_M_BRAKE, OUTPUT);
  digitalWrite(PIN_M_BRAKE, LOW);                               // assert brake

  pinMode(PIN_M_PWM, OUTPUT);
  digitalWrite(PIN_M_PWM, /*OFF*/ PWM_ACTIVE_LOW ? HIGH : LOW); // force PWM OFF level

  pinMode(PIN_M_DIR, OUTPUT);
  digitalWrite(PIN_M_DIR, LOW);
}

// -------------------- API --------------------
void motor_init();                                   // call in setup()
void read_encoder();                                 // snapshot -> MOTOR_ENCODER_VALUE (+ mirror COUNT)
void reset_encoder();                                // zero both

void motor_move(int speed_percentage);               // -100..100 (%)
void motor_stop(BRAKE_TYPE brake_method);                   // BRAKE or COAST

void motor_on_degree(int deg, int speed_percentage, BRAKE_TYPE brake_method);
bool motor_degree_accel(float dist, float accel_dist, float decel_dist, float max_speed);
void motor_encloop(void* parameters);
void showEncoder(TFT_COLUMN column, int line_number, int text_size, uint16_t text_colour, bool clearDisplay);