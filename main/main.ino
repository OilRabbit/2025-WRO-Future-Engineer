// ===== ESP32-S3 motor control + encoder (LEDC + Ticker) =====
#include <Arduino.h>
#include <Ticker.h>
#include <math.h>   // for lrintf (round-to-nearest)

// ---- Pin map (your request) ----
#define PIN_M_BRAKE   5    // M_BTAK -> GPIO5 (LOW = brake, HIGH = run)
#define PIN_M_PWM     6    // M_PWM  -> GPIO6  (LEDC PWM output)
#define PIN_M_DIR     7    // M_DIR  -> GPIO7
#define PIN_ENC_A     39   // Encoder A -> GPIO39 (interrupt pin)
#define PIN_ENC_B     38   // Encoder B -> GPIO38 (often RGB LED on some S3 boards)

// ---- PWM (LEDC) ----
#define PWM_FREQ_HZ   20000      // 20 kHz (quiet for motors)
#define PWM_BITS      8          // 0..255 like AVR
#define PWM_MAX_DUTY  ((1u << PWM_BITS) - 1)

// If your driver expects "active-low" PWM (like your AVR COM0A1:0 = 11), keep true.
const bool PWM_ACTIVE_LOW = true;

// ---- Types & globals ----
#define u32 unsigned int

// control targets
int   Target_Encoder = 4;     // counts per 10 ms
int   Current_Encoder = 0;    // counts measured in last 10 ms

// encoder counters
volatile long Encoder_Num = 0;   // accumulates pulses between control ticks (ISR writes)
long Position_Count = 0;         // total position in pulses (running sum)

// PI controller gains
float Velocity_KP = 1.5f;
float Velocity_KI = 0.5f;

// controller state
int   Motor_Pwm = 0;    // command after limiting (-254..254)
int   Test_cnt  = 0;    // to flip Target_Encoder every 5s (demo)

// scheduler
Ticker controlTicker;
volatile bool control_due = false;

// ---------- Incremental PI with anti-windup (float accumulator) ----------
int Incremental_PI(int Encoder, int Target)
{
  static float Bias = 0.0f, Pwm = 0.0f, Last_bias = 0.0f;

  Bias = (float)Target - (float)Encoder;
  float inc = Velocity_KP * (Bias - Last_bias) + Velocity_KI * Bias;

  // tentative update + clamp (anti-windup)
  float nextPwm = Pwm + inc;
  if (nextPwm > 254.0f)  nextPwm = 254.0f;
  if (nextPwm < -254.0f) nextPwm = -254.0f;

  Pwm = nextPwm;
  Last_bias = Bias;

  return (int)lrintf(Pwm);  // round to nearest int
}

float PWM_Limit(float IN, int maxv, int minv) {
  float OUT = IN;
  if (OUT > maxv) OUT = maxv;
  if (OUT < minv) OUT = minv;
  return OUT;
}

u32 myabs(long a) { return (a < 0) ? (u32)(-a) : (u32)a; }

// ---------- Motor PWM / DIR ----------
void Set_PWM(int motorPwm) {
  // Direction (flip HIGH/LOW if motor runs opposite)
  digitalWrite(PIN_M_DIR, (motorPwm >= 0) ? HIGH : LOW);

  // Map absolute PWM (0..254) to 0..PWM_MAX_DUTY
  int duty = (int)myabs(motorPwm);
  if (duty > 254) duty = 254;
  uint32_t d = (uint32_t)duty * PWM_MAX_DUTY / 254;

  // Invert if hardware expects active-low PWM
  if (PWM_ACTIVE_LOW) d = PWM_MAX_DUTY - d;

  // ESP32 core v3.x: pin-based LEDC write
  ledcWrite(PIN_M_PWM, d);
}

// ---------- 10 ms scheduler ----------
void control_10ms() { control_due = true; }

void do_control_step() {
  // snapshot and reset the interval count
  long pulses = Encoder_Num;
  Encoder_Num = 0;

  Current_Encoder = (int)pulses;     // pulses in the last 10 ms
  Position_Count += pulses;          // accumulate total position

  // demo: flip target every 5 seconds
  if (++Test_cnt == 500) {           // 500 * 10 ms = 5 s
    Test_cnt = 0;
    Target_Encoder = -Target_Encoder;
  }

  Motor_Pwm = Incremental_PI(Current_Encoder, Target_Encoder);
  Motor_Pwm = (int)PWM_Limit(Motor_Pwm, 254, -254);
  Set_PWM(Motor_Pwm);
}

// ---------- Encoder ISR (avoid ++/-- on volatile) ----------
void IRAM_ATTR READ_ENCODER() {
  // Quadrature decode: A is the interrupt source; B defines direction
  if (digitalRead(PIN_ENC_A) == LOW) {
    if (digitalRead(PIN_ENC_B) == LOW) Encoder_Num -= 1;
    else                               Encoder_Num += 1;
  } else {
    if (digitalRead(PIN_ENC_B) == LOW) Encoder_Num += 1;
    else                               Encoder_Num -= 1;
  }
}

void setup() {
  Serial.begin(115200);

  // PWM setup (ESP32 core v3.x pin-based LEDC API)
  ledcAttach(PIN_M_PWM, PWM_FREQ_HZ, PWM_BITS);
  // Start "off" (we invert later if active-low)
  ledcWrite(PIN_M_PWM, PWM_ACTIVE_LOW ? PWM_MAX_DUTY : 0);

  // Motor control pins
  pinMode(PIN_M_DIR, OUTPUT);
  pinMode(PIN_M_BRAKE, OUTPUT);
  digitalWrite(PIN_M_DIR, HIGH);
  digitalWrite(PIN_M_BRAKE, HIGH);   // HIGH = free-run, LOW = brake

  // Encoder pins
  // If your encoder is push-pull, INPUT is fine; for open-collector, keep PULLUP or add externals.
  pinMode(PIN_ENC_A, INPUT_PULLUP);
  pinMode(PIN_ENC_B, INPUT_PULLUP);

  // 10 ms control loop
  controlTicker.attach_ms(10, control_10ms);

  // Interrupt on A, both edges
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), READ_ENCODER, CHANGE);
}

void loop() {
  if (control_due) {
    control_due = false;
    do_control_step();
  }

  // Debug print every 100 ms
  static uint32_t t_last = 0;
  uint32_t now = millis();
  if (now - t_last >= 100) {
    t_last = now;
    Serial.print("IntervalCnt = ");
    Serial.print(Current_Encoder);     // pulses per 10 ms
    Serial.print("    Position = ");
    Serial.print(Position_Count);      // running total pulses
    Serial.print("    Target = ");
    Serial.print(Target_Encoder);      // desired pulses per 10 ms
    Serial.print("    Motor_Pwm = ");
    Serial.println(Motor_Pwm);         // -254..254 (after anti-windup/limit)
  }
}
