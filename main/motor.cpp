#include "HardwareSerial.h"
#include "Motor.h"

#include "motor.h"
#include <math.h>
#include <driver/pulse_cnt.h>   // PCNT (Arduino-ESP32 3.x)

// -------------------- Globals --------------------
volatile long MOTOR_ENCODER_COUNT = 0;
long          MOTOR_ENCODER_VALUE = 0;

// PWM polarity (true = active-low like your AVR COM0A=11 trick)
const bool PWM_ACTIVE_LOW = true;

// -------------------- PCNT handles --------------------
static pcnt_unit_handle_t    s_pcnt_unit = nullptr;
static pcnt_channel_handle_t s_chan_a    = nullptr;
static pcnt_channel_handle_t s_chan_b    = nullptr;

// -------------------- helpers --------------------
static inline uint32_t duty_from_percent(int p) {
  int mag = abs(p);
  if (mag > 100) mag = 100;
  uint32_t d = (uint32_t)mag * PWM_MAX_DUTY / 100;
  if (PWM_ACTIVE_LOW) d = PWM_MAX_DUTY - d;
  return d;
}

static inline void apply_pwm_dir(int p) {
  digitalWrite(PIN_M_DIR, (p >= 0) ? HIGH : LOW);
  digitalWrite(PIN_M_BRAKE, HIGH);                      // release brake
  ledcWrite(PIN_M_PWM, duty_from_percent(p));
}

// -------------------- PCNT encoder setup --------------------
static void encoder_pcnt_init() {
  pcnt_unit_config_t unit_cfg = {};
  unit_cfg.low_limit  = INT16_MIN;   // 16-bit hardware counter; read often or extend in SW if needed
  unit_cfg.high_limit = INT16_MAX;
  ESP_ERROR_CHECK(pcnt_new_unit(&unit_cfg, &s_pcnt_unit));

  // Glitch filter to suppress EMI/bounce (tune 200..2000 ns)
  pcnt_glitch_filter_config_t filt_cfg = {};
  filt_cfg.max_glitch_ns = 1000;
  ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(s_pcnt_unit, &filt_cfg));

  // Channel A: edge=A, level=B
  pcnt_chan_config_t ch_a_cfg = {};
  ch_a_cfg.edge_gpio_num  = PIN_ENC_A;
  ch_a_cfg.level_gpio_num = PIN_ENC_B;
  ESP_ERROR_CHECK(pcnt_new_channel(s_pcnt_unit, &ch_a_cfg, &s_chan_a));

  // Channel B: edge=B, level=A
  pcnt_chan_config_t ch_b_cfg = {};
  ch_b_cfg.edge_gpio_num  = PIN_ENC_B;
  ch_b_cfg.level_gpio_num = PIN_ENC_A;
  ESP_ERROR_CHECK(pcnt_new_channel(s_pcnt_unit, &ch_b_cfg, &s_chan_b));

  // Quadrature actions (standard mapping)
  ESP_ERROR_CHECK(pcnt_channel_set_edge_action(
    s_chan_a, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
  ESP_ERROR_CHECK(pcnt_channel_set_level_action(
    s_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

  ESP_ERROR_CHECK(pcnt_channel_set_edge_action(
    s_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
  ESP_ERROR_CHECK(pcnt_channel_set_level_action(
    s_chan_b, PCNT_CHANNEL_LEVEL_ACTION_INVERSE, PCNT_CHANNEL_LEVEL_ACTION_KEEP));

  ESP_ERROR_CHECK(pcnt_unit_enable(s_pcnt_unit));
  ESP_ERROR_CHECK(pcnt_unit_clear_count(s_pcnt_unit));
  ESP_ERROR_CHECK(pcnt_unit_start(s_pcnt_unit));
}

// -------------------- Public API --------------------
void motor_init() {
  // Keep brake asserted until PWM ready
  digitalWrite(PIN_M_BRAKE, LOW);

  // LEDC (Arduino-ESP32 3.x pin-based API)
  ledcAttach(PIN_M_PWM, PWM_FREQ_HZ, PWM_BITS);
  ledcWrite(PIN_M_PWM, PWM_ACTIVE_LOW ? PWM_MAX_DUTY : 0); // OFF

  pinMode(PIN_M_DIR, OUTPUT);
  digitalWrite(PIN_M_DIR, LOW);

  pinMode(PIN_M_BRAKE, OUTPUT);
  // leave brake asserted until user runs motor_move()

  // Encoder pins & PCNT
  pinMode(PIN_ENC_A, INPUT_PULLUP);
  pinMode(PIN_ENC_B, INPUT_PULLUP);
  encoder_pcnt_init();

  MOTOR_ENCODER_COUNT = 0;
  MOTOR_ENCODER_VALUE = 0;
}

void read_encoder() {
  int hw = 0;
  if (s_pcnt_unit) pcnt_unit_get_count(s_pcnt_unit, &hw);
  MOTOR_ENCODER_COUNT = (long)hw;
  MOTOR_ENCODER_VALUE = (long)hw;
}

void reset_encoder() {
  if (s_pcnt_unit) pcnt_unit_clear_count(s_pcnt_unit);
  MOTOR_ENCODER_COUNT = 0;
  MOTOR_ENCODER_VALUE = 0;
}

void motor_move(int speed_percentage) {
  if (speed_percentage > 100)  speed_percentage = 100;
  if (speed_percentage < -100) speed_percentage = -100;
  apply_pwm_dir(speed_percentage);
}

void motor_stop(BRAKE_TYPE brake_method) {
  // PWM off
  ledcWrite(PIN_M_PWM, PWM_ACTIVE_LOW ? PWM_MAX_DUTY : 0);
  if (brake_method == BRAKE) digitalWrite(PIN_M_BRAKE, LOW);   // active brake
  else                       digitalWrite(PIN_M_BRAKE, HIGH);  // coast
}

// Blocking move by degrees (sign of deg = direction)
void motor_on_degree(int deg, int speed_percentage, BRAKE_TYPE brake_method) {
  if (deg == 0 || speed_percentage == 0) { motor_stop(brake_method); return; }

  // target ticks (abs)
  long target_ticks = (long)(( (double)ENCODER_CPR * (double)abs(deg) ) / 360.0);
  if (target_ticks == 0) target_ticks = 1;

  // start reference
  read_encoder();
  long start = MOTOR_ENCODER_VALUE;

  // direction from sign of deg
  int dir = (deg >= 0) ? 1 : -1;
  motor_move(dir * abs(speed_percentage));

  const uint32_t timeout_ms = 10000; // safety
  uint32_t t0 = millis();
  for (;;) {
    read_encoder();
    long progressed = labs(MOTOR_ENCODER_VALUE - start);
    if (progressed >= target_ticks) break;
    if ((uint32_t)(millis() - t0) > timeout_ms) break;
    vTaskDelay(pdMS_TO_TICKS(2));
  }

  motor_stop(brake_method);
}

typedef enum {
  ACCEL,
  CONST_SPEED,
  DECEL,
  ENDED,
} MOTOR_ACCEL_STATE;

bool motor_degree_accel(float dist, float accel_dist, float decel_dist, float init_speed, float max_speed){
  static bool init = true;
  static float curr_speed;
	static float start_pos;
	static float acceleration;
  static MOTOR_ACCEL_STATE state;

  if (init){
    if ((accel_dist + decel_dist) > dist){
      accel_dist = dist / 2;
      decel_dist = dist / 2;
    }
    start_pos = MOTOR_ENCODER_COUNT;
    acceleration = (max_speed * 10 / (accel_dist + 1));
    curr_speed += acceleration;
    state = ACCEL;
		init = false;
	}

  vTaskDelay(10 / portTICK_PERIOD_MS);
  motor_move(curr_speed);
  if (abs(MOTOR_ENCODER_COUNT - start_pos) < accel_dist){
      if (abs(curr_speed) > abs(max_speed)){
        curr_speed = max_speed;
      }
      curr_speed += acceleration;
  } else if (abs(MOTOR_ENCODER_COUNT - start_pos) < abs(dist - decel_dist)) curr_speed = max_speed;
  else if (abs(MOTOR_ENCODER_COUNT - start_pos) < dist){
    acceleration = (-max_speed * 10 / (decel_dist + 1));
    if (abs(curr_speed) <= 10){
      if (abs(MOTOR_ENCODER_COUNT - start_pos) <= dist){
        curr_speed = 10 * max_speed / abs(max_speed);
      }
    } else{
      curr_speed += acceleration;
    }
  } else {
    init = true;
    return true;
  }
  return false;
}

void motor_encloop(void* parameters){
  while(1){
    read_encoder();
    vTaskDelay(5 / portTICK_PERIOD_MS);
  }
}

void showEncoder(TFT_COLUMN column, int line_number, int text_size, uint16_t text_colour = TFT_WHITE, bool clearDisplay = false){
  read_encoder();
  String encoder_text = "Enc:" + String(long(MOTOR_ENCODER_COUNT));
  if (column == TFT_LEFT_CLN){
    tft.clearln(TFT_LEFT_CLN, line_number);
    tft.displayLeftln(line_number, text_size, encoder_text.c_str(), text_colour, false);
  } else {
    tft.clearln(TFT_RIGHT_CLN, line_number);
    tft.displayRightln(line_number, text_size, encoder_text.c_str(), text_colour, false);
  }
}