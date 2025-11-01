### SPI LCD (ST7789 HAT)

* SCLK (1st row, pin 12) → **GPIO40**
* MOSI (1st row, pin 10) → **GPIO41**
* CS (2nd row, pin 12) → **GPIO21**
* DC (2nd row, pin 11) → **GPIO42**
* RST (1st row, pin 7) → **GPIO1**
* BL (2nd row, pin 9) → **GPIO2** (drive HIGH to turn backlight on)
* VCC (1st row, pin 1) → 3V3
* GND (2nd row, pin 3) → GND

### HAT Buttons & Joystick  *(active-LOW, use `INPUT_PULLUP`)*

* KEY1 (2nd row, pin 20) → **GPIO12**
* KEY2 (2nd row, pin 19) → **GPIO13**
* KEY3 (2nd row, pin 18) → **GPIO14**
(UNUSED) * JOY UP → **GPIO15**
(UNUSED) * JOY DOWN → **GPIO16**
(UNUSED) * JOY LEFT → **GPIO17**
(UNUSED) * JOY RIGHT → **GPIO18**
(UNUSED) * JOY PRESS → **GPIO11**

### Qwiic IMU (ICM-20948, I²C @ 3.3 V), share with Pixy2

* SDA → **GPIO9** (purple)
* SCL → **GPIO8**
* VCC → 3V3, GND → GND

### Pixy 2.1 (UART)

* SDA (3rd row, pin 1) → **GPIO8**
* SCL (5th row, pin 1) → **GPIO9**
* VCC (1st row, pin 2) → 3V3
* GND (3rd row, pin 2) → GND

### HC-SR04 x2  *(power both from 5 V; **ECHO must be level-shifted to 3.3 V**)*

* Sensor A: **TRIG → GPIO10**, **ECHO → GPIO11** (through resistor divider)
* Sensor B: **TRIG → GPIO47**, **ECHO → **GPIO48** (through resistor divider)

### L298N (single DC motor)

* ENA (PWM) → **GPIO5**
* IN1 → **GPIO6**
* IN2 → **GPIO7**
* ENCPinA → **GPIO38** (yellow)
* ENCPinB → **GPIO46** (green)
* blue wire from matrix motor (5V)
* white wire from Matrix motor (GND)
* L298N logic 5V → your 5 V rail; motor VIN → motor battery; GND common

### Servo

* Signal (orang) → **GPIO4** (LEDC @ 50 Hz)
* Power servo (red) from 5–6 V (not 3V3); GND (brown) common

#### Notes / gotchas

* **Do not use**: GPIO0 (BOOT strap), **GPIO19/20** (USB D−/D+), **GPIO45/46** (input-only; avoid for TRIG/PWM).
