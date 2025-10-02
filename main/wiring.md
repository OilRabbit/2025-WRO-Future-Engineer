### SPI LCD (ST7789 HAT)

* SCLK → **GPIO40**
* MOSI → **GPIO41**
* CS   → **GPIO21**
* DC   → **GPIO42**
* RST  → **GPIO1**
* BL   → **GPIO2** (drive HIGH to turn backlight on)
* VCC → 3V3, GND → GND

### HAT Buttons & Joystick  *(active-LOW, use `INPUT_PULLUP`)*

* KEY1 → **GPIO12**
* KEY2 → **GPIO13**
* KEY3 → **GPIO14**
* JOY UP → **GPIO15**
* JOY DOWN → **GPIO16**
* JOY LEFT → **GPIO17**
* JOY RIGHT → **GPIO18**
* JOY PRESS → **GPIO11**

### Qwiic IMU (ICM-20948, I²C @ 3.3 V)

* SDA → **GPIO9**
* SCL → **GPIO8**
* VCC → 3V3, GND → GND

### Pixy 2.1 (UART)

* ESP32 **TX** → Pixy **RX** → **GPIO6**
* ESP32 **RX** → Pixy **TX** → **GPIO7**
* Power Pixy from **5V**; GND common

### HC-SR04 x2  *(power both from 5 V; **ECHO must be level-shifted to 3.3 V**)*

* Sensor A: **TRIG → GPIO3**, **ECHO → GPIO45** (through resistor divider)
* Sensor B: **TRIG → GPIO47**, \*\*ECHO → **GPIO48** (through resistor divider)

### L298N (single DC motor)

* ENA (PWM) → **GPIO5**
* IN1 → **GPIO6**
* IN2 → **GPIO7**
* ENCPinA → **GPIO38**
* ENCPinB → **GPIO46**
* L298N logic 5V → your 5 V rail; motor VIN → motor battery; GND common

### Servo

* Signal → **GPIO4** (LEDC @ 50 Hz)
* Power servo from 5–6 V (not 3V3); GND common

#### Notes / gotchas

* **Do not use**: GPIO0 (BOOT strap), **GPIO19/20** (USB D−/D+), **GPIO45/46** (input-only; avoid for TRIG/PWM).
