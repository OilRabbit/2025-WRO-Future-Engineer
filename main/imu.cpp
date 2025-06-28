#include "MatrixMiniR4.h"
#include "imu.h"

Kalman kalmanZ;

/* Calibration values for Kalman filter */
double zeroValue[5] = { -200, 44, 660, 52.3, -18.5 };

/* Global variable for Kalman filter */
double gyroZangle = 180;
double compAngleZ = 180;
double kalmanAngleZ = 180;

/* Global variable for storing time */
unsigned long timer;

/**
 * @brief Initialize the IMU
 * 
 */
void imuInit(){
  MiniR4.Motion.resetIMUValues();
  long sum = 0;
  const int samples = 500;
  for (int i = 0; i < samples; i++) {
    sum += MiniR4.Motion.getGyro(MiniR4Motion::AxisType::Z);
    delay(1);
  }
  zeroValue[3] = sum / (double)samples;
}

double unwrap_previous_angle = 0;
double unwrap_accum_angle = 0;

void resetUnwrap(){
  unwrap_previous_angle = MiniR4.Motion.getEuler(MiniR4Motion::AxisType::Yaw);
  unwrap_accum_angle = 0;
}


/**
 * @brief Convert angle in compass to angle in gyro (i.e. will not reset to 180 when < -180)
 * 
 * @param current_angle; double; Basically is the euler angle get from the IMU
 * @return double; The angle in gyro
 */
double unwrapAngle(double current_angle){
  double delta = current_angle - unwrap_previous_angle;

  if (delta > 180) {
    delta -= 360;
  } else if (delta < -180) {
    delta += 360;
  }

  unwrap_accum_angle += delta;
  unwrap_previous_angle = current_angle;

  return unwrap_accum_angle;
}

/**
 * @brief Get the IMU value in gyro
 * 
 * @return double; The angle in gyro
 */
double getIMU(){
  return unwrapAngle(MiniR4.Motion.getEuler(MiniR4Motion::AxisType::Yaw)) * -1;
}

void resetIMU(){
  MiniR4.Motion.resetIMUValues();
  delay(100); // optional: allow IMU to stabilize
  resetUnwrap(); // <-- Reset software angle tracker
}


/**
 * @brief (UNUSED) Get the IMU z-axis rotation angle with the use of Kalman filter
 * 
 * @return double; The IMU z-axis rotation angle after applying Kalman filter
 */
double getIMUKalman(){
  kalmanZ.setQangle(0.01);
  kalmanZ.setQbias(0.0007);
  kalmanZ.setRmeasure(0.01);
  // Time delta
  double dt = (double)(micros() - timer) / 1e6;
  timer = micros(); // update timer *after* dt is calculated

  // Read raw gyro Z (in deg/sec)
  double gyroZrate = -(((double)MiniR4.Motion.getGyro(MiniR4Motion::AxisType::Z) - zeroValue[3]) / 14.375);

  // Read Euler angle from IMU (yaw)
  double accZangle = unwrapAngle(MiniR4.Motion.getEuler(MiniR4Motion::AxisType::Yaw));

  // Complementary filter
  compAngleZ = 0.93 * (compAngleZ + gyroZrate * dt) + 0.07 * accZangle;

  // Kalman filter
  kalmanAngleZ = kalmanZ.getAngle(accZangle, gyroZrate, dt);

  // Simple gyro integration (for comparison)
  gyroZangle += gyroZrate * dt;

  // Print values
  // Serial.print("Gyro: "); Serial.print(gyroZangle); Serial.print("\t");
  // Serial.print("Euler: "); Serial.print(accZangle); Serial.print("\t");
  // Serial.print("Complementary: "); Serial.print(compAngleZ); Serial.print("\t");
  // Serial.print("Kalman: "); Serial.print(kalmanAngleZ); Serial.println();
  return kalmanAngleZ;
}

/**
 * @brief A function to put into display thread for showing the z-axis rotation angle from the IMU
 * 
 * @param method; (enum) IMU_METHOD; The method used to get the angle
 * @param column; (enum) COLUMN; A enum defined in oled.h indicating which column the data should be displaced at
 * @param line_number; int; The line number where the data should be displaced at (0 ~ 3)
 * @param size; int; The size of the text being displaced (1 ~ 2)
 * @param clearDisplay; bool; Set true to clear the whole OLED display everytime before displaying the battery percentage
 */
void showIMU(IMU_METHOD method, COLUMN column, int line_number, int size, bool clearDisplay){
  static String imu_text = "\0";
  display.oledSetTextColour(BLACK);
  if (column == LEFT){
    display.oledDisplayLeftln(line_number, size, imu_text, clearDisplay);
    display.oledSetTextColour(WHITE);
    imu_text = (method == IMU_ORIGIN) ? "G:" + String(getIMU()) : "G:" + String(getIMUKalman());
    display.oledDisplayLeftln(line_number, size, imu_text, clearDisplay);
  } else if (column == MID) {
    display.oledDisplayCenterln(line_number, size, imu_text, clearDisplay);
    display.oledSetTextColour(WHITE);
    imu_text = (method == IMU_ORIGIN) ? "G:" + String(getIMU()) : "G:" + String(getIMUKalman());
    display.oledDisplayCenterln(line_number, size, imu_text, clearDisplay);
  } else {
    display.oledDisplayRightln(line_number, size, imu_text, clearDisplay);
    display.oledSetTextColour(WHITE);
    imu_text = (method == IMU_ORIGIN) ? "G:" + String(getIMU()) : "G:" + String(getIMUKalman());
    display.oledDisplayRightln(line_number, size, imu_text, clearDisplay);
  }
}


