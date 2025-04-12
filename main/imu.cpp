#include "MatrixMiniR4.h"
#include "imu.h"

Kalman kalmanZ;

double zeroValue[5] = { -200, 44, 660, 52.3, -18.5 }; // Calibration values

double gyroZangle = 180;
double compAngleZ = 180;
double kalmanAngleZ = 180;

unsigned long timer;

void imuInit(){
  MiniR4.Motion.resetIMUValues();
  long sum = 0;
  const int samples = 500;
  for (int i = 0; i < samples; i++) {
    sum += MiniR4.Motion.getGyro(MiniR4Motion::AxisType::Z);
    delay(2);
  }
  zeroValue[3] = sum / (double)samples;
}

double unwrapAngle(double current_angle){
  static double previous_angle = 0;
  static double accum_angle = 0;

  double delta = current_angle - previous_angle;

  // Handle the angle wrap-around from -180 to 180
  if (delta > 180) {
    delta -= 360;
  } else if (delta < -180) {
    delta += 360;
  }

  accum_angle += delta;
  previous_angle = current_angle;

  return accum_angle;
}

double getIMU(){
  return unwrapAngle(MiniR4.Motion.getEuler(MiniR4Motion::AxisType::Yaw)) * -1;
}

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


