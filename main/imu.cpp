#include <Arduino.h>
#include <Wire.h>
#include <ICM_20948.h>
#include "imu.h"

#define I2C_SDA 9
#define I2C_SCL 8
#define AD0_VAL 1   // 1=>0x69 (SparkFun default), 0=>0x68
#define WIRE_PORT Wire
#define SERIAL_PORT Serial

ICM_20948_I2C icm;

double imu_yaw;
double imu_pitch;
double imu_roll;

static volatile double yaw_unwrapped = 0.0;
static volatile double yaw_zero_unwrapped = 0.0;
static bool   have_prev_yaw = false;
static double prev_yaw_deg  = 0.0;
static bool   first_zero_done = false;

void imu_init(){
  WIRE_PORT.begin(I2C_SDA, I2C_SCL, 400000);
  WIRE_PORT.setClock(400000);

  bool initialized = false;
  while (!initialized){
    icm.begin(WIRE_PORT, AD0_VAL);
    // SERIAL_PORT.print(F("Initialization of the sensor returned: "));
    // SERIAL_PORT.println(icm.statusString());
    if (icm.status != ICM_20948_Stat_Ok){
      // SERIAL_PORT.println(F("Trying again..."));
      delay(500);
    } else {
      initialized = true;
    }
  }
  // SERIAL_PORT.println(F("Device connected!"));

  bool success = true;
  success &= (icm.initializeDMP() == ICM_20948_Stat_Ok);
  success &= (icm.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) == ICM_20948_Stat_Ok);
  success &= (icm.setDMPODRrate(DMP_ODR_Reg_Quat6, 0) == ICM_20948_Stat_Ok);
  success &= (icm.enableFIFO() == ICM_20948_Stat_Ok);
  success &= (icm.enableDMP() == ICM_20948_Stat_Ok);
  success &= (icm.resetDMP() == ICM_20948_Stat_Ok);
  success &= (icm.resetFIFO() == ICM_20948_Stat_Ok);
  if(!success){
    // SERIAL_PORT.println(F("Enable DMP failed!"));
    // SERIAL_PORT.println(F("Please check that you have uncommented line 29 (#define ICM_20948_USE_DMP) in ICM_20948_C.h..."));
    while (1) {}
  } else {
    // SERIAL_PORT.println(F("DMP enabled!"));
  }
}

static inline double clamp01(double v){ return v < 0.0 ? 0.0 : (v > 1.0 ? 1.0 : v); }

bool get_ypr(double& yaw, double& pitch, double& roll){
  pitch = 0.0; roll = 0.0;

  icm_20948_DMP_data_t data;
  icm.readDMPdataFromFIFO(&data);

  if ((icm.status == ICM_20948_Stat_Ok) || (icm.status == ICM_20948_Stat_FIFOMoreDataAvail)){
    if (data.header & DMP_header_bitmap_Quat6){
      const double Q30 = 1073741824.0;
      const double q1 = (double)data.Quat6.Data.Q1 / Q30; // x-vector part
      const double q2 = (double)data.Quat6.Data.Q2 / Q30; // y-vector part
      const double q3 = (double)data.Quat6.Data.Q3 / Q30; // z-vector part

      const double wsq = clamp01(1.0 - (q1*q1 + q2*q2 + q3*q3));
      const double q0  = sqrt(wsq);                       // scalar part

      // Mapping you’re using (keeps your board frame):
      const double qw = q0;
      const double qx = q2;
      const double qy = q1;
      const double qz = -q3;

      // // roll (x-axis rotation)
      // double t0 = +2.0 * (qw * qx + qy * qz);
      // double t1 = +1.0 - 2.0 * (qx * qx + qy * qy);
      // roll = atan2(t0, t1) * 180.0 / PI;

      // // pitch (y-axis rotation)
      // double t2 = +2.0 * (qw * qy - qx * qz);
      // t2 = t2 > 1.0 ? 1.0 : t2;
      // t2 = t2 < -1.0 ? -1.0 : t2;
      // pitch = asin(t2) * 180.0 / PI;

      // yaw (z-axis rotation)
      const double t3 = 2.0 * (qw*qz + qx*qy);
      const double t4 = 1.0 - 2.0 * (qy*qy + qz*qz);
      yaw = atan2(t3, t4) * 180.0 / PI;                   // [-180,180]
      return isfinite(yaw);
    }
  }
  if (icm.status != ICM_20948_Stat_FIFOMoreDataAvail) vTaskDelay(10 / portTICK_PERIOD_MS);
  return false;
}

void getYPRloop(void *){
  while(1){
    double y,p,r;
    if (!get_ypr(y,p,r)) { vTaskDelay(pdMS_TO_TICKS(10)); continue; }

    if (!have_prev_yaw) {
      // First *valid* quaternion: start from perfect zero
      prev_yaw_deg     = y;
      yaw_unwrapped     = 0.0;
      yaw_zero_unwrapped= 0.0;
      have_prev_yaw     = true;
      first_zero_done   = true;
      imu_yaw = 0.0; imu_pitch = p; imu_roll = r;
      continue;  // process next packet
    }

    // Unwrap across -180/180 seam
    double d = y - prev_yaw_deg;
    if (d > 180.0)  d -= 360.0;
    if (d < -180.0) d += 360.0;
    yaw_unwrapped += d;
    prev_yaw_deg = y;

    // Apply zero reference
    imu_yaw   = yaw_unwrapped - yaw_zero_unwrapped;
    imu_pitch = p;
    imu_roll  = r;
  }
}


void imu_resetYaw(){
  yaw_zero_unwrapped = yaw_unwrapped;
}

void showIMU(TFT_COLUMN column, int line_number, int text_size, uint16_t text_colour = TFT_WHITE, bool clearDisplay = false){
  String yaw_text = String("Yaw:") + String(imu_yaw);
  // String pitch_text = String("Pit:") + String(imu_pitch);
  // String roll_text = String("Rol:") + String(imu_roll);
  if (column == TFT_LEFT_CLN){
    tft.clearln(TFT_LEFT_CLN, line_number);
    tft.displayLeftln(line_number, text_size, yaw_text.c_str(), text_colour, false);
    // tft.clearln(TFT_LEFT_CLN, line_number + 1);
    // tft.displayLeftln(line_number + 1, text_size, pitch_text.c_str(), text_colour, false);
    // tft.clearln(TFT_LEFT_CLN, line_number + 2);
    // tft.displayLeftln(line_number + 2, text_size, roll_text.c_str(), text_colour, false);
  } else {
    tft.clearln(TFT_RIGHT_CLN, line_number);
    tft.displayRightln(line_number, text_size, yaw_text.c_str(), text_colour, false);
    // tft.clearln(TFT_RIGHT_CLN, line_number + 1);
    // tft.displayRightln(line_number + 1, text_size, pitch_text.c_str(), text_colour, false);
    // tft.clearln(TFT_RIGHT_CLN, line_number + 2);
    // tft.displayRightln(line_number + 2, text_size, roll_text.c_str(), text_colour, false);
  }
}