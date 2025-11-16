#include "tof.h"
#include <HardwareSerial.h>

// Map UARTs: ToF1->UART1, ToF2->UART2, ToF3->UART0
HardwareSerial ToF1(1);
HardwareSerial ToF2(2);
HardwareSerial ToF3(0);

TOF_STATES tof1_state = SEEK_AA1;
TOF_STATES tof2_state = SEEK_AA1;
TOF_STATES tof3_state = SEEK_AA1;

uint8_t  hdr6_t1[6];  uint8_t hdr_i_t1 = 0;  uint16_t len_t1 = 0;  uint8_t  pay_t1[MAX_PAYLOAD];  uint16_t pay_i_t1 = 0;  uint8_t  csum_t1 = 0;
uint8_t  hdr6_t2[6];  uint8_t hdr_i_t2 = 0;  uint16_t len_t2 = 0;  uint8_t  pay_t2[MAX_PAYLOAD];  uint16_t pay_i_t2 = 0;  uint8_t  csum_t2 = 0;
uint8_t  hdr6_t3[6];  uint8_t hdr_i_t3 = 0;  uint16_t len_t3 = 0;  uint8_t  pay_t3[MAX_PAYLOAD];  uint16_t pay_i_t3 = 0;  uint8_t  csum_t3 = 0;

volatile bool have_t1 = false, have_t2 = false, have_t3 = false;

/**
 *  dist: Distance measured (in cm) by the ToF sensor
 *  noise: Noise level reported by the sensor’s algorithm. Higher noise → less trustworthy reading
 *  peak: Amplitude of the laser pulse at that point (signal strength). Higher peak → stronger reflection (good alignment/reflective surface).
 *  conf: Confidence score (0–255). Higher → more reliable point.
 *  reftof: Reference time-of-flight used internally for calibration/temperature drift compensation. 
 */
int dist_t1 = 0, noise_t1 = 0, peak_t1 = 0, conf_t1 = 0, intg_t1 = 0, reftof_t1 = 0;
int dist_t2 = 0, noise_t2 = 0, peak_t2 = 0, conf_t2 = 0, intg_t2 = 0, reftof_t2 = 0;
int dist_t3 = 0, noise_t3 = 0, peak_t3 = 0, conf_t3 = 0, intg_t3 = 0, reftof_t3 = 0;

static inline bool checksumOK(const uint8_t* hdr, const uint8_t* payload, uint16_t L, uint8_t cs){
  uint16_t s = 0;
  for (uint8_t i = 0; i < 6; ++i) s += hdr[i];
  for (uint16_t i = 0; i < L; ++i) s += payload[i];
  return ((uint8_t)s) == cs;
}

static inline void fillExport(const uint8_t* p, int &dist_cm, int &noise, int &peak, int &conf, int &intg, int &reftof){
  int dmm  = (int16_t)(p[0] | (p[1] << 8));
  noise    = (int16_t)(p[2] | (p[3] << 8));
  peak     = (int)((uint32_t)p[4] | ((uint32_t)p[5] << 8) | ((uint32_t)p[6] << 16) | ((uint32_t)p[7] << 24));
  conf     = (int)p[8];
  intg     = (int)((uint32_t)p[9] | ((uint32_t)p[10] << 8) | ((uint32_t)p[11] << 16) | ((uint32_t)p[12] << 24));
  reftof   = (int16_t)(p[13] | (p[14] << 8));
  if (dmm > 0){
    int cm = (dmm + 5) / 10; if (cm < 1) cm = 1; if (cm > 300) cm = 300;
    dist_cm = cm;
  }
}

// -------- FIXED SIGNATURE (no UART_t), just HardwareSerial& --------
static void parseLoopOne(HardwareSerial &U,
                         TOF_STATES &state,
                         uint8_t *hdr6, uint8_t &hdr_i, uint16_t &len,
                         uint8_t *pay,  uint16_t &pay_i, uint8_t &csum,
                         int &dist_cm, int &noise, int &peak, int &conf, int &intg, int &reftof,
                         volatile bool &have_flag)
{
  auto resetSM = [&](){ state = SEEK_AA1; hdr_i = 0; pay_i = 0; };

  while (U.available()){
    uint8_t b = (uint8_t)U.read();

    switch (state){
      case SEEK_AA1:
        if (b == 0xAA) state = SEEK_AA2PLUS;
        break;

      case SEEK_AA2PLUS:
        if (b != 0xAA){ hdr6[0] = b; hdr_i = 1; state = READ_HDR6; }
        break;

      case READ_HDR6:
        hdr6[hdr_i++] = b;
        if (hdr_i >= 6){
          len = (uint16_t)hdr6[4] | ((uint16_t)hdr6[5] << 8);
          if (len == 0 || len > MAX_PAYLOAD){ resetSM(); break; }
          pay_i = 0; state = READ_PAY;
        }
        break;

      case READ_PAY:
        pay[pay_i++] = b;
        if (pay_i >= len) state = READ_CSUM;
        break;

      case READ_CSUM: {
        csum = b;

        do {
          uint8_t cmd = hdr6[1];
          if (!((cmd == CMD_DISTANCE_A) || (cmd == CMD_DISTANCE_B))) break;
          if (len < 4) break; // needs timestamp at end
          uint16_t dataBytes = len - 4;
          if ((dataBytes % BYTES_PER_PT) != 0 || dataBytes == 0) break;
          if (!checksumOK(hdr6, pay, len, csum)) break;

          // Select nearest valid point
          // Use point #0 to minimize CPU
          uint8_t pts = dataBytes / BYTES_PER_PT;
          if (pts >= 1 && dataBytes >= BYTES_PER_PT) {
            const uint8_t *p0 = pay; // first point
            int16_t d_mm = (int16_t)(p0[0] | (p0[1] << 8));
            // optional tiny gate: also check confidence p0[8] >= 1
            if (d_mm > 0 /* && p0[8] >= 1 */) {
              fillExport(p0, dist_cm, noise, peak, conf, intg, reftof);
              have_flag = true;
            }
          } else {
            break; // malformed frame (no points)
          }
          // uint8_t pts = dataBytes / BYTES_PER_PT;
          // int best_mm = 32767; uint8_t best_i = 255;
          // for (uint8_t i = 0; i < pts; ++i){
          //   const uint8_t *p = pay + i * BYTES_PER_PT;
          //   int16_t d_mm = (int16_t)(p[0] | (p[1] << 8));
          //   uint8_t cf   = p[8];
          //   if (cf >= 1 && d_mm > 0 && d_mm < best_mm){ best_mm = d_mm; best_i = i; }
          // }
          // if (best_i == 255) break;

          // fillExport(pay + best_i * BYTES_PER_PT, dist_cm, noise, peak, conf, intg, reftof);
          // have_flag = true;
        } while (0);

        resetSM();
      } break;
    }
  }
}

// ====== init all three UARTs ======
void tofInit() {
  ToF1.setRxBufferSize(4096);
  ToF1.begin(230400, SERIAL_8N1, TOF1_RX, TOF1_TX);

  ToF2.setRxBufferSize(4096);
  ToF2.begin(230400, SERIAL_8N1, TOF2_RX, TOF2_TX);

  // ToF3.setRxBufferSize(4096);
  // ToF3.begin(230400, SERIAL_8N1, TOF3_RX, TOF3_TX);

  tof1_state = tof2_state = tof3_state = SEEK_AA1;

  hdr_i_t1 = hdr_i_t2 = hdr_i_t3 = 0;
  pay_i_t1 = pay_i_t2 = pay_i_t3 = 0;
  len_t1 = len_t2 = len_t3 = 0;

  // avoid chaining assignments to volatiles
  have_t1 = false;
  have_t2 = false;
  have_t3 = false;
}

// ====== tasks ======
void getToF1Distloop(void *){
  for(;;){
    parseLoopOne(ToF1, tof1_state,
                 hdr6_t1, hdr_i_t1, len_t1,
                 pay_t1,  pay_i_t1, csum_t1,
                 dist_t1, noise_t1, peak_t1, conf_t1, intg_t1, reftof_t1,
                 have_t1);
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

void getToF2Distloop(void *){
  for(;;){
    parseLoopOne(ToF2, tof2_state,
                 hdr6_t2, hdr_i_t2, len_t2,
                 pay_t2,  pay_i_t2, csum_t2,
                 dist_t2, noise_t2, peak_t2, conf_t2, intg_t2, reftof_t2,
                 have_t2);
    vTaskDelay(5 / portTICK_PERIOD_MS);
  }
}

// void getToF3Distloop(void *){
//   for(;;){
//     parseLoopOne(ToF3, tof3_state,
//                  hdr6_t3, hdr_i_t3, len_t3,
//                  pay_t3,  pay_i_t3, csum_t3,
//                  dist_t3, noise_t3, peak_t3, conf_t3, intg_t3, reftof_t3,
//                  have_t3);
//     vTaskDelay(5 / portTICK_PERIOD_MS);
//   }
// }

// ===== Display helpers =====
void showToF1Dist(TFT_COLUMN column, int line_number, int text_size, uint16_t text_colour, bool){
  String s = String("T1:") + String(dist_t1) + "cm";
  if (column == TFT_LEFT_CLN){ tft.clearln(TFT_LEFT_CLN, line_number);
    tft.displayLeftln(line_number, text_size, s.c_str(), text_colour, false);
  } else { tft.clearln(TFT_RIGHT_CLN, line_number);
    tft.displayRightln(line_number, text_size, s.c_str(), text_colour, false); }
}

void showToF2Dist(TFT_COLUMN column, int line_number, int text_size, uint16_t text_colour, bool){
  String s = String("T2:") + String(dist_t2) + "cm";
  if (column == TFT_LEFT_CLN){ tft.clearln(TFT_LEFT_CLN, line_number);
    tft.displayLeftln(line_number, text_size, s.c_str(), text_colour, false);
  } else { tft.clearln(TFT_RIGHT_CLN, line_number);
    tft.displayRightln(line_number, text_size, s.c_str(), text_colour, false); }
}

void showToF3Dist(TFT_COLUMN column, int line_number, int text_size, uint16_t text_colour, bool){
  String s = String("T3:") + String(dist_t3) + "cm";
  if (column == TFT_LEFT_CLN){ tft.clearln(TFT_LEFT_CLN, line_number);
    tft.displayLeftln(line_number, text_size, s.c_str(), text_colour, false);
  } else { tft.clearln(TFT_RIGHT_CLN, line_number);
    tft.displayRightln(line_number, text_size, s.c_str(), text_colour, false); }
}
