#ifndef TOF_H
#define TOF_H

#include <Arduino.h>
#include "tft.h"

// Pins connected to ToF
#define TOF1_TX 10
#define TOF1_RX 11
#define TOF2_TX 48
#define TOF2_RX 47

typedef enum { SEEK_AA1, SEEK_AA2PLUS, READ_HDR6, READ_PAY, READ_CSUM } TOF_STATES;

static const uint8_t  CMD_DISTANCE_A = 0x02;
static const uint8_t  CMD_DISTANCE_B = 0x0D;
static const uint16_t MAX_PAYLOAD    = 320;
static const uint8_t  BYTES_PER_PT   = 15;

extern int dist_t1, noise_t1, peak_t1, conf_t1, intg_t1, reftof_t1;
extern int dist_t2, noise_t2, peak_t2, conf_t2, intg_t2, reftof_t2;

void tofInit();
void getToF1Distloop(void *parameters);
void getToF2Distloop(void *parameters);
void showToF1Dist(TFT_COLUMN column, int line_number, int text_size, uint16_t text_colour = TFT_WHITE, bool clearDisplay = false);
void showToF2Dist(TFT_COLUMN column, int line_number, int text_size, uint16_t text_colour = TFT_WHITE, bool clearDisplay = false);

#endif
