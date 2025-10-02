#pragma once
#include <Arduino.h>
#include "tft.h"
#include <SPI.h>

// Use HSPI for Pixy2, keep default SPI for TFT
#define PIXY_SCK  17
#define PIXY_MISO 16
#define PIXY_MOSI 15
#define PIXY_SS   18

// ====== Pixy2 CCC protocol constants ======
#define CCC_MAX_SIGNATURE      7

#define CCC_RESPONSE_BLOCKS    0x21
#define CCC_REQUEST_BLOCKS     0x20

// Signature bitmasks
#define CCC_SIG1               1
#define CCC_SIG2               2
#define CCC_SIG3               4
#define CCC_SIG4               8
#define CCC_SIG5               16
#define CCC_SIG6               32
#define CCC_SIG7               64
#define CCC_COLOR_CODES        128

#define CCC_SIG_ALL            0xFF   // all signatures


int pixy2_init();
int pixy2_transaction(byte *req, int reqLen, byte *reply, int maxReply);
void testing_print_pixy(TFT_COLUMN column, int line_number, int text_size,
                        uint16_t text_colour, bool clearDisplay);
