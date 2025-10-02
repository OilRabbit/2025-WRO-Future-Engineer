#ifndef ULTRA_H
#define ULTRA_H

#include <Arduino.h>
#include <NewPing.h>
#include "tft.h"

extern NewPing ultra1;
extern NewPing ultra2;

extern int ultra1Dist;
extern int ultra2Dist;

void ultraInit();
int getUltra1Dist(int max_dist_cm);
int getUltra1DistMedian(int iter, int max_dist_cm);
void getultra1Distloop(void *parameters);
void showUltra1Dist(TFT_COLUMN column, int line_number, int text_size, uint16_t text_colour, bool clearDisplay);
int getUltra2Dist(int max_dist_cm);
int getUltra2DistMedian(int iter, int max_dist_cm);
void getultra2Distloop(void *parameters);
void showUltra2Dist(TFT_COLUMN column, int line_number, int text_size, uint16_t text_colour, bool clearDisplay);

#endif
