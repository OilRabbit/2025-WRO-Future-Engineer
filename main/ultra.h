#ifndef ULTRA_H
#define ULTRA_H

#include <Arduino.h>
#include <MatrixMiniR4.h>
#include <vector>
#include <algorithm>
#include "oled.h"

extern int ultra1Dist;
extern int ultra2Dist;

struct ULTRA {
    int code;
    int trig;
    int echo;
};

extern ULTRA ultra1;
extern ULTRA ultra2;

void ultraInit(ULTRA ultra);
int getUltra1Dist();
int getUltra2Dist();
void getultra1Distloop();
void getultra2Distloop();
void showUltra1Dist(COLUMN column, int line_number, int size, bool clearDisplay);
void showUltra2Dist(COLUMN column, int line_number, int size, bool clearDisplay);

#endif
