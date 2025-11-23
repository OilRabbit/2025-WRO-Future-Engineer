#include <cmath>
#include "calculation.h"

/**
 * @brief To get the sign of a variable
 * @param num; float; the variable to get the sign
 *
 * @return int; 1, -1 or 0
*/
int sign(float num){
  if (num > 0) return 1;
  else if (num < 0) return -1;
  else return 0;
}

float deg2rad(float deg){
  return deg * M_PI / 180;
}

float rad2deg(float rad){
  return rad * 180 / M_PI;
}

float min_val(float v1, float v2){
  if (v1 < v2) return v1;
  else return v2;
}

float max_val(float v1, float v2){
  if (v1 > v2) return v1;
  else return v2;
}