#include "OC1.h"

void OpenChallenge(){
  int diff = getLaserDist(I2CPORT1) - getLaserDist(I2CPORT2);
  float percentage = 0;
  if (diff > 300) percentage = (diff > 0) ? 100 : -100;
  else percentage = (getLaserDist(I2CPORT1) - getLaserDist(I2CPORT2)) * 100 / (getLaserDist(I2CPORT1) + getLaserDist(I2CPORT2));
  steering(percentage);
  // MiniR4.M2.setPower(100);
}