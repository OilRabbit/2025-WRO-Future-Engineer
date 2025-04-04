#include "OC1.h"

void OpenChallenge(){
  int percentage = (getLaserDist(I2CPORT1) - getLaserDist(I2CPORT2)) * 100 / (getLaserDist(I2CPORT1) + getLaserDist(I2CPORT2));
  steering(percentage);
  MiniR4.M2.setPower(100);
}