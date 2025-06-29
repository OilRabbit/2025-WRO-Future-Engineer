#include "LED.h"

void led1_off(){
  MiniR4.LED.setColor(1, 0, 0, 0);
}

void led1_flashwhite(){
  MiniR4.LED.setColor(1, 255, 255, 255);
}

void led1_flashred(){
  MiniR4.LED.setColor(1, 255, 0, 0);
}

void led1_flashgreen(){
  MiniR4.LED.setColor(1, 0, 255, 0);
}

void led1_flashblue(){
  MiniR4.LED.setColor(1, 0, 0, 255);
}

void led2_off(){
  MiniR4.LED.setColor(2, 0, 0, 0);
}

void led2_flashwhite(){
  MiniR4.LED.setColor(2, 255, 255, 255);
}

void led2_flashred(){
  MiniR4.LED.setColor(2, 255, 0, 0);
}

void led2_flashgreen(){
  MiniR4.LED.setColor(2, 0, 255, 0);
}

void led2_flashblue(){
  MiniR4.LED.setColor(2, 0, 0, 255);
}
