#include "pixy2lib.h"

Pixy2I2C pixy;
COLOURED_OBJ nearestPillarGlobal;

int pixy2_init() {
  Serial.println("Init Pixy2...");
  pixy.init();
  return 0;
}

COLOURED_OBJ getNearestPillar() {
  COLOURED_OBJ nearest_pillar;
  int count = pixy.ccc.getBlocks();
  Block* blocks = pixy.ccc.blocks;
  int largestArea = 0;
  int largest_ypos = 0;
  if (count == 0) {
    nearest_pillar.colour = NO_COLOUR;
    return nearest_pillar;
  } else {
    for (int i = 0; i < count; i++) {
      if (blocks[i].m_y > largest_ypos) {
        if (blocks[i].m_signature != 3) {
          largest_ypos = blocks[i].m_y;
          // largestArea = blocks[i].m_height * blocks[i].m_width;
          nearest_pillar.height = blocks[i].m_height;
          nearest_pillar.width = blocks[i].m_width;
          nearest_pillar.area = blocks[i].m_height * blocks[i].m_width / 100;
          nearest_pillar.xpos = blocks[i].m_x;
          nearest_pillar.ypos = blocks[i].m_y;
          nearest_pillar.colour = (blocks[i].m_signature == RED) ? RED : (blocks[i].m_signature == GREEN) ? GREEN : NO_COLOUR;
        }
      }
    }
    if (nearest_pillar.ypos < CAM_MID_YPOS){
      for (int i = 0; i < count; i++){
        if (blocks[i].m_height * blocks[i].m_width > largestArea){
          if (blocks[i].m_signature != 3) {
            // largest_ypos = blocks[i].m_y;
            largestArea = blocks[i].m_height * blocks[i].m_width;
            nearest_pillar.height = blocks[i].m_height;
            nearest_pillar.width = blocks[i].m_width;
            nearest_pillar.area = blocks[i].m_height * blocks[i].m_width / 100;
            nearest_pillar.xpos = blocks[i].m_x;
            nearest_pillar.ypos = blocks[i].m_y;
            nearest_pillar.colour = (blocks[i].m_signature == RED) ? RED : (blocks[i].m_signature == GREEN) ? GREEN : NO_COLOUR;
          }
        }
      }
      nearest_pillar.num_signitures = count;
    }
  }
  return nearest_pillar;
}

void getNearestBlkloop(void *){
  while(1){
    nearestPillarGlobal = getNearestPillar();
    // Serial.println(nearestPillarGlobal.colour);
    vTaskDelay(5 / portTICK_PERIOD_MS);
  }
}

void showNearestBlk(TFT_COLUMN column, int line_number, int text_size, uint16_t text_colour = TFT_WHITE, bool clearDisplay = false){
  String red_text;
  String green_text;
  if (nearestPillarGlobal.colour == RED){
    red_text = String("R:") + String(nearestPillarGlobal.ypos) + String(", ") + String(nearestPillarGlobal.area);
    green_text = String("G:") + String("NA");
  } else if (nearestPillarGlobal.colour == GREEN){
    red_text = String("R:") + String("NA");
    green_text = String("G:") + String(nearestPillarGlobal.ypos) + String(", ") + String(nearestPillarGlobal.area);
  } else {
    red_text = String("R:") + String("NA");
    green_text = String("G:") + String("NA");
  }
  if (column == TFT_LEFT_CLN){
    tft.clearln(TFT_LEFT_CLN, line_number);
    tft.displayLeftln(line_number, text_size, red_text.c_str(), text_colour, false);
    tft.clearln(TFT_LEFT_CLN, line_number + 1);
    tft.displayLeftln(line_number + 1, text_size, green_text.c_str(), text_colour, false);
  } else {
    tft.clearln(TFT_RIGHT_CLN, line_number);
    tft.displayRightln(line_number, text_size, red_text.c_str(), text_colour, false);
    tft.clearln(TFT_RIGHT_CLN, line_number + 1);
    tft.displayRightln(line_number + 1, text_size, green_text.c_str(), text_colour, false);
  }
}