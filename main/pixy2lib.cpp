#include "esp32-hal-gpio.h"
#include "pixy2lib.h"

SPIClass pixySPI(HSPI);

struct Pixy2Version {
  uint16_t hardware;
  uint8_t firmwareMajor;
  uint8_t firmwareMinor;
  uint16_t firmwareBuild;
  char firmwareType[10];
};

static void decodeVersion(byte *reply, int len) {
  // Find sync (handle the occasional double 0xAF before 0xC1)
  int start = -1;
  for (int i = 0; i < len - 1; i++) {
    if (reply[i] == 0xAF && reply[i + 1] == 0xC1) {
      start = (i > 0 && reply[i - 1] == 0xAF) ? (i - 1) : i;
      break;
    }
  }
  if (start < 0) { Serial.println("No sync found!"); return; }

  int idx = start + 2; // past sync
  if (idx + 3 >= len) { Serial.println("Reply too short (header)!"); return; }

  // uint8_t type = reply[idx++]; // not used, but this is 0x0F for GetVersion
  idx++; // skip type
  uint16_t payloadLen = reply[idx] | (reply[idx + 1] << 8);
  idx += 2;

  if (idx + payloadLen > len) {
    Serial.println("Warning: buffer shorter than declared payload, decoding what we have...");
    payloadLen = len - idx;
  }

  Pixy2Version v;
  if (idx + 2 > len) { Serial.println("Reply too short (hardware)!"); return; }
  v.hardware = reply[idx] | (reply[idx + 1] << 8); idx += 2;

  if (idx + 1 > len) { Serial.println("Reply too short (fw major)!"); return; }
  v.firmwareMajor = reply[idx++];

  if (idx + 1 > len) { Serial.println("Reply too short (fw minor)!"); return; }
  v.firmwareMinor = reply[idx++];

  if (idx + 2 > len) { Serial.println("Reply too short (fw build)!"); return; }
  v.firmwareBuild = reply[idx] | (reply[idx + 1] << 8); idx += 2;

  // Skip the observed 0x14 0x00 padding before the ASCII firmwareType
  if (idx + 2 <= len && reply[idx] == 0x14 && reply[idx + 1] == 0x00) {
    idx += 2;
  }

  // Copy up to 10 bytes (null-terminate)
  for (int i = 0; i < 10; i++) {
    if (idx + i >= len) { v.firmwareType[i] = '\0'; break; }
    v.firmwareType[i] = (char)reply[idx + i];
    if (v.firmwareType[i] == '\0') break;
    if (i == 9) v.firmwareType[9] = '\0';
  }

  Serial.print("Hardware: "); Serial.println(v.hardware);
  Serial.print("Firmware: ");
  Serial.print(v.firmwareMajor); Serial.print(".");
  Serial.print(v.firmwareMinor);
  Serial.print(" build "); Serial.println(v.firmwareBuild);
  Serial.print("Type: "); Serial.println(v.firmwareType);
}

int pixy2_init() {
  pinMode(PIXY_SS, OUTPUT);
  digitalWrite(PIXY_SS, HIGH);

  // init Pixy2 on HSPI with your pins
  pixySPI.begin(PIXY_SCK, PIXY_MISO, PIXY_MOSI, PIXY_SS);
  delay(1000);

  Serial.println("Sending Pixy2 getVersion...");

  pixySPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE3));
  digitalWrite(PIXY_SS, LOW);

  byte req[] = {0xAE, 0xC1, 0x0E, 0x00};
  for (int i=0; i<4; i++)
    pixySPI.transfer(req[i]);

  byte reply[64];
  for (int i=0; i<32; i++) {
    reply[i] = pixySPI.transfer(0x00);
  }

  digitalWrite(PIXY_SS, HIGH);
  pixySPI.endTransaction();

  // Dump raw
  Serial.print("Reply: ");
  for (int i=0; i<32; i++) {
    Serial.print("0x"); Serial.print(reply[i], HEX); Serial.print(" ");
  }
  Serial.println();

  decodeVersion(reply, 32);

  return 0;
}

struct Block {
  uint16_t m_signature;
  uint16_t m_x;
  uint16_t m_y;
  uint16_t m_width;
  uint16_t m_height;
  int16_t  m_angle;
  uint8_t  m_index;
  uint8_t  m_age;

  void print() {
    if (m_signature > CCC_MAX_SIGNATURE) {
      Serial.printf("CC block sig: %d x:%d y:%d w:%d h:%d angle:%d idx:%d age:%d\n",
                    m_signature, m_x, m_y, m_width, m_height, m_angle, m_index, m_age);
    } else {
      Serial.printf("sig:%d x:%d y:%d w:%d h:%d idx:%d age:%d\n",
                    m_signature, m_x, m_y, m_width, m_height, m_index, m_age);
    }
  }
};

int pixy2_transaction(byte *req, int reqLen, byte *reply, int maxReply) {
  pixySPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE3));
  digitalWrite(PIXY_SS, LOW);

  for (int i = 0; i < reqLen; i++) {
    pixySPI.transfer(req[i]);
  }

  // read reply
  for (int i = 0; i < maxReply; i++) {
    reply[i] = pixySPI.transfer(0x00);
  }

  digitalWrite(PIXY_SS, HIGH);
  pixySPI.endTransaction();
  return 0;
}

int pixy2_getBlocks(Block *blocks, int maxBlocks, uint8_t sigmap) {
  // Build request: GetBlocks
  byte req[] = {0xAE, 0xC1, 0x20, 0x02, 0x00, sigmap, (uint8_t)maxBlocks};

  pixySPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE3));
  digitalWrite(PIXY_SS, LOW);

  // Send request
  for (int i = 0; i < (int)sizeof(req); i++) {
    pixySPI.transfer(req[i]);
  }

  delay(5); // give Pixy2 time to prepare reply

  // Read bulk reply
  byte reply[64];
  for (int i = 0; i < 64; i++) {
    reply[i] = pixySPI.transfer(0x00);
  }

  digitalWrite(PIXY_SS, HIGH);
  pixySPI.endTransaction();

  // Debug: print first 32 reply bytes
  Serial.print("Raw reply: ");
  for (int i=0; i<32; i++) {
    Serial.printf("0x%02X ", reply[i]);
  }
  Serial.println();

  // Find sync (0xAF 0xC1)
  int start = -1;
  for (int i=0; i<62; i++) {
    if (reply[i] == 0xAF && reply[i+1] == 0xC1) {
      start = i;
      break;
    }
  }

  if (start < 0) {
    Serial.println("No sync in GetBlocks reply");
    return 0;
  }

  int idx = start + 2;
  uint8_t type = reply[idx++];
  if (type != 0x21) { // CCC_RESPONSE_BLOCKS
    Serial.printf("Unexpected reply type: 0x%02X\n", type);
    return 0;
  }

  uint16_t payloadLen = reply[idx] | (reply[idx+1] << 8);
  idx += 2;

  if (payloadLen == 0) {
    Serial.println("No blocks detected (payloadLen=0)");
    return 0;
  }

  int numBlocks = payloadLen / sizeof(Block);
  if (numBlocks > maxBlocks) numBlocks = maxBlocks;

  memcpy(blocks, &reply[idx], numBlocks * sizeof(Block));

  return numBlocks;
}


void testing_print_pixy(TFT_COLUMN column, int line_number, int text_size, uint16_t text_colour, bool clearDisplay) {
  Block blocks[16];
  int n = pixy2_getBlocks(blocks, 16, CCC_SIG_ALL);
  if (n > 0) {
    Serial.printf("Detected %d blocks\n", n);
    for (int i = 0; i < n; i++) {
      blocks[i].print();
    }
  }

  vTaskDelay(100 / portTICK_PERIOD_MS);
  // byte req[] = {0xAE, 0xC1, 0x0E, 0x00};
  // byte reply[32];

  // pixy2_transaction(req, sizeof(req), reply, sizeof(reply));

  // // Print reply to Serial
  // Serial.print("Pixy2 Reply: ");
  // for (int i = 0; i < 32; i++) {
  //   Serial.print("0x"); Serial.print(reply[i], HEX); Serial.print(" ");
  // }
  // Serial.println();

  // // Optionally show something on TFT
  // String pixy_text = "Pix: ";
  // pixy_text += String(reply[8], HEX); // just an example field
  // if (column == TFT_LEFT_CLN) {
  //   tft.clearln(TFT_LEFT_CLN, line_number);
  //   tft.displayLeftln(line_number, text_size, pixy_text.c_str(),
  //                     text_colour, clearDisplay);
  // } else {
  //   tft.clearln(TFT_RIGHT_CLN, line_number);
  //   tft.displayRightln(line_number, text_size, pixy_text.c_str(),
  //                      text_colour, clearDisplay);
  // }
}