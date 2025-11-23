#include <SPI.h>

#define PIXY_SCK  17
#define PIXY_MISO 16
#define PIXY_MOSI 15
#define PIXY_SS   18

void setup() {
  Serial.begin(115200);
  pinMode(PIXY_SS, OUTPUT);
  digitalWrite(PIXY_SS, HIGH);

  SPI.begin(PIXY_SCK, PIXY_MISO, PIXY_MOSI, PIXY_SS);
  delay(1000);

  Serial.println("Sending Pixy2 getVersion...");

  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
  digitalWrite(PIXY_SS, LOW);

  // Send request
  byte req[] = {0xAE, 0xC1, 0x0E, 0x00};
  for (int i=0; i<4; i++) {
    SPI.transfer(req[i]);
  }

  // Read reply (try up to 20 bytes)
  Serial.print("Reply: ");
  for (int i=0; i<20; i++) {
    byte b = SPI.transfer(0x00);
    Serial.print("0x"); Serial.print(b, HEX); Serial.print(" ");
  }
  Serial.println();

  digitalWrite(PIXY_SS, HIGH);
  SPI.endTransaction();
}

void loop() {}
