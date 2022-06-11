#include <SPI.h>
#include <LoRa.h>

#define ss 10
#define rst 9
#define dio0 2

void setup() {
  Serial.begin(9600);
  while (!Serial);
  LoRa.setPins(ss, rst, dio0);
  if (!LoRa.begin(433E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
}

void loop() {
  // try to parse packet
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    // received a packet
    // read packet
    while (LoRa.available()) {
      Serial.print((char)LoRa.read());
    }

    // print RSSI of packet
    Serial.print(",");
    Serial.println(LoRa.packetRssi());
  }
}
