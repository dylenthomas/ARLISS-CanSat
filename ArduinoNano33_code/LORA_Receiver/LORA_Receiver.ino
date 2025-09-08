#include <SPI.h>
#include <LoRa.h>
#include "Neo6M_UBXParser.h"

#define CS_PIN     10
#define RST_PIN    9   // Or -1 if not connected
#define IRQ_PIN   -1   // Not using DIO1

#define freq 915E6
#define preamble_len 8 //12

GPSParser gps;
GPSParser::posllhData GPS_posllh;
GPSParser::statusData GPS_status;
GPSParser::velnedData GPS_velned;

double targetLat, targetLon;

void setup() {
  Serial.begin(11520);
  while (!Serial);

  pinMode(RST_PIN, OUTPUT);
  //pinMode(53, OUTPUT);
  //digitalWrite(53, HIGH);

  LoRa.setPins(CS_PIN, RST_PIN, IRQ_PIN);  // CS, RESET, IRQ
  
  while (!LoRa.begin(freq)) {
    Serial.println("LoRa init failed!");
  }

  //LoRa.setSpreadingFactor(12);  // valid: 6–12
  //LoRa.setSignalBandwidth(7.8E3);  // Hz; valid: 7.8E3 – 500E3
  //LoRa.setCodingRate4(8);  // valid: 5–8 (represents 4/5 … 4/8)
  //LoRa.setTxPower(20);  // dBm, valid: 2–20 (depends on hardware)
  LoRa.setPreambleLength(preamble_len);
  //LoRa.setSyncWord(0x12);

  Serial.println("LoRa receiver ready.");
  Serial1.begin(9600);
  delay(100);
}

void loop() {
  while (Serial1.available()) {
    unsigned char c = Serial1.read();
    gps.addByte(c);
  }

  if (gps.posllhChanged()) GPS_posllh = gps.getPOSLLH();
  if (gps.statusChanged()) GPS_status = gps.getSTATUS();
  if (gps.velnedChanged()) GPS_velned = gps.getVELNED();

  String incoming = "";
  int packetSize = LoRa.parsePacket();

  if (packetSize) {
    while (LoRa.available()) {
        incoming += (char)LoRa.read();
    }
    incoming.trim();

    int commaIndex = incoming.indexOf(',');
    if (commaIndex > 0) {
        targetLat = incoming.substring(0, commaIndex).toFloat();
        targetLon = incoming.substring(commaIndex + 1).toFloat();
    }

    //Serial.println(LoRa.packetRssi());
    Serial.print(GPS_velned.heading);
    Serial.print(",");
    Serial.print(GPS_posllh.lat);
    Serial.print(",");
    Serial.print(GPS_posllh.lon);
    Serial.print(",");
    Serial.print(targetLat);
    Serial.print(",");
    Serial.println(targetLon);
  }
}