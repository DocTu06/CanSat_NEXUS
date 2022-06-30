#include <NRFLite.h>

#include <MQUnifiedsensor.h>
#include <MPU6050_light.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <BH1750.h>
#include <SPI.h>
#include <SD.h>
int RXPin = 16;
int TXPin = 17;
int GPSBaud = 9600;
int ctimee;
int timee = 0;
#define SD_CS_PIN 5
#define SEALEVELPRESSURE_HPA (1013.25)
TinyGPSPlus gps;
File myFile;
Adafruit_BME280 bme;
BH1750 lightMeter;
const int MPU_addr = 0x68;
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
int air_purity;
SoftwareSerial gpsSerial(RXPin, TXPin);

void setup()
{
  Serial.begin(9600);
  pinMode(25, OUTPUT);
  delay(30000);
  digitalWrite(25, HIGH);
  gpsSerial.begin(9600);
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  if (!bme.begin(0x76)) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }
  Serial.print("Initializing SD card...");
  if (!SD.begin(SD_CS_PIN))
  {
    Serial.println("initialization failed!");
    return;
  }
  lightMeter.begin();


}

void loop()
{

  myFile = SD.open("/cansat.txt", "a");
  myFile.println(gps.location.lat(), 6);
  myFile.println(gps.location.lng(), 6);
  myFile.println(gps.altitude.meters());
  myFile.print(gps.date.month());
  myFile.print("/");
  myFile.print(gps.date.day());
  myFile.print("/");
  myFile.println(gps.date.year());
  myFile.print(gps.time.hour());
  myFile.print(":");
  myFile.print(gps.time.minute());
  myFile.print(":");
  myFile.print(gps.time.second());
  myFile.print(".");
  myFile.println(gps.time.centisecond());
  myFile.println();
  myFile.println();
  myFile.println(bme.readTemperature());
  myFile.println(bme.readPressure() / 100.0F);
  myFile.println(bme.readAltitude(SEALEVELPRESSURE_HPA));
  myFile.println(bme.readHumidity());
  float lux = lightMeter.readLightLevel();
  myFile.println(lux);
  myFile.println();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14, true); 
  AcX = Wire.read() << 8 | Wire.read();
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read(); 
  Tmp = Wire.read() << 8 | Wire.read(); 
  GyX = Wire.read() << 8 | Wire.read(); 
  GyY = Wire.read() << 8 | Wire.read(); 
  GyZ = Wire.read() << 8 | Wire.read();
  myFile.println(AcX);
  myFile.println(AcY);
  myFile.println(AcZ);
  myFile.println(GyX);
  myFile.println(GyY);
  myFile.println(GyZ);
  air_purity = analogRead(A0);
  myFile.print(air_purity);
  myFile.close();
}
