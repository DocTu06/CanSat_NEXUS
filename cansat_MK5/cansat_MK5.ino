#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include <LoRa.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"


//library declatrations
#define SEALEVELPRESSURE_HPA (1013.25)//for altitude calculation
#define ss 25
#define rst 14
#define dio0 2
int i = 0;
char c;
bool newdata = false;
//LoRa module pins
int RXPin = 16;
int TXPin = 17;
//GPS pins
int GPSBaud = 9600;
const int MPU_addr = 0x68;
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
//mpu6050 variables
TinyGPSPlus gps;
//initialize gps object
SoftwareSerial gpsSerial(RXPin, TXPin);
//gps module declarations
Adafruit_BME280 bme;
//temperature, humidity and pressure sensor declaration
unsigned long prevmillis = 0;
const long interval = 500;
//variables for Lora delay
void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  //disabling brownout protection because ealrier models of the esp32 gave false brownout errors
  pinMode(32, OUTPUT);/set buzzer pin as output
  delay(40000);
  digitalWrite(32,HIGH);
  Wire.begin(); //begin i2c communication
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  bme.begin(0x76); //initialize bme280
  pinMode(5, OUTPUT);
  pinMode(25, OUTPUT);

  //set the cs pins of the SPI modules as outputs to modify them(the libraries can't do that)
  Serial.begin(9600); //begin serial comunication
  LoRa.setPins(ss, rst, dio0);
  if (!LoRa.begin(433E6)) {
    Serial.println("Starting LoRa failed!");
  }
  //LoRa check
  if (!SD.begin(5)) {
    Serial.println("Card Mount Failed");
  }
  //SD check
  gpsSerial.begin(GPSBaud); //GPS initialization
}

void loop() {
  while (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      unsigned long currentmillis = millis();
      digitalWrite(25, HIGH);
      digitalWrite(5, LOW);
      //select the SD card module
      File file = SD.open("/nexus.txt", FILE_APPEND); //open the file
      file.print(bme.readTemperature());
      file.print(",");
      file.print(bme.readPressure() / 100.0F);
      file.print(",");
      file.print(bme.readHumidity());
      file.print(",");
      //bme280 data
      file.print(gps.location.lat(), 6);
      file.print(",");
      file.print(gps.location.lng(), 6);
      file.print(",");
      file.print(gps.time.hour());
      file.print(":");
      file.print(gps.time.minute());
      file.print(":");
      file.print(gps.time.second());
      file.print(".");
      file.print(gps.time.centisecond());
      file.print(",");
      //gps data
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
      file.print(AcX);
      file.print(",");
      file.print(AcY);
      file.print(",");
      file.print(AcZ);
      file.print(",");
      file.print(GyX);
      file.print(",");
      file.print(GyY);
      file.print(",");
      file.print(GyZ);
      file.print(",");
      file.print(analogRead(A0));
      file.print("t");
      //accelerometer and gyroscope data
      file.close(); //close file
      if (currentmillis - prevmillis >= 500)
      {
        prevmillis  = currentmillis;
        digitalWrite(5, HIGH);
        digitalWrite(25, LOW);
        //select the LoRa module
        Serial.println("Sending packet");
        LoRa.beginPacket(); //begin sending data
        LoRa.print(bme.readTemperature());
        LoRa.print(",");
        LoRa.print(bme.readPressure() / 100.0F);
        LoRa.print(",");
        LoRa.print(bme.readHumidity());
        LoRa.print(",");
        //bme280 data
        LoRa.print(gps.location.lat(), 6);
        LoRa.print(",");
        LoRa.print(gps.location.lng(), 6);
        LoRa.print(",");
        LoRa.print(analogRead(A0));
        //GPS data
        LoRa.endPacket(); //send data
        //if  one second had passed, send data through LoRa
      }
    }
  }
}
