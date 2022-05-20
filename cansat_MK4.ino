
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <BH1750.h>
#include <SPI.h>
#include <SD.h>
//Create software serial object to communicate with SIM800L
int RXPin = 16;
int TXPin = 17;
int GPSBaud = 9600;
int ctimee;
int timee = 0;
//gps pins and baudrate
#define SD_CS_PIN 5
//micro sd card chipselect pin
#define SEALEVELPRESSURE_HPA (1013.25)
TinyGPSPlus gps;
//gps object
File myFile;
Adafruit_BME280 bme;
BH1750 lightMeter;
const int MPU_addr = 0x68;
// I2C address of the MPU-6050
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
int air_purity;
SoftwareSerial gpsSerial(RXPin, TXPin);
SoftwareSerial mySerial(26, 27); //SIM800L Tx & Rx is connected to Arduino #3 & #2

void setup()
{
  //Begin serial communication with Arduino and Arduino IDE (Serial Monitor)
  Serial.begin(9600);
  pinMode(25,OUTPUT);
    delay(30000);
  digitalWrite(25,HIGH);
  //Begin serial communication with Arduino and SIM800L
  gpsSerial.begin(GPSBaud);
  mySerial.begin(9600);
Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  // PWR_MGMT_1 register
  Wire.write(0);
  // set to zero (wakes up the MPU-6050)
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
  //sd check
  lightMeter.begin();

  
}

void loop()
{

  myFile = SD.open("/cansat.txt", "a");
  //open "cansat.txt" file in append mode
  
  while (gpsSerial.available() > 0)
  {
    if (gps.encode(gpsSerial.read()))
    {
      ctimee = millis();
      displayInfo();
      //displays gps info when available to serial monitor
      filedata();
      //stores gps data to micro sd card
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
      Wire.requestFrom(MPU_addr, 14, true); // request a total of 14 registers
      AcX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
      AcY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
      AcZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
      Tmp = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
      GyX = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
      GyY = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
      GyZ = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
      myFile.println(AcX);
      myFile.println(AcY);
      myFile.println(AcZ);
      myFile.println(GyX);
      myFile.println(GyY);
      myFile.println(GyZ);
      air_purity = analogRead(A0);
      myFile.println(air_purity);
      myFile.close();
      Serial.println("Done");
      //to see when a writing was succesfull
    }
    
  }



  // If 5000 milliseconds pass and there are no characters coming in
  // over the software serial port, show a "No GPS detected" error
  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println("No GPS detected");
    while (true);
  }
    mySerial.println("AT"); //Once the handshake test is successful, it will back to OK

 
  delay(80);
  mySerial.write(26);
  /*
   * 
   * 
   * 
   * 
   * 
   * 
   * 
   * 
   * 
   */
      
  if(ctimee - timee >=1500)
  {
    timee = ctimee;
     mySerial.println("AT+CMGF=1"); // Configuring TEXT mode
  mySerial.println("AT+CMGS=\"+40748835779\"");//change ZZ with country code and xxxxxxxxxxx with phone number to sms
  mySerial.print(String(gps.location.lat(), 6) + " " + String(gps.location.lng(), 6));
  delay(20);
  Wire.beginTransmission(MPU_addr);
      Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
      Wire.endTransmission(false);
      Wire.requestFrom(MPU_addr, 14, true); // request a total of 14 registers
      AcX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
      AcY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
      AcZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
      Tmp = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
      GyX = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
      GyY = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
      GyZ = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
   mySerial.println("AT+CMGF=1"); // Configuring TEXT mode
  mySerial.println("AT+CMGS=\"+40748835779\"");//change ZZ with country code and xxxxxxxxxxx with phone number to sms
  mySerial.print(String(bme.readTemperature()) + " " + String(bme.readPressure() / 100.0F) + " " + String(bme.readHumidity()) + " " + String(lightMeter.readLightLevel()) + " " + String(AcX) + " " + String(AcY) + " " + String(AcZ) + " " + String(GyX) + " " + String(GyY) + " " + String(GyZ) + " " + String(analogRead(A0)));
  //text content
  }
}

void displayInfo()
{
  if (gps.location.isValid())
  {
    Serial.print("Latitude: ");
    Serial.println(gps.location.lat(), 6);
    Serial.print("Longitude: ");
    Serial.println(gps.location.lng(), 6);
    Serial.print("Altitude: ");
    Serial.println(gps.altitude.meters());
  }
  else
  {
    Serial.println("Location: Not Available");
  }

  Serial.print("Date: ");
  if (gps.date.isValid())
  {
    Serial.print(gps.date.month());
    Serial.print("/");
    Serial.print(gps.date.day());
    Serial.print("/");
    Serial.println(gps.date.year());
  }
  else
  {
    Serial.println("Not Available");
  }

  Serial.print("Time: ");
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(":");
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(":");
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(".");
    if (gps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.println(gps.time.centisecond());
  }
  else
  {
    Serial.println("Not Available");
  }

  Serial.println();
  Serial.println();
  delay(80);
  
}

void filedata()
{
  if (gps.location.isValid())
  {
   
    myFile.println(gps.location.lat(), 6);
 
    myFile.println(gps.location.lng(), 6);

    myFile.println(gps.altitude.meters());
  }
  else
  {
    myFile.println("Location: Not Available");
  }

 
  if (gps.date.isValid())
  {
    myFile.println(gps.date.month());
    myFile.print("/");
    myFile.print(gps.date.day());
    myFile.print("/");
    myFile.println(gps.date.year());
  }
  else
  {
    myFile.println("Not Available");
  }

 
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) myFile.print(F("0"));
    myFile.print(gps.time.hour());
    myFile.print(":");
    if (gps.time.minute() < 10) myFile.print(F("0"));
    myFile.print(gps.time.minute());
    myFile.print(":");
    if (gps.time.second() < 10) myFile.print(F("0"));
    myFile.print(gps.time.second());
    myFile.print(".");
    if (gps.time.centisecond() < 10) myFile.print(F("0"));
    myFile.println(gps.time.centisecond());
  }
  else
  {
    myFile.println("Not Available");
  }

  myFile.println();
  myFile.println();
}


void updateSerial()
{
  delay(500);
  while (Serial.available()) 
  {
    mySerial.write(Serial.read());//Forward what Serial received to Software Serial Port
  }
  while(mySerial.available()) 
  {
    Serial.write(mySerial.read());//Forward what Software Serial received to Serial Port
  }
}
