/*
 * gps module pin1 tx-  arduino 19(rx1)
 * gps module pin2 rx-  arduino 18(tx1)
 * 
 * co2 module pin1 rx-  arduino 17(tx2)
 * co2 module pin2 tx-  arduino 16(rx2)
 * co2 module pin3 VCC
 * co2 module pin4 GND
 * 
 * DHT22 pin1 VCC
 * DHT22 pin2 SIG -     aurduino digital2
 * DHT22 pin3 GND
 * 
 * dust module red    -   5V
 * dust module black  -   GND
 * dust module yellow -   arduino 14(tx3)
 * dust module blue   -   arduino 15(rx3)
 * 
 * 
 */




#include <SoftwareSerial.h>

#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <TinyGPS.h>
TinyGPS gps;

#define DHTPIN            2         // Pin which is connected to the DHT sensor.
#define DHTTYPE           DHT22     // DHT 22 (AM2302)

DHT_Unified dht(DHTPIN, DHTTYPE);

uint32_t delayMS;


int myDelay = 2000;
byte addArray[] = { 0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79 };
 
char dataValue[9];
String dataString = "";


float pm25; //2.5um particles detected in ug/m3
float pm10; //10um particles detected in ug/m3

void setup()
{
  Serial.begin(115200);
  Serial1.begin(9600);
  Serial2.begin(9600);
  Serial3.begin(9600); //SDS021 reports at 1Hz at 9600bps
  dht.begin();
 
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  dht.humidity().getSensor(&sensor);
  delayMS = sensor.min_delay / 1000;

}

void loop()
{

  Serial.print("ID: Car0 ");

  bool newData = false;

  // For one second we parse GPS data and report some key values
  for (unsigned long start = millis(); millis() - start < 1000;)
  {
    while (Serial1.available())
    {
      char c = Serial1.read();
      // Serial.write(c); // uncomment this line if you want to see the GPS data flowing
      if (gps.encode(c)) // Did a new valid sentence come in?
        newData = true;
    }
  }

  if (newData)
  {
    float flat, flon;
    unsigned long age;
    gps.f_get_position(&flat, &flon, &age);
  //  Serial.print("LAT= ");
    Serial.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
    Serial.print(" ");
 //   Serial.print(" LON= ");
    Serial.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
  }else{
  //  Serial.print("LAT=");
    Serial.print("****");
  //  Serial.print(" LON=");
    Serial.print("  ****");
  }
 
      

  
  
  sensors_event_t event;  
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    Serial.print(" Temperature: ");
    Serial.print("*****");
    //Serial.print("*C");
  }
  else {
    Serial.print(" Temperature: ");
    Serial.print(event.temperature);
   // Serial.print("*C");
  }
  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    Serial.print(" Humidity: ");
    Serial.print("*****");
   // Serial.print("%");
  }
  else {
    Serial.print(" Humidity: ");
    Serial.print(event.relative_humidity);
   // Serial.print("%");
  }

  Serial2.write(addArray, 9);
  Serial2.readBytes(dataValue, 9);
  int resHigh = (int) dataValue[2];
  int resLow  = (int) dataValue[3];
  int pulse = (256*resHigh)+resLow;
  dataString = String(pulse);
  Serial.print(" PPM: ");
  Serial.print(pulse);

  if (dataAvailable())
  {
    Serial.print(" dust[PM2.5]: ");
    Serial.print(pm25, 1);
   // Serial.print("ug/m3");
    Serial.print(" dust[PM10]: ");
    Serial.println(pm10, 1);
   // Serial.println("ug/m3");
    //Serial.println();
  }
  else
  {
    Serial.print(" dust[PM2.5]: ");
    Serial.print("*****");
   // Serial.print("ug/m3");
    Serial.print(" dust[PM10]: ");
    Serial.println("*****");
 //   Serial.println("ug/m3");
  }

  
  delay(60000);
  
}

boolean dataAvailable(void)
{
  //Spin until we hear meassage header byte
  long startTime = millis();

  while (1)
  {
    while (!Serial3.available())
    {
      delay(1);
      if (millis() - startTime > 1500) return (false); //Timeout error
    }

    if (Serial3.read() == 0xAA) break; //We have the message header
  }

  //Read the next 9 bytes
  byte sensorValue[10];
  for (byte spot = 1 ; spot < 10 ; spot++)
  {
    startTime = millis();
    while (!Serial3.available())
    {
      delay(1);
      if (millis() - startTime > 1500) return (false); //Timeout error
    }

    sensorValue[spot] = Serial3.read();
  }

  //Check CRC
  byte crc = 0;
  for (byte x = 2 ; x < 8 ; x++) //DATA1+DATA2+...+DATA6
    crc += sensorValue[x];
  if (crc != sensorValue[8])
    return (false); //CRC error

  //Update the global variables
  pm25 = ((float)sensorValue[3] * 256 + sensorValue[2]) / 10;
  pm10 = ((float)sensorValue[5] * 256 + sensorValue[4]) / 10;

  return (true); //We've got a good reading!
}
