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
#include <TinyGPS.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

#define DHTPIN            2         // Pin which is connected to the DHT sensor.
#define DHTTYPE           DHT22     // DHT 22 (AM2302)



DHT_Unified dht(DHTPIN, DHTTYPE);

uint32_t delayMS;

TinyGPS gps;

int myDelay = 2000;
byte addArray[] = { 0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79 };
 
char dataValue[9];
String dataString = "";
static void gpsdump(TinyGPS &gps);
static bool feedgps();
static void print_float(float val, float invalid, int len, int prec);
static void print_int(unsigned long val, unsigned long invalid, int len);
static void print_date(TinyGPS &gps);
static void print_str(const char *str, int len);

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
  bool newdata = false;
  unsigned long start = millis();
  
  // Every second we print an update
  while (millis() - start < 5)
  {
    if (feedgps())
      newdata = true;
  }
  Serial.print("ID: PROTO0 ");
  gpsdump(gps);

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

  
  delay(500);
  
}

static void gpsdump(TinyGPS &gps)
{
  float flat, flon;
  unsigned long age;
  gps.f_get_position(&flat, &flon, &age);
  print_float(flat, TinyGPS::GPS_INVALID_F_ANGLE, 9, 5);
  print_float(flon, TinyGPS::GPS_INVALID_F_ANGLE, 10, 5);
}

static void print_int(unsigned long val, unsigned long invalid, int len)
{
  char sz[32];
  if (val == invalid)
    strcpy(sz, "*******");
  else
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i=strlen(sz); i<len; ++i)
    sz[i] = ' ';
  if (len > 0) 
    sz[len-1] = ' ';
  Serial.print(sz);
  feedgps();
}

static void print_float(float val, float invalid, int len, int prec)
{
  char sz[32];
  if (val == invalid)
  {
    strcpy(sz, "*******");
    sz[len] = 0;
        if (len > 0) 
          sz[len-1] = ' ';
    for (int i=7; i<len; ++i)
        sz[i] = ' ';
    Serial.print(sz);
  }
  else
  {
    Serial.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1);
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    for (int i=flen; i<len; ++i)
      Serial.print(" ");
  }
  feedgps();
}

static void print_date(TinyGPS &gps)
{
  int year;
  byte month, day, hour, minute, second, hundredths;
  unsigned long age;
  gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &age);
  if (age == TinyGPS::GPS_INVALID_AGE)
    Serial.print("*******    *******    ");
  else
  {
    char sz[32];
    sprintf(sz, "%02d/%02d/%02d %02d:%02d:%02d   ",
        month, day, year, hour, minute, second);
    Serial.print(sz);
  }
  print_int(age, TinyGPS::GPS_INVALID_AGE, 5);
  feedgps();
}

static void print_str(const char *str, int len)
{
  int slen = strlen(str);
  for (int i=0; i<len; ++i)
    Serial.print(i<slen ? str[i] : ' ');
  feedgps();
}

static bool feedgps()
{
  while (Serial1.available())
  {
    if (gps.encode(Serial1.read()))
      return true;
  }
  return false;
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
