//------------------------------------------------------------------
// BTLE - thermometer and hydometer using DHT11 or DHT22 sensor
//
// Dean Gienger, Feb 12, 2024
//
// - uses Adafruit DHT library
// - Credit Rui Santos - randomnerdtutorials.com - BLE example tutorial
//
// DHTxx sensor uses 1-wire protocol and enables reading of
// temperature and humidity.
// A library from Adafruit (DHT Library) is driving this
// The DHTxx device uses 3.3V and Ground, with one signal wire
// connected to GPIO 4.   The driver initializes the device and
// enables reading of temperature and humidity.
//
// On startup, a BLE device is created and starts advertising.
// There is one service with three characteristics:
// temperature (read only, degrees F)
// humidity (read only, percent)
// time and date (read/write)
//
// Time and date format is yyyy/mm/dd hh:mm:ss 
// For example 2024/01/23 12:34:56
//
// If you write the time and date characteristic, it will
// try to use the value written to set the internal clock
//
// The clock is maintained to the accuracy of the clock oscillator
// of the ESP32.
//
//
//------------------------------------------------------------------

//-------- includes --------------
#include <BLEDevice.h>  // BLE driver
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <ESP32Time.h>  // RTC time functions
#include "DHT.h"        // temperature sensor driver - had to mod to make _type public not private
#include <time.h>
#include <SPI.h>
#include <SPIFFS.h>

//------- Signal LED for indicator purposes ---
#define LED 22 
#define LEDOFF HIGH
#define LEDON  LOW
// LED blinks whenever the device is updating it's time/humidity reading

// Config file
// If the file config.ini exists in the SPIFFS of the device it is
// read and values are used to configure the operation of the sensor
// It is a text file with three lines:
//
// Example:
//SERVERNAME="Deans TempHumidity Sensor"
//SENSOR=DHT11
//UPDATERATE=5

// SERVERNAME is the name that will be given when the advertised device
// comes up on BLE
// SENSOR can be either DHT11 or DHT22
// UPDATERATE is the number of seconds between each time the sensor is read
// (minimum is 2 seconds, maximum is 60 seconds)

#define SIGNON "\nBLE - THERMOMETER V1.3\n"
// configuration file name
#define CONFIGFN "/config.ini"
// backup file name - stores time periodically to keep clock "sort of" on time
#define BACKUPFN "/backup.dat"

//-----------------------------------------------------------------
// config file info - read from /config.ini on startup
char bleServerName[64];
int sensorIsDHT11 = true;
unsigned long updateRateSec = 5; // must be evenly divisible into 60, 1,2,5,10,15,20,30,60
int tempIsF = true;

//-----------------------------------------------------------------
// real time clock (software based, not backed up for power failures
// Subject to some drift as it's based on the low cost crystal osc
// on the ESP32.
ESP32Time rtc(0);  // 0 from GMT by default

//----------- Sensor configuration --------
#define DHTPIN 4     // Digital pin connected to the DHT sensor

// For the DHT11 module I purchased, there were three pins
// Looking at the back they are labeled "-  output  +"
// - is connected to ESP32 GND
// + is connected to ESP32 3.3V
// output is connected go ESP32 GPIO 4 pin
  
int lastSecond = -1;
int lastHr = -1;
bool deviceConnected = false;
unsigned long secondCtr = 0;

//---------------------------------------------------------------
// All kinds of descriptor mumbo-jumbo needed in order
// to set up a BLE server :)
// UUID - descriptors for characteristics - generated using:
//      https://www.uuidgenerator.net/
// Generate 4 UUIDs, one for the service, one for each characteristic
//
// Three characteristics: temperature, humidity, and time-date

#define SERVICE_UUID "b7972d95-e930-4144-beb0-6a6e8b9a3d23"
#define TEMP_UUID    "20b5e09a-f998-47f0-aae3-4b361ebc8233"
#define HUM_UUID     "5ed64822-1dc1-4ebd-8e23-f0847e380841"
#define TIME_UUID    "711d51a8-f76b-4c24-ad4a-8a3059d2489b"

BLECharacteristic dhtTemperatureFahrenheitCharacteristics(TEMP_UUID, 
  BLECharacteristic::PROPERTY_NOTIFY |
  BLECharacteristic::PROPERTY_READ );
BLEDescriptor dhtTemperatureFahrenheitDescriptor(BLEUUID((uint16_t)0x2902));

BLECharacteristic dhtHumidityCharacteristics(HUM_UUID, 
  BLECharacteristic::PROPERTY_NOTIFY | 
  BLECharacteristic::PROPERTY_READ);
BLEDescriptor dhtHumidityDescriptor(BLEUUID((uint16_t)0x2903));

BLECharacteristic dhtTimeCharacteristics(TIME_UUID, 
  BLECharacteristic::PROPERTY_NOTIFY | 
  BLECharacteristic::PROPERTY_READ |
  BLECharacteristic::PROPERTY_WRITE);
BLEDescriptor dhtTimeDescriptor(BLEUUID((uint16_t)0x2903));

// just some utility routines for diagnostic printouts to the serial port
void println(char* s) { Serial.println(s); }
void println(String s) { Serial.println(s); }
void print(char* s) { Serial.print(s); }
void print(String s) { Serial.print(s); }

//--------------------------------------------------------------------
// BLE: Callbacks for onConnect and onDisconnect
class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
  };
  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    pServer->startAdvertising();
  }
};

//--------------------------------------------------------------------
// Get value as integer from some digits, return -1 on error
// For parsing the date time string written to the BLD characteristic
int getDigits(char* p, int nDigits)
{
  int res = -1;
  int val = 0;
  for (int i = 0; i < nDigits; i++)
  {
    char c = *p++;
    if ((c >= '0') && (c <= '9'))
    {
      val = val*10 + (c - '0');
    }
    else
    {
      return -1;
    }
  }
  return val;
}

//--------------------------------------------------------------------
// Set RTC time from received string of form 
// "2024/02/23 12:34:56"
//  0123456789012345678
void setRtcTime(char* tstring)
{
  
  if (strlen(tstring) < 19) return;
  
  int tyr = getDigits(&tstring[0],4);
  int tmo = getDigits(&tstring[5],2);
  int tda = getDigits(&tstring[8],2);

  int thr = getDigits(&tstring[11],2);
  int tmn = getDigits(&tstring[14],2);
  int tse = getDigits(&tstring[17],2);

  if ((tyr>=2024) && (tyr<=2100) &&
      (tmo>=1) && (tmo <= 12) &&
      (tda>=1) && (tda <= 31) &&
      (thr>=0) && (thr <= 23) &&
      (tmn>=0) && (tmn <= 59) &&
      (tse>=0) && (tse <= 59) &&
      (tstring[4] == '/') &&
      (tstring[7] == '/') &&
      (tstring[13] == ':') &&
      (tstring[16] == ':'))
  {
    // todo - check day of month >30,31,28
    //Serial.printf("Setting time: %d/%d/%d %d:%d:%d\n",tyr,tmo,tda,thr,tmn,tse,0);
    
    rtc.setTime(tse,tmn,thr,tda,tmo,tyr);
    lastSecond = tse;
    lastHr = thr;
    String ttag = rtc.getTime("%Y/%m/%d %H:%M:%S");
    Serial.println(ttag);
  }
}

char setTimeData[32] = "";
//--------------------------------------------------------------------
// callback for when time characteristic is written by some client
//
class CharacteristicCallBack : public BLECharacteristicCallbacks
{
public:
  void onWrite(BLECharacteristic *characteristic_) override
  {
    // client writes a date-time string to set the BLE device clock
    // Expect it to be exactly this format "2024/02/23 12:34:56"
    std::string ttag = characteristic_->getValue();
    strncpy(setTimeData, (char*)ttag.c_str(),31);
    // copy it to a special setting variable setTimeData
    // next 1 second task interval it will be picked up
    // and used to set the RTC
    //Serial.print("Time was written: ");
    //Serial.println((char*)ttag.c_str());
  }
};

//----------------------------------------------------------
// read line from input text file
int readln(File finp, uint8_t* buf, int maxlen)
{
  // return true on successful read, false on EOF
  // 10 or 13 (LF, CR) or both are EOL indicators
  int len=0;
  int eof=false;

  buf[0]=0;
  while (len<(maxlen-1))
  {
    if (!finp.available())
    {
      eof=true;
      break;
    }
    char c = finp.read();
    if (c < 0) 
    {
      eof=true;
      break; // EOF
    }
    if (c==13) continue; // ignore CR
    if (c==10) break; // end-of-line
    buf[len++]=c;
  }
  buf[len]=0; // null terminate
  return !eof;
}

//----------------------------------------------------------
// retrieve a value for a key in the config file
void readKey(char* configFn, char* key, char* outbuf, int maxlen)
{
  outbuf[0] = 0; // returning null string on error 
  //
  // Config file is key=value format
  // SSID=mywifi
  // PASSWORD=mypassword
  // TIMEZONE=-8
  // OFFSET=123590 
  //
  // pass in key with trailing = sign!!! 
  // readKey("/test.cfg","MYKEY=", outbuf, 127);

  File finp = SPIFFS.open(configFn, FILE_READ);
  if (!finp)
  {
    print("Unable to read config file");
    return;
  }
  // scan file and look for key
  char buf[128];
  int n = strlen(key);
  while (readln(finp, (uint8_t*) buf, 127))
  {
    if (strncmp(buf,key,n) == 0) // found
    { 
      println(buf);
      strncpy(outbuf,&buf[n],maxlen);
      break;
    }
  }
  finp.close();
 
}

//------------------------------------------------------------
// Save the time in a backup file as TIME=2024/02/24 12:34:56
// use readKey to retrieve time
void backupTimeValue() 
{
  // TIME=2023/02/24 12:34:56
  String buf="TIME="+rtc.getTime("%Y/%m/%d %H:%M:%S");
  File fout = SPIFFS.open(BACKUPFN, FILE_WRITE);
  if (!fout)
  {
    print("Unable to write backup file");
    return;
  }
  fout.println(buf);
  fout.close();
}

//----------------------------------------------------------


BLEServer *pServer;
BLEService *dhtService;
BLECharacteristic *pCharacteristicTemp;
BLECharacteristic *pCharacteristicHum;
BLECharacteristic *pCharacteristicTime;

// controller for the DHT sensor 
DHT dht(DHTPIN, DHT11); // change later to DHT22 - set dht.setType(DHT22) or DHT11

//--------------------------------------------------------------------
// Setup tasks - called one time on power up
void setup() 
{
  char tmpbuf[32];

  //-- start up serial port for diagnostics
  Serial.begin(115200);
  Serial.println(SIGNON);


  //-- initialize signal LED to blink when in operation
  pinMode(LED,OUTPUT);
  digitalWrite(LED,LEDON);


  //-- initialize the RTC
  rtc.setTime(0,0,0,1,1,2024);

  
  //-- mount SPIFFS and read config file
  bleServerName[0] = '\0'; // "my diy temp sensor name"
  sensorIsDHT11 = true; // SENSOR=DHT11 or DHT12
  updateRateSec = 10; // read every 10 seconds

  if(!SPIFFS.begin(true))
  {
    print("An Error has occurred while mounting SPIFFS");
  }
  else
  {
    // read config file
    readKey(CONFIGFN,"SERVERNAME=",bleServerName,63);
    readKey(CONFIGFN,"SENSOR=",tmpbuf,15);
    sensorIsDHT11 = (strcmp(tmpbuf, "DHT11") == 0);
    readKey(CONFIGFN,"UPDATERATE=",tmpbuf,15); // update rate in seconds
    int x = atoi(tmpbuf);
    if (x < 2) x = 2;
    if (x > 60) x = 60;
    updateRateSec = x;
    tempIsF = true;
    readKey(CONFIGFN,"UNITS=",tmpbuf,15); // UNITS=C or UNITS=F, F is default temperature unit
    char c = tmpbuf[0];
    if ((c=='C') || (c == 'c')) tempIsF = false;

    readKey(BACKUPFN,"TIME=",tmpbuf,31);
    setRtcTime(tmpbuf);
  }

  // default server name if we can't read it from the configuration file
  if (bleServerName[0] == '\0')   strcpy(bleServerName,"DIY Temp Humidity Sensor");

  Serial.print("BTLE Name: "); Serial.println(bleServerName);


  //-- initialize the DHT - 11 sensor (or DHT-22)
  dht.setType(sensorIsDHT11 ? DHT11 : DHT22);
  dht.begin();
  

  //-- Create the BLE Device
  // We need to initialize the device with a name
  BLEDevice::init(bleServerName);

  // Then we need to create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Then we need to create the BLE Service that will hold characteristics
  dhtService = pServer->createService(SERVICE_UUID);

  // Then we need to create the Characteristics and Create a BLE Descriptor of each one
  // Temperature
  dhtService->addCharacteristic(&dhtTemperatureFahrenheitCharacteristics);
  dhtTemperatureFahrenheitDescriptor.setValue("DHT temperature Fahrenheit");
  dhtTemperatureFahrenheitCharacteristics.addDescriptor(&dhtTemperatureFahrenheitDescriptor);

  // Humidity
  dhtService->addCharacteristic(&dhtHumidityCharacteristics);
  dhtHumidityDescriptor.setValue("DHT humidity");
  dhtHumidityCharacteristics.addDescriptor(new BLE2902());

  // Time
  //  time is writable, so there's a call back for when a client writes the time
  dhtTimeCharacteristics.setCallbacks(new CharacteristicCallBack()); 
  dhtService->addCharacteristic(&dhtTimeCharacteristics);
  dhtTimeDescriptor.setValue("Date and Time yyyy/mo/da hr:mn:ss");
  dhtTimeCharacteristics.addDescriptor(new BLE2902());
  
  // Now we can start the service running
  dhtService->start();

  // Finally let's start advertising that this server is alive
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pServer->getAdvertising()->start();
  std::string myaddr = BLEDevice::getAddress().toString();
  Serial.print("BLE: Advertising and awaiting a client connection on ");
  Serial.println((char*)myaddr.c_str());
  

  //-- initialization for the app  
  lastSecond = rtc.getSecond();
  lastHr = rtc.getHour();
  secondCtr = 0;

  // LED has been on during initialization
  digitalWrite(LED,LEDOFF);
}

char tmpbuf[32];
//--------------------------------------------------------------------
// Called forever after setup() completes
void loop() 
{
  //-- detect when each second ticks by on the rtc
  int sec = rtc.getSecond();
  if (sec != lastSecond)
  {
    lastSecond = sec;
    secondCtr++;

    // inside this if() we do things every 1 second    
    // -- time characteristic handling
    if (strlen(setTimeData) > 0)
    {
      // if a client wrote a time, we set our internal clock to that time/date
      setRtcTime(setTimeData);
      setTimeData[0] = '\0';
      backupTimeValue();
    }
    else
    {
      // otherwise we update the characteristic value with current time/date
      String ttag = rtc.getTime("%Y/%m/%d %H:%M:%S");
      dhtTimeCharacteristics.setValue((char*)ttag.c_str());
    }

    // -- temperature and humidity characteristic handling
    if ((secondCtr % updateRateSec) == 0) // every 5 seconds or so
    {
      digitalWrite(LED,LEDON);
      // Read temperature as Fahrenheit 
      dht.setType(sensorIsDHT11 ? DHT11 : DHT22);
      float tempF = dht.readTemperature(tempIsF);
      // Read humidity
      float hum = dht.readHumidity();
      
      // Update temperature
      tmpbuf[0] = '\0';
      sprintf(tmpbuf,"%.1f", tempF);
      //dtostrf(tempF, 6, 2, tmpbuf);
      //Set temperature Characteristic value and notify connected client
      dhtTemperatureFahrenheitCharacteristics.setValue(tmpbuf);
      dhtTemperatureFahrenheitCharacteristics.notify();
      Serial.print("Temperature Fahrenheit: ");
      Serial.print(tmpbuf);
      Serial.print(" ºF");
      
      // update humidity characteristic
      tmpbuf[0] = '\0';
      sprintf(tmpbuf,"%.0f",hum);
      //dtostrf(hum, 6, 2, tmpbuf);
      //Set humidity Characteristic value and notify connected client
      dhtHumidityCharacteristics.setValue(tmpbuf);
      dhtHumidityCharacteristics.notify();   
      Serial.print(" - Humidity: ");
      Serial.print(tmpbuf);
      Serial.println(" %");

      // LED is on during 1 second processing
      // 1 second processing has completed, so turn it off
      digitalWrite(LED,LEDOFF);
    }
  }
  // Hourly tasks
  int hr = rtc.getHour();
  if (hr != lastHr)
  {
    lastHr = hr;
    backupTimeValue(); // once per hour, save current time
    // on reset, if there's a saved time value, use that as
    // the startup time
  }
}
