#define DHT22
#define BMP280
#define BH1750
#define SHARP_DUSTSENSOR
#define MQ-135

#define THING_SPEAK_INTEGRATION
#define HTTP_CLIENT_DELAY 200
#define SAMPLING_INTERVAL 60000

//#define SERIAL_OUTPUT
#define SD_CARD
#define SD_FAT_LIB
#define RTC

#include "secrets.h"

#ifdef DHT22
  #include <SimpleDHT.h>
  #define DHT_PIN 2
  SimpleDHT22 dht22(DHT_PIN);
#endif

#ifdef BMP280
  #include <Wire.h>
  #include <BMx280I2C.h>
  
  #define BMP_I2C_ADDRESS 0x76
  //create a BMx280I2C object using the I2C interface with I2C Address 0x76
  BMx280I2C bmx280(BMP_I2C_ADDRESS);
#endif

#ifdef BH1750
  #include <BH1750FVI.h>

  BH1750FVI LightSensor(BH1750FVI::k_DevModeContHighRes);
#endif

#ifdef SHARP_DUSTSENSOR
  const int sharpLEDPin = 8;      // Arduino digital pin X connect to sensor LED.
  const int sharpVoPin = A3;      // Arduino analog pin Y connect to sensor Vo.
  const float sharpK = 0.5;       // Use the typical sensitivity in units of V per 100ug/m3.
  static float sharpVoc = 0.6;    // Set the typical output voltage in Volts when there is zero dust. 
                                  // (not CONST since lower values maybe be measured during runtime) 
#endif

#ifdef MQ-135
  #include <MQ135.h>
  
  #define MQ135_PIN A1
  
  MQ135 mq135_sensor = MQ135(MQ135_PIN);
#endif

#include "WiFiEsp.h"

// Emulate Serial1 on pins 6/7 if not present - For ARDUINO UNO
#ifndef HAVE_HWSERIAL1
  #include "SoftwareSerial.h"
  SoftwareSerial Serial1(6, 7); // UNO RX, UNO TX
#endif

char ssid[] = SECRET_SSID;        // the network SSID (name)
char pass[] = SECRET_PASS;        // the network password

int status = WL_IDLE_STATUS;     // the Wifi radio's status

#ifdef THING_SPEAK_INTEGRATION
  #define MAX_ITER_THING_SPEAK 5
  static byte iterationsTS = MAX_ITER_THING_SPEAK;
#endif

WiFiEspClient client;

#ifdef SD_CARD
  #include <SPI.h>

  #ifdef SD_FAT_LIB
    #include "SdFat.h"
    SdFat SD;
  #else
    #include <SD.h>
  #endif

  #define SPI_SPEED SD_SCK_MHZ(4)

  const uint8_t chipSelect = 4; // SD chip select pin.
  const int8_t DISABLE_CHIP_SELECT = 10; //Ethernet ChipSelect
#endif

#ifdef RTC
  #include "RTClib.h"

  RTC_DS3231 rtc;
#endif

#define METRICS_COUNT 6
#define METRIC_IDX_TEMP 0
#define METRIC_IDX_HUMD 1
#define METRIC_IDX_PRESS 2
#define METRIC_IDX_LIGHT 3
#define METRIC_IDX_DUST 4
#define METRIC_IDX_GASCO2 5


// ---------------------------------------------------------------
// Helper functions to print a data value to the serial monitor
// ---------------------------------------------------------------
#ifdef SERIAL_OUTPUT
void printDivider() {
  int i;
  for (i = 1; i < 50; ++i) {
    Serial.print("=");
  }
  Serial.println();
}
#endif

#ifdef SERIAL_OUTPUT
void printSensorSamplingInitiation(String sensor) {
  printDivider();
  Serial.print("Sampling ");  
  Serial.print(sensor);
  Serial.println(" ...");
  Serial.println();
}
#endif

void printFValue(String text, float value, String units, bool isLast = false, bool addNewLine = false) {
  Serial.print(text);
  Serial.print(" = ");
  Serial.print(value);
  Serial.print(units);
  if (isLast) {
    Serial.println();
  } else {
    Serial.print(", ");
  }
  if (addNewLine) {
    Serial.println();
  }
}

// ---------------------------------------------------------------
// Helper functions for memory management
// ---------------------------------------------------------------
void softReset() {
  // Restarts program from beginning but does not reset the peripherals and registers
  asm volatile ("  jmp 0");  
}

int freeRam() {
  // Calculates available free RAM 
  extern int __heap_start,*__brkval;
  int v;
  return (int)&v - (__brkval == 0 ? (int)&__heap_start : (int) __brkval); 
}

// ---------------------------------------------------------------
// Setup function
// ---------------------------------------------------------------
void setup() {
  // Start the hardware serial port for the serial monitor (debugging).
  Serial.begin(9600);
  // Initialize serial port for ESP8266 (WiFi) module
  Serial1.begin(9600);

  #ifdef SD_CARD
    if (!SD.begin(chipSelect)) {
      Serial.println(F("FAIL SD Card"));
    }
  #endif

  #ifdef RTC
    if (!rtc.begin()) {
      Serial.println(F("FAIL RTC"));
      //while (1);
    } else {
      //if (rtc.lostPower()) {
      //  Serial.println(F("RESET RTC"));
      //  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
      //}
    }
  #endif
  
  // initialize ESP WiFi module
  #ifdef SERIAL_OUTPUT
    Serial.println(F("WiFi init - pre"));
  #endif
  WiFi.init(&Serial1);
  #ifdef SERIAL_OUTPUT
    Serial.println(F("WiFi init - post"));
  #endif

  // check for the presence of the shield
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println(F("FAIL WiFi"));
    // don't continue
    while (true);
  }

  // attempt to connect to WiFi network
  while ( status != WL_CONNECTED) {
    #ifdef SERIAL_OUTPUT
      Serial.print(F("Attempting to connect to WPA SSID: "));
      Serial.println(ssid);
    #endif
    // Connect to WPA/WPA2 network
    status = WiFi.begin(ssid, pass);
  }

  IPAddress ip = WiFi.localIP();
  Serial.println(ip);
  #ifdef SERIAL_OUTPUT
    printWifiStatus();
  #endif

  #ifdef BMP280
    Wire.begin();

    //begin() checks the Interface, reads the sensor ID (to differentiate between BMP280 and BME280)
    //and reads compensation parameters.
    if (!bmx280.begin())
    {
      Serial.println(F("FAIL BMx280"));
      while (1);
    }
    #ifdef SERIAL_OUTPUT
      Serial.println(F(">> Pressure Sensor - BMP280"));
    #endif

    //reset sensor to default parameters.
    bmx280.resetToDefaults();

    //by default sensing is disabled and must be enabled by setting a non-zero
    //oversampling setting.
    //set an oversampling setting for pressure and temperature measurements. 
    bmx280.writeOversamplingPressure(BMx280MI::OSRS_P_x16);
    bmx280.writeOversamplingTemperature(BMx280MI::OSRS_T_x16);
  #endif

  #ifdef BH1750
    LightSensor.begin();
  #endif
  
  #ifdef SHARP_DUSTSENSOR
    // Set LED pin for output.
    pinMode(sharpLEDPin, OUTPUT);
  
    // Wait two seconds for startup.
    delay(2000);
    #ifdef SERIAL_OUTPUT
      Serial.println(F(">> Dust Sensor - SHARP GP2Y1014AU0F"));
    #endif
  #endif  
}

// ---------------------------------------------------------------
// Loop function.
// ---------------------------------------------------------------
void loop() {
  unsigned long startTime = millis();
  Serial.println(startTime);
  // if there's incoming data from the net connection send it out the serial port
  // this is for debugging purposes only
  while (client.available()) {
    char c = client.read();
    Serial.write(c);
  }
  
  #ifdef SERIAL_OUTPUT
    Serial.println();
    Serial.println(F(">>> Sampling ..."));
  #endif

  float *sensorData;
  sensorData = retrieveSensorData();

  #ifdef SERIAL_OUTPUT
    Serial.println();
  #endif

  // close any connection before send a new request
  // this will free the socket on the WiFi shield
  client.stop();

  #ifdef THING_SPEAK_INTEGRATION
    // Send data to ThingSpeak every MAX_ITER_THING_SPEAK iterations (minutes)
    Serial.println(iterationsTS);
    if (iterationsTS >= MAX_ITER_THING_SPEAK) {
      iterationsTS = 0;
      writeToThingSpeak(sensorData, METRICS_COUNT);
    }
    iterationsTS = (byte)(iterationsTS + 1);
  #endif

  #ifdef SD_CARD
    // Log to file
    writeToLog(sensorData, METRICS_COUNT);
  #endif

  sendAlert(F("IoTAlert"), sensorData, METRICS_COUNT);
  
  Serial.println(millis());
  delay(SAMPLING_INTERVAL - (millis() - startTime));
}

#ifdef SERIAL_OUTPUT
void printWifiStatus() {
  Serial.println(F("You're connected to the network"));

  // print the SSID of the network
  Serial.print(F("SSID: "));
  Serial.println(WiFi.SSID());

  // print WiFi shield's IP address
  IPAddress ip = WiFi.localIP();
  Serial.print(F("IP Address: "));
  Serial.println(ip);
}
#endif

float * retrieveSensorData() {
  // Initialize array for sensors sampling: 
  // [0] Temperature, [1] Humidity, [2] Pressure, [3] Light, [4] DustDensity, [5] CO2
  static float ret[] = {-1, -1, -1, -1, -1, -1, -1};
  
  #ifdef SERIAL_OUTPUT
    printSensorSamplingInitiation(F("DHT22"));
  #endif
  #ifdef DHT22
    // Use read2 to get a float data, such as 10.1*C
    float temperature = 0;
    float humidity = 0;
    int err = SimpleDHTErrSuccess;
    if ((err = dht22.read2(&temperature, &humidity, NULL)) != SimpleDHTErrSuccess) {
      Serial.print(F("DHT22 RD FAIL, err="));
      Serial.println(err);
    }
    else {
      ret[METRIC_IDX_TEMP] = (float)temperature;
      ret[METRIC_IDX_HUMD] = (float)humidity;

      printFValue(F("Temperature"), (float)temperature, F(" *C"), true, false);
      printFValue(F("Humidity"), (float)humidity, F(" RH%"), true, true);

      #ifdef SERIAL_OUTPUT
        printDivider();
      #endif
    }
  #endif

  #ifdef BMP280
    ret[METRIC_IDX_PRESS] = retrievePressureData();
  #endif
  #ifdef BH1750
    ret[METRIC_IDX_LIGHT] = retrieveIlluminanceData();
  #endif
  #ifdef SHARP_DUSTSENSOR
    ret[METRIC_IDX_DUST] = retrieveDustData();
  #endif
  #ifdef MQ-135
    ret[METRIC_IDX_GASCO2] = retrieveCO2Data(ret[METRIC_IDX_TEMP], ret[METRIC_IDX_HUMD]);
  #endif
  return ret;
}

#ifdef BMP280
float retrievePressureData() {

  #ifdef SERIAL_OUTPUT
    printSensorSamplingInitiation(F("BMP280"));
  #endif

  if (!bmx280.measure())
  {
    Serial.println(F("BMP280 RD FAIL"));
    return -1;
  }

  //wait for the measurement to finish
  do
  {
    delay(100);
  } while (!bmx280.hasValue());

  float value = bmx280.getPressure() / 100;
  if(isnan(value)) {
    Serial.println(F(">>> BMP280: NaN"));
    value = -1;
  }

  printFValue(F("Pressure"), (float)value, F(" hPa (1 hPa = 100 Pa)"), true, false);

  #ifdef SERIAL_OUTPUT
    printDivider();
  #endif

  return value;
}
#endif

#ifdef BH1750
float retrieveIlluminanceData() {

  #ifdef SERIAL_OUTPUT
    printSensorSamplingInitiation(F("Light Sensor"));
  #endif

  uint16_t lux = LightSensor.GetLightIntensity();
  printFValue(F("Light"), (float)lux, F(" Lux"), true, true);

  return lux;  
}
#endif

#ifdef SHARP_DUSTSENSOR
float retrieveDustData() {

  #ifdef SERIAL_OUTPUT
    printSensorSamplingInitiation(F("Dust Sensor"));
  #endif

  // Turn on the dust sensor LED by setting digital pin LOW.
  digitalWrite(sharpLEDPin, LOW);

  // Wait 0.28ms before taking a reading of the output voltage as per spec.
  delayMicroseconds(280);

  // Record the output voltage. This operation takes around 100 microseconds.
  int VoRaw = analogRead(sharpVoPin);
  
  // Turn the dust sensor LED off by setting digital pin HIGH.
  digitalWrite(sharpLEDPin, HIGH);

  // Wait for remainder of the 10ms cycle = 10000 - 280 - 100 microseconds.
  delayMicroseconds(9620);

  // Use averaging if needed.
  float Vo = VoRaw;
  #ifdef USE_AVG
    VoRawTotal += VoRaw;
    VoRawCount++;
    if (VoRawCount >= N) {
      Vo = 1.0 * VoRawTotal / N;
      VoRawCount = 0;
      VoRawTotal = 0;
    }
  #endif // USE_AVG

  // Compute the output voltage in Volts.
  Vo = Vo * (5.0 / 1024.0);
  printFValue(F("Vo"), Vo*1000.0, F(" mV"), true, false);

  // Convert to Dust Density in units of ug/m3.
  float dV = Vo - sharpVoc;
  if ( dV < 0 ) {
    dV = 0;
    sharpVoc = Vo;
    printFValue(F("SHARP: Voc"), sharpVoc, F(" <<<"), true, true);
  }
  float dustDensity = dV / sharpK * 100.0;
  printFValue(F("Dust Density"), dustDensity, F(" ug/m3"), true, true);

  return dustDensity;
}
#endif

#ifdef MQ-135
float retrieveCO2Data(float temperature, float humidity) {

  #ifdef SERIAL_OUTPUT
    printSensorSamplingInitiation(F("CO2 Sensor"));
  #endif

  float rzero = mq135_sensor.getRZero();
  float correctedRZero = mq135_sensor.getCorrectedRZero(temperature, humidity);
  float resistance = mq135_sensor.getResistance();
  float ppm = mq135_sensor.getPPM();
  float correctedPPM = mq135_sensor.getCorrectedPPM(temperature, humidity);

  Serial.print("MQ135 RZero: ");
  Serial.print(rzero);
  Serial.print("\t Corrected RZero: ");
  Serial.print(correctedRZero);
  Serial.print("\t Resistance: ");
  Serial.print(resistance);
  Serial.print("\t PPM: ");
  Serial.print(ppm);
  Serial.print("\t Corrected PPM: ");
  Serial.print(correctedPPM);
  Serial.println("ppm");  

  printFValue(F("CO2"), correctedPPM, F(" PPM"), true, true);
  
  return correctedPPM;
}
#endif

void writeFields(float * data, int length) {
    for (int i = 0; i < length; i++) {
      client.print(F("&field"));
      client.print(i+1);
      client.print("=");
      client.print(data[i]);
    }  
}

#ifdef THING_SPEAK_INTEGRATION
void writeToThingSpeak(float * data, int length) {
  // close any connection before send a new request
  // this will free the socket on the WiFi shield
  client.stop();
  
  __FlashStringHelper * server = F("api.thingspeak.com");

  Serial.println(server);
  // if you get a connection, report back via serial
  //if (client.connectSSL("api.thingspeak.com", 443)) {
  // Connecting with plain HTTP since SNI certificates (wildcard) are not supported by wifi library
  if (client.connect("api.thingspeak.com", 80)) {
    Serial.println(F("Connected"));
    // Make a HTTP request

    client.print(F(THINGSPEAK_WRITE_APIKEY));

    writeFields(data, length);
    client.print(F(" HTTP/1.1\r\nHost: api.thingspeak.com\r\n"));
    
    client.println(F("Connection: close"));
    client.println();

    delay(HTTP_CLIENT_DELAY);

    // if there's incoming data from the net connection send it out the serial port
    // this is for debugging purposes only
    while (client.available()) {
      char c = client.read();
      Serial.write(c);
    }    
    //client.stop();
    Serial.println();
  }
}
#endif

#ifdef SD_CARD
void writeToLog(float * data, int length) {
  File file = SD.open(F("log.txt"), FILE_WRITE);
  if (file) {
    Serial.print(F("LOG: "));

    // Write data to file.  Start with log time in millis or get time from RTC module.
    #ifdef RTC
      DateTime now = rtc.now();

      char buf2[] = "YYYYMMDD hh:mm:ss";
      datetimeToString(now, buf2);
      file.print(buf2);
      Serial.print(buf2);
    #else
      file.print(millis());
    #endif
    
    for (int i = 0; i < length; i++) {
      file.write(',');
      file.print(data[i]);
      
      Serial.print(',');
      Serial.print(data[i]);
    }
    file.println();   
    
    Serial.println();
    Serial.println();
    file.close();
  } else {
    Serial.println(F("FAIL OPEN log.txt"));
  }
}
#endif


#ifdef RTC
char * datetimeToString(DateTime dt, char *buffer) {
  uint8_t hourReformatted;

  for (size_t i = 0; i < strlen(buffer) - 1; i++) {
    if (buffer[i] == 'h' && buffer[i + 1] == 'h') {
      buffer[i] = '0' + dt.hour() / 10;
      buffer[i + 1] = '0' + dt.hour() % 10;
    }
    if (buffer[i] == 'm' && buffer[i + 1] == 'm') {
      buffer[i] = '0' + dt.minute() / 10;
      buffer[i + 1] = '0' + dt.minute() % 10;
    }
    if (buffer[i] == 's' && buffer[i + 1] == 's') {
      buffer[i] = '0' + dt.second() / 10;
      buffer[i + 1] = '0' + dt.second() % 10;
    }
    if (buffer[i] == 'D' && buffer[i + 1] == 'D') {
      buffer[i] = '0' + dt.day() / 10;
      buffer[i + 1] = '0' + dt.day() % 10;
    }
    if (buffer[i] == 'M' && buffer[i + 1] == 'M') {
      buffer[i] = '0' + dt.month() / 10;
      buffer[i + 1] = '0' + dt.month() % 10;
    }
    if (buffer[i] == 'Y' && buffer[i + 1] == 'Y' && buffer[i + 2] == 'Y' &&
        buffer[i + 3] == 'Y') {
      buffer[i] = '2';
      buffer[i + 1] = '0';
      buffer[i + 2] = '0' + ((dt.year() - 2000) / 10) % 10;
      buffer[i + 3] = '0' + (dt.year() - 2000) % 10;
    } else if (buffer[i] == 'Y' && buffer[i + 1] == 'Y') {
      //buffer[i] = '0' + (yOff / 10) % 10;
      //buffer[i + 1] = '0' + yOff % 10;
    }

  }
  return buffer;
}
#endif


void alertIfLimitsExceeded(float * data, int length) {

  
}

void sendAlert(__FlashStringHelper * message, float * data, int length) {
  // close any connection before send a new request
  // this will free the socket on the WiFi shield
  client.stop();

  // if you get a connection, report back via serial
  if (client.connect("iot.varouxis.net", 80)) {
    Serial.println(F("Connected"));
    // Make a HTTP request

    client.print(F(SEND_ALERT_URL));
    client.print(message);
    writeFields(data, length);
    
    client.println(F(" HTTP/1.1\r\nHost: iot.varouxis.net\r\n"));

    delay(HTTP_CLIENT_DELAY);

    // if there's incoming data from the net connection send it out the serial port
    // this is for debugging purposes only
    while (client.available()) {
      char c = client.read();
      Serial.write(c);
    }  
    //client.stop();
    Serial.println(); 
  }
}
/*
What are safe levels of CO and CO2 in rooms?
CO2
  
250-400ppm  Normal background concentration in outdoor ambient air
400-1,000ppm  Concentrations typical of occupied indoor spaces with good air exchange
1,000-2,000ppm  Complaints of drowsiness and poor air.
2,000-5,000 ppm   Headaches, sleepiness and stagnant, stale, stuffy air. Poor concentration, loss of attention, increased heart rate and slight nausea may also be present.
5,000   Workplace exposure limit (as 8-hour TWA) in most jurisdictions.
>40,000 ppm   Exposure may lead to serious oxygen deprivation resulting in permanent brain damage, coma, even death.
CO
  
9 ppm   CO Max prolonged exposure (ASHRAE standard)
35 ppm  CO Max exposure for 8 hour work day (OSHA)
800 ppm   CO Death within 2 to 3 hours
12,800 ppm  CO Death within 1 to 3 minutes 
*/
