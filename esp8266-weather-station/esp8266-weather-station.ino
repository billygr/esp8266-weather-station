// ESP8266 weather station
// Data upload to emoncms
// D0/GPIO16 needs to be connect to RST pin to return from deep sleep
// Garbage on default serial port due to the boot rom
// https://github.com/espressif/esptool/wiki/ESP8266-Boot-ROM-Log
// TODO: If neeeded use the second serial
// If using a BMP280 SD0 to 3V3 => I2C address BMP280 0x77
// BME280 is also on 0x77

#include <ESP8266WiFi.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

const char* ssid = "ssid";
const char* password = "password";

// EMONCMS settings
const char* host = "host";
const char* apikey = "0123456789ABCDEF";
const char* node = "0";

#define ONE_WIRE_BUS D3  // DS18B20 pin
#define BATTERY_SENSE A0 // ADC pin to measure battery voltage

Adafruit_BME280 bme; // I2C
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature DS18B20(&oneWire);

unsigned int ADCValue=0;
float VBAT=0.0;

void setup () {

  Serial.begin(115200);         //Serial connection

  ADCValue = analogRead(BATTERY_SENSE);
  VBAT = ADCValue / 1023.0;
  VBAT = VBAT * 4.2;

  // Disable the WiFi persistence to reduce the writes to the flash
  // https://github.com/esp8266/arduino/issues/1054
  WiFi.persistent(false);

  WiFi.disconnect();
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);   //WiFi connection

  //while (WiFi.status() != WL_CONNECTED) {
  //  delay(1000);
  //  Serial.println("Waiting for connection...");
  //}

  //Wait to connect to Wi-Fi for 20 seconds, if not reboot, probably wrong SSID/PASSWORD
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    Serial.print(".");
    delay(20000);
    ESP.restart();
  }

  if (!bme.begin(0x76)) {
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    Serial.print(".");
    delay(20000);
    ESP.restart();
 }
  Serial.println();
  Serial.print("Connected to WiFi: ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  float temperature = getTemperature();
  float pressure =  bme.readPressure()/100;
  float humidity = bme.readHumidity();

  Serial.print("Temperature: ");
  Serial.println(temperature);

  Serial.print("Humidity: ");
  Serial.println(humidity);
  
  //Serial.print("BME280 Temperature = ");
  //Serial.print(bme.readTemperature());
  //Serial.println(" *C");

  Serial.print("Pressure = ");
  Serial.print(pressure);
  Serial.println(" hPa");

  Serial.print("Approx altitude = ");
  Serial.print(bme.readAltitude(1013.25)); // this should be adjusted to your local forcase
  Serial.println(" m");

  Serial.print("Battery voltage = ");
  Serial.println(VBAT);

  Serial.println();

  WiFiClient client;
  const int httpPort = 80;
  if (!client.connect(host, httpPort)) {
    Serial.println("Connection failed");
    return;
  }

  char outstr[15];
  dtostrf(temperature,4, 2, outstr);
  String temperaturestring = outstr;

  Serial.print("Temperature String:");
  Serial.println(temperaturestring);

  dtostrf(pressure,4, 2, outstr);
  String pressurestring = outstr;

  Serial.print("Pressure String:");
  Serial.println(pressurestring);

  dtostrf(VBAT,4, 2, outstr);
  String voltagestring = outstr;

  String url = "/emoncms/input/post.json?json={Temperature_ESP8266:"+ temperaturestring +",Pressure_ESP8266:"+ pressurestring +",Voltage_ESP8266:"+voltagestring +"}&apikey="+apikey;

  Serial.print("Requesting URL: ");
  Serial.println(url);

  client.print(String("GET ") + url + " HTTP/1.1\r\n" +
               "Host: " + host + "\r\n" + 
               "Connection: close\r\n\r\n");
  delay(2000);

  // Read the server response and print it to serial
  while(client.available()){
    String line = client.readStringUntil('\r');
    Serial.print(line);
  }

  Serial.println();
  Serial.println("Closing connection");

  Serial.println("Going to sleep mode");

  //Sleep for 5 minute
  ESP.deepSleep(5 * 60 * 1000000, WAKE_RF_DEFAULT);
  delay(1000);
}

float getTemperature() {
  float temp;
  //do {
    DS18B20.requestTemperatures(); 
    temp = DS18B20.getTempCByIndex(0);
  //  delay(100);
  //} while (temp == 85.0 || temp == (-127.0));
  //TODO: -127.00 means nothing is connected
  //Change the code to detect if indeed the DS18B20 is connected
  return temp;
}

void loop() {
}
