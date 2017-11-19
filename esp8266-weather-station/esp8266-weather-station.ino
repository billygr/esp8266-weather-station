// ESP8266 weather station
// Data upload to emoncms
// D0/GPIO16 needs to be connect to RST pin to return from deep sleep
// Garbage on default serial port due to the boot rom
// https://github.com/espressif/esptool/wiki/ESP8266-Boot-ROM-Log
// TODO: If neeeded use the second serial

#include <ESP8266WiFi.h>
#include <OneWire.h>
#include <DallasTemperature.h>

const char* ssid = "ssid";
const char* password = "password";

const char* host = "host";
const char* apikey = "0123456789ABCDEF";

#define ONE_WIRE_BUS D2  // DS18B20 pin
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature DS18B20(&oneWire);

void setup () {

  Serial.begin(115200);         //Serial connection

  // Disable the WiFi persistence to reduce the writes to the flash
  // https://github.com/esp8266/arduino/issues/1054
  WiFi.persistent(false);
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

  Serial.println();
  Serial.print("Connected to WiFi: ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  float temperature = getTemperature();

  Serial.print("Temperature: ");
  Serial.println(temperature);


  WiFiClient client;
  const int httpPort = 80;
  if (!client.connect(host, httpPort)) {
    Serial.println("Connection failed");
    return;
  }

  char outstr[15];
  dtostrf(temperature,4, 2, outstr);
  String temperaturestring= outstr;

  Serial.print("Temperature String:");
  Serial.println(temperaturestring);

  String url = "/emoncms/input/post.json?json={Temperature_ESP8266:"+ temperaturestring +"}&apikey="+apikey;

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
