// ESP8266 weather station
// Data upload to emoncms
// D0/GPIO16 needs to be connect to RST pin to return from deep sleep
// Garbage on default serial port due to the boot rom
// https://github.com/espressif/esptool/wiki/ESP8266-Boot-ROM-Log
// TODO: If neeeded use the second serial
// TODO: BME compesate for altitude, convert your current altitude in m to ft, 
// divide /20 or /30 subscract that value from a close METAR station (Q = hpa)
// If using a BMP280 SD0 to 3V3 => I2C address BMP280 0x77
// BME280 is also on 0x77

#include <ESP8266WiFi.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <PubSubClient.h>

const char* ssid = "ssid";
const char* password = "password";
const char* mqtt_server = "broker.mqtt-dashboard.com";

#define ONE_WIRE_BUS 0  // DS18B20 pin D3/GPIO0 on ESP8266
#define BATTERY_SENSE A0 // ADC pin to measure battery voltage

#define BME280_I2C_ADDRESS 0x76

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

  //Wait to connect to Wi-Fi for 20 seconds, if not reboot, probably wrong SSID/PASSWORD
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    Serial.print(".");
    delay(20000);
    ESP.restart();
  }

  if (!bme.begin(BME280_I2C_ADDRESS)) {
    Serial.println("Could not find a valid BMΕ280 sensor, check wiring!");
    Serial.print(".");
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
  
  Serial.print("BME280 Temperature = ");
  Serial.print(bme.readTemperature());
  Serial.println(" *C");

  Serial.print("Pressure = ");
  Serial.print(pressure);
  Serial.println(" hPa");

  Serial.print("Approx altitude = ");
  Serial.print(bme.readAltitude(1013.25)); // this should be adjusted to your local forcase
  Serial.println(" m");

  Serial.print("Battery voltage = ");
  Serial.println(VBAT);

  Serial.println();

  WiFiClient WiFiESP8266client;
  PubSubClient MQTTclient(WiFiESP8266client);

  if (!WiFiESP8266client.connect(mqtt_server, 1883)) {
    Serial.println("Connection failed");
    Serial.println("Going to sleep mode for 5 minutes and rebooting");
    //Sleep for 5 minutes to avoid battery drain and then reboot hopefully the server will be up
    ESP.deepSleep(5 * 60 * 1000000, WAKE_RF_DEFAULT);
    delay(1000);
    ESP.restart();
    return;
  }

  char outstr[15];

  dtostrf(temperature,4, 2, outstr);
  String temperaturestring = outstr;
  Serial.print("Temperature String:");
  Serial.println(temperaturestring);

  dtostrf(humidity,4, 2, outstr);
  String humiditystring = outstr;
  Serial.print("Humidity String:");
  Serial.println(humiditystring);

  dtostrf(pressure,4, 2, outstr);
  String pressurestring = outstr;
  Serial.print("Pressure String:");
  Serial.println(pressurestring);

  dtostrf(VBAT,4, 2, outstr);
  String voltagestring = outstr;

// mqtt send here
  MQTTclient.setServer(mqtt_server, 1883);
  if (MQTTclient.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (MQTTclient.connect(clientId.c_str())) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      MQTTclient.publish("outTopic", "hello world");
    } else {
      Serial.print("failed, rc=");
      Serial.print(MQTTclient.state());
    }
  }
  delay(2000);

  // Read the server response and print it to serial
  while(WiFiESP8266client.available()){
    String line = WiFiESP8266client.readStringUntil('\r');
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
