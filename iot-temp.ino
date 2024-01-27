#include "DHT.h"
#include "WiFi.h"
#include <ArduinoJson.h>
#include <MQTT.h>
#include <LiquidCrystal.h>

#include "arduino_secrets.h"

// Temperature and humidity sensor
#define DHT_PIN 22
#define DHT_TYPE DHT11
DHT dht(DHT_PIN, DHT_TYPE);
const float temp_offset = 2;

// Soil moisture sensor
const int soil_pin = 35;

// LCD
const int rs = 27, en = 26, d4 = 25, d5 = 33, d6 = 32, d7 = 23;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
const int lcd_contrast_pin = 14;
const int lcd_contrast = 75;

// ! Define env vars in arduino_secrets.h
const char wifi_ssid[] = WIFI_SSID;
const char wifi_pass[] = WIFI_PASS;

WiFiClient wifiClient;
// We need to ensure the buffer is large enough to handle the JSON payload, else
// it will be truncated.
MQTTClient mqttClient(512);

const char mqtt_broker[] = MQTT_BROKER;
const int mqtt_port = MQTT_PORT;
const char mqtt_username[] = MQTT_USERNAME;
const char mqtt_password[] = MQTT_PASSWORD;

String mqtt_topic = MQTT_TOPIC;

// Home Assistant discovery metadata
String device_name = DEVICE_NAME;
String device_model = DEVICE_MODEL;
String device_fw_version = DEVICE_FW_VERSION;
String device_manufacturer = DEVICE_MANUFACTURER;
bool is_discovery_complete = false;

const long interval = 10000;
unsigned long previousMillis = 0;
String unique_id;

void setup()
{
  Serial.begin(115200);

  analogWrite(lcd_contrast_pin, lcd_contrast);
  lcd.begin(16, 2);

  delay(1000);
  while (!Serial)
  {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  initWiFi(wifi_ssid, wifi_pass);

  initMqtt(mqtt_broker, mqtt_port);
  mqttConnect(mqtt_username, mqtt_password);

  dht.begin();
}

void loop()
{
  mqttClient.loop();

  if (!mqttClient.connected())
  {
    mqttConnect(mqtt_username, mqtt_password);
  }

  // Home Assistant discovery process
  if (!is_discovery_complete)
  {
    MqttHomeAssistantDiscovery();
    is_discovery_complete = true;
  }

  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval)
  {
    // Save the last time a message was sent
    previousMillis = currentMillis;

    // Read humidity
    float humidity = dht.readHumidity();
    // Read temperature as Celsius
    float tempC = dht.readTemperature() - temp_offset;
    // Read soil moisture
    float soil_moisture = map(analogRead(soil_pin), 0, 2000, 0, 100);

    // Check if any reads failed
    if (isnan(humidity) || isnan(tempC))
    {
      Serial.println("Failed to read from DHT sensor.");
    }
    else
    {
      Serial.print("Humidity: [");
      Serial.print(humidity);
      Serial.print("%]");

      Serial.print("  |  ");

      Serial.print("Temperature: [");
      Serial.print(tempC);
      Serial.print("°C]");

      Serial.print("  |  ");

      Serial.print("Soil moisture: [");
      Serial.print(soil_moisture);
      Serial.print("%]");
      Serial.println();

      char tempC_out[20];
      dtostrf(tempC, 2, 2, tempC_out);
      char humidity_out[20];
      dtostrf(humidity, 2, 2, humidity_out);
      char soil_moisture_out[20];
      dtostrf(soil_moisture, 2, 2, soil_moisture_out);

      // Write to LCD
      lcd.setCursor(0, 0);
      lcd.print("Temp:     ");
      lcd.print(tempC_out);
      lcd.print("C");
      lcd.setCursor(0, 1);
      lcd.print("Humidity: ");
      lcd.print(humidity_out);
      lcd.print("%");

      // Write to Home Assistant
      StaticJsonDocument<200> payload;
      payload["temperature"] = tempC_out;
      payload["humidity"] = humidity_out;
      payload["soil_moisture"] = soil_moisture_out;

      String strPayload;
      serializeJson(payload, strPayload);

      sendMqttMessage(mqtt_topic, strPayload);
    }

    Serial.println();
  }
}

void initWiFi(const char ssid[], const char password[])
{
  WiFi.mode(WIFI_STA);
  WiFi.enableIpV6();

  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi with SSID [");
  Serial.print(ssid);
  Serial.print("]...");

  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(1000);
  }

  // Generate a unique ID from the MAC address
  byte mac[6];
  WiFi.macAddress(mac);
  unique_id = String(mac[0], HEX) + String(mac[1], HEX) + String(mac[2], HEX) + String(mac[3], HEX) + String(mac[4], HEX) + String(mac[5], HEX);

  Serial.println();
  Serial.print("Connected to the network with IP: [");
  Serial.print(WiFi.localIP());
  Serial.print("] and MAC address [");
  Serial.print(unique_id);
  Serial.println("].");
  Serial.println();
}

void initMqtt(const char broker[], const int port)
{
  Serial.print("Attempting to initialise MQTT client for broker: [");
  Serial.print(broker);
  Serial.print(":");
  Serial.print(port);
  Serial.println("])...");

  mqttClient.begin(broker, port, wifiClient);
  mqttClient.onMessage(MqttReceiverCallback);

  Serial.println("Initialised MQTT client.");
}

void mqttConnect(const char username[], const char password[])
{
  Serial.print("Attempting to connect to the MQTT broker as user [");
  Serial.print(username);
  Serial.println("]...");

  if (!mqttClient.connect(device_name.c_str(), username, password))
  {
    Serial.print(".");
    delay(1000);
  }

  Serial.println("Connected to the MQTT broker.");
  Serial.println();
}

void sendMqttMessage(String topic, String message)
{
  Serial.print("Sending message to topic: [");
  Serial.print(topic);
  Serial.print("] with payload [");
  Serial.print(message);
  Serial.println("]...");

  mqttClient.publish(topic, message);

  Serial.print("Message sent.");
  Serial.println();
}

// Unit name must be one of: https://www.home-assistant.io/integrations/sensor/#device-class
void InitMqttHomeAssistantDiscoveryConfig(String unit_name, String unit_class, String unit_of_meas, String topic)
{
  String strPayload;
  StaticJsonDocument<600> payload;

  String discoveryTopic = "homeassistant/sensor/esp32iotsensor/" + device_name + "_" + unit_name + "/config";

  payload["name"] = device_name + "." + unit_name;
  payload["uniq_id"] = unique_id + "_" + unit_name;
  payload["stat_t"] = topic;
  payload["dev_cla"] = unit_class;
  payload["val_tpl"] = "{{ value_json." + unit_name + " | is_defined }}";
  payload["unit_of_meas"] = unit_of_meas;
  JsonObject device = payload.createNestedObject("device");
  device["name"] = device_name;
  device["model"] = device_model;
  device["sw_version"] = device_fw_version;
  device["manufacturer"] = device_manufacturer;
  JsonArray identifiers = device.createNestedArray("identifiers");
  identifiers.add(unique_id);

  serializeJson(payload, strPayload);

  sendMqttMessage(discoveryTopic, strPayload);
}

void MqttHomeAssistantDiscovery()
{
  Serial.println("Establishing Home Assistant discovery...");
  InitMqttHomeAssistantDiscoveryConfig("temperature", "temperature", "°C", mqtt_topic);
  InitMqttHomeAssistantDiscoveryConfig("humidity", "humidity", "%", mqtt_topic);
  InitMqttHomeAssistantDiscoveryConfig("soil_moisture", "moisture", "%", mqtt_topic);
  Serial.println("Established Home Assistant discovery.");
  Serial.println();
}

void MqttReceiverCallback(String &topic, String &payload)
{
  if (String(topic) == String("homeassistant/status"))
  {
    Serial.println("Status message changed on homeassistant/status. This might indicate that Home Assistant has restarted.");
    Serial.println();
    if (payload == "online")
      MqttHomeAssistantDiscovery();
  }
}