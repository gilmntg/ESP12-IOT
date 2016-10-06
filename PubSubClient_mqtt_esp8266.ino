/*
 Basic ESP8266 MQTT example

 This sketch demonstrates the capabilities of the pubsub library in combination
 with the ESP8266 board/library.

 It connects to an MQTT server then:
  - publishes "hello world" to the topic "outTopic" every two seconds
  - subscribes to the topic "inTopic", printing out any messages
    it receives. NB - it assumes the received payloads are strings not binary
  - If the first character of the topic "inTopic" is an 1, switch ON the ESP Led,
    else switch it off

 It will reconnect to the server if the connection is lost using a blocking
 reconnect function. See the 'mqtt_reconnect_nonblocking' example for how to
 achieve the same result without blocking the main loop.

 To install the ESP8266 board, (using Arduino 1.6.4+):
  - Add the following 3rd party board manager under "File -> Preferences -> Additional Boards Manager URLs":
       http://arduino.esp8266.com/stable/package_esp8266com_index.json
  - Open the "Tools -> Board -> Board Manager" and click install for the ESP8266"
  - Select your ESP8266 in "Tools -> Board"

*/

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <FS.h>
#include <ArduinoOTA.h>
#include "Timer.h"

#define HOSTNAME "ESP8266-1-OTA-" ///< Hostename. The setup function adds the Chip ID at the end.

// Update these with values suitable for your network.

const char* ssid = "gmontag-room";
const char* password = "gm3351324";
const char* mqtt_server = "raspberrypi-desktop";

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;


//GPIO#                            0     1     2     3     4     5     6      7     8       9    10     11    12     13    14    15   16
int ESP12E_gpio_exist_list[] = { true, false, true, false, true, true, false, false, false, true, true, false, true, true, true, true, true };
char* ESP12E_gpio_num_list[] = { "0", "1", "2", "3", "4", "5", "6", "7", "8", "9" ,"A", "B", "C", "D", "E", "F", "G" };

char* gpio_leading_topic = "/esp/gpio/pin";
char* gpio_mode_topic = "/mode";
char* gpio_val_topic = "/val";

enum topic_tail_desc { MODE_TOPIC, VAL_TOPIC };

void build_topic(int pin, topic_tail_desc tail_desc, char* out_topic, unsigned int length) {
  if (strlen(gpio_leading_topic) + strlen(ESP12E_gpio_num_list[pin]) + (tail_desc == MODE_TOPIC) ? strlen(gpio_mode_topic) : strlen(gpio_val_topic) <= length - 1) {
    memset(out_topic, '\0', length);
    strcat(out_topic, gpio_leading_topic);
    strcat(out_topic, ESP12E_gpio_num_list[pin]);
    if (tail_desc == MODE_TOPIC) {
      strcat(out_topic, gpio_mode_topic);
    } else {
      strcat(out_topic, gpio_val_topic);
    }
  } else {
    Serial.print("out_topic length too short");
  }
}
void pin_mode_callback(char* topic, byte* payload, unsigned int length) {
  //extract the pin number from the topic:
  //first verify that the topic is of type "mode" (otherwise bail out): 
  if (!strcmp(&topic[length-5], "mode")) {
    Serial.print("topic=");
    Serial.println(topic);
    for (int i=0; i<length; i++) {
      Serial.print((char)payload[i]);
    }
  }
}

#define TOPIC_LENGTH 30
  char topic[TOPIC_LENGTH];

void subscribe_all_gpios_mode() {
  //client.setCallback(pin_mode_callback);
  int num_gpios = sizeof(ESP12E_gpio_exist_list)/sizeof(int);
  for (int i = 0; i < num_gpios ; i++ ) {
    if (ESP12E_gpio_exist_list[i]) {
      build_topic(i, MODE_TOPIC, topic, TOPIC_LENGTH);
      client.subscribe(topic, 1);//qos = 1
    }
  }
}


void setup_wifi() {

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  // Switch on the LED if an 1 was received as first character
  if ((char)payload[0] == '1') {
    digitalWrite(2, LOW);   // Turn the LED on (Note that LOW is the voltage level
    // but actually the LED is on; this is because
    // it is acive low on the ESP-01)
  } else {
    digitalWrite(2, HIGH);  // Turn the LED off by making the voltage HIGH
  }

}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    //id,NULL,NULL,willTopic,willQos,willRetain,willMessage, cleanSession
    if (client.connect("ESP8266Client", NULL, NULL, NULL, 0, 0, NULL, false)) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("outTopic", "hello from ESP");
      // ... and resubscribe
      client.subscribe("inTopic", 1);//qos = 1
      subscribe_all_gpios_mode();
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup() {

  Serial.println("\r\n");
  Serial.print("Chip ID: 0x");
  Serial.println(ESP.getChipId(), HEX);

  // Set Hostname.
  String hostname(HOSTNAME);
  hostname += String(ESP.getChipId(), HEX);
  WiFi.hostname(hostname);

  // Print hostname.
  Serial.println("Hostname: " + hostname);
  //Serial.println(WiFi.hostname());

  // Start OTA server.
  ArduinoOTA.setHostname((const char *)hostname.c_str());
  ArduinoOTA.begin();


  pinMode(2, OUTPUT);     // Initialize the BUILTIN_LED pin as an output
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

    // Start OTA server.
  ArduinoOTA.setHostname((const char *)hostname.c_str());
  ArduinoOTA.begin();

}

void loop() {

  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  long now = millis();
  if (now - lastMsg > 2000) {
    lastMsg = now;
    ++value;
    snprintf (msg, 75, "hello from ESP #%ld", value);
    Serial.print("Publish message: ");
    Serial.println(msg);
    client.publish("outTopic", msg);
  }

  // Handle OTA server.
  ArduinoOTA.handle();
  yield();

}


