


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

#define HOSTNAME "ESP12E-" ///< Hostename. The setup function adds the Chip ID at the end
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
bool ESP12E_gpio_exist_list[] = { true, false, true, false, true, true, false, false, false, true, true, false, true, true, true, true, true };
                                        
String outtopic("");

String get_full_hostname() {
  String my_hostname(HOSTNAME);
  my_hostname += String(ESP.getChipId(), HEX);
  return my_hostname;
}

String topic_cmd() {
  return get_full_hostname() + String("/cmd");
}

String gpio_list_payload() {
    String tmp("");
  for (int i=0; i<sizeof(ESP12E_gpio_exist_list)/sizeof(bool); i++) {
    if (ESP12E_gpio_exist_list[i]) {
      tmp += String(i, HEX);
      tmp += String(",");
    }
  }
  return tmp;
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

String build_payload_string(byte* payload, unsigned int length) {
  String ret("");
  for (int i=0; i<length; i++) {
    ret += String((char)payload[i]);
  }
  return ret;
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  if (String(topic) == topic_cmd()) {
    Serial.println("DEBUG: topic OK");
    //parse the  payload
    //payload can be either:
    // direction <<gpio#>> output OR direction <<gpio#>> input <<poll_period>>
    String payload_string = build_payload_string(payload, length);
    int offset = 0;
    if (payload_string.substring(offset, offset + strlen("out-topic")) == String("out-topic")) {
      offset += strlen("out-topic") + 1;
      Serial.print("Received  command to set out-topic to: ");
      outtopic = payload_string.substring(offset);
      Serial.println(outtopic);
    } else if (payload_string.substring(offset, offset + strlen("write")) == String("write")) {
      offset += strlen("write") + 1;
      int gpio_num = payload_string.substring(offset, offset+2).toInt();
      offset += 3;
      Serial.print("Received  command to write GPIO# ");
      Serial.print(gpio_num);
      int value = payload_string.substring(offset).toInt();
      Serial.print(" To value: ");
      Serial.println(value);
      pinMode(gpio_num, OUTPUT); 
      digitalWrite(gpio_num, value & 0x01);
    } else if (payload_string.substring(offset, offset + strlen("read")) == String("read")) {
      offset += strlen("read") + 1;
      int gpio_num = payload_string.substring(offset, offset+2).toInt();
      offset += 3;
      Serial.print("Received  command to read from GPIO# ");
      Serial.println(gpio_num);
      pinMode(gpio_num, INPUT); 
      if (outtopic != "") {
        client.publish((get_full_hostname() + String("/") + outtopic).c_str(), String(digitalRead(gpio_num)&0x01, HEX).c_str());//TODO
      }
    } else if (payload_string.substring(offset, offset + strlen("list")) == String("list")) {
        Serial.println("Received command to list all GPIOs");
        if (outtopic != "") {
          client.publish((get_full_hostname() + String("/") + outtopic).c_str(), gpio_list_payload().c_str());
        }
    } else {
      Serial.println("unknown command:" + payload_string.substring(offset));
    }
  }

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
    //                 id,               NULL, NULL, willTopic,willQos,willRetain,willMessage, cleanSession
    if (client.connect(get_full_hostname().c_str(), NULL, NULL, NULL,     0,      0,         NULL,        false)) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("outTopic", "hello from ESP");
      //Publish to "iot-devices" topic, our device hostname - used by other clients subscribed to iot-devices to register our device into their Database
      client.publish("iot-devices", get_full_hostname().c_str());
      // ... and resubscribe
      client.subscribe("inTopic", 1);//qos = 1
      //other client: publish to topic: "FULL-HOSTNAME/cmd", payload: "out-topic <<topic>>" - will set the 
      //out-topic for all input related commands
      //Other client: publish to topic: "FULL-HOSTNAME/cmd" payload: "write <<gpio#>> <<val>>" - will configure GPIO# as output 
      //and write the value
      //other client: publish to topic: "FULL-HOSTNAME/cmd" payload: "read <<gpio#>>" will configure GPIO# to direction=INPUT
      //, read this gpio and publish to topic: "FULL-HOSTNAME/<<out_topic>>" with the payload: <<value>>
      //other client: publish to topic: "FULL-HOSTNAME/cmd" payload: "list" - will cause this device to publish the gpio list 
      //into topic: FULL-HOSTNAME/<<out_topic>> the payload: (e.g.) "0,1,2,4,5" as the existing GPIOs in this device
      client.subscribe(topic_cmd().c_str(), 1);//qos = 1
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
    // Start OTA server.
  ArduinoOTA.setHostname((const char *)hostname.c_str());
  ArduinoOTA.begin();

  pinMode(2, OUTPUT);     // Initialize the BUILTIN_LED pin as an output
  Serial.begin(115200);

  client.setCallback(callback);
  client.setServer(mqtt_server, 1883);

  setup_wifi();

}

void loop() {

  if (!client.connected()) {
    reconnect();
  }
  client.loop();
#if 0
  long now = millis();
  if (now - lastMsg > 2000) {
    lastMsg = now;
    ++value;
    snprintf (msg, 75, "hello from ESP #%ld", value);
    Serial.print("Publish message: ");
    Serial.println(msg);
    client.publish("outTopic", msg);
  }
#endif
  // Handle OTA server.
  ArduinoOTA.handle();
  yield();

}


