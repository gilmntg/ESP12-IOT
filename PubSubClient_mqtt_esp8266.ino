


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
#include "SimpleTimer.h"

extern "C" {
#include "user_interface.h"
}


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

//SimpleTimer timer;

class Blinker {

  public:
    enum  BlinkRate { BLINK_RATE_OFF, BLINK_RATE_SLOW, BLINK_RATE_FAST, BLINK_RATE_ON };
    enum  LedState  { LED_STATE_OFF = 1, LED_STATE_ON = 0 };

  private:
    static LedState m_led_state;
    enum Timeout { SLOW_RATE_TIMEOUT = 500, FAST_RATE_TIMEOUT = 100 };
    static int m_led_gpio;
//    int m_timer_num;
    os_timer_t m_timer;


  public:
    Blinker(int gpio_num) {
      m_led_gpio = gpio_num;
      m_led_state = LED_STATE_OFF;
      pinMode(m_led_gpio, OUTPUT);
      digitalWrite(m_led_gpio, m_led_state);
    }
    ~Blinker() {
      os_timer_disarm(&m_timer);
      m_led_state = LED_STATE_OFF;
      digitalWrite(m_led_gpio, m_led_state);
    };
    void SetRate(BlinkRate r) {
      switch (r) {
        case BLINK_RATE_OFF:
          os_timer_disarm(&m_timer);
          m_led_state = LED_STATE_OFF;
          digitalWrite(m_led_gpio, m_led_state);
          break;
        case BLINK_RATE_SLOW:
          os_timer_disarm(&m_timer);
          os_timer_setfn(&m_timer, (os_timer_func_t *)ToggleLed, NULL); //pArg = NULL for now
          os_timer_arm(&m_timer, SLOW_RATE_TIMEOUT,  1); //repetitive
          break;
        case BLINK_RATE_FAST:
            os_timer_disarm(&m_timer);
            os_timer_setfn(&m_timer, (os_timer_func_t *)ToggleLed, NULL); //pArg = NULL for now
            os_timer_arm(&m_timer, FAST_RATE_TIMEOUT,  1); //repetitive
          break;
        case BLINK_RATE_ON:
          os_timer_disarm(&m_timer);
          m_led_state = LED_STATE_ON;
          digitalWrite(m_led_gpio, m_led_state);
          break;
      }
    }

  private:
    static void ToggleLed(void *pArg) {
      if (m_led_state == LED_STATE_OFF) {
        m_led_state = LED_STATE_ON;
      } else {
        m_led_state = LED_STATE_OFF;
      }
      digitalWrite(m_led_gpio, m_led_state);
    }
};

Blinker::LedState Blinker::m_led_state = Blinker::LED_STATE_OFF;
int Blinker::m_led_gpio = -1;  //dummy value

Blinker active_led(2); //Indicates connection to Wifi and connection to MQTT status


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
  for (int i = 0; i < sizeof(ESP12E_gpio_exist_list) / sizeof(bool); i++) {
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
  for (int i = 0; i < length; i++) {
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
    // "write <<gpio#>> <value>>" OR "read <<gpio#>>" OR "list" OR "out-topic <<some string>>"

    char* token = strtok((char*)payload, " \0\n");
    if (String(token) == "out-topic") {
      token = strtok(NULL, " \0\n");
      if (token != NULL) {
        outtopic = String(token);
        Serial.print("Received command to set out-topic to: ");
        Serial.println(outtopic);
      } else {
        Serial.println("Invalid out-topic payload");
      }
    } else if (String(token) == "write") {
      token = strtok(NULL, " \0\n");
      if (token != NULL) {
        int gpio_num = String(token).toInt();
        token = strtok(NULL, " \n");
        if (token != NULL) {
          int gpio_val = String(token).toInt() & 0x01;
          Serial.print("Received command to write to GPIO# ");
          Serial.print(gpio_num);
          Serial.print(" The value: ");
          Serial.println(gpio_val);
          pinMode(gpio_num, OUTPUT);
          digitalWrite(gpio_num, gpio_val & 0x01);
        } else {
          Serial.println("Invalid gpio_val argument to \"write\" topic");
        }
      } else {
        Serial.println("Invalid gpio_num argument to \"write\" topic");
      }
    } else if (String(token) == "read") {
      token = strtok(NULL, " \0\n");
      if (token != NULL) {
        Serial.print("token = ");
        Serial.println(token);
        int gpio_num = String(token).toInt();
        Serial.print("Received command to read from GPIO# ");
        Serial.println(gpio_num);
        pinMode(gpio_num, INPUT);
        if (outtopic != "") {
          client.publish((get_full_hostname() + String("/") + outtopic).c_str(), String(digitalRead(gpio_num) & 0x01, HEX).c_str()); //TODO
        }
      } else {
        Serial.println("Invalid gpio_num argument to \"read\" topic");
      }
    } else if (String(token) == "list") {
      Serial.println("Received command to list all existing GPIOs");
      if (outtopic != "") {
        client.publish((get_full_hostname() + String("/") + outtopic).c_str(), gpio_list_payload().c_str());
      }
    } else  {
      Serial.print("unknown command: ");
      Serial.println((char*)payload);
    }
  }
}


void reconnect() {

  // Loop until we're reconnected
  active_led.SetRate(Blinker::BLINK_RATE_FAST);
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
  active_led.SetRate(Blinker::BLINK_RATE_ON);
}


void setup() {

  active_led.SetRate(Blinker::BLINK_RATE_OFF);

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

  active_led.SetRate(Blinker::BLINK_RATE_SLOW);
  setup_wifi();
  active_led.SetRate(Blinker::BLINK_RATE_FAST);
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    active_led.SetRate(Blinker::BLINK_RATE_SLOW);
    setup_wifi();
    active_led.SetRate(Blinker::BLINK_RATE_FAST);
  }

  if (!client.connected()) {
    reconnect();
  } 

  client.loop();
  //run the led blink timer

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


