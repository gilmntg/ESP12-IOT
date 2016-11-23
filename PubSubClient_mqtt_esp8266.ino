


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

#include <IRremoteESP8266.h>



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


//GPIO#                            0     1     2      3      4     5      6      7      8      9     10    11     12     13     14    15    16
bool ESP12E_gpio_exist_list[] = { true, false, false, false, true, false, false, false, false, true, true, false, false, false, true, true, true };

String debugtopic("");

class ADCSampler {
  private:
#define MAX_SAMPLES 1000
    static byte m_buff[MAX_SAMPLES]; //up to 1000 samples
    static os_timer_t m_timer;
    static int m_current_samples;
    static int m_samples_to_get;
    int m_samp_rate_ms;
  public:
    ADCSampler() {
      m_current_samples = 0;
      m_samples_to_get = 0;
      os_timer_disarm(&m_timer);
      os_timer_setfn(&m_timer, (os_timer_func_t *)SampleCB, NULL); //pArg = NULL for now
    //  os_timer_arm(&m_timer, m_samp_rate_ms,  1); //repetitive
    }
    ~ADCSampler() {
      os_timer_disarm(&m_timer);
    };
    static void SampleCB(void *pArg) {
      if (m_current_samples < m_samples_to_get) {
        m_buff[m_current_samples++] = analogRead(0);
      } else {
        os_timer_disarm(&m_timer);
      }
    }
    bool StartSampling(int nsamps, int samp_rate_ms) {
      if (nsamps > MAX_SAMPLES)
        return false;
      m_samples_to_get = nsamps;
      m_samp_rate_ms = samp_rate_ms;
      m_current_samples = 0;
      os_timer_disarm(&m_timer);
      os_timer_arm(&m_timer, m_samp_rate_ms,  1); //repetitive
      return true;
    }
    int GetNumAvailableSamples() {
      return m_current_samples;
    }
    void ReadSamples(byte *buff) {
      memcpy(buff, m_buff, m_current_samples);
    }
};


byte ADCSampler::m_buff[MAX_SAMPLES] = {0}; //up to 1000 samples
os_timer_t ADCSampler::m_timer;
int ADCSampler::m_current_samples = 0;
int ADCSampler::m_samples_to_get = 0;

ADCSampler sampler;


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

enum protocol {
  PROT_UNDEFINED = -1,
  PROT_RC5,
  PROT_RC6,
  PROT_NEC,
  PROT_SONY,
  PROT_PANASONIC,
  PROT_JVC,
  PROT_SAMSUNG,
  PROT_WHYNTER,
  PROT_DISH,
  PROT_SHARP,
  PROT_COOLIX
};
String protocol_names_lut[] = {
  "RC5",
  "RC6",
  "NEC",
  "SONY",
  "PANASONIC",
  "JVC",
  "SAMSUNG",
  "WHYNTER",
  "DISH",
  "SHARP",
  "COOLIX"
};

int get_protocol_from_name(String& name) {
  for (int i=0; i<sizeof(protocol_names_lut)/sizeof(protocol_names_lut[0]); i++) {
    if (name == protocol_names_lut[i]) {
      return i;
    }
  }
  return PROT_UNDEFINED;
}
IRsend irsend(5); //using GPIO5 for IR

void ir_send_prot(int prot, unsigned int command, int nbits) {
  switch (prot) {
    case PROT_RC5:
      irsend.sendRC5(command, nbits);
    break;
    case PROT_RC6:
      irsend.sendRC6(command, nbits);
    break;
    case PROT_NEC:
      irsend.sendNEC(command, nbits);
    break;
    case PROT_SONY:
      irsend.sendSony(command, nbits);
    break;
    case PROT_PANASONIC:
      irsend.sendPanasonic(command, nbits);
    break;
    case PROT_JVC:
      irsend.sendJVC(command, nbits, 0);//repeat = 0
    break;
    case PROT_SAMSUNG:
      irsend.sendSAMSUNG(command, nbits);
    break;
    case PROT_WHYNTER:
      irsend.sendWhynter(command, nbits);
    break;
    case PROT_DISH:
      irsend.sendDISH(command, nbits);
    break;
    case PROT_SHARP:
      irsend.sendSharp(command, nbits);
    break;
    case PROT_COOLIX:
      irsend.sendCOOLIX(command, nbits);
    break;
    default:
    break;
  }//switch
}

unsigned int irSignalBuf[100]; //raw buffer for IR irsend.sendRaw()
unsigned char samples[100]; //raw buffer for ADC samples sending

void mqtt_debug_print( const char *msg ) {
    Serial.print(msg);
    if (debugtopic != "") {
      client.publish((get_full_hostname() + String("/") + debugtopic).c_str(), msg);
    }
}
void mqtt_debug_println( const char *msg ) {
    Serial.println(msg);
    if (debugtopic != "") {
      client.publish((get_full_hostname() + String("/") + debugtopic).c_str(), (msg + String("\n")).c_str());
    }
}

#define MAX_PAYLOAD_LEN 100
char strtok_payload[MAX_PAYLOAD_LEN];
#define TOKEN_DELIM " \0\n"

byte analog_samples_buff[1000]; 


void callback(char* topic, byte* payload, unsigned int length) {
  if (length > MAX_PAYLOAD_LEN) {
    Serial.println("Too long payload in MQTT message");
    return;
  }
  memset(strtok_payload, 0, sizeof(strtok_payload));//strtok requires a buffer with NULL chars at it's end to work on
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
    strtok_payload[i] = payload[i];
  }
  Serial.println();

  if (String(topic) == topic_cmd()) {
    mqtt_debug_println("topic OK");
    //parse the  payload
    //payload can be either:
    // "write <<gpio#>> <value>>" OR "read <<gpio#>>" OR "list" OR "out-topic <<some string>>"

    char* token = strtok(strtok_payload, TOKEN_DELIM);
    Serial.println(String("first token (from payload) = ") + String(token));
    /*******************************************************************************************/
    if (String(token) == "debug-topic") {
      token = strtok(NULL, TOKEN_DELIM);
      Serial.println("token = " + String(token));
      if (token != NULL) {
        debugtopic = String(token);
        Serial.print("Received command to set debug-topic to: ");
        Serial.println(debugtopic);
      } else {
        Serial.println("Invalid debug-topic payload");
      }
    /*******************************************************************************************/
    } else if (String(token) == "write") {
      token = strtok(NULL, TOKEN_DELIM);
      if (token != NULL) {
        int gpio_num = String(token).toInt();
        token = strtok(NULL, TOKEN_DELIM);
        if (token != NULL) {
          int gpio_val = String(token).toInt() & 0x01;
          mqtt_debug_print("Received command to write to GPIO# ");
          mqtt_debug_print(String(gpio_num, DEC).c_str());
          mqtt_debug_print(" The value: ");
          mqtt_debug_println(token);
          pinMode(gpio_num, OUTPUT);
          digitalWrite(gpio_num, gpio_val & 0x01);
        } else {
          mqtt_debug_println("Invalid gpio_val argument to \"write\" topic");
        }
      } else {
        mqtt_debug_println("Invalid gpio_num argument to \"write\" topic");
      }
    /*******************************************************************************************/
    } else if (String(token) == "read") {
      token = strtok(NULL, TOKEN_DELIM);
      if (token != NULL) {
        mqtt_debug_print("token = ");
        mqtt_debug_println(token);
        int gpio_num = String(token).toInt();
        //get result topic
        token = strtok(NULL, TOKEN_DELIM);
        if (token != NULL) {
          String restopic = String(token);
          mqtt_debug_print("Received command to read from GPIO# ");
          mqtt_debug_print(String(gpio_num, DEC).c_str());
          mqtt_debug_print(" into result topic:");
          mqtt_debug_println(restopic.c_str());
          pinMode(gpio_num, INPUT);
          if (restopic != "") {
            client.publish((get_full_hostname() + String("/") + restopic).c_str(), String(digitalRead(gpio_num) & 0x01, HEX).c_str()); //TODO
          }
        } else {
          mqtt_debug_println("Invalid result_topic argument to \"read\" topic");
        }
      } else {
        mqtt_debug_println("Invalid gpio_num argument to \"read\" topic");
      }
    } 
    /*******************************************************************************************/
    else if (String(token) == "startsamp") { //Start A/D sampling command
        token = strtok(NULL, TOKEN_DELIM);
        if (token != NULL) {
          int num = String(token).toInt();
          mqtt_debug_print("Received command to Start A/D#0, num samples = ");
          mqtt_debug_print(String(num).c_str());
          sampler.StartSampling(num, 10); //10 ms sampling rate
          //client.publish((get_full_hostname() + String("/") + restopic).c_str(), String(digitalRead(gpio_num) & 0x01, HEX).c_str()); //TODO
          //dump last N samples from A/D circular buffer
        } else {
          mqtt_debug_println("Invalid num_samples argument for A/D start sampling command");
        }
    } 
    /*******************************************************************************************/
    else if (String(token) == "dump") { //Dump all available A/D samples command
        token = strtok(NULL, TOKEN_DELIM);
        if (token != NULL) {
          String restopic = String(token);
          mqtt_debug_print("Received command to dump A/D#0 samples buffer ");
          mqtt_debug_print(" into result topic:");
          mqtt_debug_println(restopic.c_str());
          sampler.ReadSamples(analog_samples_buff);
          for (int i=0; i<sampler.GetNumAvailableSamples(); i++) {
            mqtt_debug_print(String(analog_samples_buff[i]).c_str());
          }
          //client.publish((get_full_hostname() + String("/") + restopic).c_str(), String(digitalRead(gpio_num) & 0x01, HEX).c_str()); //TODO
          //dump last N samples from A/D circular buffer
        } else {
          mqtt_debug_println("Invalid restopic argumenr for A/D dump command");
        }
    } 
    /*******************************************************************************************/
    else if (String(token) == "list") {
       //get result topic
      token = strtok(NULL, TOKEN_DELIM);
      if (token != NULL) {
        String restopic = String(topic);
        mqtt_debug_println("Received command to list all existing GPIOs");
        client.publish((get_full_hostname() + String("/") + restopic).c_str(), gpio_list_payload().c_str());
      } else {
        mqtt_debug_println("Invalid result_topic to list all existing GPIOs command");
      }
    /*******************************************************************************************/
    } else if (String(token) == "irsendprot") {
      enum protocol prot;
      unsigned long command;
      unsigned int nbits;
      //parsing protocol
      token = strtok(NULL, TOKEN_DELIM);
      if (token != NULL) {
        String protocol_name = String(token);
        prot = (enum protocol)get_protocol_from_name(protocol_name);
        if (prot == PROT_UNDEFINED) {
          mqtt_debug_println("Undefined protocol");
          return;
        }
      } else {
        mqtt_debug_println("Expected a valid protocol name!");
        return;
      }
      //parsing command
      token = strtok(NULL, TOKEN_DELIM);
      if (token != NULL) {
        command = (unsigned long)strtoul(token, NULL, 16); //assuming hex number format 0xNNNNNNNN
      } else {
        mqtt_debug_println("Expected a valid command!");
        return;
      }
      //parsing nbits
      token = strtok(NULL, TOKEN_DELIM);
      if (token != NULL) {
        nbits = String(token).toInt();
      } else {
        mqtt_debug_println("Expected a valid nbits!");
        return;
      }
      //sending the IR command
      String debug_str = String("Sending IR command on protocol ") + protocol_names_lut[prot] + String(": 0x") + String(command, HEX);
      mqtt_debug_println(debug_str.c_str());
      ir_send_prot(prot, command, nbits);
    /*******************************************************************************************/
    } else if (String(token) == "irsendraw") {
      unsigned int nbits;
      unsigned int nitems;
      unsigned int freq;
      
      memset(irSignalBuf, 0, sizeof(irSignalBuf));
      //parsing nitems
      token = strtok(NULL, TOKEN_DELIM);
      if (token != NULL) {
        nitems = String(token).toInt();
        if ((nitems > 100) || (nitems < 0)) {
          mqtt_debug_println("Invalid nutems value");
          return;
        }
        for (int i=0; i<nitems; i++) {
          token = strtok(NULL, TOKEN_DELIM);
          if (token != NULL) {
            irSignalBuf[i] = String(token).toInt();
          }
        }
      } else {
        mqtt_debug_println("Expected a valid npairs parameter");
        return;
      }
      //parsing freq kHz
      token = strtok(NULL, TOKEN_DELIM);
      if (token != NULL) {
        freq = String(token).toInt();
      } else {
        mqtt_debug_println("Expected a valid frequency parameter");
        return;
      }
      //sending the IR command
      irsend.sendRaw(irSignalBuf, nitems, freq);
    /*******************************************************************************************/
    } else  {
      String debug_str = String("unknown command: ") + String((char*)payload);
      mqtt_debug_println(debug_str.c_str());
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

  irsend.begin();

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

//  A2DSampler a2d_sampler(10);
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


