


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

//const char* ssid = "gmontag-room";
//const char* password = "gm3351324";
const char* mqtt_server = "raspberrypi-desktop";


WiFiClient espClient;
PubSubClient client(espClient);


long lastMsg = 0;
char msg[50];
int value = 0;


String get_full_hostname() {
  String my_hostname(HOSTNAME);
  my_hostname += String(ESP.getChipId(), HEX);
  return my_hostname;
}

String debugtopic("");

class WiFiScanner {
  struct {
    char ssid[50];
    int rssi;
  } m_scanResults[10];
  int m_numNetworksFound;
    
  public:
    WiFiScanner() {
      WiFi.mode(WIFI_STA);
      WiFi.disconnect();
      delay(100);
      m_numNetworksFound = 0;
      memset(m_scanResults, 0, sizeof(m_scanResults));
    };
    ~WiFiScanner() {};

    int ScanNetworks() {
        memset(m_scanResults, 0, sizeof(m_scanResults));
        m_numNetworksFound = WiFi.scanNetworks();
        Serial.println("scan done");
        if (m_numNetworksFound == 0) {
          Serial.println("no networks found");
          return 0;
        }  else {
          Serial.print(m_numNetworksFound);
          Serial.println(" networks found");
          for (int i = 0; i < m_numNetworksFound; ++i)
          {
            // SSID and RSSI for each network found
            m_scanResults[i].rssi = WiFi.RSSI(i); 
            strcpy(m_scanResults[i].ssid, WiFi.SSID(i).c_str()); 
          }
          return m_numNetworksFound;
        }
    }

    char* GetBestNetworkSsid() {
      int maxRssi = -32767;
      int maxIndex = 0;
      for (int i=0; i<m_numNetworksFound; i++) {
        if (m_scanResults[i].rssi >= maxRssi) {
          maxRssi = m_scanResults[i].rssi;
          maxIndex = i;
        }
      }
      return m_scanResults[maxIndex].ssid;
    }
};

class CyclicBuff {
#define MAX_BUFF_SIZE 1000 //10 seconds of 10ms samples
    int m_buff[MAX_BUFF_SIZE]; 
    int m_num_items;
    int m_write_pointer;
    int m_read_pointer;
  public:
    CyclicBuff() : m_num_items(0), m_write_pointer(0), m_read_pointer(0) {
      memset(m_buff, 0, sizeof(m_buff));
    }
    ~CyclicBuff() {};

    bool Full() {
      return m_num_items == MAX_BUFF_SIZE;
    }
    bool Empty() {
      return m_num_items == 0;
    }
    int Available() {
      return m_num_items;
    }
    bool Read(int& val) {
      noInterrupts();
      if (Empty()) {
        interrupts();
        return false;
      }
      val = m_buff[m_read_pointer];
      m_read_pointer = (m_read_pointer+1) % MAX_BUFF_SIZE;
      m_num_items--;
      interrupts();
      return true;
    }
    bool Write(int& val) {
      noInterrupts();
      if (Full()) {
        interrupts();
        return false;
      }
      m_buff[m_write_pointer] = val;
      m_write_pointer = (m_write_pointer+1) % MAX_BUFF_SIZE;
      m_num_items++;
      interrupts();
      return true;
    }
};

//instantiate the class
CyclicBuff samples_cyc_buff;

static void OSTimerBaseCB(void *pArg);

class OSTimerBase {
    os_timer_t m_timer;
    int m_rate_ms;
  public:
    OSTimerBase() 
      : m_rate_ms(0) {
      os_timer_disarm(&m_timer);
      os_timer_setfn(&m_timer, (os_timer_func_t *)OSTimerBaseCB, this);
    }
    virtual ~OSTimerBase() {
        os_timer_disarm(&m_timer);
    };
    void SetRate( int rate_ms ) {
      m_rate_ms = rate_ms;
    }
    bool Start() {
      if (m_rate_ms != 0) {
        os_timer_disarm(&m_timer);
        os_timer_arm(&m_timer, m_rate_ms,  1); //repetitive
        return true;
      } else {
          Serial.println("OSTimerBase::Start() Failed: m_rate_ms = 0!");
        return false;
      }
    }
    void Stop() {
        os_timer_disarm(&m_timer);
    }
    //The actual work is delegated to the derived classes
    virtual void DoTheWork() = 0;
};

static void OSTimerBaseCB( void *pArg) {
  ((OSTimerBase*)pArg)->DoTheWork();
};
 
static void SampleCB(void *pArg);

class ADCSampler : public OSTimerBase {
    CyclicBuff* m_buff; 
  public:
    ADCSampler(CyclicBuff* p_buff) {
      m_buff = p_buff;
      Stop();
    };
    ~ADCSampler() {
        Stop();
    };
    CyclicBuff* GetBuffPtr() {
      return m_buff;
    };
  
    bool StartSampling(int samp_rate_ms) {
      SetRate(samp_rate_ms);
      Start();
      return true;
    };
    void StopSampling() {
        Stop();
    };
   private:
    //override
    void DoTheWork() {
      int sample = analogRead(0);
      //modify the sample so it would never have \0 bytes in its payload
      //"\0" do not pass correctly in strings...
      //The client receiving the sample, needs to to the reverse of this operation to restore the sample value
      sample <<= 2;
      sample += 0xf003;
      m_buff->Write(sample); //ignoring fullness here
    }
};

//statics
//CyclicBuff* ADCSampler::m_buff = &samples_buff;
//os_timer_t ADCSampler::m_timer;
//int ADCSampler::m_samp_rate_ms = 1000;

//instantiate the class
ADCSampler sampler(&samples_cyc_buff);

class Blinker : public OSTimerBase {

  public:
    enum  BlinkRate { BLINK_RATE_OFF, BLINK_RATE_SLOW, BLINK_RATE_FAST, BLINK_RATE_ON };
    enum  LedState  { LED_STATE_OFF = 1, LED_STATE_ON = 0 };

  private:
    LedState m_led_state;
    enum Timeout { SLOW_RATE_TIMEOUT = 500, FAST_RATE_TIMEOUT = 100 };
    int m_led_gpio;
  public:
    Blinker(int gpio_num) {
      m_led_gpio = gpio_num;
      m_led_state = LED_STATE_OFF;
      pinMode(m_led_gpio, OUTPUT);
      digitalWrite(m_led_gpio, m_led_state);
    };
    ~Blinker() {
      m_led_state = LED_STATE_OFF;
      digitalWrite(m_led_gpio, m_led_state);
    };
    void SetBlinkRate(BlinkRate r) {
      switch (r) {
        case BLINK_RATE_OFF:
          Stop();
          m_led_state = LED_STATE_OFF;
          digitalWrite(m_led_gpio, m_led_state);
          break;
        case BLINK_RATE_SLOW:
          Stop();
          SetRate(SLOW_RATE_TIMEOUT);
          Start();
          break;
        case BLINK_RATE_FAST:
            Stop();
            SetRate(FAST_RATE_TIMEOUT);
            Start();
          break;
        case BLINK_RATE_ON:
          Stop();
          m_led_state = LED_STATE_ON;
          digitalWrite(m_led_gpio, m_led_state);
          break;
      }
    };
    //override
    void DoTheWork() {
      ToggleLed();
    }
  private:
     void ToggleLed() {
      if (m_led_state == LED_STATE_OFF) {
        m_led_state = LED_STATE_ON;
      } else {
        m_led_state = LED_STATE_OFF;
      }
      digitalWrite(m_led_gpio, m_led_state);
    };
};

//instantiate the class
Blinker active_led(2); //Indicates connection to Wifi and connection to MQTT status

String topic_cmd() {
  return get_full_hostname() + String("/cmd");
}

void setup_wifi() {
  char* ssid;
  const char* password = "gm3351324";
  int numNetworks;

  WiFiScanner wifi_scanner;

  numNetworks = wifi_scanner.ScanNetworks();
  Serial.print("Found ");
  Serial.print(numNetworks);
  Serial.println(" Networks");
  ssid = wifi_scanner.GetBestNetworkSsid();
  Serial.print("Best network is: ");
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
  
#if 0
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
#endif
  
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
};

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

#define IRSIGNAL_MAX_LEN 256
unsigned int irSignalBuf[IRSIGNAL_MAX_LEN]; //raw buffer for IR irsend.sendRaw()
unsigned int irSignalBufLen;

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
    if (String(token) == "debug-topic-set") {
      token = strtok(NULL, TOKEN_DELIM);
      Serial.println("token = " + String(token));
      if (token != NULL) {
        debugtopic = String(token);
        Serial.print("Received command to set debug-topic to: ");
        Serial.println(debugtopic);
      } else {
        Serial.println("Invalid debug-topic payload");
      }
    }
    /*******************************************************************************************/
    else if (String(token) == "debug-topic-clr") {
        debugtopic = String("");
        Serial.print("Received command to clear debug-topic");
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
    else if (String(token) == "startA2D") { //Start A/D sampling command
        token = strtok(NULL, TOKEN_DELIM);
        if (token != NULL) {
          String restopic = String(token);
          mqtt_debug_print("Received command to Start A/D#0");
          mqtt_debug_print(" into result topic: ");
          mqtt_debug_println(restopic.c_str());
          sampler.StartSampling(10); //10 ms sampling rate
          } else {
            mqtt_debug_println("Invalid restopic argumenr for A/D startA2D command");
          }
    } 
    /*******************************************************************************************/
    else if (String(token) == "stopA2D") { //Stop A/D sampling command
        sampler.StopSampling(); 
    }
    /*******************************************************************************************/
    else if (String(token) == "irsendprot") {
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
    } else if (String(token) == "irrawbegin") {
        memset(irSignalBuf, 0, sizeof(irSignalBuf));
        irSignalBufLen = 0;
    /*******************************************************************************************/
    } else if (String(token) == "irrawpart") {
      while (irSignalBufLen < IRSIGNAL_MAX_LEN) {
        token = strtok(NULL, TOKEN_DELIM);
        if ((token == NULL) || (token == "")) {
          break;
        }
        irSignalBuf[irSignalBufLen++] = String(token).toInt();
      };
    /*******************************************************************************************/
    } else if (String(token) == "irrawend") {
//        for (int i=0; i<irSignalBufLen; i++) {
//          Serial.print(irSignalBuf[i]);
//          Serial.print(" ");    
//        }
        irsend.sendRaw(irSignalBuf, irSignalBufLen, 38);
    /*******************************************************************************************/
    } else  {
      String debug_str = String("unknown command: ") + String((char*)payload);
      mqtt_debug_println(debug_str.c_str());
    }
  }
}

void reconnect() {

  // Loop until we're reconnected
  active_led.SetBlinkRate(Blinker::BLINK_RATE_FAST);
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
      //And resubscribe to <hostname>/cmd to start receiving commands from clients
      client.subscribe(topic_cmd().c_str(), 1);//qos = 1
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
  active_led.SetBlinkRate(Blinker::BLINK_RATE_ON);
}


void setup() {

  irsend.begin();

  active_led.SetBlinkRate(Blinker::BLINK_RATE_OFF);

  Serial.println("\r\n");
  Serial.print("Chip ID: 0x");
  Serial.println(ESP.getChipId(), HEX);

  // Set Hostname.
  String hostname(HOSTNAME);
  hostname += String(ESP.getChipId(), HEX);
  WiFi.hostname(hostname);

  // Print hostname.
  Serial.println("Hostname: " + WiFi.hostname());
  //Serial.println(WiFi.hostname());

  // Start OTA server.
  // Start OTA server.
  ArduinoOTA.setHostname((const char *)hostname.c_str());
  ArduinoOTA.begin();

  pinMode(2, OUTPUT);     // Initialize the BUILTIN_LED pin as an output
  Serial.begin(115200);

  client.setCallback(callback);
  client.setServer(mqtt_server, 1883);

  active_led.SetBlinkRate(Blinker::BLINK_RATE_SLOW);
  setup_wifi();
  active_led.SetBlinkRate(Blinker::BLINK_RATE_FAST);

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

  String result_topic = get_full_hostname() + String("/") + String("adc");
  int payload_size = MQTT_MAX_PACKET_SIZE-5-2 - result_topic.length() - 2;
//  long now = millis();
//  if (now - lastMsg > 2000) {
//    lastMsg = now;
//    ++value;
//    snprintf (msg, 75, "hello from ESP #%ld", value);
//    Serial.print("Publish message: ");
//    Serial.println(msg);
//    if (!client.publish("outTopic", msg)) {
//      Serial.println("Publish failed");
//    }
//    if (samples_cyc_buff.Empty()) {
//      Serial.println("Nothing to publish");
//    }
//  }
    while (samples_cyc_buff.Available() >= payload_size/2) { //each sample takes two chars
      //publish a frame
      int bytes_to_publish = payload_size;
      Serial.println("Available = " + String(samples_cyc_buff.Available()));
      char pub_msg[payload_size+2];
      memset(pub_msg, 0, sizeof(pub_msg));
      int i = 0;
      while (bytes_to_publish > 0) { //some spare payload size to pad with '\0'
        int sample;
        if (samples_cyc_buff.Read(sample)) {
          pub_msg[i++] = sample & 0xff;
          pub_msg[i++] = (sample >> 8) & 0xff;
          bytes_to_publish -= 2;
        } else {
          break;//while
        }
      }//while bytes_to_publish
      Serial.println("publishing frame = " + String(pub_msg));
      Serial.println("Into topic:" + result_topic);
      if (!client.publish(result_topic.c_str(), pub_msg, payload_size)) {
        Serial.println("Publish failed");
      }
    }
  // Handle OTA server.
  ArduinoOTA.handle();

  yield();

}


