


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
#include <EEPROM.h>
#include <WiFiClient.h>



extern "C" {
#include "user_interface.h"
}

#include <IRremoteESP8266.h>
#include <IRsend.h>  // Needed if you want to send IR commands.


#define HOSTNAME "ESP12E-" ///< Hostename. The setup function adds the Chip ID at the end
#define MAX_TIME_TO_TRY_BEFORE_REVERTING_TO_WEB_PORTAL_MS 300000

String get_full_hostname() {
  String my_hostname(HOSTNAME);
  my_hostname += String(ESP.getChipId(), HEX);
  return my_hostname;
}

// Update these with values suitable for your network.

//const char* ssid = "gmontag-room";
//const char* password = "gm3351324";
const char* mqtt_server = "raspberrypi-desktop";


WiFiClient espClient;
PubSubClient client(espClient);

class EepromData {

  public:
    enum { eeprom_magic=0x07121960   };
    enum { ApListSize=3, SsidStrLen=33, PassStrLen=65, MqttHostNameStrLen=33 };
    enum eCredIndex { eCred0, eCred1, eCred2 };
    struct ApCred {
      char ssid[SsidStrLen];
      char password[PassStrLen];
    };
  private:
    struct {
      uint32_t magic;
      ApCred ap_list[ApListSize];
      char mqtt_host_name[MqttHostNameStrLen];
      uint8_t mqtt_ip[4];
    } m_data;
    bool m_cached;
    
  public:
    EepromData() {
      m_cached = false;
      memset((void*)&m_data, 0, sizeof(m_data));
    }
    ~EepromData() {}
  
    void Begin() {
      EEPROM.begin(512);
      //Clear();//TODO - Remove!!!
      int address = 0;
#if 0      
      //read magic
      m_data.magic |= (uint32_t)EEPROM.read(address++) << 24;
      m_data.magic |= (uint32_t)EEPROM.read(address++) << 16;
      m_data.magic |= (uint32_t)EEPROM.read(address++) << 8;
      m_data.magic |= (uint32_t)EEPROM.read(address++) << 0;
      Serial.println("");
      Serial.print("Eeprom magic = 0x");
      Serial.println(m_data.magic, HEX);
      for (int i=0; i < ApListSize; i++) {
        //read ssid
        for (int j=0; j<SsidStrLen; j++) {
          m_data.ap_list[i].ssid[j] = char(EEPROM.read(address++));
        }
        Serial.print("Eeprom ssid: ");
        Serial.print(i);
        Serial.print("  = ");
        Serial.println(m_data.ap_list[i].ssid);
        //read password
        for (int j=0; j<PassStrLen; j++) {
          m_data.ap_list[i].password[j] = char(EEPROM.read(address++));
        }
        Serial.print("Eeprom password: ");
        Serial.print(i);
        Serial.print("  = ");
        Serial.println(m_data.ap_list[i].password);
      }
      for (int i=0; i<MqttHostNameStrLen; i++) {
        m_data.mqtt_host_name[i] = char(EEPROM.read(address++));
      }
      Serial.print("Eeprom mqtt: ");
      Serial.println(m_data.mqtt_host_name);
#endif
      uint8_t *m_data_p = (uint8_t *)&m_data;
      for (int i=0; i<sizeof(m_data); i++) {
        m_data_p[i] = EEPROM.read(address++);
      }
      Serial.println("");
      Serial.print("Eeprom magic = 0x");
      Serial.println(m_data.magic, HEX);
      for (int i=0; i < ApListSize; i++) {
        //read ssid
        Serial.print("Eeprom ssid: ");
        Serial.print(i);
        Serial.print("  = ");
        Serial.println(m_data.ap_list[i].ssid);
        //read password
        Serial.print("Eeprom password: ");
        Serial.print(i);
        Serial.print("  = ");
        Serial.println(m_data.ap_list[i].password);
      }
      Serial.print("Eeprom mqtt: ");
      Serial.print(m_data.mqtt_ip[0]);Serial.print(".");
      Serial.print(m_data.mqtt_ip[1]);Serial.print(".");
      Serial.print(m_data.mqtt_ip[2]);Serial.print(".");      
      Serial.println(m_data.mqtt_ip[3]);
      m_cached = true;
    }
    //for testing only
    void Clear() {
      for (int i=0; i<512; i++) {
        EEPROM.write(i, 0);
      }
      EEPROM.commit();
    }
    bool Valid() {
//      return false;
      if (!m_cached)
        return false;
      return m_data.magic == eeprom_magic;
    }
    bool GetPasswordForSsid( String ssid, String& password) {
      if (!Valid())
        return false;
      for (int i=0; i<ApListSize; i++) {
        if (String(m_data.ap_list[i].ssid) == ssid) {
          password = String(m_data.ap_list[i].password);
          return true;
        }
      }
      return false;
    }
    bool GetApCred(eCredIndex num, ApCred& cred) {
      if (!Valid())
        return false;
      strcpy(cred.ssid, m_data.ap_list[num].ssid);
      strcpy(cred.password, m_data.ap_list[num].password);
      return true;
    }
    bool SetApCred(eCredIndex num, ApCred& cred) {
      strcpy(m_data.ap_list[num].ssid, cred.ssid);
      strcpy(m_data.ap_list[num].password, cred.password);
      return true;
    }
  
    bool GetMqttHostName(String& mqtt_host_name) {
      if (!Valid())
        return false;
      mqtt_host_name = String(m_data.mqtt_host_name);
      return true;
    }
    bool SetMqttHostName(String& mqtt_host_name) {
      strcpy(m_data.mqtt_host_name, mqtt_host_name.c_str());
      return true;
    }
    bool SetMqttIpByte(const int byte_num, String& data) {
      if ((byte_num >=0) && (byte_num<=3))
        m_data.mqtt_ip[byte_num] = data.toInt();
        return true;
      return false;
    }
    bool GetMqttIpBytes(uint8_t bytes[]) {
      if (!Valid())
        return false;
      memcpy(bytes, m_data.mqtt_ip, 4);
        return true;
    }

    void Commit() {
      int address = 0;
      m_data.magic = eeprom_magic;
#if 0      
      //write magic
      EEPROM.write(address++, (byte)((eeprom_magic >> 24) & 0xff));
      EEPROM.write(address++, (byte)((eeprom_magic >> 16) & 0xff));
      EEPROM.write(address++, (byte)((eeprom_magic >> 8) & 0xff));
      EEPROM.write(address++, (byte)((eeprom_magic >> 0) & 0xff));
      for (int i=0; i < ApListSize; i++) {
        //write ssid
        Serial.println("commiting ssid");
        for (int j=0; j<SsidStrLen; j++) {
          EEPROM.write(address++, m_data.ap_list[i].ssid[j]);
          Serial.print(m_data.ap_list[i].ssid[j]);
        }
        Serial.println("");
        Serial.println("commiting pass");
        //write password
        for (int j=0; j<PassStrLen; j++) {
          EEPROM.write(address++, m_data.ap_list[i].password[j]);
          Serial.print(m_data.ap_list[i].password[j]);
        }
      }
      Serial.println("");
      Serial.println("commiting mqtt");
      for (int i=0; i<MqttHostNameStrLen; i++) {
        EEPROM.write(address++, m_data.mqtt_host_name[i]);
        Serial.print(m_data.mqtt_host_name[i]);
      }
      Serial.println("");
#endif
      uint8_t *m_data_p = (uint8_t *)&m_data;
      for (int i = 0; i < sizeof(m_data); i++) {
        EEPROM.write(address++, (byte)(*m_data_p));
        m_data_p++;
      }
      EEPROM.commit();
    }
};

EepromData eeprom_data;


/************************************************Taken from Example*****************************************/
MDNSResponder mdns;
WiFiServer server(80);


String st;

void launchWeb(int webtype) {
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println(WiFi.localIP());
  Serial.println(WiFi.softAPIP());
  if (!MDNS.begin("esp8266")) {
    Serial.println("Error setting up MDNS responder!");
    while(1) { 
      delay(1000);
    }
  }
  Serial.println("mDNS responder started");
  // Start the server
  server.begin();
  Serial.println("Server started");   
  int b = 20;
  int c = 0;
  while(b == 20) { 
     b = mdns1(webtype);
     //handle OTA upgrade
     ArduinoOTA.handle();
  }
}

void setupAP(void) {
  
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
  int n = WiFi.scanNetworks();
  Serial.println("scan done");
  if (n == 0)
    Serial.println("no networks found");
  else
  {
    Serial.print(n);
    Serial.println(" networks found");
    for (int i = 0; i < n; ++i)
     {
      // Print SSID and RSSI for each network found
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.print(WiFi.SSID(i));
      Serial.print(" (");
      Serial.print(WiFi.RSSI(i));
      Serial.print(")");
      Serial.println((WiFi.encryptionType(i) == ENC_TYPE_NONE)?" ":"*");
      delay(10);
     }
  }
  Serial.println(""); 
  st = "<ul>";
  for (int i = 0; i < n; ++i)
    {
      // Print SSID and RSSI for each network found
      st += "<li>";
      st +=i + 1;
      st += ": ";
      st += WiFi.SSID(i);
      st += " (";
      st += WiFi.RSSI(i);
      st += ")";
      st += (WiFi.encryptionType(i) == ENC_TYPE_NONE)?" ":"*";
      st += "</li>";
    }
  st += "</ul>";
  delay(100);
  WiFi.mode(WIFI_OFF);
  delay(100);
  WiFi.mode(WIFI_AP);
  const char* ssid = "ESP12E-AP";//get_full_hostname().c_str();
  Serial.print("Opening AP with ssid =");
  Serial.println(ssid);
  WiFi.softAP(ssid);
  Serial.println("softap");
  Serial.println("");
  launchWeb(1);
  Serial.println("over");
}

int mdns1(int webtype)
{
  // Check for any mDNS queries and send responses
  MDNS.update();
  
  
  // Check if a client has connected
  WiFiClient client = server.available();
  if (!client) {
    return(20);
  }
  Serial.println("");
  Serial.println("New client");

  // Wait for data from client to become available
  while(client.connected() && !client.available()){
    delay(1);
   }
  
  // Read the first line of HTTP request
  String req = client.readStringUntil('\r');
  
  // First line of HTTP request looks like "GET /path HTTP/1.1"
  // Retrieve the "/path" part by finding the spaces
  int addr_start = req.indexOf(' ');
  int addr_end = req.indexOf(' ', addr_start + 1);
  if (addr_start == -1 || addr_end == -1) {
    Serial.print("Invalid request: ");
    Serial.println(req);
    return(20);
   }
  req = req.substring(addr_start + 1, addr_end);
  Serial.print("Request: ");
  Serial.println(req);
  client.flush(); 
  String s;
  if ( webtype == 1 ) {
      if (req == "/")
      {
        IPAddress ip = WiFi.softAPIP();
        String ipStr = String(ip[0]) + '.' + String(ip[1]) + '.' + String(ip[2]) + '.' + String(ip[3]);
        s = "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n<!DOCTYPE HTML>\r\n<html>Hello from ESP8266 at ";
        s += ipStr;
        s += "<p>";
        s += st;
//        s += "<form method='get' action='a'><label>SSID: </label><input name='ssid' length=32><input name='pass' length=64><input type='submit'></form>";
        s += "<form method='get' action='a'>";
        s += "<TABLE BORDER='1'>";
        s += "<TR>";
        s += "<TD><label>SSID: </label><input name='ssid1' length=32><label>PASS: </label><input name='pass1' length=64></TD>";
        s += "</TR>";
        s += "<TR>";
        s += "<TD><label>SSID: </label><input name='ssid2' length=32><label>PASS: </label><input name='pass2' length=64></TD>";
        s += "</TR>";
        s += "<TR>";
        s += "<TD><label>SSID: </label><input name='ssid3' length=32><label>PASS: </label><input name='pass3' length=64></TD>";
        s += "<TR>";
        s += "<TD><label>MQTTSERVER: </label><input name='mqtt_ip0' length=3><input name='mqtt_ip1' length=3><input name='mqtt_ip2' length=3><input name='mqtt_ip3' length=3></TD>";
        s += "</TR>";
        s += "</TABLE>";
        s += "<input type='submit'></form>";
        s += "</html>\r\n\r\n";
        Serial.println("Sending 200");
      }
      else if ( req.startsWith("/a?ssid1=") ) {
        // /a?ssid1=blahhhh&pass1=poooo&ssid2=blahhh&pass2=pooo&ssid3=blahhh&pass3=pooo&mqtt_ip0=aaa&mqtt_ip1=bbb&mqtt_ip2=ccc&mqtt_ip3=ddd
//        Serial.println("clearing eeprom");
//        for (int i = 0; i < 96; ++i) { EEPROM.write(i, 0); }
        String qsid1 = req.substring(9,req.indexOf('&'));
        Serial.println(qsid1);
        Serial.println("");
        String qpass1 = req.substring(req.indexOf("pass1")+6, req.indexOf("ssid2")-1);
        Serial.println(qpass1);
        Serial.println("");
        String qsid2 = req.substring(req.indexOf("ssid2")+6, req.indexOf("pass2")-1);
        Serial.println(qsid2);
        Serial.println("");
        String qpass2 = req.substring(req.indexOf("pass2")+6, req.indexOf("ssid3")-1);
        Serial.println(qpass2);
        Serial.println("");
        String qsid3 = req.substring(req.indexOf("ssid3")+6, req.indexOf("pass3")-1);
        Serial.println(qsid3);
        Serial.println("");
        String qpass3 = req.substring(req.indexOf("pass3")+6, req.indexOf("mqtt_ip0")-1);
        Serial.println(qpass3);
        Serial.println("");
        String qmqtt_ip0 = req.substring(req.indexOf("mqtt_ip0")+9, req.indexOf("mqtt_ip1")-1);
        String qmqtt_ip1 = req.substring(req.indexOf("mqtt_ip1")+9, req.indexOf("mqtt_ip2")-1);
        String qmqtt_ip2 = req.substring(req.indexOf("mqtt_ip2")+9, req.indexOf("mqtt_ip3")-1);
        String qmqtt_ip3 = req.substring(req.indexOf("mqtt_ip3")+9, req.length());
        Serial.print(qmqtt_ip0);Serial.print(".");Serial.print(qmqtt_ip1);Serial.print(".");Serial.print(qmqtt_ip2);Serial.print(".");Serial.println(qmqtt_ip3);

        
        Serial.println("writing eeprom...");
        EepromData::ApCred cred;
        
        strcpy(cred.ssid, qsid1.c_str());
        strcpy(cred.password, qpass1.c_str());
        eeprom_data.SetApCred(EepromData::eCred0, cred);

        strcpy(cred.ssid, qsid2.c_str());
        strcpy(cred.password, qpass2.c_str());
        eeprom_data.SetApCred(EepromData::eCred1, cred);
        
        strcpy(cred.ssid, qsid3.c_str());
        strcpy(cred.password, qpass3.c_str());
        eeprom_data.SetApCred(EepromData::eCred2, cred);

        //eeprom_data.SetMqttHostName(qmqtt_server);
        eeprom_data.SetMqttIpByte(0, qmqtt_ip0);
        eeprom_data.SetMqttIpByte(1, qmqtt_ip1);
        eeprom_data.SetMqttIpByte(2, qmqtt_ip2);
        eeprom_data.SetMqttIpByte(3, qmqtt_ip3);

        eeprom_data.Commit();
        s = "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n<!DOCTYPE HTML>\r\n<html>Hello from ESP8266 ";
        s += "Found ";
        s += req;
        s += "<p> saved to eeprom... reset to boot into new wifi</html>\r\n\r\n";
      }
      else
      {
        s = "HTTP/1.1 404 Not Found\r\n\r\n";
        Serial.println("Sending 404");
      }
  } 
  else
  {
      if (req == "/")
      {
        s = "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n<!DOCTYPE HTML>\r\n<html>Hello from ESP8266";
        s += "<p>";
        s += "</html>\r\n\r\n";
        Serial.println("Sending 200");
      }
      else if ( req.startsWith("/cleareeprom") ) {
        s = "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n<!DOCTYPE HTML>\r\n<html>Hello from ESP8266";
        s += "<p>Clearing the EEPROM<p>";
        s += "</html>\r\n\r\n";
        Serial.println("Sending 200");  
        Serial.println("clearing eeprom");
        for (int i = 0; i < 96; ++i) { EEPROM.write(i, 0); }
        EEPROM.commit();
      }
      else
      {
        s = "HTTP/1.1 404 Not Found\r\n\r\n";
        Serial.println("Sending 404");
      }       
  }
  client.print(s);
  Serial.println("Done with client - rebooting...");
  ESP.restart();
  //return(20);
}
 
 /************************************************Till Here**************************************************/




long lastMsg = 0;
char msg[50];
int value = 0;



String debugtopic("");


class WiFiScanner {
  struct {
    String ssid;
    int rssi;
    bool excluded;
  } m_scanResults[10];
  int m_numNetworksFound;
    
  public:
    WiFiScanner() {
      WiFi.mode(WIFI_STA);
      WiFi.disconnect();
      delay(100);
      m_numNetworksFound = 0;
      for (int i=0; i<10; i++) {
        m_scanResults[i].ssid = "";
        m_scanResults[i].rssi = -32767;
        m_scanResults[i].excluded = false;
      }
    };
    ~WiFiScanner() {};

    int ScanNetworks() {
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
              m_scanResults[i].ssid = WiFi.SSID(i); 
              m_scanResults[i].excluded = false;
            }
            return m_numNetworksFound;
        }
    }

    void ExcludeNetworkSsid(String to_exclude) {
      for (int i=0; i<m_numNetworksFound; i++) {
        if (m_scanResults[i].ssid == to_exclude) {
          m_scanResults[i].excluded = true;
          break;
        }
      }
    }

    String GetBestNetworkSsid() {
      int maxRssi = -32767;
      int maxIndex = 0;
      for (int i=0; i<m_numNetworksFound; i++) {
        if ((m_scanResults[i].rssi >= maxRssi) && (m_scanResults[i].excluded == false)) {
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

bool setup_wifi() {
  String best_ssid, best_password;
  const char* password; // = "gm3351324"; //TODO - remove!!!
  int numNetworks;
  EepromData::ApCred cred;
  int wifi_connection_wait;

  WiFiScanner wifi_scanner;

  // Set Wifi Hostname.
  String hostname(HOSTNAME);
  hostname += String(ESP.getChipId(), HEX);
  WiFi.hostname(hostname);

  // Print Wifi hostname.
  Serial.println("Hostname: " + WiFi.hostname());


  numNetworks = wifi_scanner.ScanNetworks();
  if (numNetworks == 0) {
    Serial.println("No WiFi Networks found - waiting...!!");
    return false;
  }
  Serial.print("Found ");
  Serial.print(numNetworks);
  Serial.println(" Networks");
  
#define WIFI_CONNECTION_WAIT_MAX 100
  
  for (int i=0; i<numNetworks; i++) {
    best_ssid = wifi_scanner.GetBestNetworkSsid();
    Serial.print("Best SSID is: ");
    Serial.println(best_ssid);
    //find password for best AP
    if (!eeprom_data.GetPasswordForSsid(best_ssid, best_password)) {
      Serial.println("Didn't find best AP in EEPROM - trying next best");
      wifi_scanner.ExcludeNetworkSsid(best_ssid);
      continue;
    }
    Serial.print("Found best AP in EEPROM, trying to associate using ssid = ");
    Serial.print(best_ssid.c_str());
    Serial.print(" password = ");
    Serial.println(best_password.c_str());
    WiFi.mode(WIFI_STA);
    WiFi.begin(best_ssid.c_str(), best_password.c_str());
    wifi_connection_wait = 0;
    while ((WiFi.status() != WL_CONNECTED) && (wifi_connection_wait < WIFI_CONNECTION_WAIT_MAX)) {
      delay(500);
      Serial.print(".");
      wifi_connection_wait++;
    }
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("Failed association to this network, trying next best...");
      wifi_scanner.ExcludeNetworkSsid(best_ssid);
      continue;
    }
    //success
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
    return true;
  }//for i
  //if we got here, we failed
  return false;
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
uint16_t irSignalBuf[IRSIGNAL_MAX_LEN]; //raw buffer for IR irsend.sendRaw()
uint16_t irSignalBufLen;

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
void invalidate_eeprom_and_reboot() {
  eeprom_data.Clear(); //invalidating EEPROM
  ESP.restart();
}

void reconnect() {

  int timeout_ms = 0;

  // Loop until we're reconnected
  active_led.SetBlinkRate(Blinker::BLINK_RATE_FAST);
  while (!client.connected()) {
    // Wait 5 seconds before retrying
    delay(5000);
    timeout_ms += 5000;
    Serial.print("time = ");
    Serial.print(timeout_ms/1000, DEC);
    Serial.print(" out of ");
    Serial.print(MAX_TIME_TO_TRY_BEFORE_REVERTING_TO_WEB_PORTAL_MS/1000, DEC);
    Serial.println(" seconds before reverting to portal...");
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
      Serial.println(client.state());
      if (timeout_ms >= MAX_TIME_TO_TRY_BEFORE_REVERTING_TO_WEB_PORTAL_MS) {
        Serial.println("Timeout expired - rebooting into web portal...");
        invalidate_eeprom_and_reboot();
      }
    }
  }
  active_led.SetBlinkRate(Blinker::BLINK_RATE_ON);
}


void setup() {

  Serial.begin(115200);
  irsend.begin();
  eeprom_data.Begin();
  
  active_led.SetBlinkRate(Blinker::BLINK_RATE_OFF);

  Serial.println("\r\n");
  Serial.print("Chip ID: 0x");
  Serial.println(ESP.getChipId(), HEX);
  Serial.println(eeprom_data.Valid() ? "EEPROM is valid" : "EEPROM is not valid");
  if (!eeprom_data.Valid()) {
    Serial.println("EEPROM data is invalid - going to start a web server/portal on WiFi: ESP12E-AP IP: 192.168.4.1 to configure WiFi/MQTT credentials");
    setupAP();
  }

  // Start OTA server.
  String hostname(HOSTNAME);
  hostname += String(ESP.getChipId(), HEX);
  ArduinoOTA.setHostname((const char *)hostname.c_str());
  ArduinoOTA.begin();

  pinMode(2, OUTPUT);     // Initialize the BUILTIN_LED pin as an output

  client.setCallback(callback);
  uint8_t mqtt_ip[4];
  if (eeprom_data.GetMqttIpBytes(mqtt_ip)) {
      Serial.print("Setting MQTT Server address: ");
      Serial.print(mqtt_ip[0]);Serial.print(".");
      Serial.print(mqtt_ip[1]);Serial.print(".");
      Serial.print(mqtt_ip[2]);Serial.print(".");
      Serial.println(mqtt_ip[3]);
      client.setServer(IPAddress(mqtt_ip[0], mqtt_ip[1], mqtt_ip[2], mqtt_ip[3])/*mqtt_host_name.c_str()*/, 1883);
//    client.setServer(mqtt_host_name.c_str(), 1883);
  } else {
    Serial.println("EEPROM data is invalid - going to reboot into a web server to configure WiFi/MQTT credentials");
    setupAP();
  }

  active_led.SetBlinkRate(Blinker::BLINK_RATE_SLOW);
  if (!setup_wifi()) {
    Serial.println("Failed setup_wifi() - going to open a web server to configure WiFi credentials");
    setupAP();
  }
  active_led.SetBlinkRate(Blinker::BLINK_RATE_FAST);

//  A2DSampler a2d_sampler(10);
}

void loop() {
  long now;
  static long wifi_good = 0;
  long wifi_bad;

  if (WiFi.status() != WL_CONNECTED) {
    active_led.SetRate(Blinker::BLINK_RATE_SLOW);
    if (!setup_wifi()) {
      wifi_bad = millis();
      if (wifi_bad - wifi_good >= MAX_TIME_TO_TRY_BEFORE_REVERTING_TO_WEB_PORTAL_MS) {
        Serial.println("Timeout trying to setup_wifi()based on current eeprom values - Invalidating eeprom and rebooting into web portal to re-configure WiFi");
        invalidate_eeprom_and_reboot();
      }
    }
  } else {
    wifi_good = millis();
    active_led.SetRate(Blinker::BLINK_RATE_FAST);
  }

  if (!client.connected()) {
    reconnect();
  } 

  client.loop();
  //run the led blink timer
/*
  long now = millis();
  static long last;
  if (now - last > 2000) {
    last = now;
    int val = EEPROM.read(0);
    EEPROM.write(0, val+1);
    EEPROM.commit();
    Serial.print("EEPROM value = ");
    Serial.println(val);
  }
*/
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


