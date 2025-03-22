
#include "userdata.h"
#include <ArduinoOTA.h>
#include "PubSubClient.h"
#include <WiFi.h>
#include <esp_wifi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include "Wire.h"
#include <ArduinoJson.h>
#include "time.h"


// ****************** Uncomment to enable functions*****************
// #define debug_rtc
// #define DEBUG                 // uncomment for debugging
// #define MBUS_TEMP
// #define MBUS_BASE             // Base Definitions for MBUS
// #define MBUS_WIND             // Windspeed and Direction
// #define MBUS_OAQ              // Outdoor Particulate Sensor
// #define MBUS_WPSI             // water pressure
// #define MBUS_RAIN             // rain sensor
// #define MQTT_SEND_DEBUG_ON    // turn on MQTT send output
// #define MQTT_NODEFUNC_DEBUG
// #define PING_TANKS;
// #define RELAYS;
// ****************************************************************




#ifdef PING_TANKS
  #include <PingSerial.h>
  #include <SoftwareSerial.h>
  PingSerial tank1(Tank1EchoPin,Tank1TrigPin, 0, 1000);
  PingSerial tank2(Tank2EchoPin,Tank2TrigPin, 0, 1000);
#endif


#define ACT1 22               // Actuator pin (LED or relay to ground)  //D6 on Wemos unnecessary
#define MQCON 23              // MQTT link indicator D7 op Wemos unnecessary
#define BTN 19                 // Button pin (also used for Flash setting)  D3 op Wemos  unnecessary


#define SERIAL_BAUD 115200
#define HOLDOFF 1000          // blocking period between button messages

#ifdef RELAYS
  #define REL1 32               // DID16 fan direction
  #define REL2 33               // DID17
  #define REL3 25               // DID18
  #define REL4 26               // DID19
#endif

long  TXinterval = 0;        // periodic transmission interval in seconds
long  TIMinterval = 0;        // timer interval in seconds

#ifdef BME680
  #include "bsec.h"
  bool bmp680_not_present = false;
  long  BMEinterval = 0;
#endif

// long  FlowStatus = 0;
long  mpu_sample_rate = 25; //ms
long  timer2_sample_rate = 2.273; // 2.273
long  lastPeriod_timer2 = -1;
long  lastPeriod_mpu6050 = -1;


long  UptimeInterval = 120;  // Poll EnviroPMsensor every 30 minutes
bool  ackButton = false;      // flag for message on button press
bool  toggleOnButton = true;  // toggle output on button press
bool  fallBackSsi = false;    // toggle access point
unsigned long then = 0;

#ifdef PING_TANKS
  int tank1_int = 300000; //30 min
  int tank2_int = 300000; //30 min
  unsigned long intervals[3] = { 0, tank1_int,  tank2_int };
  int current_interval_index = 0;
  long  lastPeriod_tank = -1;
  #include <Coloria.h>    
#endif


//  VARIABLES
int   DID;                  // Device ID
int   error;                // Syntax error code
long  lastPeriod = -1;      // timestamp last transmission
long  lastPeriod_bme = -1;



long  lastBtnPress = -1;    // timestamp last buttonpress
long  lastMinute = -1;      // timestamp last minute
long  upTime = 0;           // uptime in minutes
int   ACT1State;            // status ACT1 output

#ifdef RELAYS
  bool  REL1State;
  bool  REL2State;
  bool  REL3State;
  bool  REL4State;

  bool  prevREL1State;
  bool  prevREL2State;
  bool  prevREL3State;
  bool  prevREL4State;
#endif

bool  mqttNotCon = true;      // MQTT broker not connected flag
int   signalStrength;         // radio signal strength
bool  wakeUp = true;          // wakeup indicator
bool  setAck = true;          // acknowledge receipt of actions
bool  curState = true;        // current button state
bool  lastState = true;       // last button state
bool  timerOnButton = false;  // timer output on button press
bool  msgBlock = false;       // flag to hold button message
bool  readAction;             // indicates read / set a vaendiflue
bool  send0, send1, send2, send3, send5, send6, send7, send8, send9;
bool  send10, send11, send12, send13, send16, send17, send18, send19, send40, send41, send42, send46;
bool  send48, send49, send50, send51, send52, send53, send54, send55, send59, send60, send61;
bool  send99, send20, send21, send22, send23, send24, send25, send26, send27, send56, send57;  // message triggers

String  IP;                   // IPaddress of ESP
char  buff_topic[30];         // mqtt topic
char  buff_msg[32];           // mqtt message
char  clientName[13];         // Mqtt client name
char  wifi_ssid[20];          // Wifi SSID name
char  wifi_password[20];      // Wifi password
bool reported[15] = {false, false, false, false, false, false, false, false , false, false, false, false, false, false, false};


#ifdef MBUS_BASE
  #include <ModbusMaster.h>
  int errorcnt =0;
  int cycle =0;
  //uint8_t j, result;

  /*!
    We're using a MAX485-compatible RS485 Transceiver.
    Rx/Tx is hooked up to the hardware serial port at 'Serial'.
    The Data Enable and Receiver Enable pins are hooked up as follows:
  */
  #define MAX485_DE      5
  #define MAX485_RE_NEG  4
  // instantiate ModbusMaster object
  ModbusMaster node;
#endif



#ifdef MBUS_WIND
  uint8_t Node_1 = 1;            //Windspeed
  uint8_t Node_2 = 2;           // Winddir
  float node1_data[]{-1,-1,-1,-1,-1,-1,-1,-1};
  float node2_data[]{-1,-1,-1,-1,-1,-1,-1,-1};
  float cur_wind_speed = 0;
  float cur_wind_dir = 0;
  float cur_wind_gear = 0;
  float max_wind_speed = 0;
  float max_wind_dir = 0;
  long  lastPeriod_node1_rtu_wind_speed = -1;
  long  node1_rtu_wind_speed_interval = 30;    // in seconds  
#endif

#ifdef MBUS_OAQ
  uint8_t Node_3 = 3;           // EnviroPMSensor
  float node3_data[]{-1,-1,-1,-1,-1,-1,-1,-1};
  float pm2510ug_output_pm25;
  float pm2510ug_output_pm10;
#endif

#ifdef MBUS_WPSI    // Water Pressure Sensor

  uint8_t Node_4 = 4;           
  int16_t node4_data[]{-1,-1,-1,-1,-1,-1,-1,-1};
  uint8_t wpsi_addr;
  uint8_t wpsi_baud;
  uint8_t wpsi_unit;
  uint8_t wpsi_dp;
  double wpsi_output_mpa;
  double wpsi_output_psi;  
  int16_t wpsi_minRange;
  int16_t wpsi_maxRange;
  long  node4_rtu_wpsi_interval = 60;
  long  lastPeriod_node4_rtu_wpsi = -1;

  // Func 06 addr 00 0-255 change address
  // Func 06 addr 01 0-7   change baud


#endif

#ifdef MBUS_RAIN
  uint16_t DayRain;
  uint16_t InstRain;
  uint16_t YesterdayRain;
  uint16_t SensorTotalRain;
  uint16_t HourlyRainfall;
  uint16_t LastHourRainfall;  
  uint16_t Max24Rain;
  uint8_t Max24Rain_TimeStart;
  uint8_t Max24Rain_TimeEnd;
  uint16_t Min24Rain;
  uint8_t Min24Rain_TimeStart;
  uint8_t Min24Rain_TimeEnd;


  uint8_t RainClockYear;
  uint8_t RainClockMonth;
  uint8_t RainClockDay;
  uint8_t RainClockHour;
  uint8_t RainClockMin;
  uint8_t RainClockSecond;

  uint8_t Node_5 = 5;           // Rain Sensor

  uint16_t node5_data[]{-1,-1,-1,-1,-1,-1,-1,-1};
  uint16_t node5_time[]{-1,-1,-1,-1,-1,-1,-1,-1};
#endif

unsigned long epochTime; 
#define countof(a) (sizeof(a) / sizeof(a[0]))

#ifdef rtc_ds3231
  #include <RtcDS3231.h>
  RtcDS3231<TwoWire> Rtc(Wire);
#endif

#include "rtc.h"

#ifdef I2C_MUX
  #include "TCA9548.h"
  PCA9545 MP(0x73);
  uint8_t channels = 0;
#endif

#ifdef rtc_ds3231
  void RTC_Update(){
    RtcDateTime now = Rtc.GetDateTime();//rtc
    #ifdef debug_rtc
      Serial.println("************************************");
      Serial.println("RTC time before update in RTC_Update");
      printDateTime(now);
      Serial.println("NTP time before update in RTC_Update");  
      printLocalTime();
      Serial.println();
    #endif      
    RtcDateTime timeToSet;
    timeToSet.InitWithEpoch32Time(getTime());
    #ifdef debug_rtc
      Serial.println("timeToSet value before update in RTC_Update");
    #endif
    Rtc.SetDateTime(timeToSet);
    #ifdef debug_rtc
      printDateTime(timeToSet);
      Serial.println();
      Serial.println("RTC time after update in RTC_Update");
      now = Rtc.GetDateTime();
      printDateTime(now); 
      Serial.println("Set RTC to NTP complete in RTC_Update");
      Serial.println("*************************************");
    #endif   
  }

  bool RTC_Valid(){
    bool boolCheck = true; 
    if (!Rtc.IsDateTimeValid()){
      Serial.println("RTC lost confidence in the DateTime!  Updating DateTime");
      boolCheck = false;
      RTC_Update();    
    }
    if (!Rtc.GetIsRunning())
    {
      Serial.println("RTC was not actively running, starting now.  Updating Date Time");
      Rtc.SetIsRunning(true);
      boolCheck = false;
      RTC_Update();
    }
  }

  void readRtcTemp() {
    RtcTemperature temp = Rtc.GetTemperature();
    if (!wasError("loop GetTemperature")) {
      //temp.Print(Serial);
      // you may also get the temperature as a float and print it
      // tempAHT = ((9.0 / 5.0) * (humiditySensor.getTemperature()) + 32.0);
      Serial.print((9.0 / 5.0) * (temp.AsFloatDegC()) + 32.0);
      Serial.println("F");
    }
  }
#endif


void mqttSubs(char* topic, byte* payload, unsigned int length);

#ifdef OLED
  #include <Adafruit_SSD1306.h>
  #define SCREEN_WIDTH 128 // OLED display width, in pixels
  #define SCREEN_HEIGHT 64 // OLED display height, in pixels
  Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
#endif

#ifdef BME680
  #define SEALEVELPRESSURE_HPA (1013.25)
  String output;
  Bsec iaqSensor;
#endif

#ifdef MBUS_RAIN
  #include <Preferences.h>
  Preferences preferences;

  #include "rain.h"
#endif

#include "Freenove_WS2812_Lib_for_ESP32.h"

#define LEDS_COUNT  1   // The number of led
#define LEDS_PIN	  16  // define the pin connected to the led strip
#define CHANNEL		  0   // RMT module channel

Freenove_ESP32_WS2812 strip = Freenove_ESP32_WS2812(LEDS_COUNT, LEDS_PIN, CHANNEL, TYPE_GRB);
//                         GREEN       , LIME      , YELLOW        , ORANGE      , RED         , MAGENTA     , BROWN         , WHITE         , OFF
uint8_t m_color[9][3] = { {0, 128, 0}, {0, 255, 0}, {255, 255, 0}, {255, 165, 0}, {255, 0, 0}, {255, 0, 255}, {165, 42, 42}, {255, 255, 255}, {0, 0, 0}  };



#include "functions.h"
#include "mpu6050.h"

WiFiClient espClient;
PubSubClient client(mqtt_server, 1883, mqttSubs, espClient); // instantiate MQTT client
const char* host = HOST;

//===============================================================================================

void pubMQTT(String topic, String topic_val) { // publish MQTT message to broker
  #ifdef MQTT_SEND_DEBUG_ON
    Serial.print("topic " + topic + " value:");
    Serial.println(String(topic_val).c_str());
  #endif
  // digitalWrite(MQCON, HIGH);
  client.publish(topic.c_str(), String(topic_val).c_str(), true);
  // digitalWrite(MQCON, LOW);
}

void mqttSubs(char* topic, byte* payload, unsigned int length) {  

  int i;
  error = 4;                    // assume invalid device until proven otherwise
  #ifdef DEBUG
    // Serial.print("Message arrived [");
    // Serial.print(topic);
    // Serial.print("] ");
    // for (int i = 0; i < length; i++) {
    //   Serial.print((char)payload[i]);
    // }
    // Serial.println();
  #endif
  if (strlen(topic) == 20) {            // correct topic length ?  was 27
    DID = (topic[18] - '0') * 10 + topic[19] - '0'; // extract device ID from MQTT topic  was 25 en 26
    payload[length] = '\0';           // terminate string with '0'
    String strPayload = String((char*)payload); // convert to string
    readAction = (strPayload == "READ");      // 'READ' or 'SET' value
    if (length == 0) {
      error = 2; // no payload sent
    }
    else {
      if (DID == 0) {     // uptime
        if (readAction) {
          send0 = true;
          error = 0;
        } else error = 3;           // invalid payload; do not process
      }
      if (DID == 1) {     // transmission interval
        error = 0;
        if (readAction) {
          send1 = true;
        } else {                // set transmission interval
          TXinterval = strPayload.toInt();
          if (TXinterval < 10 && TXinterval != 0) TXinterval = 10; // minimum interval is 10 seconds
        }
      }
      if (DID == 2) {     // RSSI
        if (readAction) {
          send2 = true;
          error = 0;
        } else error = 3;           // invalid payload; do not process
      }
      if (DID == 3) {     // version
        if (readAction) {
          send3 = true;
          error = 0;
        } else error = 3;           // invalid payload; do not process
      }
      if (DID == 4) {
        error = 0;   // flowstatus
        if (readAction) {          
          // send4 = true;
        } else{
          #ifdef flowSensor
            flowStatus = strPayload.toInt();
          #endif
        }
      }      
      if (DID == 5) {     // ACK
        if (readAction) {
          send5 = true;
          error = 0;
        } else if (strPayload == "ON") {
          setAck = true;
          if (setAck) send5 = true;
          error = 0;
        } else if (strPayload == "OFF") {
          setAck = false;
          if (setAck) send5 = true;
          error = 0;
        } else error = 3;
      }
      if (DID == 6) {     // toggle / timer mode selection
        if (readAction) {
          send6 = true;
          error = 0;
        } else if (strPayload == "ON") {    // select toggle mode
          toggleOnButton = true;
          if (setAck) send6 = true;
          error = 0;
        } else if (strPayload == "OFF") {   // select timer mode
          toggleOnButton = false;
          if (setAck) send6 = true;
          error = 0;
        } else error = 3;
      }
      if (DID == 7) {     // Timer interval
        error = 0;
        if (readAction) {
          send7 = true;
        } else {                // set timer interval
          TIMinterval = strPayload.toInt();
          if (TIMinterval < 5 && TIMinterval != 0) TIMinterval = 5; // minimum interval is 5 seconds
        }
      }
      if (DID == 8) {     // Timer interval
        error = 0;
        if (readAction) {
          send8 = true;
        } else {                // set timer interval
        //TIMShower = strPayload.toInt();
        //if (TIMShower <3&& TIMShower !=0) TIMShower = 3; // minimum interval is 3 min
        //Serial.print(TIMShower);
        }
      }
      if (DID == 9) {  // Timer interval
        error = 0;
        if (readAction) {
          send9 = true;
        } else {      // set timer interval
          // TIMShower2 = strPayload.toInt();
          // Serial.print(TIMShower2);
        }
      }
      if (DID == 10) {    // IP
        if (readAction) {
          send10 = true;
          error = 0;
        } else error = 3;           // invalid payload; do not process
      }
      if (DID == 11) {    //SSID
        if (readAction) {
          send11 = true;
          error = 0;
        } else error = 3;
      }
      if (DID == 12) {    //PW
        if (readAction) {
          send12 = true;
          error = 0;
        } else error = 3;
      }
      if (DID == 13) {    // Timer interval
        error = 0;
        if (readAction) {
          send13 = true;
          error = 0;
        } else error = 3; 
      }
      #ifdef RELAYS
        if (DID == 16) {    // state of Relay1  GPIO1 TX
          if (readAction) {
            send16 = true;
            error = 0;
          } else if (strPayload == "ON") {
            REL1State = 1; //mind you,the relay used will switch on at LOW
            digitalWrite(REL1, REL1State);
            if (setAck) send16 = true;
            error = 0;
          } else if (strPayload == "OFF") {
            REL1State = 0;
            digitalWrite(REL1, REL1State);
            if (setAck) send16 = true;
            error = 0;
          } else error = 3;
        }      
        if (DID == 17) {    // state of Relay2
          if (readAction) {
            send17 = true;
            error = 0;
          } else if (strPayload == "ON") {
            REL2State = 1;
            digitalWrite(REL2, REL2State);
            if (setAck) send17 = true;
            error = 0;
          } else if (strPayload == "OFF") {
            REL2State = 0;
            digitalWrite(REL2, REL2State);
            if (setAck) send17 = true;
            error = 0;
          } else error = 3;
        }
        if (DID == 18) {    // state of Relay3
          if (readAction) {
            send18 = true;
            error = 0;
          } else if (strPayload == "ON") {
            REL3State = 1;
            digitalWrite(REL3, REL3State);
            if (setAck) send18 = true;
            error = 0;
          } else if (strPayload == "OFF") {
            REL3State = 0;
            digitalWrite(REL3, REL3State);
            if (setAck) send18 = true;
            error = 0;
          } else error = 3;
        }
        if (DID == 19) {    // state of Relay4  GPIO3 Rx
        // Serial.end();
          if (readAction) {
            send19 = true;
            error = 0;
          } else if (strPayload == "ON") {
            REL4State = 1;
            digitalWrite(REL4, REL4State);
            if (setAck) send19 = true;
            error = 0;
          } else if (strPayload == "OFF") {
            REL4State = 0;
            digitalWrite(REL4, REL4State);
            if (setAck) send19 = true;
            error = 0;
          } else error = 3;
        }
        if (DID == 20) {    // state of GDE PWM 1
          if (readAction) {
            send20 = true;
            error = 0;
          } else if (strPayload == "ON") {
            // PWM1State = 1;
            // GDE.writeGPIO(PWM1, PWM1State);
            if (setAck) send20 = true;
            error = 0;
          } else if (strPayload == "OFF") {
            // PWM1State = 0;
            // GDE.writeGPIO(PWM1, PWM1State);
            if (setAck) send20 = true;
            error = 0;
          } else error = 3;
        }
      #endif
      if (DID == 21) {    // state of GDE PWM 2
        if (readAction) {
          send21 = true;
          error = 0;
        } else if (strPayload == "ON") {
          // PWM2State = 1;
          // GDE.writeGPIO(PWM2, PWM2State);
          if (setAck) send21 = true;
          error = 0;
        } else if (strPayload == "OFF") {
          // PWM2State = 0;
          // GDE.writeGPIO(PWM2, PWM2State);
          if (setAck) send21 = true;
          error = 0;
        } else error = 3;
      }
      if (DID == 22) {    // state of GDE PWM 3
        if (readAction) {
          send22 = true;
          error = 0;
        } else if (strPayload == "ON") {
          // PWM3State = 1;
          // GDE.writeGPIO(PWM3, PWM3State);
          if (setAck) send22 = true;
          error = 0;
        } else if (strPayload == "OFF") {
          // PWM3State = 0;
          // GDE.writeGPIO(PWM3, PWM3State);
          if (setAck) send22 = true;
          error = 0;
        } else error = 3;
      }
      if (DID == 23) {    // state of GDE PWM 4
        if (readAction) {
          send23 = true;
          error = 0;
        } else if (strPayload == "ON") {
          // PWM4State = 1;
          // GDE.writeGPIO(PWM4, PWM4State);
          if (setAck) send23 = true;
          error = 0;
        } else if (strPayload == "OFF") {
          // PWM4State = 0;
          // GDE.writeGPIO(PWM4, PWM4State);
          if (setAck) send23 = true;
          error = 0;
        } else error = 3;
      }
      if (DID == 24) {    // state of GDE PWM 5
        if (readAction) {
          send24 = true;
          error = 0;
        } else if (strPayload == "ON") {
          // PWM5State = 1;
          // GDE.writeGPIO(PWM5, PWM5State);
          if (setAck) send24 = true;
          error = 0;
        } else if (strPayload == "OFF") {
          // PWM5State = 0;
          // GDE.writeGPIO(PWM5, PWM5State);
          if (setAck) send24 = true;
          error = 0;
        } else error = 3;
      }
      if (DID == 25) {    // state of GDE PWM 6
        if (readAction) {
          send25 = true;
          error = 0;
        } else if (strPayload == "ON") {
          // PWM6State = 1;
          // GDE.writeGPIO(PWM6, PWM6State);
          if (setAck) send25 = true;
          error = 0;
        } else if (strPayload == "OFF") {
          // PWM6State = 0;
          // GDE.writeGPIO(PWM6, PWM6State);
          if (setAck) send25 = true;
          error = 0;
        } else error = 3;
      }
      if (DID == 26) {    // state of GDE PWM 7
        if (readAction) {
          send26 = true;
          error = 0;
        } else if (strPayload == "ON") {
          // PWM7State = 1;
          // GDE.writeGPIO(PWM7, PWM7State);
          if (setAck) send26 = true;
          error = 0;
        } else if (strPayload == "OFF") {
          // PWM7State = 0;
          // GDE.writeGPIO(PWM7, PWM7State);
          if (setAck) send26 = true;
          error = 0;
        } else error = 3;
      }
      if (DID == 27) {    // state of GDE PWM 8
        if (readAction) {
          send27 = true;
          error = 0;
        } else if (strPayload == "ON") {
          // PWM8State = 1;
          // GDE.writeGPIO(PWM8, PWM8State);
          if (setAck) send27 = true;
          error = 0;
        } else if (strPayload == "OFF") {
          // PWM8State = 0;
          // GDE.writeGPIO(PWM8, PWM8State);
          if (setAck) send27 = true;
          error = 0;
        } else error = 3;
      }
      if (DID == 46) {    //coop door
        if (readAction) {
          send46 = true;
          error = 0;
        } else error = 3;
      }
      if (DID == 49) {    //batterij
        if (readAction) {
          send49 = true;
          error = 0;
        } else error = 3;
      }
      if (DID == 50) {    //DHT11
        if (readAction) {
          send50 = true;
          error = 0;
        } else error = 3;
      }
      if (DID == 51) {    //BMP
        if (readAction) {
          send51 = true;
          error = 0;
        } else error = 3;
      }
      if (DID == 52) {    //BH1750
        if (readAction) {
          send52 = true;
          error = 0;
        } else error = 3;
      }
      if (DID == 53) {    //pcf8591
        //DHT11
        if (readAction) {
          send53 = true;
          error = 0;
        } else error = 3;
      }
      if (DID == 55) {    //clear rain totals 
        //DHT11
        if (readAction) {
          send55 = true;
        } else if (strPayload == "ON") {
            #ifdef MBUS_RAIN
            // rainFallSecImperial = 0;
            // rainFallSecMetric = 0;
            // rainHour = 0;
            // rainDay = 0;
            // preferences.putFloat("rainHour", rainHour); // save hourly rain data
            // preferences.putFloat("rainDay", rainDay); // save daily rain data
            #endif    
            // send55 = true;
          if (setAck) send55 = true;
          error = 0;
        } else if (strPayload == "OFF") {
          // PWM2State = 0;
          // GDE.writeGPIO(PWM2, PWM2State);
          if (setAck) send55 = true;
          error = 0;          
        } else error = 3;
      }
      if (DID == 59) {
        if (readAction) {
          send59 = true;
          error = 0;
        } else error = 0;
      }
      if (DID == 60) {
        if (readAction) {
          send60 = true;
          error = 0;
        } else error = 0;
      }
    }
  } else error = 1;
  if (error != 0) {             // send error message
    sprintf(buff_topic, "home/nb/node%02d/dev91", nodeId);
    sprintf(buff_msg, "syntax error %d", error);
    pubMQTT(buff_topic, buff_msg);
  }
}

void readMacAddress(){
  uint8_t baseMac[6];
  esp_err_t ret = esp_wifi_get_mac(WIFI_IF_STA, baseMac);
  if (ret == ESP_OK) {
    Serial.printf("%02x:%02x:%02x:%02x:%02x:%02x\n",
                  baseMac[0], baseMac[1], baseMac[2],
                  baseMac[3], baseMac[4], baseMac[5]);
  } else {
    Serial.println("Failed to read MAC address");
  }
}

void connectMqtt() {                // reconnect to mqtt broker
  sprintf(buff_topic, "home/sb/node%02d/#", nodeId);
  sprintf(clientName, "ESP32_%02d_%s", nodeId, FuntionCode);
  if (!client.loop()) {
    mqttNotCon = true; 
  #ifdef DEBUG
    Serial.print("Connect to MQTT broker...");
  #endif
    if (client.connect(clientName, "home/disconnected", 0, true, clientName)) {
      mqttNotCon = false; 
      client.subscribe(buff_topic);

  #ifdef DEBUG
    Serial.println("connected");
    Serial.println();    
  #endif
    } else {
  #ifdef DEBUG
      Serial.println("Failed, try again in 5 seconds");
  #endif
      delay(5000);
    }
    // digitalWrite(MQCON, mqttNotCon);      // adjust MQTT link indicator
  }
}

void connectWifi() {                // reconnect to Wifi
  while (WiFi.status() != WL_CONNECTED ) {
    int i = 0;
    if (!WiFi.config(local_IP, gateway, subnet, primaryDNS)){
      Serial.println("STA Failed to configure");
    }
    if (fallBackSsi)                  // select main or fallback access point
    { strcpy(wifi_ssid, wifi_ssid_B);
      strcpy(wifi_password, wifi_password_B);
    }
    else
    { strcpy(wifi_ssid, wifi_ssid_A);
      strcpy(wifi_password, wifi_password_A);
    }
  #ifdef DEBUG
    Serial.println();
    Serial.print("Connecting to  ");
    Serial.print(wifi_ssid);
  #endif
    WiFi.begin(wifi_ssid, wifi_password);
    while ((WiFi.status() != WL_CONNECTED) && (i < 20)) {
      delay(1000);
      i++;
  #ifdef DEBUG
      Serial.print(".");
  #endif
    }  
    fallBackSsi = !fallBackSsi;             // toggle access point
  }
  #ifdef DEBUG
    Serial.println("WiFi connected");
    Serial.print("IP address: ");
    //  Serial.println(IP);
    Serial.println(WiFi.localIP().toString());
    IP = WiFi.localIP().toString().c_str();
    Serial.print("[DEFAULT] ESP32 Board MAC Address: ");
    readMacAddress();
  #endif
}

void sendMsg() {                // send any outstanding messages
  int i;
  // digitalWrite(MQCON, HIGH);
  if (wakeUp) {   // send wakeup message
    wakeUp = false;
    sprintf(buff_topic, "home/nb/node%02d/dev99", nodeId);
    sprintf(buff_msg, "NODE %d WAKEUP: %s",  nodeId, clientName);
    send99 = false;
    pubMQTT(buff_topic, buff_msg);
  }
  if (send0) {    // send uptime
    sprintf(buff_topic, "home/nb/node%02d/dev00", nodeId);
    sprintf(buff_msg, "%d", upTime);
    send0 = false;
    pubMQTT(buff_topic, buff_msg);
  }
  if (send1) {    // send transmission interval
    sprintf(buff_topic, "home/nb/node%02d/dev01", nodeId);
    sprintf(buff_msg, "%d", TXinterval);
    send1 = false;
    pubMQTT(buff_topic, buff_msg);
  }
  if (send2) {    // send signalStrength
    sprintf(buff_topic, "home/nb/node%02d/dev02", nodeId);
    signalStrength = WiFi.RSSI();
    sprintf(buff_msg, "%d", signalStrength);
    send2 = false;
    pubMQTT(buff_topic, buff_msg);
  }
  if (send3) {    // send software version
    sprintf(buff_topic, "home/nb/node%02d/dev03", nodeId);
    for (i = 0; i < sizeof(VERSION); i++) {
      buff_msg[i] = VERSION[i];
    }
    buff_msg[i] = '\0';
    send3 = false;
    pubMQTT(buff_topic, buff_msg);
  }
  if (send5) {    // send ACK state
    sprintf(buff_topic, "home/nb/node%02d/dev05", nodeId);
    if (!setAck) sprintf(buff_msg, "OFF");
    else sprintf(buff_msg, "ON");
    pubMQTT(buff_topic, buff_msg);
    send5 = false;
  }
  if (send6) {    // send toggleOnButton state
    sprintf(buff_topic, "home/nb/node%02d/dev06", nodeId);
    if (!toggleOnButton) sprintf(buff_msg, "OFF");
    else sprintf(buff_msg, "ON");
    pubMQTT(buff_topic, buff_msg);
    send6 = false;
  }
  if (send7) {    // send timer value
    sprintf(buff_topic, "home/nb/node%02d/dev07", nodeId);
    // sprintf(buff_msg, "%d", TIMinterval);
    pubMQTT(buff_topic, buff_msg);
    send7 = false;
  }
  if (send8) {    // send timer value
    sprintf(buff_topic, "home/nb/node%02d/dev08", nodeId);
    //sprintf(buff_msg, "%d", TIMShower);
    pubMQTT(buff_topic, buff_msg);
    send8 = false;
    //pubMQTT(buff_topic, buff_msg);
  }
  if (send9) {    // send timer value
    sprintf(buff_topic, "home/nb/node%02d/dev09", nodeId);
    //sprintf(buff_msg, "%d", TIMShower2);
    pubMQTT(buff_topic, buff_msg);
    send9 = false;
    //pubMQTT(buff_topic, buff_msg);
  }
  if (send10) {   // send IP address
    sprintf(buff_topic, "home/nb/node%02d/dev10", nodeId);
    for (i = 0; i < 16; i++) {
      buff_msg[i] = IP[i];
    }
    buff_msg[i] = '\0';
    pubMQTT(buff_topic, buff_msg);
    send10 = false;
  }
  if (send11) {   // send SSID
    sprintf(buff_topic, "home/nb/node%02d/dev11", nodeId);
    for (i = 0; i < 20; i++) {
      buff_msg[i] = wifi_ssid[i];
    }
    buff_msg[i] = '\0';
    pubMQTT(buff_topic, buff_msg);
    send11 = false;
  }
  if (send12) {   // send PW
    sprintf(buff_topic, "home/nb/node%02d/dev12", nodeId);
    for (i = 0; i < 20; i++) {
      buff_msg[i] = wifi_password[i];
    }
    buff_msg[i] = '\0';
    pubMQTT(buff_topic, buff_msg);
    send12 = false;
  }
  if (send13) {   // send timer value
    sprintf(buff_topic, "home/nb/node%02d/dev13", nodeId);
    // sprintf(buff_msg, "%d", moistTimer);
    pubMQTT(buff_topic, buff_msg);
    send13 = false;
    //pubMQTT(buff_topic, buff_msg);
  }
  #ifdef RELAYS
    if (send16) {   // send RELAY1 state
      sprintf(buff_topic, "home/nb/node%02d/dev16", nodeId);
      if (prevREL1State != REL1State){
        if (REL1State == 0) sprintf(buff_msg, "OFF");
        if (REL1State == 1) sprintf(buff_msg, "ON");
        pubMQTT(buff_topic, buff_msg);
        prevREL1State = REL1State;
        send16 = false;
      }
      send16 = false;
    }
    if (send17) {   // send RELAY2 state
      sprintf(buff_topic, "home/nb/node%02d/dev17", nodeId);
      if (prevREL2State != REL2State){
        if (REL2State == 0) sprintf(buff_msg, "OFF");
        if (REL2State == 1) sprintf(buff_msg, "ON");
        pubMQTT(buff_topic, buff_msg);
        prevREL2State = REL2State;
        send17 = false;
      }
      send17 = false;
    }
    if (send18) {   // send RELAY3 state
      sprintf(buff_topic, "home/nb/node%02d/dev18", nodeId);
      if (prevREL3State != REL3State){    
        if (REL3State == 0) sprintf(buff_msg, "OFF");
        if (REL3State == 1) sprintf(buff_msg, "ON");
        pubMQTT(buff_topic, buff_msg);
        prevREL3State = REL3State;
        send18 = false;
      }
      send18 = false;
    }
    if (send19) {   // send RELAY4 state
      sprintf(buff_topic, "home/nb/node%02d/dev19", nodeId);
      if (prevREL4State != REL4State){
        if (REL4State == 0) sprintf(buff_msg, "OFF");
        if (REL4State == 1) sprintf(buff_msg, "ON");
        pubMQTT(buff_topic, buff_msg);
        prevREL4State = REL4State;
        send19 = false;
      }
      send19 = false;
    }
  #endif

  if (send20) {   // send PWM1 state
    sprintf(buff_topic, "home/nb/node%02d/dev20", nodeId);
    // if (PWM1State == 0) sprintf(buff_msg, "OFF");
    // if (PWM1State == 1) sprintf(buff_msg, "ON");
    pubMQTT(buff_topic, buff_msg);
    send20 = false;
  }
  if (send21) {   // send PWM2 state
    sprintf(buff_topic, "home/nb/node%02d/dev21", nodeId);
    // if (PWM2State == 0) sprintf(buff_msg, "OFF");
    // if (PWM2State == 1) sprintf(buff_msg, "ON");
    pubMQTT(buff_topic, buff_msg);
    send21 = false;
  }
  if (send22) {   // send PWM3 state
    sprintf(buff_topic, "home/nb/node%02d/dev22", nodeId);
    // if (PWM3State == 0) sprintf(buff_msg, "OFF");
    // if (PWM3State == 1) sprintf(buff_msg, "ON");
    pubMQTT(buff_topic, buff_msg);
    send22 = false;
  }
  if (send23) {   // send PWM4 state
    sprintf(buff_topic, "home/nb/node%02d/dev23", nodeId);
    // if (PWM4State == 0) sprintf(buff_msg, "OFF");
    // if (PWM4State == 1) sprintf(buff_msg, "ON");
    pubMQTT(buff_topic, buff_msg);
    send23 = false;
  }
  if (send24) {   // send PWM5 state
    sprintf(buff_topic, "home/nb/node%02d/dev24", nodeId);
    // if (PWM5State == 0) sprintf(buff_msg, "OFF");
    // if (PWM5State == 1) sprintf(buff_msg, "ON");
    pubMQTT(buff_topic, buff_msg);
    send24 = false;
  }
  if (send25) {   // send PWM6 state
    sprintf(buff_topic, "home/nb/node%02d/dev25", nodeId);
    // if (PWM6State == 0) sprintf(buff_msg, "OFF");
    // if (PWM6State == 1) sprintf(buff_msg, "ON");
    pubMQTT(buff_topic, buff_msg);
    send25 = false;
  }
  if (send26) {   // send PWM7 state
    sprintf(buff_topic, "home/nb/node%02d/dev26", nodeId);
    // if (PWM7State == 0) sprintf(buff_msg, "OFF");
    // if (PWM7State == 1) sprintf(buff_msg, "ON");
    pubMQTT(buff_topic, buff_msg);
    send26 = false;
  }
  if (send27) {   // send PWM8 state
    sprintf(buff_topic, "home/nb/node%02d/dev27", nodeId);
    // if (PWM8State == 0) sprintf(buff_msg, "OFF");
    // if (PWM8State == 1) sprintf(buff_msg, "ON");
    pubMQTT(buff_topic, buff_msg);
    send27 = false;
  }  
  if (send40) {   // send button pressed message
    sprintf(buff_topic, "home/nb/node%02d/dev40", nodeId);
    if (ACT1State == 0) sprintf(buff_msg, "OFF");
    if (ACT1State == 1) sprintf(buff_msg, "ON");
    pubMQTT(buff_topic, buff_msg);
    send40 = false;
  }
  if (send48) {   // bme680
    sprintf(buff_topic, "home/nb/node%02d/dev48", nodeId);
    #ifdef BME680
      StaticJsonDocument<256> doc;
      // doc["dev"] = "48";
      JsonArray data = doc.createNestedArray("bme680");
      data.add(round2(((9.0 / 5.0) * (iaqSensor.temperature) + 32.0)));
      data.add(round2(iaqSensor.humidity));
      data.add(round2((iaqSensor.pressure * 0.01) / pow(1 - (160 / 44330.0), 5.255)));    
      data.add(round2(iaqSensor.gasResistance));
      data.add(round2(iaqSensor.iaqAccuracy));
      data.add(round2(iaqSensor.iaq));
      data.add(round2(iaqSensor.staticIaq));
      data.add(round2(iaqSensor.co2Equivalent));
      data.add(round2(iaqSensor.breathVocEquivalent));
      char out[128];
      int b = serializeJson(doc, out);
      // Serial.print("bytes = ");
      // Serial.println(b,DEC);
      pubMQTT(buff_topic, out);
    #endif
    send48 = false;
  }
  if (send49) {   // modbus 8ch Temp Node 1
    sprintf(buff_topic, "home/nb/node%02d/dev49", nodeId);
    #ifdef MBUS_TEMP
      StaticJsonDocument<256> node1;
      node1["dev"] = "49";
      JsonArray data = node1.createNestedArray("N1_T8");
      data.add(serialized(String(node1_data[0],2)));
      // data.add(serialized(String(node1_data[1],2)));
      // data.add(serialized(String(node1_data[1],2)));    
      // data.add(serialized(String(node1_data[1],2)));
      // data.add(serialized(String(node1_data[1],2)));
      // data.add(serialized(String(node1_data[1],2)));
      // data.add(serialized(String(node1_data[1],2)));
      // data.add(serialized(String(node1_data[1],2)));
      // data.add(cycle);
      char out[128];
      int b = serializeJson(node1, out);
      // Serial.print("bytes = ");
      // Serial.println(b,DEC);
      pubMQTT(buff_topic, out);
    #endif
    send49 = false;
  }
  if (send50) {   // modbus 8ch Temp Node 2
    sprintf(buff_topic, "home/nb/node%02d/dev50", nodeId);
    #ifdef MBUS_TEMP
      StaticJsonDocument<256> node2;
      node2["dev"] = "50";
      JsonArray data = node2.createNestedArray("N2_T8");
      data.add(serialized(String(node2_data[0],2)));
      data.add(serialized(String(node2_data[1],2)));
      // data.add(serialized(String(node2_data[2],2)));    
      // data.add(serialized(String(node2_data[3],2)));
      // data.add(serialized(String(node2_data[4],2)));
      // data.add(serialized(String(node2_data[5],2)));
      // data.add(serialized(String(node2_data[6],2)));
      // data.add(serialized(String(node2_data[7],2)));
      // data.add(cycle);
      char out[128];
      int b = serializeJson(node2, out);
      // Serial.print("bytes = ");
      // Serial.println(b,DEC);
      pubMQTT(buff_topic, out);
    #endif
    send50 = false;
  }
  if (send51) {   // modbus wind
    sprintf(buff_topic, "home/nb/node%02d/dev51", nodeId);
    #ifdef MBUS_WIND
      StaticJsonDocument<256> wind;
      JsonArray data = wind.createNestedArray("wind");
      data.add(serialized(String(cur_wind_speed,2)));
      data.add(serialized(String(cur_wind_dir,2)));
      data.add(serialized(String(cur_wind_gear,2)));
      data.add(serialized(String(max_wind_speed,2)));
      data.add(serialized(String(max_wind_dir,2)));
      char out[128];
      int b = serializeJson(wind, out);
      // Serial.print("bytes = ");
      // Serial.println(b,DEC);
      pubMQTT(buff_topic, out);
    #endif
    send51 = false;
  }  
  if (send54) {   // ping tanks
    sprintf(buff_topic, "home/nb/node%02d/dev54", nodeId);
    #ifdef PING_TANKS
      StaticJsonDocument<256> doc;
      JsonArray data = doc.createNestedArray("tanks");
      data.add(serialized(String(TankTotalValue,2)));
      data.add(TankTotal_Status);    
      data.add(serialized(String(Tank1Value,2)));
      data.add(Tank1_Status);
      data.add(serialized(String(Tank2Value,2)));
      data.add(Tank2_Status);
      data.add(distance);              
      data.add(round2(tank1Temperature));
      data.add(round2(tank2Temperature));
      data.add(tank1_int/10000);
      data.add(tank2_int/10000);
      char out[128];        
      int b = serializeJson(doc, out);
      // Serial.print("bytes = ");
      // Serial.println(b,DEC);
      pubMQTT(buff_topic, out);
    #endif
    send54 = false;
  }
  if (send55) {   // rain
    sprintf(buff_topic, "home/nb/node%02d/dev55", nodeId);
    #ifdef MBUS_RAIN
      StaticJsonDocument<256> doc;
      JsonArray data = doc.createNestedArray("rain");
      data.add(InstRain);  //mm in 5 mins 
      data.add(InstRain / 25.4); // inches in 5 mins
      data.add(HourlyRainfall / 25.4); // inch in hour
      data.add(DayRain / 25.4);  // inch in day             
      char out[128];
      int b = serializeJson(doc, out);
      // Serial.print("bytes = ");
      // Serial.println(b,DEC);
      pubMQTT(buff_topic, out);
    #endif
    send55 = false;
  }
  if (send56) {   // mbus oaq
    sprintf(buff_topic, "home/nb/node%02d/dev56", nodeId);
    #ifdef MBUS_OAQ
      StaticJsonDocument<256> doc;
      JsonArray data = doc.createNestedArray("pm2510ug");
      data.add(serialized(String(pm2510ug_output_pm25)));  
      data.add(serialized(String(pm2510ug_output_pm10)));             
      char out[128];
      int b = serializeJson(doc, out);
      // Serial.print("bytes = ");
      // Serial.println(b,DEC);
      pubMQTT(buff_topic, out);
    #endif
    send56 = false;
  }
  if (send57) {   // wspi
    sprintf(buff_topic, "home/nb/node%02d/dev57", nodeId);
    #ifdef MBUS_WPSI
      StaticJsonDocument<256> doc;
      JsonArray data = doc.createNestedArray("wpsi");
      data.add(serialized(String(wpsi_output_mpa / 1000, 3)));  
      data.add(serialized(String(wpsi_output_psi / 1000, 3)));                          
      char out[128];
      int b = serializeJson(doc, out);
      // Serial.print("bytes = ");
      // Serial.println(b,DEC);
      pubMQTT(buff_topic, out);
    #endif
    send57 = false;
  }          
  //  Serial.println(); 
}

void scan_i2c (){
  byte error, address;
  int nDevices;
  nDevices = 0;
  for (address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    #ifdef OLED
      display.setCursor(0,0);
      display.clearDisplay();
    #endif
    if (error == 0)
    {
      #ifdef OLED
        display.setCursor(0,0);
        display.clearDisplay();
        display.setTextSize(2);
        display.println("I2C Found");
        display.println("");      
        display.setTextSize(3);
      #endif
      Serial.print("I2C device found at address 0x");   
      if (address < 16){
        Serial.print("0");
        #ifdef OLED
          display.print("0");
        #endif
      }
      Serial.print(address, HEX);
      Serial.println("");
      #ifdef OLED
        display.print(address,HEX);      
        display.println("");
        display.display();
      #endif
      delay(100);
      nDevices++;
    }
    else if (error == 4)
    {
      // display.setCursor(0,0);
      // display.clearDisplay();
      Serial.print("Unknow error at address 0x");
      // display.print("Error at:");
      if (address < 16)
        Serial.print("0");
        // display.print("0");
      Serial.println(address, HEX);
      // display.print(address, HEX);
      delay(100);
    }
  }
  if (nDevices == 0){
    // display.setCursor(0,0);
    // display.clearDisplay();    
    Serial.println("No I2C devices found\n");
    // display.println("NONE");
    delay(100);    
  }
  else{
    // display.setCursor(0,0);
    // display.clearDisplay();        
    Serial.println("done\n");
    // display.println("DONE");
    delay(100);    
  }
}

#ifdef I2C_MUX
  void scan_tca9545(){
    for (uint8_t t=0; t<channels; t++) {
      MP.enableChannel(t);
      Serial.print("Scanning Mux Bus: ");
      Serial.println(t);
      scan_i2c();
      MP.disableChannel(t);  
    }
    MP.disableAllChannels();
  }
#endif



void setup() { 
  Serial.begin(115200);
  Serial.println();
  Wire.begin();
  setAck=true;
  // scan_i2c();

	strip.begin();
	strip.setBrightness(10);  


  #ifdef MBUS_BASE  
   // Modbus communication runs at 19200 baud
    Serial2.begin(115200,SERIAL_8N1);
    pinMode(MAX485_RE_NEG, OUTPUT);
    pinMode(MAX485_DE, OUTPUT);
    // Init in receive mode
    digitalWrite(MAX485_RE_NEG, 0);
    digitalWrite(MAX485_DE, 0);
    node.begin(1, Serial2);
    // Callbacks allow us to configure the RS485 transceiver correctly
    node.preTransmission(preTransmission);
    node.postTransmission(postTransmission);
    Serial.println("MBUS BASE INIT COMPLETE");
    // send51 = true;
  #endif

  #ifdef I2C_MUX
    if (MP.begin() == false){
      Serial.println("COULD NOT CONNECT");
    }
    channels = MP.channelCount();
  


    Serial.print("CHAN:\t");
    Serial.println(MP.channelCount());
    MP.disableAllChannels();
    Serial.println("done...");
    Serial.println();
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.setCursor(0,0);
    display.clearDisplay();  
    scan_i2c();
    delay(1000);

    Serial.println("enabling bus 0 bme680");
    MP.enableChannel(0);
    scan_i2c();  
    display.setCursor(0,0);
    display.setTextSize(1);
    display.print(VERSION);
    display.display();
    delay(1000);
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE); 
    // pinMode(ACT1, OUTPUT);          // set pin of radio indicator
    // pinMode(MQCON, OUTPUT);         // set pin for MQTT connection indicator
  #endif

  #ifdef RELAYS
    pinMode(REL1, OUTPUT);          // set pin of radio indicator
    pinMode(REL2, OUTPUT);          // set pin of radio indicator
    pinMode(REL3, OUTPUT);          // set pin of radio indicator
    pinMode(REL4, OUTPUT);          // set pin of radio indicator
    // pinMode(REEDPIN,INPUT);

    digitalWrite(REL1, LOW);
    digitalWrite(REL2, LOW);
    digitalWrite(REL3, LOW);
    digitalWrite(REL4, LOW);      
    // digitalWrite(MQCON, LOW);         // switch off MQTT connection indicator
    // digitalWrite(ACT1, LOW);       // switch off radio indicator



    REL1State = LOW;
    REL2State = LOW;
    REL3State = LOW;
    REL4State = LOW;  

    digitalWrite(REL1, LOW);// Relay Switch 1  
    digitalWrite(REL2, LOW);// Relay Switch 2
    digitalWrite(REL3, LOW);// Relay Switch 3
    digitalWrite(REL4, LOW);// Relay Switch 4

    prevREL1State = HIGH; // set to force mqtt send 1st loop
    prevREL2State = HIGH;
    prevREL3State = HIGH;
    prevREL4State = HIGH;
  #endif

  #ifdef BME680
    iaqSensor.begin(BME680_I2C_ADDR_SECONDARY, Wire);
    output = "\nBSEC library version " + String(iaqSensor.version.major) + "." + String(iaqSensor.version.minor) + "." + String(iaqSensor.version.major_bugfix) + "." + String(iaqSensor.version.minor_bugfix);    
    Serial.println(output);
    checkIaqSensorStatus();

    if (bmp680_not_present != true){  
      bsec_virtual_sensor_t sensorList[10] = {
      BSEC_OUTPUT_RAW_TEMPERATURE,
      BSEC_OUTPUT_RAW_PRESSURE,
      BSEC_OUTPUT_RAW_HUMIDITY,
      BSEC_OUTPUT_RAW_GAS,
      BSEC_OUTPUT_IAQ,
      BSEC_OUTPUT_STATIC_IAQ,
      BSEC_OUTPUT_CO2_EQUIVALENT,
      BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
      BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
      BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
    };
    iaqSensor.updateSubscription(sensorList, 10, BSEC_SAMPLE_RATE_LP);
    checkIaqSensorStatus();
    // Print the header
    output = "Timestamp [ms], raw temperature [°C], pressure [hPa], raw relative humidity [%], gas [Ohm], IAQ, IAQ accuracy, temperature [°C], relative humidity [%], Static IAQ, CO2 equivalent, breath VOC equivalent";
    // Serial.println(output);  
    }
  #endif

  send0 = true;   // uptime
  send1 = true;    // interval for push messages
  send2 = true;    // RSSI
  send3 = true;    // Version
  send5 = false;   // ACK
  send6 = false;   // set/reset btn toggle
  send7 = false;   // timerinterval (sec)
  send8 = false;   // timer1 (min)
  send9 = false;   // timer2 (minm)
  send10 = true;   // send IP on startup
  send11 = false;   // SSID
  send13 = false;   // timer voor moisture
  #ifdef RELAYS
    send16 = true;   // read/set RELAY1 output
    send17 = true;   // read/set RELAY2 output
    send18 = true;   // read/set RELAY3 output
    send19 = true;   // read/set RELAY4 output
  #endif
  send20 = false;  // read/set PWM1   output
  send21 = false;  // read/set PWM2   output
  send22 = false;  // read/set PWM3   output
  send23 = false;  // read/set PWM4   output
  send24 = false;  // read/set PWM5   output
  send25 = false;  // read/set PWM6   output
  send26 = false;  // read/set PWM7   output
  send27 = false;  // read/set PWM8   output    
  send40 = false;
  send46 = false; 
  send48 - false; 
  send49 = false; 
  send50 = false; 
  send51 = false; 
  send52 = false; 
  send53 = false; 
  send54 = false; 
  send59 = false;
  send60 = false;
  #ifdef PING_TANKS
    tank1.begin();
    tank2.begin();
    //Create storage space for last readings in read/write mode
    preferences.begin("persist", false);
  #endif

  init_mpu6050();

  // stream_enabled = 1;

  connectWifi();

  // NTP
  // Start NTP Time Client
  // Init and get the time
  // configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  initTime("EST5EDT,M3.2.0,M11.1.0");
  #ifdef debug_rtc
    Serial.println();
    printLocalTime();
    Serial.println();
  #endif


  #ifdef rtc_ds3231
    Rtc.Begin();
    Serial.println();
    #ifdef debug_rtc
      RtcDateTime now = Rtc.GetDateTime();//rtc
      Serial.println("RTC time before update in setup");
      printDateTime(now);
    #endif  
    RTC_Update();
    #ifdef debug_rtc
      delay(2000);
      now = Rtc.GetDateTime();//rtc
      Serial.println("RTC time after update in setup");  
      printDateTime(now);
      Serial.println();
    #endif
    RtcDateTime now = Rtc.GetDateTime();//rtc
  #endif
  


  if (!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time");
  }

  second = timeinfo.tm_sec;
  minute = timeinfo.tm_min;
  hour = timeinfo.tm_hour;
  day = timeinfo.tm_yday + 1;
  month = timeinfo.tm_mon + 1;
  year = timeinfo.tm_year + 1900;
  weekday = timeinfo.tm_wday +1;

  #ifdef MBUS_RAIN
    node5_rtu_time();

    Serial.println();
    if(hour != RainClockHour){
      Serial.println("Rtc and RainClockHour does not match");
      Serial.print(hour);
      Serial.print(" = ");
      Serial.println(RainClockHour);
      Serial.print(hour, HEX);
      Serial.print(" = ");
      Serial.println(RainClockHour, HEX);          
    }

    if(hour == RainClockHour){
      Serial.println("Rtc and RainClockHour matches");
      Serial.print(hour);
      Serial.print(" = ");
      Serial.println(RainClockHour);
      Serial.print(hour, HEX);
      Serial.print(" = ");
      Serial.println(RainClockHour, HEX);          
    }
    Serial.println(); 
  #endif



  // flowStatus = 0;
  // Read_HCSR04(2);//************************************
  // send54 = true;


  //Start counting rain data
  // attachInterrupt(digitalPinToInterrupt(REEDPIN),reedSwitch_ISR, RISING);  // <----- changed to this line
  #ifdef MBUS_BASE

    count = 0;
 
    #ifdef MBUS_RAIN
      lastSecondRain = 0;
      last5MinutesRain = 0;
      // lastPulseCountRain = 0;
      // currentPulseCountRain = 0;
      // rain5min = 0;
      // rainHour = 0;
      // rainDay = 0;
    #endif

    if (firstRun == 1){
      #ifdef MBUS_RAIN
        // rainDay = 0.770;
        // preferences.putFloat("rainHour", rainHour); // save hourly rain data
        // preferences.putFloat("rainDay", rainDay); // save daily rain data
      #endif
      preferences.putFloat("tank1_int", tank1_int); // save 
      preferences.putFloat("tank2_int", tank2_int); // save
      firstRun = 0;      
    } 
    tank1_int = preferences.getInt("tank1_int", 0); //30 sec
    tank2_int = preferences.getInt("tank2_int", 0); //30 sec
    #ifdef MBUS_RAIN
      // rainHour = preferences.getFloat("rainHour", 0); // Restore the hourly rainfall data
      // rainDay = preferences.getFloat("rainDay", 0); // Restore the daily rainfall data
      // rainFallSecImperial = 0; //rain in last measuremet period
      //daysRain = 0;
      send55 = true;
    #endif
  #endif

  // Port defaults to 3232
  // ArduinoOTA.setPort(3232);
  // Hostname defaults to esp3232-[MAC]
  ArduinoOTA.setHostname(HOST);

  // No authentication by default
  ArduinoOTA.setPassword("admin");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3"); 

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else // U_SPIFFS
      type = "filesystem";
    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end() 
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR)         Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR)   Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR)     Serial.println("End Failed");
  });

  ArduinoOTA.begin(); 
}

void loop() {                       // Main program loop
  ArduinoOTA.handle();
  if (WiFi.status() != WL_CONNECTED) {            // Wifi connected ?
    connectWifi();
  }

  if (!client.connected()) {                  // MQTT connected ?
    connectMqtt();
  }
  client.loop();

  #ifdef BME680
    check_iaqSens();  
  #endif

  #ifdef rtc_ds3231
    if (!Rtc.IsDateTimeValid()){
      RTC_Update();
    }
    RtcDateTime now = Rtc.GetDateTime();//rtc
  #endif

  


  if (!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time");
  }

  #ifdef OLED
    display.setCursor(0,0);
    display.clearDisplay();
  #endif
 
  second = timeinfo.tm_sec;
  minute = timeinfo.tm_min;
  hour = timeinfo.tm_hour;
  day = timeinfo.tm_yday + 1;
  month = timeinfo.tm_mon + 1;
  year = timeinfo.tm_year + 1900;
  weekday = timeinfo.tm_wday +1;

  currentHour = hour;
  currentMinute = minute;
  currentSecond =  second;

  #ifdef MBUS_RAIN
    switch (currentMinute) {
      case 0:
        if (reported[0] == false){
          reported[11] = false;

          #ifdef MBUS_RAIN          
            node5_rtu_rain();
            rain5min_update();
            send55 = true;          
          #endif

          // #ifdef MBUS_OAQ          
          //   node3_rtu_oaq();
          //   send56 = true;
          // #endif 

          reported[0] = true;
        }
        break;
      case 5:
        if (reported[1] == false){
          reported[0] = false;

          #ifdef MBUS_RAIN          
            node5_rtu_rain();
            rain5min_update();
            send55 = true;
          #endif

          // #ifdef MBUS_OAQ          
          //   node3_rtu_oaq();
          //   send56 = true;
          // #endif

          reported[1] = true;       
        }
        break;
      case 10:
        if (reported[2] == false){
          reported[1] = false;

          #ifdef MBUS_RAIN          
            node5_rtu_rain();
            rain5min_update();
            send55 = true;
          #endif

          // #ifdef MBUS_OAQ          
          //   node3_rtu_oaq ();
          //   send56 = true;
          // #endif

          reported[2] = true;          
        }
        break;
      case 15:
        if (reported[3] == false){
          reported[2] = false;

          #ifdef MBUS_RAIN          
            node5_rtu_rain();
            rain5min_update();          
            send55 = true;
          #endif

          // #ifdef MBUS_OAQ          
          //   node3_rtu_oaq();
          //   send56 = true;
          // #endif

          reported[3] = true;
        }
        break;
      case 20:
        if (reported[4] == false){
          reported[3] = false;

          #ifdef MBUS_RAIN          
            node5_rtu_rain();
            rain5min_update();         
            send55 = true;
          #endif

          // #ifdef MBUS_OAQ          
          //   node3_rtu_oaq();
          //   send56 = true;
          // #endif

          reported[4] = true;
        }
        break;
      case 25:
        if (reported[5] == false){
          reported[4] = false;

          #ifdef MBUS_RAIN          
            node5_rtu_rain();
            rain5min_update();          
            send55 = true;
          #endif

          // #ifdef MBUS_OAQ          
          //   node3_rtu_oaq();
          //   send56 = true;
          // #endif

          reported[5] = true;
        }
        break;
      case 30:
        if (reported[6] == false){
          reported[5] = false;

          #ifdef MBUS_RAIN          
            node5_rtu_rain();
            rain5min_update();          
            send55 = true;
          #endif

          // #ifdef MBUS_OAQ          
          //   node3_rtu_oaq();
          //   send56 = true;
          // #endif

          reported[6] = true;
        }
        break;
      case 35:
        if (reported[7] == false){
          reported[6] = false;

          #ifdef MBUS_RAIN          
            node5_rtu_rain();
            rain5min_update();          
            send55 = true;
          #endif

          // #ifdef MBUS_OAQ          
          //   node3_rtu_oaq();
          //   send56 = true;
          // #endif

          reported[7] = true;
        }
        break;
      case 40:
        if (reported[8] == false){
          reported[7] = false;

          #ifdef MBUS_RAIN          
            node5_rtu_rain();
            rain5min_update();
            send55 = true;
          #endif

          // #ifdef MBUS_OAQ          
          //   node3_rtu_oaq();
          //   send56 = true;
          // #endif

          reported[8] = true;
        }
        break;
      case 45:
        if (reported[9] == false){
          reported[8] = false;

          #ifdef MBUS_RAIN          
            node5_rtu_rain();
            rain5min_update();         
            send55 = true;
          #endif

          // #ifdef MBUS_OAQ          
          //   node3_rtu_oaq();
          //   send56 = true;
          // #endif

          reported[9] = true;
        }
        break;
      case 50:
        if (reported[10] == false){
          reported[9] = false;

          #ifdef MBUS_RAIN          
            node5_rtu_rain();
            rain5min_update();
            send55 = true;
          #endif

          // #ifdef MBUS_OAQ          
          //   node3_rtu_oaq();
          //   send56 = true;
          // #endif
          reported[10] = true;
        }
        break;
      case 55:
          if (reported[11] == false){
            reported[10] = false;

            #ifdef MBUS_RAIN
              node5_rtu_rain();
              rain5min_update();
              send55 = true;
            #endif

            // #ifdef MBUS_OAQ
            //   node3_rtu_oaq();
            //   send56 = true;
            // #endif

            reported[11] = true;
            reported[12] = false;          
          }
          break;          
    }
    if (currentMinute == 59 && currentSecond == 59){
      if (reported[12] == false){
        reported[11] = false;
        #ifdef MBUS_RAIN
          node5_rtu_rain ();      
          rain5min_update();
          // rainFallSecMetric = 0;
          // rainHour = 0;
          preferences.putFloat("rainHour", HourlyRainfall / 25.4); // save hourly rain data
          send55 = true;
        #endif
        // #ifdef MBUS_OAQ
        //   node3_rtu_oaq ();
        //   send56 = true;
        // #endif
        reported[12] = true; 
      }  // reset hourly counter
    }
    if ((currentHour == 0) && (currentMinute == 1)){ //Reset Daily counts at 00:59
      if (reported[13] == false){
        #ifdef MBUS_RAIN
          // node5_rtu_rain ();      
          // rain5min_update();
          // preferences.putFloat("rainHour", rainHour); // save hourly rain data
          // preferences.putFloat("rainDay", rainDay); // save daily rain data            
          // send55 = true;
        #endif
        reported[13] = true;
      }
    if ((currentHour == 0) && (currentMinute == 2)){ //Reset Daily counts at 00:59
        reported[13] = false;      
      }
    }
  #endif		        
  

  long currentMillis = millis();

  #ifdef PING_TANKS
    unsigned long nowTANKS = millis();
    if (flowStatus == 0){
      tank1_int = 300000; // set tank1 read interval to 30sec
      tank2_int = 300000; // set tank2 read interval to 30sec
      intervals[1] = tank1_int;
      intervals[2] = tank2_int;
      preferences.putFloat("tank1_int", tank1_int); // save 
      preferences.putFloat("tank2_int", tank2_int); // save
      // Serial.println(tank1_int); 
    }

    if (flowStatus == 1) {
      tank1_int = 15000; // set tank1 read interval to 15 minutes
      tank2_int = 15000; // set tank2 read interval to 15 minutes
      intervals[1] = tank1_int;
      intervals[2] = tank2_int;
      preferences.putFloat("tank1_int", tank1_int); // save 
      preferences.putFloat("tank2_int", tank2_int); // save
      // Serial.println(tank1_int);       
    }

    if (nowTANKS - then >= intervals[current_interval_index]) {
      switch (current_interval_index) {
      case 0:
          // Serial.println("Stop");
          break;
      case 1:
          // detachInterrupt(REEDPIN);
          // storeVarRain();
          tank1.request_distance();
          tank1.request_temperature();
          // Read_HCSR04(1);
          // send54 = true;
          // restoreVarRain();
          // attachInterrupt(digitalPinToInterrupt(REEDPIN),reedSwitch_ISR, FALLING);
          break;
      case 2:
          // detachInterrupt(REEDPIN);
          // storeVarRain();
          tank2.request_distance();
          tank2.request_temperature();
          // Read_HCSR04(2);
          // send54 = true;
          // restoreVarRain();        
          // attachInterrupt(digitalPinToInterrupt(REEDPIN),reedSwitch_ISR, FALLING);
          break;
      }
      then = nowTANKS;
      // increment index and wrap it back to zero, if it goes to 4 
      current_interval_index = (current_interval_index + 1) % 3; 
    }

    if (tank1_data_available = true){
      tank1_data_available = tank1.data_available();
      proc_tank1();    
    }

    if (tank2_data_available = true){
      tank2_data_available = tank2.data_available();
      proc_tank2();    
    }
  #endif

  // // DETECT INPUT CHANGE
  // // not necessary for current application
  // curState = digitalRead(BTN);                // Read button  
  // msgBlock = ((millis() - lastBtnPress) < HOLDOFF);     // hold-off time for additional button messages

  // if (!msgBlock &&  (curState != lastState)) {        // input changed ?
  //   delay(5);
  //   lastBtnPress = millis();                // take timestamp
  //   if (setAck) send40 = true;                // send button message
  //   if (curState == LOW) {
  //     if (toggleOnButton) {
  //       // button in toggle state ?
  //       Serial.println(ACT1State);
  //       ACT1State = !ACT1State;               // toggle output
  //       // digitalWrite(ACT1, ACT1State);
  //       Serial.println(ACT1State);        
  //       send16 = true;                    // send message on status change
  //     }
  //     else if (TIMinterval > 0 && !timerOnButton) {       // button in timer state ?
  //       timerOnButton = true;               // start timer interval
  //       Serial.println(ACT1State);
  //       ACT1State = HIGH;                 // switch on ACT1
  //       // digitalWrite(ACT1, ACT1State);
  //       Serial.println(ACT1State);
  //       send16 = true;
  //     }
  //   }
  //   lastState = curState;
  // }

  // // TIMER CHECK

  // if (TIMinterval > 0 && timerOnButton) {           // =0 means no timer
  //   if ( millis() - lastBtnPress > TIMinterval * 1000) {  // timer expired ?
  //     timerOnButton = false;                // then end timer interval
  //     ACT1State = LOW;                  // and switch off Actuator
  //     // digitalWrite(ACT1, ACT1State);
  //     send16 = true;
  //   }
  // }



  #ifdef MBUS_WIND
    if (node1_rtu_wind_speed_interval > 0) {
      // delay(2);
      if (currentMillis - lastPeriod_node1_rtu_wind_speed >= (node1_rtu_wind_speed_interval * 1000)) {
        lastPeriod_node1_rtu_wind_speed = currentMillis;
        // Serial.println("Entered Node_1 Read");      
        // digitalWrite(MAX485_RE_NEG, 0);
        // digitalWrite(MAX485_DE, 0);
        node1_rtu_wind_speed(); // read windspeed
        delay(200);
        // digitalWrite(MAX485_RE_NEG, 0);
        // digitalWrite(MAX485_DE, 0);
        node2_rtu_wind_dir(); // read wind direction and wind gear
        calc_wind_max();      
      }
    }
  #endif

  #ifdef MBUS_WPSI
    if (node4_rtu_wpsi_interval > 0) { 
      if (currentMillis - lastPeriod_node4_rtu_wpsi >= (node4_rtu_wpsi_interval * 1000)) {
        lastPeriod_node4_rtu_wpsi = currentMillis;
        // digitalWrite(MAX485_RE_NEG, 0);
        // digitalWrite(MAX485_DE, 0);
        node4_rtu_wpsi ();
        send57 = true;
      }
    }  
  #endif 

  #ifdef MBUS_OAQ
    if (currentMillis - lastMinute >= (UptimeInterval * 1000)) {    
      lastMinute = currentMillis;
      #ifdef MBUS_OAQ
        node3_rtu_oaq ();
        send56 = true;
      #endif
      upTime++;
    }
  #endif

  if (timer2_sample_rate > 0) {
    if (currentMillis - lastPeriod_timer2 >= (timer2_sample_rate)) {
      lastPeriod_timer2 = currentMillis;
      buzzer_timer_callback();
    }    
  } 

  if (mpu_sample_rate > 0) {
    if (currentMillis - lastPeriod_mpu6050 >= (mpu_sample_rate)) {
      lastPeriod_mpu6050 = currentMillis;
      imu_read_interrupt_callback();
    }    
  } 

  proc_mpu6050();


  // PERIODIC TRANSMISSION
  if (TXinterval > 0) { 
    if (currentMillis - lastPeriod >= (TXinterval * 1000)) {
      lastPeriod = currentMillis;  
      send0 = false;     // uptime
      send1 = false;     // interval
      send2 = false;     // send RSSI
      send3 = false;     // send version
      send8 = false;                  
      send9 = false;                 
      send10 = false;    // send IP address
      send11 = false; 
      send13 = false;   // 
      send16 = true;    // read/set RELAY1 output
      send17 = true;    // read/set RELAY2 output
      send18 = true;    // read/set RELAY3 output
      send19 = true;    // read/set RELAY4 output
      send20 = false;   // read/set PWM1 output
      send21 = false;   // read/set PWM2 output
      send22 = false;   // read/set PWM3 output
      send23 = false;   // read/set PWM4 output
      send24 = false;   // read/set PWM5 output
      send25 = false;   // read/set PWM6 output
      send26 = false;   // read/set PWM7 output
      send27 = false;   // read/set PWM8 output    
      send41 = false;   //
      send42 = false;   //
      send46 = false;   //
      send48 = false;   //
      send49 = false;   //
      send51 = false;   //
      send54 = false;   //
      send53 = false;   //
      send55 = false;   //
      send59 = false;   //
      send60 = false;   // 
      send61 = false;   //
    }    
  }

  sendMsg();            // send any mqtt messages   
  
}   // end loop
