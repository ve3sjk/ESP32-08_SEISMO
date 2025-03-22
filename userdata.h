#define VERSION "ESP32_08_SEISMO"
#define HOST  "ESP32_08_SEISMO"
#define TRANSMITINTERVAL 30
#define wifi_ssid_A "XXXXXXXXX"          // wifi station name
#define wifi_password_A "XXXXXXXXXXX"      // wifi password
#define wifi_ssid_B "XXXXXXXXXX"          // fallback wifi station name; if not present use same as A
#define wifi_password_B "XXXXXXXXXXX"      // fallback wifi password; if not present use same as A
#define mqtt_server "192.168.1.207"       // mqtt server IP  >make this yr MQTTserver IP
#define nodeId 8                         // node ID
#define FuntionCode "SEI"                 // 3 char device primary function code

IPAddress local_IP(192, 168, 1, 4);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress primaryDNS(192,168,1,1); 

#ifdef flowSensor
  //water flow meter 1
  int flowStatus = 0;
#endif

#ifdef PING_TANKS

  #define tank1_on
  #define tank2_on
  #define tanktotal_on

  int Tank1TrigPin = 27;
  int Tank1EchoPin = 34;
  int Tank2TrigPin = 13;
  int Tank2EchoPin = 35;

  //**variables used in read_water_sensor function
  long duration, distance; // Duration used to calculate distance
  float water_height = 0; //water height in inches
  float filledvolume = 0; // amount of tank volume filled with water
  float water_litres = 0; // amount of tank volume filled in litres
  float echo = 0; // returned value in litres

  byte tank1_data_available;
  byte tank2_data_available;
  uint16_t tank1Distance;
  uint16_t tank2Distance;
  uint16_t tank1Temperature;
  uint16_t tank2Temperature;


  float Tank1RawValue = 0;
  float Tank2RawValue = 0;
  float Tank3RawValue = 0;
  float Tank4RawValue = 0;
  float TankTotalRawValue = 0;

  float Tank1Value = 0;
  float Tank2Value = 0;
  float Tank3Value = 0;
  float Tank4Value = 0;
  float TankTotalValue = 0;
  float PrevTankTotalValue = 0;
    //**Tank 1
  int Tank1Value_Low = 50.00;
  int Tank1Value_High = 1000.00;
  int Tank1Pump_ON = 200.00;
  int Tank1Pump_OFF = 190.00;
  boolean Tank1MixPump_Enabled = true;
  String Tank1_Status = "OK";

  //**Tank 2
  int Tank2Value_Low = 50.00;
  int Tank2Value_High = 1000.00;
  int Tank2Pump_ON = 200.00;
  int Tank2Pump_OFF = 190.00;
  boolean Tank2MixPump_Enabled = true;
  String Tank2_Status = "OK";

  //**Tank 3
  int Tank3Value_Low = 50.00;
  int Tank3Value_High = 200.00;
  int Tank3Pump_ON = 200.00;
  int Tank3Pump_OFF = 190.00;
  boolean Tank3MixPump_Enabled = true;
  String Tank3_Status = "OK";

  //**Tank 4
  int Tank4Value_Low = 50.00;
  int Tank4Value_High = 200.00;
  int Tank4Pump_ON = 200.00;
  int Tank4Pump_OFF = 190.00;
  boolean Tank4MixPump_Enabled = true;
  String Tank4_Status = "OK";

  //**Tank Total
  int TankTotalValue_Low = 100.00;
  int TankTotalValue_High = 2000.00;
  String TankTotal_Status = "OK";
#endif


