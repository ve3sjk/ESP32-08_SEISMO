char timestamp[32];
int cur_hour;
int cur_minute;
char timeHour[3];
char timeMinute[3];
struct tm timeinfo;
uint8_t second;
uint8_t minute;
uint8_t hour;
uint8_t day;
uint8_t month;
uint8_t year;
int weekday;
uint8_t currentHour = 0;
uint8_t currentMinute = 0;
uint8_t currentSecond = 0;

int firstRun = 0;
const char* ntpServer = "pool.ntp.org";
const char* ntpServer1 = "pool.ntp.org";
const char* ntpServer2 = "time.nist.gov";
const long  gmtOffset_sec = -18000;
const int   daylightOffset_sec = 0;
// #include <ESP32Time.h>
//ESP32Time rtc;
// ESP32Time rtc(gmtOffset_sec);

//#include <NTPClient.h>
// // Start NTP time sync
// timeClient.begin();
// timeClient.update();

const char* time_zone = "EST5EDT,M3.2.0,M11.1.0";  // TimeZone rule for Europe/Rome including daylight adjustment rules (optional)


void setTimezone(String timezone){
  #ifdef rtc_debug
    Serial.printf("  Setting Timezone to %s\n",timezone.c_str());
  #endif
  setenv("TZ",timezone.c_str(),1);  //  Now adjust the TZ.  Clock settings are adjusted to show the new local time
  tzset();
}

void initTime(String timezone){
  static struct tm timeinfo;
  #ifdef rtc_debug
    Serial.println("Setting up time");
  #endif
  configTime(0, 0, "pool.ntp.org");    // First connect to NTP server, with 0 TZ offset
  if(!getLocalTime(&timeinfo)){
    Serial.println("  Failed to obtain time");
    return;
  }
  #ifdef rtc_debug
    Serial.println("  Got the time from NTP");
  #endif
  // Now we can set the real timezone
  setTimezone(timezone);
}

void printLocalTime(){
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time 1");
    return;
  }
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S zone %Z %z ");
  // Serial.println("Time variables");
  // strftime(timeHour,3, "%H", &timeinfo);
  // Serial.println(timeHour); 
}

void setTime(int yr, int month, int mday, int hr, int minute, int sec, int isDst){
  struct tm tm;

  tm.tm_year = yr - 1900;   // Set date
  tm.tm_mon = month-1;
  tm.tm_mday = mday;
  tm.tm_hour = hr;      // Set time
  tm.tm_min = minute;
  tm.tm_sec = sec;
  tm.tm_isdst = isDst;  // 1 or 0
  time_t t = mktime(&tm);
  Serial.printf("Setting time: %s", asctime(&tm));
  struct timeval now = { .tv_sec = t };
  settimeofday(&now, NULL);
}

// Function that gets current epoch time
unsigned long getTime() {
  time_t now;
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    //Serial.println("Failed to obtain time");
    return(0);
  }
  time(&now);
  return now-14400;
}

#ifdef rtc_ds3231
  bool wasError(const char* errorTopic = ""){
    uint8_t error = Rtc.LastError();
    if (error != 0){
      // we have a communications error
      // see https://www.arduino.cc/reference/en/language/functions/communication/wire/endtransmission/
      // for what the number means
      Serial.print("[");
      Serial.print(errorTopic);
      Serial.print("] WIRE communications error (");
      Serial.print(error);
      Serial.print(") : ");

      switch (error)
      {
      case Rtc_Wire_Error_None:
          Serial.println("(none?!)");
          break;
      case Rtc_Wire_Error_TxBufferOverflow:
          Serial.println("transmit buffer overflow");
          break;
      case Rtc_Wire_Error_NoAddressableDevice:
          Serial.println("no device responded");
          break;
      case Rtc_Wire_Error_UnsupportedRequest:
          Serial.println("device doesn't support request");
          break;
      case Rtc_Wire_Error_Unspecific:
          Serial.println("unspecified error");
          break;
      case Rtc_Wire_Error_CommunicationTimeout:
          Serial.println("communications timed out");
          break;
      }
      return true;
    }
    return false;
  }
  void printDateTime(const RtcDateTime& dt){
      char datestring[20];

      snprintf_P(datestring, 
              countof(datestring),
              PSTR("%02u/%02u/%04u %02u:%02u:%02u"),
              dt.Month(),
              dt.Day(),
              dt.Year(),
              dt.Hour(),
              dt.Minute(),
              dt.Second() );
      Serial.println(datestring);
  }

#endif


void formatTime() {
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    return;
  }
}


