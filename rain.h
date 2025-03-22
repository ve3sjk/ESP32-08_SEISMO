
int count;
// unsigned int decimalPlaces = 3;

#ifdef MBUS_RAIN

  #define FIVEMINUTES (300 *1000L)  // 30 Seconds, for 5 minutes will need changed to 300000 milliseconds
  // #define REEDPIN 23
  // #define RainCalImperial 0.011 //inch
  // #define RainCalMetric 0.2794  //mm




  int hourCount;
  unsigned long lastSecondRain,last5MinutesRain;
  // float lastPulseCountRain;
  // int currentPulseCountRain;
  // float rain5min;
  // float rainHour;
  // float rainDay;
  // float rainFallSecImperial;
  // float rainFallSecMetric;



  //float daysRain;

  int tmp_count;
  int tmp_hourCount;
  unsigned long tmp_lastSecondRain,tmp_last5MinutesRain;
  float tmp_lastPulseCountRain;
  int tmp_currentPulseCountRain;
  float tmp_rain5min;
  float tmp_rainHour;
  float tmp_rainDay;
  float tmp_rainFall;

  float tmp_daysRain;


  unsigned int decimalPlaces = 3;

  void rain5min_update(){

    // rainFallSecMetric = ((node5_data[1]) / 10); // 
    // rainFallSecImperial = rainFallSecMetric / 25.4;
    // rain5min = rainFallSecImperial;
    // rainHour = ((HourlyRainfall / 10) / 25.4);
    // rainDay  = ((DayRain / 10) / 25.4);

    Serial.print("Rain Var (mm): ");
    Serial.print(DayRain);
    Serial.print(", ");
    Serial.print(InstRain);
    Serial.print(", ");    
    Serial.print(YesterdayRain);
    Serial.print(", ");    
    Serial.print(SensorTotalRain);
    Serial.print(", ");
    Serial.print(HourlyRainfall);
    Serial.print(", ");    
    Serial.print(LastHourRainfall);
    Serial.print(", ");
    Serial.print(Max24Rain);
    Serial.print(", ");
    Serial.print(Max24Rain_TimeStart, HEX);
    Serial.print(", ");    
    Serial.print(Max24Rain_TimeEnd, HEX);
    Serial.print(", ");           
    Serial.print(Min24Rain);
    Serial.print(", ");     
    Serial.print(Min24Rain_TimeStart, HEX);
    Serial.print(", ");    
    Serial.print(Min24Rain_TimeEnd, HEX);
    Serial.println();    

  
    #ifdef eqmode
      Coloria::print(Coloria::BG_BRIGHT_RED, "Current Time is: ");
      // Serial.print("Current Time is: ");
      if (currentHour < 10){
        // Coloria::print(Coloria::RED, Coloria::UNDERLINE, "0");
        Serial.print("0");
      }
      Serial.print(currentHour);
      Coloria::print(Coloria::BG_BRIGHT_RED, ":");  
      // Serial.print(":");
      if (currentMinute < 10){
        // Coloria::println(Coloria::RED, Coloria::UNDERLINE, "0");    
        Serial.print("0");
      }
      Serial.print(currentMinute);
      Coloria::print(Coloria::BG_BRIGHT_RED, ":");  
      // Serial.print(":");
      if (currentSecond < 10){
        // Coloria::println(Coloria::RED, Coloria::UNDERLINE, "0");    
        Serial.print("0");
      }
      Serial.print(currentSecond);
      Coloria::print(Coloria::BG_BRIGHT_RED, " 5m:");
      Serial.print(InstRain / 25.4, 3);
      Coloria::print(Coloria::BG_BRIGHT_RED, " 60m:");  
      Serial.print(HourlyRainfall / 25.4, 3);
      Coloria::print(Coloria::BG_BRIGHT_RED, " 24h:");
      Serial.println(DayRain / 25.4, 3);
    #endif

    preferences.putFloat("rainHour", DayRain / 25.4); // save hourly rain data
    preferences.putFloat("rainDay", DayRain / 25.4); // save daily rain data
  }

#endif




