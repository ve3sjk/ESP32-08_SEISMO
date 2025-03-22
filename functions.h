  // Helper function definitions

  // rounds a number to 2 decimal places
  // example: round(3.14159) -> 3.14
  double round2(double value) {
    return (int)(value * 100 + 0.5) / 100.0;
  }

  float roundoff(float num,int precision){
        int temp=(int )(num*pow(10,precision));
        int num1=num*pow(10,precision+1);
        temp*=10;
        temp+=5;
        if(num1>=temp)
                num1+=10;
        num1/=10;
        num1*=10;
        num=num1/pow(10,precision+1);
        return num;
  }

  void print_decimal_point_string(uint16_t num, uint8_t dec_place){
    char temp_buffer[7]; // uint16_t can be 5 digits + '.' + '\0'
    char *temp_ptr = &temp_buffer[sizeof(temp_buffer)-1]; // start pointing to the end of the buffer

    *temp_ptr-- = '\0'; // null terminate the string.  todo - i might have lost a few cycles here if your optimizer sucks.
    do
    {
      *temp_ptr-- = num % 10 + '0'; // convert least significant digit
      if(--dec_place == 0) *temp_ptr-- = '.';
      num /= 10;
    } 
    while (num != 0);

    Serial.print(++temp_ptr); // print string, although we should have decremented one past it in the loop.
    return;
  }

  void errLeds(void){
    pinMode(23, OUTPUT);
    digitalWrite(23, HIGH);
    delay(100);
    digitalWrite(23, LOW);
    delay(100);
  }



  #ifdef BME680
    void checkIaqSensorStatus(void)
    {
      if (iaqSensor.status != BSEC_OK) {
        if (iaqSensor.status < BSEC_OK) {
          output = "BSEC error code : " + String(iaqSensor.status);
          if (bmp680_not_present != true){
            Serial.println(output);
          }        
          bmp680_not_present = true;
        } else {
          output = "BSEC warning code : " + String(iaqSensor.status);
          Serial.println(output);
        }
      }
    
      if (iaqSensor.bme680Status != BME680_OK) {
        if (iaqSensor.bme680Status < BME680_OK) {
          output = "BME680 error code : " + String(iaqSensor.bme680Status);
          if (bmp680_not_present != true){
            Serial.println(output);
          }
          bmp680_not_present = true;
        } else {
          output = "BME680 warning code : " + String(iaqSensor.bme680Status);
          Serial.println(output);
        }
      }
    }
    
    int bsecloopcount = 0;

    void check_iaqSens(){   
      unsigned long time_trigger = millis();
      if (iaqSensor.run()) { // If new data is available
        output = String(time_trigger);
        output += ", " + String(((9.0 / 5.0) * (iaqSensor.rawTemperature) + 32.0));
        output += ", " + String(((iaqSensor.pressure * 0.01) / pow(1 - (160 / 44330.0), 5.255)));    
        output += ", " + String(iaqSensor.rawHumidity);
        output += ", " + String(iaqSensor.gasResistance);
        output += ", " + String(iaqSensor.iaq);
        output += ", " + String(iaqSensor.iaqAccuracy);
        output += ", " + String(iaqSensor.temperature);
        output += ", " + String(iaqSensor.humidity);
        output += ", " + String(iaqSensor.staticIaq);
        output += ", " + String(iaqSensor.co2Equivalent);
        output += ", " + String(iaqSensor.breathVocEquivalent);
        bsecloopcount++;
        if (bsecloopcount >= 20*6){
          send48 = true;
          bsecloopcount = 0;
        } 
      } else {
        checkIaqSensorStatus();
      }
      display.setTextSize(2); 
      // Serial.print("Temperature = "); Serial.print(((9.0 / 5.0) * (iaqSensor.rawTemperature) + 32.0));
      // Serial.println(" *F");
      display.print("T: "); display.println(((9.0 / 5.0) * (iaqSensor.rawTemperature) + 32.0));//  display.println(" *F");
    
      // Serial.print("Pressure = "); Serial.print(((iaqSensor.pressure * 0.01) / pow(1 - (160 / 44330.0), 5.255))); Serial.println(" hPa");
      display.print("P: "); display.println(((iaqSensor.pressure * 0.01) / pow(1 - (160 / 44330.0), 5.255)));//  display.println(" hPa");
    
      // Serial.print("Humidity = "); Serial.print(iaqSensor.humidity); Serial.println(" %");
      display.print("H: "); display.println(iaqSensor.humidity);//  display.println(" %");
    
      // Serial.print("IAQ = "); Serial.print(iaqSensor.staticIaq); Serial.println("");
      display.print("IA:"); display.println(iaqSensor.staticIaq);// display.println("");
    
      // Serial.print("CO2 equiv = "); Serial.print(iaqSensor.co2Equivalent); Serial.println("");
      display.print("CO:"); display.println(iaqSensor.co2Equivalent);// display.println("");
    
      // Serial.print("Breath VOC = "); Serial.print(iaqSensor.breathVocEquivalent); Serial.println("");
      display.print("VO:"); display.println(iaqSensor.breathVocEquivalent);// display.println("");
    
      // Serial.println();
      display.display();
    }
  #endif

  #ifdef PING_TANKS
    float read_water_sensor (int tank) {

      int tanknum1 = tank;

      if (tanknum1 == 1){
        #ifdef eqmode
          Serial.print("current tank number");
          Serial.print(tanknum1);
        #endif
        distance = tank1Distance;
      }
      if (tanknum1 == 2){
        #ifdef eqmode
          Serial.print("current tank number");
          Serial.print(tanknum1);
        #endif
        distance = tank2Distance;
      }

      if (distance > 38){ //39fullh
        distance = 38;

      }
      // convert air space to water height
      water_height = 38-distance;
      #ifdef eqmode
        Serial.print("  Water Height: ");
        Serial.println(water_height);
      #endif
      // calculate volume of water in tank 37.5x37.5x39.750
      filledvolume = 1706.25*(water_height);
      // convert to litres equivalent
      water_litres = filledvolume*0.01638706;
      // set return value to requirements
      echo = water_litres;
      return echo;
    }
    void Read_HCSR04(int tank_number){
    
      int tanknum = tank_number;
      // Serial.print("current tank number");
      // Serial.println(tanknum);
      if (tanknum == 1){
        #if defined(tank1_on)
          Tank1RawValue = (read_water_sensor(tanknum));
        #else
          Tank1RawValue = 0;
          Tank1Value =0;
        #endif
        }
      if (tanknum == 2){
        #if defined(tank2_on)
          Tank2RawValue = (read_water_sensor(tanknum));
        #else
          Tank2RawValue = 0;
          Tank2Value =0;
        #endif
        }
      if (tanknum == 3){
        #if defined(tank3_on)
        Tank3RawValue = (read_water_sensor(tanknum));
        #else
        Tank3RawValue = 0;
        Tank3Value =0;
        #endif
        }
      if (tanknum == 4){
        #if defined(tank4_on)
          Tank4RawValue = (read_water_sensor(tanknum));
        #else
          Tank4RawValue = 0;
          Tank4Value =0;
        #endif
      }

      #if defined(tanktotal_on)  
        TankTotalRawValue = Tank1RawValue + Tank2RawValue + Tank3RawValue + Tank4RawValue;
      #else
        TankTotalRawValue = 0;
        TankTotalValue = 0;
      #endif  

      Tank1Value = Tank1RawValue;    
      Tank2Value = Tank2RawValue;
      Tank3Value = Tank3RawValue;
      Tank4Value = Tank4RawValue;  
      TankTotalValue = TankTotalRawValue;
      PrevTankTotalValue = TankTotalValue;

      //TankTotal
      if (TankTotalValue < TankTotalValue_Low) {
        TankTotal_Status = "LOW";
      }  
      else if (TankTotalValue > TankTotalValue_Low && TankTotalValue < TankTotalValue_High) {
        TankTotal_Status = "OK";
      } 
      else if (TankTotalValue > TankTotalValue_High) {
        TankTotal_Status = "HIGH";
      }

      //Tank1
      if (Tank1Value < Tank1Value_Low) {
        Tank1_Status = "LOW";
      } 
      else if (Tank1Value > Tank1Value_Low && Tank1Value < Tank1Value_High) {
        Tank1_Status = "OK";
      } 
      else if (Tank1Value > Tank1Value_High) {
        Tank1_Status = "HIGH";
      }

      //Tank2
      if (Tank2Value < Tank2Value_Low) {
        Tank2_Status = "LOW";
      } 
      else if (Tank2Value > Tank2Value_Low && Tank2Value < Tank2Value_High) {
        Tank2_Status = "OK";
      } 
      else if (Tank2Value > Tank2Value_High) {
        Tank2_Status = "HIGH";
      }

      //Tank3
      if (Tank3Value < Tank3Value_Low) {
        Tank3_Status = "LOW";
      } 
      else if (Tank3Value > Tank3Value_Low && Tank3Value < Tank3Value_High) {
        Tank3_Status = "OK";
      } 
      else if (Tank3Value > Tank3Value_High) {
        Tank3_Status = "HIGH";
      }

      //Tank4
      if (Tank4Value < Tank4Value_Low) {
        Tank4_Status = "LOW";
      } 
      else if (Tank4Value > Tank4Value_Low && Tank4Value < Tank4Value_High) {
        Tank4_Status = "OK";
      } 
      else if (Tank4Value > Tank4Value_High) {
        Tank4_Status = "HIGH";
      }
      if (tanknum == 2){
        send54 = true; // dont send data to mqtt unless both tanks have been read
      }

    }
    void proc_tank1 (){

      if (tank1_data_available & DISTANCE) {
          tank1Distance = tank1.get_distance();
          // Serial.print("Distance to Water Tank 1: ");
          // Serial.print(tank1Distance);
          // Serial.print("  ");
          tank1Distance = tank1Distance / 25.4;
          Read_HCSR04(1);      
          // Serial.println(tank1Distance);
      }

      if (tank1_data_available & TEMPERATURE) {
          // Serial.print("Airtemp Tank 1: ");
          tank1Temperature = tank1.get_temperature();
          tank1Temperature = ((9.0 / 5.0) * (tank1Temperature) + 32.0);
          // Serial.print("Airtemp Tank 1: ");
          // Serial.println(tank1Temperature);
      }

    }
    void proc_tank2 (){
      if (tank2_data_available & DISTANCE) {
          tank2Distance = tank2.get_distance();    
          // Serial.print("Distance to Water Tank 2: ");
          // Serial.print(tank2Distance);
          // Serial.print("  ");
          tank2Distance = tank2Distance / 25.4;
          Read_HCSR04(2);            
          // Serial.println(tank2Distance);
      }
      if (tank2_data_available & TEMPERATURE) {
          tank2Temperature = tank2.get_temperature();
          tank2Temperature = ((9.0 / 5.0) * ( tank2Temperature) + 32.0);    
          // Serial.print("Airtemp Tank 2: ");
          // Serial.println(tank2Temperature);
      }
    }
  #endif




  #ifdef MBUS_BASE 
    void preTransmission(){
      uint32_t t3_5 = (1000000 * 39) / 9600 + 500; // +500us : to be safe
      delayMicroseconds(t3_5);
      digitalWrite(MAX485_RE_NEG, 1);
      digitalWrite(MAX485_DE, 1);
    }
    void postTransmission(){
      digitalWrite(MAX485_RE_NEG, 0);
      digitalWrite(MAX485_DE, 0);
      Serial2.flush();
    }
  #endif 
 
  #ifdef MBUS_WIND
    void calc_wind_max(){
      cur_wind_speed = node1_data[0];    //  mph
      cur_wind_gear = node2_data[0];     //  0-7
      cur_wind_dir = node2_data[1];      //  0-359
      if (max_wind_speed < cur_wind_speed){
        max_wind_speed = cur_wind_speed;
        max_wind_dir = cur_wind_dir;
      }
      send51 = true;
    }
    void node1_rtu_wind_speed(){
      uint8_t j, result;
      node.setSlaveId(Node_1);
      result = node.readHoldingRegisters(0, 1);
      // Serial.print("Node_1 Result: ");
      // Serial.println(result);    
      if (result == node.ku8MBSuccess)
      {
        // Serial.print("Node 1: ");
        for (j = 0; j < 1; j++)
        {
          node1_data[j] = node.getResponseBuffer(j);
          node1_data[j] = ((node1_data[j] / 10) / 0.44704);         
          // Serial.print(node1_data[j]);
          // Serial.print(":");
          // Serial.print(node1_data[j]);
          // Serial.print(" ");
        }
        // Serial.print(" ");
        // Serial.print("EC: ");
        // Serial.print(errorcnt);
        // Serial.print(" ");        
        // Serial.print("CY: ");
        // Serial.println(cycle);
        // Serial.println();  
        cycle++;
      }
      else {
        errorcnt++;
        cycle++;
        Serial.print("Node_1 ER: ");
        Serial.print(result, HEX);
        Serial.print(" ");
        Serial.print("EC: ");
        Serial.println(errorcnt);
      }
    }
    void node2_rtu_wind_dir(){
      uint8_t j, result;
      node.setSlaveId(Node_2);
      // slave: read (8) 16-bit registers starting at register 0 to RX buffer
      result = node.readHoldingRegisters(0, 2);
      // Serial.print("Node_2 Result: ");
      // Serial.println(result);
      // do something with data if read is successful
      if (result == node.ku8MBSuccess)
      {     
        for (j = 0; j < 2; j++)
        {
          node2_data[j] = node.getResponseBuffer(j);    
          //Serial.print(node2_data[j]);
          // Serial.print(":");
          // Serial.print(data[j]);
          //Serial.print(" ");
        }
        // Serial.print(" ");
        //Serial.print("EC: ");
        //Serial.print(errorcnt);
        //Serial.print(" ");        
        //Serial.print("CY: ");
        //Serial.println(cycle);
        //Serial.println(); 
        cycle++;
        // send50=true;           
      }
      else {
        errorcnt++;
        cycle++;
        Serial.print("Node_2 ER: ");
        Serial.print(result);
        Serial.print(" ");      
        Serial.print("EC: ");
        Serial.println(errorcnt);
      }

    }
  #endif

  #ifdef MBUS_OAQ
    void node3_rtu_oaq(){
      uint8_t j, result;
      node.setSlaveId(Node_3);
      // slave: read (8) 16-bit registers starting at register 0 to RX buffer
      result = node.readHoldingRegisters(0, 2);
      #ifdef MQTT_NODEFUNC_DEBUG
        Serial.println("*****************************************");
        Serial.print("Node_3 Result: ");
        Serial.print(result, HEX);
        Serial.print(" : ");        
      #endif
      if (result == node.ku8MBSuccess)
      {     
        for (j = 0; j < 2; j++)
        {
          node3_data[j] = node.getResponseBuffer(j);
          #ifdef MQTT_NODEFUNC_DEBUG  
            Serial.print(node3_data[j]);
            if (j < 1){
              Serial.print(", ");
            }
          #endif
        }
        pm2510ug_output_pm25 = node3_data[0];
        pm2510ug_output_pm10 = node3_data[1];
        #ifdef MQTT_NODEFUNC_DEBUG     
          Serial.print(" -- ");
          Serial.print("EC: ");
          Serial.print(errorcnt);
          Serial.print(" ");        
          Serial.print("CY: ");
          Serial.println(cycle);
          Serial.println();
        #endif 
        cycle++;
        // send50=true;   
      }
      else {
        errorcnt++;
        cycle++;
        Serial.print("Node_3 ER: ");
        Serial.print(result, HEX);
        Serial.print(" ");
        Serial.print("EC: ");
        Serial.println(errorcnt);
      }
    }
  #endif

  #ifdef MBUS_WPSI
    void node4_rtu_wpsi(){
      uint8_t j, result;
      node.setSlaveId(Node_4);
      // slave: read (8) 16-bit registers starting at register 0 to RX buffer
      result = node.readHoldingRegisters(0, 7);
      if (result == 226){
        delay(10);
        result = node.readHoldingRegisters(0, 7);
      }
      #ifdef MQTT_NODEFUNC_DEBUG
        Serial.println("******************************************");
        Serial.print("Node_4 Result: ");
        Serial.print(result, HEX);
        Serial.print(" : ");
      #endif
      if (result == node.ku8MBSuccess)
      {     
        for (j = 0; j < 8; j++)
        {
          node4_data[j] = node.getResponseBuffer(j);    
          #ifdef MQTT_NODEFUNC_DEBUG   
            Serial.print(node4_data[j]);
            if (j < 7){
              Serial.print(", ");
            }  
          #endif
        }
        wpsi_addr = node4_data[0];
        wpsi_baud = node4_data[1];
        wpsi_unit = node4_data[2];
        wpsi_dp = node4_data[3];
        wpsi_output_mpa = (node4_data[4]);
        wpsi_minRange = node4_data[5];
        wpsi_maxRange = node4_data[6];

        wpsi_output_psi = wpsi_output_mpa * 145.03773773;                  
     
        #ifdef MQTT_NODEFUNC_DEBUG     
          Serial.print(" -- ");
          Serial.print("EC: ");
          Serial.print(errorcnt);
          Serial.print(" ");        
          Serial.print("CY: ");
          Serial.println(cycle);
          Serial.println();
        #endif  
        cycle++;
        // send50=true;           
      }
      else {
        errorcnt++;
        cycle++;
        Serial.print("Node_4 ER: ");
        Serial.print(result, HEX);
        Serial.print(" ");      
        Serial.print("EC: ");
        Serial.println(errorcnt);
      }  
    }
  #endif

  #ifdef MBUS_RAIN

    void node5_rtu_rain(){
      uint8_t j, result;
      node.setSlaveId(Node_5);
      // slave: read (8) 16-bit registers starting at register 0 to RX buffer
      result = node.readHoldingRegisters(0, 6);
      if (result == 224 || result == 226){
        delay(10);
        result = node.readHoldingRegisters(0, 10);
      }      
      #ifdef MQTT_NODEFUNC_DEBUG
        Serial.println("*********************************************");
        Serial.print("Node_5 Result: ");
        Serial.print(result, HEX);
        Serial.print(" : ");
      #endif
      if (result == node.ku8MBSuccess)
      {     
        for (j = 0; j < 10; j++)
        {
          node5_data[j] = node.getResponseBuffer(j);    
          #ifdef MQTT_NODEFUNC_DEBUG
            if (j < 7){
              Serial.print(node5_data[j]);
              Serial.print(", ");
            }
            else if (j == 7 || j == 9){
                Serial.print(node5_data[j], HEX);
                if (j == 7){
                  Serial.print(", ");
                }                
              }
            else if(j == 8){
              Serial.print(node5_data[j]);
              Serial.print(", ");
            }
          #endif
        } 
        #ifdef MQTT_NODEFUNC_DEBUG     
          Serial.print(" -- ");
          Serial.print("EC: ");
          Serial.print(errorcnt);
          Serial.print(" ");        
          Serial.print("CY: ");
          Serial.println(cycle);
          // Serial.println();
        #endif

        DayRain = node5_data[0] / 10;
        InstRain = node5_data[1] / 10;
        YesterdayRain = node5_data[2] / 10;
        SensorTotalRain = node5_data[3] / 10;
        HourlyRainfall = node5_data[4] / 10;
        LastHourRainfall = node5_data[5] / 10;  
        Max24Rain = node5_data[6] / 10;
        Max24Rain_TimeStart = highByte(node5_time[7]);
        Max24Rain_TimeEnd = lowByte(node5_time[7]);
        Min24Rain = node5_data[8] / 10;
        Min24Rain_TimeStart = highByte(node5_time[9]);
        Min24Rain_TimeEnd = lowByte(node5_time[9]);     
        cycle++;
        // send50=true;   
      }         
      else {
        errorcnt++;
        cycle++;
        Serial.print("Node_5 ER: ");
        Serial.print(result, HEX);
        Serial.print(" ");      
        Serial.print("EC: ");
        Serial.println(errorcnt);
      }

    }

    void node5_rtu_time(){
      uint8_t j, result;
      node.setSlaveId(Node_5);
      // slave: read (8) 16-bit registers starting at register 0 to RX buffer
      result = node.readHoldingRegisters(52, 3);
      if (result == 224 || result == 226){
        delay(100);
        result = node.readHoldingRegisters(52, 3);
      }      
      #ifdef MQTT_NODEFUNC_DEBUG
        Serial.println("********************************************");
        Serial.print("Node_5 Result: ");
        Serial.print(result, HEX);
        Serial.print(" : ");
      #endif
      if (result == node.ku8MBSuccess)
      {     
        for (j = 0; j < 3; j++)
        {
          node5_time[j] = node.getResponseBuffer(j);    
          #ifdef MQTT_NODEFUNC_DEBUG
            Serial.print(node5_time[j], HEX);
            if (j < 2){
              Serial.print(", ");
            }  
          #endif
        }
        #ifdef MQTT_NODEFUNC_DEBUG     
          Serial.print(" -- ");
          Serial.print("EC: ");
          Serial.print(errorcnt);
          Serial.print(" ");        
          Serial.print("CY: ");
          Serial.println(cycle);
          // Serial.println();
        #endif
        RainClockYear = highByte(node5_time[0]);
        RainClockMonth = lowByte(node5_time[0]);
        RainClockDay = highByte(node5_time[1]);
        RainClockHour = lowByte(node5_time[1]);
        RainClockMin = highByte(node5_time[2]);
        RainClockSecond = lowByte(node5_time[2]);
        // Serial.println();
        Serial.print("Rain Sensor Time: ");
        Serial.print(RainClockYear, HEX);
        Serial.print("-");
        Serial.print(RainClockMonth, HEX);
        Serial.print("-");
        Serial.print(RainClockDay, HEX);
        Serial.print(" ");
        Serial.print(RainClockHour, HEX);
        Serial.print(":");
        Serial.print(RainClockMin, HEX);
        Serial.print(":");
        Serial.println(RainClockSecond, HEX);        
        cycle++;
        // send50=true;   
      }         
      else {
        errorcnt++;
        cycle++;
        Serial.print("Node_5 ER: ");
        Serial.print(result, HEX);
        Serial.print(" ");      
        Serial.print("EC: ");
        Serial.println(errorcnt);
        Serial.println();
      }

    }
  #endif






