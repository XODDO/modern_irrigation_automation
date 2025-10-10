#include <Arduino.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_GFX.h>

#include <GxEPD2_BW.h>
#include <GxEPD2_3C.h>
#include <Fonts/FreeMonoBold9pt7b.h>
#include <Fonts/FreeSansBold24pt7b.h> // FreeSerifBold24pt7b
#include <Fonts/FreeMonoOblique18pt7b.h>
#include <Fonts/FreeSans12pt7b.h>
#include <Fonts/FreeSans9pt7b.h>
#include<Fonts/FreeSansBold18pt7b.h>

#include "esp_timer.h"
#include "esp_wifi.h"
#include <esp_now.h>
#include <WiFi.h>

#include "Battery.h"
#include "Buzzer.h"

#include "esp_sleep.h"
#include "driver/gpio.h"

// Peer info 
esp_now_peer_info_t peerInfo;

uint8_t broadcastAddress[] = {0xF4, 0x65, 0x0B, 0xE8, 0xC6, 0x30}; // MUARIK CURVED LARGE F4:65:0B:E8:C6:30

StaticJsonDocument<256> JSON_data; //100 bytes is very tight. With multiple floats + strings you’ll overflow.
//JsonDocument JSON_data;

//
Buzzer buzzer(14);   //uint8_t buzzer = 14;     // buzzer pin
Battery batt(35, 8.45, 1);         // pin, scaling factor, num batteries

uint8_t high = 25; // green
uint8_t medium = 27; //orange
uint8_t low   = 26;//red
uint8_t indicator = 2;


uint8_t sensorPin = 36; uint8_t soilTempPin = 39;
uint8_t solarPin = 34; //uint8_t batteryPin = 35;
uint8_t sensorRelay = 33;

// base class GxEPD2_GFX can be used to pass references or pointers to the display instance as parameter, uses ~1.2k more code
// enable or disable GxEPD2_GFX base class
#define ENABLE_GxEPD2_GFX 0

// ESP32 CS(SS)=5,SCL(SCK)=18,SDA(MOSI)=23,BUSY=15,RES(RST)=2,DC=0

// 1.54'' EPD Module
   GxEPD2_BW<GxEPD2_154_D67, GxEPD2_154_D67::HEIGHT> LCD(GxEPD2_154_D67(/*CS=5*/ 5, /*DC=*/ 4, /*RES=*/ 13, /*BUSY=*/ 15)); // GDEH0154D67 200x200, SSD1681
// 1.54'' EPD Module (3-Color: B/W/R)
// GxEPD2_3C<GxEPD2_154_D67, GxEPD2_154_D67::HEIGHT> LCD(GxEPD2_154_D67(/*CS=*/5, /*DC=*/4, /*RES=*/13, /*BUSY=*/15)); // GDEW0154Z17 200x200, SSD1681

bool battery_low_registered = false;
// ===== Callbacks =====
void beepLow() {
  if(!battery_low_registered) {
      Serial.println("Battery Critically low");
      buzzer.beep(2, 800, 500);
      battery_low_registered = true;
  }
}

void recovered() {
  if(battery_low_registered) {
      Serial.println("Battery recovered above 7.2V");
      battery_low_registered = false;
  }
}

// ===== FreeRTOS Tasks =====
void TaskFastPoll(void *pvParams) {
  // ~10ms periodic: buttons, buzzer
  const TickType_t buzzDelay = pdMS_TO_TICKS(10);
  while (1) {
    buzzer.update();
    //buttons.update();
    vTaskDelay(buzzDelay);
  }
}

bool enable_light_sleep = false;
bool enable_deep_sleep = false;

// in low power: wake up once every hour and check battery and return to deep sleep, in light wake up every 15mins, update readings, in no sleep, never sleep
enum SLEEPMODE {LOW_POWER, MODERATE, EXCELLENT_MODE};
SLEEPMODE sleepMode = LOW_POWER;
char batt_str[12] = ".";
//modify sleep modes according to available battery power
void TaskBattery(void *pvParams) {
  // ~1s periodic: battery read
  const TickType_t xDelay = pdMS_TO_TICKS(5*60*1000); // each 5 mins ... but when not in deep sleep
  while (1) {
    batt.update();
    dtostrf(batt.getVoltage(), 3, 1, batt_str); //strncat(batt_str, "V", 2); //  //BATTERY LOGIC // dtostrf(voltage, 3, 1, batt_str); 
            
    Serial.printf("Voltage: %s\n", batt_str);

    if(batt.getVoltage() <= 2*3.6){ sleepMode = LOW_POWER; Serial.println("Sleep Mode: LOW POWER"); }
    else if(batt.getVoltage() > 2*3.6 && batt.getVoltage() <= 2*3.9){
          sleepMode = MODERATE; Serial.println("Sleep Mode: MODERATE POWER"); 
    } 
    else {// batt.getVoltage() > 2*3.9 
          sleepMode = EXCELLENT_MODE; Serial.println("Sleep Mode: EXCELLENT POWER"); 
    }

    if (batt.isLow()) { Serial.println("Battery LOW (with hysteresis)"); sleepMode = LOW_POWER; }
    
    vTaskDelay(xDelay);
  }
}
 const char SensorAddress[10] = "MUARIK_1"; /// GAYAZA, WAKISO";
 const char sensorID2[20] = "IRRI-KIT, MUARIK 1";


 const char DeviceID[60] = "Field Wireless Soil Moisture Sensor"; 


// GLOBAL VARIABLES TO BE USED
float soilPercent = 0.00f;  float soilTemp = 0.00f;   const uint8_t wiltingPt = 15;    const uint8_t fieldCapacity = 40; // for 100% 

char soil_Moi[8] = ".";
char soil_Tem[8] = ".";

char batt_volts[7] = ".";
char soil_moisture_char[7] = "x.x";
char soil_temp_char[7] = "x.x";


void sense_reading();
float soil_moisture_calibrator(uint16_t working_reading);


typedef struct moistureData{ // all stringified
      char sendable_data_bundle[200] = "";
   }  moistureData; 

moistureData senderObject;

uint8_t currentScreen = 1;
uint32_t update_frequency = 30; // 15 or 30 seconds when power is high, 15mins when moderate, 1 hour if low

void setup(){
      Serial.begin(115200); Serial.println("\nBooting....");
      buzzer.begin(); //pinMode(buzzer, OUTPUT);
  
  //  POWER MANAGEMENT   
      batt.begin(); //pinMode(batteryPin, INPUT);
         
 
  // Tasks
      xTaskCreatePinnedToCore(TaskFastPoll, "FastPoll", 2048, NULL, 1, NULL, 1);
      Serial.println("\tBuzzer Task initialized!");

      buzzer.beep(1,50,0);

      LCD.init(115200, true, 50, false);

  //THE KEY COMPONENT
      pinMode(sensorPin, INPUT); pinMode(soilTempPin, INPUT); Serial.println("Sensor initialized!"); delay(100);
      pinMode(sensorRelay, OUTPUT); digitalWrite(sensorRelay, HIGH); Serial.println("Sensor Relato Toggled!");

    //INDICATORS
      pinMode(indicator, OUTPUT);  digitalWrite(indicator, HIGH);
      pinMode(low, OUTPUT); digitalWrite(low, HIGH); delay(500);
      pinMode(medium, OUTPUT); digitalWrite(medium, HIGH); delay(500);
      pinMode(high, OUTPUT); digitalWrite(high, HIGH); delay(500);
      Serial.println("Indicators initialized!"); delay(100);

 

      BootScreen();
      delay(2000);
      Serial.println("Screen Booted!");
      
      batt.onLow(beepLow);
      batt.onRecovered(recovered);

      Serial.println("Checking Battery Voltage...");
      MonitorBattery();
      
    //  xTaskCreatePinnedToCore(TaskBattery, "Battery_Check", 4096, NULL, 1, NULL, 1);
      
      Serial.println("Selecting Sleep Mode based on battery voltage...");
      // set sleep mode
      switch(sleepMode){ // {LOW_POWER, MODERATE, EXCELLENT_MODE}
        case LOW_POWER: enable_deep_sleep = true; enable_light_sleep = false; 
        break;

        case MODERATE: enable_deep_sleep = false; enable_light_sleep = true; 
        break;
        
        case EXCELLENT_MODE: enable_deep_sleep = false; enable_light_sleep = false; 
        break;
        

      }

     
    //if too low, go to sleep and display Shut down due to Low Power, onl;y awake after an hour, run set up and on reaching battery low, go back to sleep

    //if moderate, wake up and do all work once every 15 mins   

    // if excellent, never sleep, keep logging data once every 5 minutes


      power_Screen(); // UPDATE DISPLAY
      delay(1000);

      LCD.hibernate();
    

    // if power is high enough, proceed to turn on WiFi
        WiFi.begin();
        WiFi.mode(WIFI_STA);

      // Initilize ESP-NOW
      if (esp_now_init() != ESP_OK) {
           Serial.println("Error initializing ESP-NOW");
          //  return;
      }

      else { Serial.println("ESP-NOW intialized successfully");

              // Register SEND callback
              esp_now_register_send_cb(OnDataSent);
              
              // Register peer
              memcpy(peerInfo.peer_addr, broadcastAddress, 6);
              peerInfo.channel = 0;  
              peerInfo.encrypt = false;
              
              // Add peer        
              if (esp_now_add_peer(&peerInfo) != ESP_OK){
                  Serial.println("Failed to add peer");
              //  return;
              } else Serial.println("Peer Added Successfully!");

      }
      delay(1000);

/*
     if(enable_deep_sleep){
        Serial.println("\tDEEP SLEEPING for 1 hour...");
        esp_sleep_enable_timer_wakeup(60ULL * 1000000ULL); // 3600ULL * 1000000ULL // 1 hour in µs
        esp_deep_sleep_start();
        Serial.println("Entered Deep Sleep"); // this line should never be reached
      }
      if(enable_light_sleep){
          Serial.println("\tLIGHT SLEEPING for 15 minutes...");
          esp_sleep_enable_timer_wakeup(30ULL * 1000000ULL); // 900ULL * 1000000ULL // 15 min in µs
          esp_light_sleep_start();
            // After wake: execution resumes here
          Serial.println("Woke up from LIGHT SLEEP");
      }
      if(!enable_light_sleep && !enable_deep_sleep) {
        Serial.println("No Sleep Mode.");
        Serial.print("Loop refresh frequency set to "); Serial.println(update_frequency);
      }
  */        

     // core jobs
        sense_reading();
        soilMoi_Screen();
        LCD.hibernate();
        buzzer.beep(2, 50, 50);


}


uint16_t delvs = 0;
bool successfully_delivered = false;

void OnDataSent(const wifi_tx_info_t *mac_addr, esp_now_send_status_t status) {
  /*
    // You can still access the peer MAC address like this:
    char macStr[18];
    snprintf(macStr, sizeof(macStr),
             "%02X:%02X:%02X:%02X:%02X:%02X",
             mac_addr->broadcastAddress[0], mac_addr->broadcastAddress[1], mac_addr->broadcastAddress[2],
             mac_addr->broadcastAddress[3], mac_addr->broadcastAddress[4], mac_addr->broadcastAddress[5]);

    Serial.print("Last Packet Sent to: ");
    Serial.println(macStr);
  */
    if (status == ESP_NOW_SEND_SUCCESS) {
        successfully_delivered = true;    delvs++;
        // LONGISH buzzer chirp
         buzzer.beep(1, 100, 50);
    } else {
        successfully_delivered = false;
    }
}



uint64_t now_now = 0, prev = 0;
void loop() {
    now_now = esp_timer_get_time()/1000000; // each sekonda
   
   if((now_now - prev) >= update_frequency){ // once each 15 seconds 

            sense_reading();
            MonitorBattery();       
            send_as_JSON();
            Serial.println(); Serial.printf("Time check: %llu\n", now_now);
            update_display();

 /*
   if(enable_deep_sleep){
        Serial.println("\tDEEP SLEEPING for 1 hour...");
        esp_sleep_enable_timer_wakeup(60ULL * 1000000ULL); // 3600ULL * 1000000ULL // 1 hour in µs
        esp_deep_sleep_start();
        Serial.println("Entered Deep Sleep"); // this line should never be reached
      }
      if(enable_light_sleep){
          Serial.println("\tLIGHT SLEEPING for 15 minutes...");
          esp_sleep_enable_timer_wakeup(30ULL * 1000000ULL); // 900ULL * 1000000ULL // 15 min in µs
          esp_light_sleep_start();
            // After wake: execution resumes here
          Serial.println("Woke up from LIGHT SLEEP");
      }
      if(!enable_light_sleep && !enable_deep_sleep) {
        Serial.println("No Sleep Mode.");
        Serial.print("Loop refresh frequency set to "); Serial.println(update_frequency);
      }
 */
        prev = now_now; 
    }


}



void MonitorBattery(){
    batt.update();
    dtostrf(batt.getVoltage(), 3, 1, batt_str); //strncat(batt_str, "V", 2); //  //BATTERY LOGIC // dtostrf(voltage, 3, 1, batt_str); 
            
    Serial.printf("Voltage: %s\n", batt_str);

    if(batt.getVoltage() <= 2*3.6){ sleepMode = LOW_POWER; Serial.println("Sleep Mode: LOW POWER"); }
    else if(batt.getVoltage() > 2*3.6 && batt.getVoltage() <= 2*3.9){
          sleepMode = MODERATE; Serial.println("Sleep Mode: MODERATE POWER"); 
    } 
    else {// batt.getVoltage() > 2*3.9 
          sleepMode = EXCELLENT_MODE; Serial.println("Sleep Mode: EXCELLENT POWER"); 
    }

    if (batt.isLow()) { Serial.println("Battery LOW (with hysteresis)"); sleepMode = LOW_POWER; }
    
   // vTaskDelay(const TickType_t xDelay = pdMS_TO_TICKS(5*60*1000););
  
}

uint64_t sent_bytes = 1;
char packets_sent[12] = ""; 

uint64_t sends = 0; // 0 TO 4BILLION
char transmissions[12] = ""; //10 digit value: 1Million sends over | many years

// Data gonna be like = {SensorID1:"SensorA1", "Moisture1":25.87, Temperature1:32.87, Voltage: 8.2, Sends:256718};
const char sensor_position[10] = "Sensor A1";
bool sent_successfully = false;
bool send_as_JSON(){ // bundle of JOY

  sent_successfully = false;
  JSON_data.clear();   // removes all keys/values, resets capacity usage
  // Assign values
  JSON_data["Position1"]     = sensor_position; // string
  JSON_data["Moisture1"]     = soilPercent;   // keep as float // hopefully at 2dp
  JSON_data["Temperature1"]  = soilTemp;      // keep as float // 2 decimal places
  JSON_data["Voltage1"]      = batt.getVoltage();       // keep as float
  JSON_data["Sends1"]        = sends;         // very long integer

  // Serialize with controlled float precision
  serializeJsonPretty(JSON_data, Serial);   // optional: preview

  // Write to buffer with limited decimals
  serializeJson(JSON_data, senderObject.sendable_data_bundle); /*
  serializeJson(JSON_data, senderObject.sendable_data_bundle, sizeof(senderObject.sendable_data_bundle),   [](Print& p, double val) { p.print(val, 2); });    // This custom "serializer" forces 2 decimal places for floats
  //ArduinoJson::serializeJson(JSON_data, senderObject.sendable_data_bundle, ArduinoJson::SerializationOptions::withDecimals(2));
*/
  sent_bytes = measureJson(JSON_data);
  // Send via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t*)&senderObject, sizeof(senderObject));
    

  if (result == ESP_OK) {
      sent_successfully = true;
      sends++;
      ltoa(sends, transmissions, 10);
      Serial.println(); Serial.print(sent_bytes); Serial.println(" Serialized JSON Bytes sent");
  } else {
      Serial.println("Send Failed!");
  }

  if (sent_successfully) {
    //Serial.print(senderObject.sendable_data_bundle);
    Serial.println(" Sent");
  }
    
  
  return sent_successfully;


}


     uint32_t movin_avg = 0;

     float newMax = 0.00, newMin = 0.00;

     float maxRange = 0.00;
     float variance = 0.00; 
char ShortTime[32] = ".";
char last_soil_moisture[10] = "x.x%";
char last_recording_time[12] = "21:45:00";

float recent_soil_data[6] = {0.00, 0.00, 0.00, 0.00, 0.00}; //6 slots ...but ... 5 readings
char  recent_soil_moistures[6][8] = {"XX", "YY", "ZZ"}; //6 slots for 5 most recent readings e.g. 10.4%
char  recent_soil_times[6][8] = {"xx:xx", "yy:yy", "10:35"}; //5 sets of seven each 10:35


void sense_reading(){

     Serial.println("Now Scanning the Soil..."); Serial.println();
         
     digitalWrite(sensorRelay, HIGH); 
      
     
      uint16_t lowest = 4090; // pull it so high so that it is always pushed downwards to the smallest soil moisture reading 
      uint16_t highest = 0;
      maxRange = 0.00;

      uint16_t readings = 0;

      uint16_t _12bit_val = 0; uint16_t soilTemp_raw = 0;
      movin_avg = 0;
     
      float readings_duration = 0.00; float reading_period = 0.0;

      
      int64_t start_reading = esp_timer_get_time(); // returns microseconds

 while(readings < 500){ readings++; // ~13,000-16,000 readings in 1 sec without any delay
    
                delayMicroseconds(500); // to stretch 100 reads over a half second reading duration

                //vTaskDelay(5/portTICK_PERIOD_MS);
                  _12bit_val = analogRead(sensorPin);
                if(_12bit_val > highest) highest = _12bit_val;
                if(_12bit_val < lowest) lowest   = _12bit_val;
                
                //noise_councillor();
                movin_avg += _12bit_val;
              /*  
                  temp_average   = float(total) / float(reading);
                  temp_variance = temp_variance + ((sensorVal - temp_average)*(sensorVal - temp_average));
              */
            // Serial.print("Counter: "); Serial.print(readings); Serial.print(" Reading = "); Serial.println(_12bit_val);

          }


     int64_t stop_reading = esp_timer_get_time();
     readings_duration = float(stop_reading - start_reading)/1000.0; // by a million
     reading_period    = (readings_duration/float(readings));
     
       
       movin_avg /= (readings); 
      
       soilPercent = soil_moisture_calibrator(movin_avg); //the algorithm for soil moisture
    
       soilTemp_raw = analogRead(soilTempPin); Serial.print("Raw Temp Reading: "); Serial.println(soilTemp_raw);
       soilTemp = ((soilTemp_raw)/(4095.0-0.0) * (40.0 + 80.0)); // measurement range: -40 to 80deg
      // if(soilTemp > 32.0) soilTemp = (soilTemp - 32.0) * 5/9.0;
       soilTemp -= 40.0; 

    //RECORD THE READING  & TIME
      dtostrf(soilPercent, 3, 1, soil_moisture_char); // strcat(last_soil_moisture, "%");
      dtostrf(soilTemp, 3, 1, soil_temp_char);  
      strcpy(last_recording_time, ShortTime); 
      

          maxRange = (newMax - newMin); // spike identification

          
       if(soilPercent <= 15.0){

         digitalWrite(low, HIGH);    delay(50);
         digitalWrite(medium, LOW); delay(50);
         digitalWrite(high, LOW);   delay(50);
    
       }

       else if(soilPercent <= 25.0){ // if 15 - 25
         digitalWrite(low, LOW);    delay(50);
         digitalWrite(medium, HIGH); delay(50);
         digitalWrite(high, LOW);   delay(50);

       }
       else { // if > 25
         digitalWrite(low, LOW);    delay(50);
         digitalWrite(medium, LOW); delay(50);
         digitalWrite(high, HIGH);   delay(50);

       }
    

     digitalWrite(sensorRelay, LOW); buzzer.beep(1,50,0); // TO SIGNIFY END OF READING

     Serial.print("Reading Duration: ");    Serial.print(readings_duration, 4); Serial.println(" milliseconds");
     Serial.print("Soil Temperatures: ");   Serial.println(soilTemp);
     Serial.print("Period of a reading: "); Serial.print(reading_period, 6);    Serial.println(" milliseconds");
    
                  Serial.print("Last Direct: "); Serial.println(_12bit_val);
                  Serial.print("Readings: "); Serial.println(readings);
                  Serial.print("Highest Reading: "); Serial.println(highest);
                  Serial.print("Lowest Reading: ");  Serial.println(lowest);
                  Serial.print("Range: "); Serial.println(highest - lowest); 
  if(highest > 0) { Serial.print("Percent Error: "); Serial.println(((highest - lowest)*100)/highest); }
                  Serial.print("Soil Moisture: "); Serial.print(soilPercent); Serial.println("%");
                  Serial.println();

}


  //   const float reading_range = 2800.0; // 50 - 2850
       const float reading_range = 4095.0; // 0 - 4095

float soil_moisture_calibrator(uint16_t working_reading){ Serial.println("Running Calibrator..."); delay(50);

      float refined_val = (float)working_reading; Serial.print("Computed Average: "); Serial.println(working_reading);
            refined_val = refined_val / float(reading_range); Serial.print("Semi Refined:"); Serial.println(refined_val); // min => 5, max => 4000
        if(refined_val <= 0.90)  { refined_val = refined_val * fieldCapacity; } //making 40 to be the highest possible reading ... 
      else refined_val = refined_val * 100.0; //showing the 100%
          
     return refined_val;
}



void update_display(){
    if(currentScreen == 1) soilMoi_Screen();
    if(currentScreen == 2) dataUpload_Screen();
}

void dataUpload_Screen(){

}

void power_Screen(){
  
  LCD.firstPage();
  do {
    LCD.setRotation(2);
    LCD.fillScreen(GxEPD_WHITE);
    
    LCD.setTextColor(LCD.epd2.hasColor ? GxEPD_RED : GxEPD_BLACK);
    LCD.setFont(&FreeSans12pt7b);
    LCD.setCursor(50, 120); LCD.print("Shutdown");
    //    batt_str
    LCD.fillRoundRect(30, 20, 140, 60, 30, GxEPD_BLACK);
    LCD.fillRoundRect(35, 25, 130, 50, 30, GxEPD_WHITE);
    LCD.fillCircle(60, 50, 22, GxEPD_BLACK);

    LCD.setFont(&FreeSansBold18pt7b); LCD.setCursor(86, 64); LCD.print("OFF"); //FreeSansBold18pt7b
    //LCD.fillCircle(100, 40, 25, GxEPD_BLACK);
    //LCD.fillCircle(100, 40, 18, GxEPD_WHITE);

        //BATTERY
    LCD.fillRoundRect(50, 144, 88, 40, 5, GxEPD_BLACK); LCD.fillRoundRect(54, 148, 80, 32, 5, GxEPD_WHITE); LCD.fillRect(45, 155, 5, 20, GxEPD_BLACK);  
    LCD.setCursor(60, 175); LCD.print(batt.getVoltage()); 
    
  } while (LCD.nextPage());
}

const char HelloWorld[] = "AgroSense Wireless";
const char HelloWeACtStudio[] = "IntelliSys Uganda";


void BootScreen(){ Serial.println("Starting Display...");
  LCD.setRotation(1);
  LCD.setFont(&FreeMonoBold9pt7b);
  LCD.setTextColor(GxEPD_BLACK);
  int16_t tbx, tby; uint16_t tbw, tbh;
  LCD.getTextBounds(HelloWorld, 0, 0, &tbx, &tby, &tbw, &tbh);
  // center the bounding box by transposition of the origin:
  uint16_t x = ((LCD.width() - tbw) / 2) - tbx;
  uint16_t y = ((LCD.height() - tbh) / 2) - tby;
  LCD.setFullWindow();
  LCD.firstPage();
  do
  {
    LCD.fillScreen(GxEPD_WHITE);
    LCD.setCursor(x, y-tbh);
    LCD.print(HelloWorld);
    LCD.setTextColor(LCD.epd2.hasColor ? GxEPD_RED : GxEPD_BLACK);
    LCD.getTextBounds(HelloWeACtStudio, 0, 0, &tbx, &tby, &tbw, &tbh);
    x = ((LCD.width() - tbw) / 2) - tbx;
    LCD.setCursor(x, y+tbh);
    LCD.print(HelloWeACtStudio);
  }
  while (LCD.nextPage());
}

char moi_char[10] = "23";

char device_name[25] = "Soil Moisture (VWC)";

void soilMoi_Screen(){
    LCD.setRotation(2);
    LCD.setFont(&FreeMonoBold9pt7b);

  // do this outside of the loop
  int16_t tbx, tby; uint16_t tbw, tbh;

  // center the heading
  LCD.getTextBounds(device_name, 0, 0, &tbx, &tby, &tbw, &tbh);
  uint16_t header_x = ((LCD.width() - tbw) / 2) - tbx;
  uint16_t header_y = ((LCD.height() / 4) - tbh / 2) - tby;

  // center the reading
  LCD.getTextBounds(moi_char, 0, 0, &tbx, &tby, &tbw, &tbh);
  uint16_t moi_x = ((LCD.width() - tbw) / 2) - tbx;
  uint16_t moi_y = ((LCD.height() * 3 / 5) - tbh / 2) - tby;


  LCD.firstPage();
  do {
        LCD.fillScreen(GxEPD_WHITE);
        LCD.setTextSize(1);

      //  HEADER
          LCD.setFont(&FreeSans9pt7b);
          LCD.setCursor(header_x+20, header_y-20); LCD.print(device_name); 
        
      //  BODY
          LCD.setTextColor(GxEPD_BLACK);  LCD.setCursor((moi_x+70), (moi_y-30));  LCD.println("%");
        
          LCD.setFont(&FreeSansBold24pt7b); // FreeSerifBold24pt7b // FreeSansBold24pt7b
          LCD.setTextSize(2);   LCD.setCursor(moi_x-40, moi_y);  LCD.println(moi_char);

     //   FOOTNOTE with Sent Messages and Battery Level
          footer();
  } while (LCD.nextPage());
 
}


void partial_update_Screen(){
  Serial.println("testing for helloFullScreenPartialMode");
  const char fullscreen[] = "full screen update";
  const char fpm[] = "fast partial mode";
  const char spm[] = "slow partial mode";
  const char npm[] = "no partial mode";
  LCD.setPartialWindow(0, 0, LCD.width(), LCD.height());
  
  LCD.setRotation(2);
  LCD.setFont(&FreeMonoBold9pt7b);
  if (LCD.epd2.WIDTH < 104) LCD.setFont(0);
  LCD.setTextColor(GxEPD_BLACK);
  const char* updatemode;
  if (LCD.epd2.hasFastPartialUpdate)
  {
    updatemode = fpm;
    Serial.println("Screen Has Fast PArtial Update");
  }
  else if (LCD.epd2.hasPartialUpdate)
  {
    updatemode = spm;
    Serial.println("Screen Has  Partial Update");
  }
  else
  {
    updatemode = npm;
    Serial.println("Screen has NO Partial Update");
  }


  // do this outside of the loop
  int16_t tbx, tby; uint16_t tbw, tbh;

  // center the heading
  LCD.getTextBounds(device_name, 0, 0, &tbx, &tby, &tbw, &tbh);
  uint16_t header_x = ((LCD.width() - tbw) / 2) - tbx;
  uint16_t header_y = ((LCD.height() / 4) - tbh / 2) - tby;

  // center the reading
  LCD.getTextBounds(moi_char, 0, 0, &tbx, &tby, &tbw, &tbh);
  uint16_t moi_x = ((LCD.width() - tbw) / 2) - tbx;
  uint16_t moi_y = ((LCD.height() * 3 / 5) - tbh / 2) - tby;




  LCD.firstPage();
  do
  {
        LCD.fillScreen(GxEPD_WHITE);
        LCD.setTextSize(1);

    //  HEADER
        LCD.setFont(&FreeSans9pt7b);
        LCD.setCursor(header_x+20, header_y-20); LCD.print(device_name); 
        
    //  BODY
        LCD.setTextColor(GxEPD_BLACK);  LCD.setCursor((moi_x+70), (moi_y-30));  LCD.println("%");
      
        LCD.setFont(&FreeSansBold24pt7b); // FreeSerifBold24pt7b // FreeSansBold24pt7b
        LCD.setTextSize(2);   LCD.setCursor(moi_x-40, moi_y);  LCD.println(moi_char);

    //  FOOTNOTE with Sent Messages and Battery Level
        footer();
  /*
   
    LCD.print(HelloWorld);
    LCD.setCursor(utx, uty);
    LCD.print(fullscreen);
    LCD.setCursor(umx, umy);
    LCD.print(updatemode);

  */
  }
  while (LCD.nextPage());
  Serial.println("helloFullScreenPartialMode testing done");
}



bool minute_count_passed = false; bool wifi_state = false;

char bytes_char[10] = "";

void footer(){
        //    upward curve
         //   LCD.fillRect(0, 160, 200, 40, GxEPD_WHITE); // GxEPD_BLACK
         //   LCD.fillRoundRect(12, 155, 186, 5, 8, GxEPD_BLACK);
              LCD.drawFastHLine(10, 165, 180, GxEPD_BLACK);
            // LCD.fillCircle(0, 295, 2, GxEPD_BLACK);
            
           
            //NETWORK BARS
            LCD.fillRect(10, 194, 6, 5, GxEPD_BLACK); LCD.fillRect(20, 184, 6, 15, GxEPD_BLACK); LCD.fillRect(30, 173, 6, 26, GxEPD_BLACK);
            
          // DATA UPLOAD ICON
            LCD.fillRect(80, 180, 30, 20, GxEPD_BLACK); LCD.fillRect(84, 180, 22, 14, GxEPD_WHITE);
            LCD.fillRect(93, 176, 4, 15, GxEPD_BLACK);  LCD.fillTriangle(88, 175, 94, 170, 100, 175, GxEPD_BLACK);
       
                 
          //BATTERY
            LCD.fillRoundRect(150, 174, 38, 20, 3, GxEPD_BLACK); LCD.fillRoundRect(154, 178, 30, 12, 3, GxEPD_WHITE); LCD.fillRect(146, 179, 5, 10, GxEPD_BLACK);  

            LCD.setFont(&FreeSans9pt7b); // LCD.setFont(&FreeSans9pt7b);
            LCD.setTextColor(GxEPD_BLACK); 
    
            LCD.setTextSize(1);
            LCD.setCursor(42, 192);  LCD.setFont(); LCD.print(wifi_state?"ON":"OFF"); 

          if(sends > 0)  {  LCD.setFont(); LCD.setCursor(115, 185); LCD.print(sends);   }
           
          if(batt.getVoltage() >= 0.00) {  LCD.setFont(); LCD.setCursor(160, 180); LCD.print(batt.getVoltage()); }
               
}



void showPartialUpdate(){
  // some useful background
  BootScreen();
  // use asymmetric values for test
  uint16_t box_x = 10;
  uint16_t box_y = 15;
  uint16_t box_w = 70;
  uint16_t box_h = 20;
  uint16_t cursor_y = box_y + box_h - 6;
  if (LCD.epd2.WIDTH < 104) cursor_y = box_y + 6;
  float value = 13.95;
  uint16_t incr = LCD.epd2.hasFastPartialUpdate ? 1 : 3;
  LCD.setFont(&FreeMonoBold9pt7b);
  if (LCD.epd2.WIDTH < 104) LCD.setFont(0);
  LCD.setTextColor(GxEPD_BLACK);
  // show where the update box is
  for (uint16_t r = 0; r < 4; r++)
  {
    LCD.setRotation(r);
    LCD.setPartialWindow(box_x, box_y, box_w, box_h);
    LCD.firstPage();
    do
    {
      LCD.fillRect(box_x, box_y, box_w, box_h, GxEPD_BLACK);
      //LCD.fillScreen(GxEPD_BLACK);
    }
    while (LCD.nextPage());
    delay(2000);
    LCD.firstPage();
    do
    {
      LCD.fillRect(box_x, box_y, box_w, box_h, GxEPD_WHITE);
    }
    while (LCD.nextPage());
    delay(1000);
  }
  //return;
  // show updates in the update box
  for (uint16_t r = 0; r < 4; r++)
  {
    LCD.setRotation(r);
    LCD.setPartialWindow(box_x, box_y, box_w, box_h);
    for (uint16_t i = 1; i <= 10; i += incr)
    {
      LCD.firstPage();
      do
      {
        LCD.fillRect(box_x, box_y, box_w, box_h, GxEPD_WHITE);
        LCD.setCursor(box_x, cursor_y);
        LCD.print(value * i, 2);
      }
      while (LCD.nextPage());
      delay(500);
    }
    delay(1000);
    LCD.firstPage();
    do
    {
      LCD.fillRect(box_x, box_y, box_w, box_h, GxEPD_WHITE);
    }
    while (LCD.nextPage());
    delay(1000);
  }
}
