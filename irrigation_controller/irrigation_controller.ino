/*
  IVANOVIC IRRIGATION CONTROLLER
*/ #define FW_VERSION "v1.3.2-ESP32-IrrigationController"

#include <Arduino.h>
#include <Wire.h> // for RTC
#include <SPI.h> // for EPD
#include "RTClib.h"

#include <Adafruit_GFX.h> // for EPD
#include <GxEPD2_BW.h> // for black AND white EPD
#include <GxEPD2_3C.h>

#include <WiFi.h>       // For high-level Wi-Fi control (mode, STA/AP setup)
#include <esp_wifi.h>   // For ESP-NOW internal Wi-Fi APIs
#include <esp_now.h>    // The ESP-NOW API itself

#include <Update.h> //FOR UPDATING OVER THE AIR
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <ArduinoJson.h>

#include "Battery.h"
#include "Buzzer.h"

#include "esp_timer.h"
#include "esp_sleep.h"
#include "driver/gpio.h"

#include "config.h"
#include "credentials.h"

#include <Fonts/FreeMonoBold9pt7b.h>
#include <Fonts/FreeMonoBold12pt7b.h>
#include <Fonts/FreeSansBold24pt7b.h> // FreeSerifBold24pt7b
#include <Fonts/FreeSansBold18pt7b.h>
#include <Fonts/FreeSans12pt7b.h>
#include <Fonts/FreeSansBold9pt7b.h>
#include <Fonts/FreeMono9pt7b.h>
#include <Fonts/FreeSans9pt7b.h>
#include <Fonts/FreeMono12pt7b.h>

// Peer info 
esp_now_peer_info_t peerInfo;

// class ARDUINOJSON_DEPRECATED("use JsonDocument instead") StaticJsonDocument

//JASON
JsonDocument JSON_sendable;
//StaticJsonDocument<1024> JSON_sendable; //for sending data
//JsonDocument JSON_data; // for dynamic amounts of data
char dataBundle[1024] = "S1:28.37";

//StaticJsonDocument<256> JSON_data; 
JsonDocument JSON_data;

typedef struct moiData{ // all stringified
      char received_data_bundle[200] = ""; // Payload is limited to 250 bytes.
   }  moiData; 

moiData fetch;

RTC_DS3231 real_time; //REALTIME CLOCK

// function prototypes
bool initializeWirelessCommunication();
void OnSensorData_received(const uint8_t * mac, const uint8_t *incomingData, int len);
void initialize_indicators();
void extract_readings();
void beepLowBattery();
void BatteryRecovered();
void MonitorBattery();

void update_display();
void BootScreen();
void homepage();
void footer();
void graphpage();
void tablescreen();
void logs();
void uploading_screen();

bool switch_radios(); // turn OFF ESP-NOW, turn ON WiFi
bool bind_into_JSON(); // generate a data bundle
bool upload(char dataSets[1024]); // upload to Web of IoT... when not successful upload to ThingsPeak
void switch_back_esp_now(); // 

void uptime();
void initialize_RTC();
void query_rtc();
//void irrigate(float average_reading);

Buzzer buzzer(14);   //uint8_t buzzer = 14;     // buzzer pin
Battery batt(32, 14.40, 1);         // pin, scaling factor, num batteries


unsigned long Channel_ID = SECRET_CH_ID;
const char * WriteAPIKey= SECRET_WRITE_APIKEY;

int keyIndex = 0;            // your network key Index number (needed only for WEP)
WiFiClient  client;

const char* serverName = "https://www.webofiot.com/irrigation/muarik_irrikit/server.php";


// base class GxEPD2_GFX can be used to pass references or pointers to the LCD instance as parameter, uses ~1.2k more code
// enable or disable GxEPD2_GFX base class
#define ENABLE_GxEPD2_GFX 0



// ESP32 CS(SS)=5,SCL(SCK)=18,SDA(MOSI)=23,
//  uint8_t CS_ = 5,  DC_ = 15, RES_ = 13, BUSY_ = 4;
  uint8_t CS_ = 5,  DC_ = 4, RES_ = 13, BUSY_ = 15;

// 4.2'' EPD Module
//   GxEPD2_BW<GxEPD2_420_GDEY042T81, GxEPD2_420_GDEY042T81::HEIGHT> LCD(GxEPD2_420_GDEY042T81(CS_, DC_, RES_, BUSY_)); // 400x300, SSD1683
     GxEPD2_3C<GxEPD2_420c_GDEY042Z98, GxEPD2_420c_GDEY042Z98::HEIGHT> LCD(GxEPD2_420c_GDEY042Z98(CS_, DC_, RES_, BUSY_)); // 400x300, SSD1683

     // with 4 power stages
typedef enum {
    POWER_CRITICAL = 0,  // < 7.2V
    POWER_LOW,           // 7.2V - 7.8V  
    POWER_MODERATE,      // 7.8V - 8.2V
    POWER_EXCELLENT      // > 8.2V
} power_state_t;

power_state_t get_power_state(uint8_t level) {

      if(level  <= 0) return POWER_CRITICAL; // < 3.6V
      if(level  == 1) return POWER_LOW;    //  < 3.9V
      if(level  == 2) return POWER_MODERATE; // < 4.1V
      return POWER_EXCELLENT; //  if(level  >= 3)

}
power_state_t current_power;  // start low until measured shows high

uint64_t dynamic_interval = (10 * 1000); // ms DEFAULT: 10 secs // @Excellent Power

const uint64_t power_critical_sleep_duration = (2 * 3600ULL * 1000000ULL); // 2 hours
const uint64_t power_low_sleep_duration = (900ULL * 1000000ULL); // 15 minutes
const uint64_t power_mod_sleep_duration = (300ULL * 1000000ULL); // 5 minutes
const uint64_t power_excellent_delay = (10 * 1000); // 10 seconds

SemaphoreHandle_t xDataMutex = xSemaphoreCreateMutex();

uint8_t indicator = 2; // onboard LED

bool can_send_wirelessly = false;

#define PRIORITY_HIGH   4 // so that the buzzer is not kept waiting for LCD/send/heavy Serial
#define PRIORITY_NORMAL 1
#define PRIORITY_VERY_HIGH 7 // so that everything else waits ... radio switches

int8_t currentScreen = -1; // negative for very low power

// ===== FreeRTOS Tasks =====
void VeryFastTask(void *pvParams);
void CombinedTasks(void *pvParams);
void ScreenTask(void * pvParams);
void ManageIrrigation(void * pvParams);
void upload_task(void * pvParams);

float average_soil_moisture = 0.00;

bool clock_is_working = false;
char ShortTime[32] = "10:10";
char SystemTime[48] = "10:10:25";
char SystemDate[50] = "10/10/25";


float sensor_1_moisture = 0.00;
float sensor_2_moisture = 0.00;
uint64_t sensor_1_transmissions = 0;
uint64_t sensor_2_transmissions = 0;

uint8_t sensor_1_PowerMode = 0;
uint8_t sensor_2_PowerMode = 0;

char sensor1_POWER_XTIX[32] = ".";
char sensor2_POWER_XTIX[32] = ".";


float sensor_1_voltage = 0.0;
float sensor_2_voltage = 0.0;
char node_1_last_seen[32] = "HH:MM";
char node_2_last_seen[32] = "HH:MM";
float sensor_1_readings[30]; // up to 10 readings ago
char sensor_1_time_stamps[30][32] = {"10:10", "10:11", "10:12", "10:13"};

float sensor_2_readings[30]; // up to 30 readings
char sensor_2_time_stamps[30][32] = {"10:10", "10:11", "10:12", "10:13"};

uint8_t active_sensors = 0;

void setup() {
      Serial.begin(115200); delay(500); Serial.println(); Serial.printf("------Firmware %s booting...\n", FW_VERSION);
      Serial2.begin(115200, SERIAL_8N1, 16, 17);
      buzzer.begin(); // comes with pinMode(buzzer, OUTPUT);
         
  // CORE TASK 1
      xTaskCreatePinnedToCore(VeryFastTask, "BuzzingCheck", 2048, NULL, PRIORITY_HIGH, NULL, 1);
      Serial.println("\tBuzzer Task initialized!");
      
      buzzer.beep(1,50,0); // now this can go off in exactly 50ms before the loop comes in

  // DISPLAY
      LCD.init(115200, true, 50, false); vTaskDelay(pdMS_TO_TICKS(500)); // for the hardware SPI to connect to start transactions
      BootScreen(); LCD.hibernate(); vTaskDelay(pdMS_TO_TICKS(2000));
      Serial.println("Screen Boot Complete!");
       
  //INDICATORS
     initialize_indicators();

  // POWER MANAGEMENT   
      batt.begin(); //comes with pinMode(batteryPin, INPUT); and analogReadResolution(12) and analogSetAttenuation(ADC_11db);
      batt.onLow(beepLowBattery);
      batt.onRecovered(BatteryRecovered);

      Serial.println("[BATTERY] Performing initial voltage check...");
      MonitorBattery(); //       current_power = get_power_state(batt.getLevel());

      
       // But if the RTC is not available, do not keep everyone else waiting
        initialize_RTC();
     if(clock_is_working) query_rtc(); // query RTC but do not hang in there long if no response

  //UI MGT TASK
    xTaskCreatePinnedToCore(ScreenTask, "UpdateDisplay", 4096, NULL, PRIORITY_NORMAL, NULL, 1);

    xTaskCreatePinnedToCore(ManageIrrigation, "IrrigationController", 2048, NULL, PRIORITY_NORMAL, NULL, 1);
    
    // 12288 safer for HTTPS 
    xTaskCreatePinnedToCore(upload_task, "Upload to the Internet", 12288, NULL, PRIORITY_VERY_HIGH, NULL, 0); // core 0
    
 //after establishing power state
      initializeWirelessCommunication();
      Serial.println("Wireless Communication Initialized!");

    currentScreen = 1; update_display(); vTaskDelay(pdMS_TO_TICKS(2000));
    digitalWrite(indicator, LOW); // turn OFF after init

}

void ManageIrrigation(void * pvParams) { 

    int valve_checkin_frequency = (10*1000); // ~10s

    while (true) {
        //irrigate(average_soil_moisture);
        vTaskDelay(pdMS_TO_TICKS(valve_checkin_frequency));
    }

}

const TickType_t upload_interval = pdMS_TO_TICKS(10 * 60 * 1000);  // 10 min

void upload_task(void *pvParams) {
    TickType_t prev_wake_time = xTaskGetTickCount();

    while (true) {
        currentScreen = 10;
        update_display(); // Indicate upload in progress

        bool wifi_enabled = switch_radios(); // Switch from ESP-NOW to WiFi

        if (wifi_enabled) {
            bool json_ready = bind_into_JSON();
            bool upload_success = false;

            if (json_ready) {
                upload_success = upload(dataBundle);
            }

            if (upload_success) {
                Serial.println("Upload successful.");
            } else {
                Serial.println("Upload failed. Will retry later.");
            }

            switch_back_esp_now(); // Return to ESP-NOW mode
        } else {
            Serial.println("WiFi unavailable. Retrying next cycle.");
            switch_back_esp_now(); // Ensure radio state restored
        }

        UBaseType_t highWaterMark = uxTaskGetStackHighWaterMark(NULL);
        Serial.print("Upload Task Stack Remaining: ");  Serial.println(highWaterMark);
        Serial.println();

        // Sleep until the next upload cycle
        vTaskDelayUntil(&prev_wake_time, upload_interval);
    }
}


bool switch_radios(){

  return true;

}

/* EXAMPLE EXPECTED ON THE SERVER SIDE
    [
      "Sensor_1":{  
                "Position":"SENSOR A1", 
                "Moisture": sensor_1_moisture, 
                "Voltage":sensor_1_voltage, 
                "PwrMde":sensor1_POWER_XTIX, 
                "Last_Seen":node_1_last_seen,
                "Sends":sensor_1_transmissions
              },
      "Sensor_2":{  
                "Position":"SENSOR A2", 
                "Moisture": sensor_2_moisture, 
                "Voltage":sensor_2_voltage, 
                "PwrMde":sensor2_POWER_XTIX, 
                "Last_Seen":node_2_last_seen,
                "Sends":sensor_2_transmissions,
                //"MoreData":sensor_2_readings, sensor_2_time_stamps // probably nested arrays
              },
      Solenoid:{
                "State":"CLOSED", 
                "Last_Irrigation":"10:10:25", 
                "Duration":0, 
                "Cycles_Today":0
              },
      "Water_Usage_Liters":0.00,
      "System":{
                "Time":SystemTime, 
                "Date":SystemDate, 
                "FW_Version":FW_VERSION,
                "Battery_Voltage": batt.getVoltage(),
                "Average_Moisture": average_soil_moisture
              },
            
      
      ]

*/

float SystemBatteryVoltage = 0.0;

char solenoid_state[16] = "CLOSED";
char solenoid_last_irrigation[16] = "10:10:25";
int solenoid_duration = 0;
int solenoid_cycles_today = 0;
float water_usage_liters = 0.00;

// For debug logging
#define LOG(msg)  Serial.println(msg)
//JsonDocument JSON_sendable; // Static is deprecated 

bool bind_into_JSON(){
           LOG("[JSON] Starting JSON binding process...");
           bool bound_successfully = false;

            // --- Populate Sensor 1 Data ---
          JsonObject Sensor_1 = JSON_sendable.createNestedObject("Sensor_1");
          Sensor_1["Position"]     = "SENSOR A1";
          Sensor_1["Moisture"]     = sensor_1_moisture;
          Sensor_1["Voltage"]      = sensor_1_voltage;
          Sensor_1["PwrMde"]       = sensor1_POWER_XTIX;
          Sensor_1["Last_Seen"]    = node_1_last_seen;
          Sensor_1["Sends"]        = sensor_1_transmissions;

          LOG("[JSON] Sensor 1 data added.");

          // ---  Populate Sensor 2 Data ---
          JsonObject Sensor_2 = JSON_sendable.createNestedObject("Sensor_2");
          Sensor_2["Position"]     = "SENSOR A2";
          Sensor_2["Moisture"]     = sensor_2_moisture;
          Sensor_2["Voltage"]      = sensor_2_voltage;
          Sensor_2["PwrMde"]       = sensor2_POWER_XTIX;
          Sensor_2["Last_Seen"]    = node_2_last_seen;
          Sensor_2["Sends"]        = sensor_2_transmissions;

          // Optional: include recent readings and timestamps (nested arrays)
          JsonArray readings = Sensor_2.createNestedArray("Readings");
          JsonArray times    = Sensor_2.createNestedArray("Timestamps");
          for (int i = 0; i < 30; i++) {
              readings.add(sensor_2_readings[i]);
              times.add(sensor_2_time_stamps[i]);
          }

          LOG("[JSON] Sensor 2 data and nested arrays added.");

          // --- 4️⃣ Solenoid / Irrigation Section ---
          JsonObject Solenoid = JSON_sendable.createNestedObject("Solenoid");
          Solenoid["State"]           = solenoid_state;
          Solenoid["Last_Irrigation"] = solenoid_last_irrigation;
          Solenoid["Duration"]        = solenoid_duration;
          Solenoid["Cycles_Today"]    = solenoid_cycles_today;

          LOG("[JSON] Solenoid data added.");

          // --- 5️⃣ Water usage ---
          JSON_sendable["Water_Usage_Liters"] = water_usage_liters;
          LOG("[JSON] Water usage added.");

          // --- 6️⃣ System block ---
          JsonObject System = JSON_sendable.createNestedObject("System");
          System["Time"]             = SystemTime;
          System["Date"]             = SystemDate;
          System["FW_Version"]       = FW_VERSION;
          System["Battery_Voltage"]  = SystemBatteryVoltage;
          System["Average_Moisture"] = average_soil_moisture;

          LOG("[JSON] System data added.");

          // --- 7️⃣ Serialize into a buffer or string ---
          char payload[2048];
          size_t len = serializeJson(JSON_sendable, payload, sizeof(payload));

          if (len > 0) {
            bound_successfully = true;
            LOG("[JSON] Serialization successful.");
            LOG("[JSON] Payload size: " + String(len) + " bytes");
            LOG("[JSON] --- Payload Preview ---");
            Serial.println(payload); // optional: can be commented out if too large
          } else {
            LOG("[JSON] Serialization failed!");
          }
          
  return bound_successfully;

}


// IRRIGATION CONTROLLER 
char solenoid_issue[120] = "...";
char solenoid_fix[120] = "...";

const int irrigation_valve_pin = 33; // GPIO pin controlling the irrigation valve
const int irrigation_duration_seconds = 30; // Duration to keep the valve open
const int irrigation_interval_seconds = 3600; // Minimum interval between irrigations

const uint64_t max_irrigation_duration = 120; // in seconds
const int max_irrigation_cycles_per_day = 5; // Max cycles per day
int irrigation_cycles_today = 0;
const float wilting_point  = 18.0; // below which irrigation must be done
const float critical_point = 20.0; // threshold to trigger irrigation to start // hysteresis
const float field_capacity = 25.0; // above which irrigation is not needed

int64_t irrigation_start_time = 0; // in microseconds
bool is_irrigating = false; bool irrigation_completed = false;

// irrigation logic based on average soil moisture
// IF IRRIGATION HAS BEEN ON FOR max_irrigation_duration, FORCED TIMEOUT [EITHER TANK EMPTY OR SENSING FAULT]
// LOGS INTO A CHAR[] the states: IRRIGATING / NOT IRRIGATING, last irrigation time, duration, cycles today, duration
// if has not irrigated in a full day, reset cycles today to 0, report it
// take care of impossible situations [avg_read > 100 or < 1.0]
// measure batt voltage befopre and after irrigation cycle to see voltage drop
// if voltage drop is significant, dont start irrigation, log["can't oirrigate, low battery"]

/*
void irrigate(float average_reading){
  bool should_irrigate = false;
  if(average_reading < decision_point){ // a figure above wilting point and less than field capacity 
    should_irrigate = true;
  }
  else {
      should_irrigate = false; // defensive programming just in case it is returning from some irrigation state
  }

  if(should_irrigate && !is_irrigating){ // if the reading is low enough and not already irrigating

    if((irrigation_cycles_today < max_irrigation_cycles_per_day)){
        Serial.println(">>> Starting Irrigation Cycle...");
        digitalWrite(irrigation_valve_pin, HIGH); // Open valve
        is_irrigating = true;
        irrigation_start_time = esp_timer_get_time(); // in microseconds
    }
    else {
      if(irrigation_cycles_today >= max_irrigation_cycles_per_day){
          Serial.println(">>> Maximum irrigation cycles reached for today.");
          irrigation_duration = 0;
        }

        else {
        irrigation_duration = (esp_timer_get_time() - irrigation_start_time)/1e6; // in microseconds
            }
      }
  }
    else { // either shouldn't irrigate or already irrigating
      if(is_irrigating){
         Serial.println(">>> System Irrigating...");

      }
      if(!should_irrigate){
        Serial.println(">>> Irrigation Cycle Completed.");
        Serial.println(">>> Soil moisture adequate, no irrigation needed.");
        digitalWrite(irrigation_valve_pin, LOW); // Close valve
        is_irrigating = false;
        irrigation_cycles_today++;
      }
    }

    
  
}

*/
char flow_sensor_issue[120] = "...";
char flow_sensor_fix[120] = "...";

#define MAX_FLOW_RATE_LPM 10.0 // maximum flow rate in liters per minute
const uint8_t flow_meter_pin = 34; // analog pin connected to the flow meter

float measure_flow_rate(bool irrigating_status, uint64_t irrigation_duration_seconds){
  bool flow_meter_working = false;
  float flow_rate = 0.0;
  float cummulated_reading = 0.0;

  if(is_irrigating){
    // start measuring flow
    for(int i = 0; i<10; i++){
        int raw_reading = analogRead(flow_meter_pin); // assuming readings go from 0 to 4095
        cummulated_reading += (float)raw_reading;
        
        vTaskDelay(pdMS_TO_TICKS(100)); // small delay between readings
    }
    cummulated_reading /= 10.0; // average reading
    
    flow_rate = ((cummulated_reading)/4096 )* MAX_FLOW_RATE_LPM;
      
    // implement a settling period 

    if(flow_rate < 0.1){ // the lowest impossible and settling period has passed
        strcpy(flow_sensor_issue, "Flow sensor reading too low during irrigation.");
        strcpy(flow_sensor_fix, "Check sensor connection or calibration.");
        flow_meter_working = false;
    }
    else {
        flow_meter_working = true;
        strcpy(flow_sensor_issue, "Flow sensor operating normally.");
        strcpy(flow_sensor_fix, "No action needed.");
    }

    
  }
  else {
    flow_rate = 0.0; // no flow when not irrigating
  }


  return flow_rate;
}

void switch_back_esp_now(){

}

bool upload(char dataSets[1024]){ // upload the serialized JSON

  return true;
}


// like checking on boiling milk few times every second
void VeryFastTask(void * pvParams) { 
  uint8_t checkin_frequency = 10; // ~10ms
 
  const TickType_t xDelay = pdMS_TO_TICKS(checkin_frequency);
  while (1) {
    buzzer.update(); // OFF if duration etuuse // maybe also buttons.update();
    
    vTaskDelay(xDelay); // yield to la skedula
  }

}

void ScreenTask(void * pvParams){
      while(true){
        update_display();
        vTaskDelay(pdMS_TO_TICKS(60000)); // ~ 1 minute
    } 
}
uint64_t now_now = 0;

void loop() {
  now_now = esp_timer_get_time()/1e6; // in seconds 
  if(!clock_is_working) uptime();
  // if(clock_is_working) query_rtc();

  Serial.print("Sensor 1 Readings: {"); 
  Serial2.print("{");
   for(int i=0; i<30; i++){ // Serial.print(i+1);
        Serial.print(sensor_1_readings[i]);
        Serial2.print(sensor_1_readings[i]);
        if(i<29){  Serial.print(",  ");  Serial2.print(",  "); }
   } Serial.println("}"); Serial2.println("}");
   
   Serial.print("Sensor 2 Readings: {"); 
   Serial2.print("{");
   for(int i=0; i<30; i++){// Serial.print(i+1);
        Serial.print(sensor_2_readings[i]);
        Serial2.print(sensor_2_readings[i]);
        if(i<29) { Serial.print(", "); Serial2.print(",  "); }
   } Serial.println("}");  Serial2.println("}");

   Serial.println();
   Serial2.println();
   

      delay(5000);  // currentScreen++;

}

char time_str[32] = "";

void uptime(){
        uint64_t seconds = now_now;
        uint8_t hours   = (seconds / 3600) % 24;
        uint8_t minutes = (seconds % 3600) / 60;
        uint8_t secs    = 1+(seconds % 60);

        sprintf(time_str, "%02u:%02u:%02u", hours, minutes, secs);
        Serial.print("Uptime Check: ");Serial.println(time_str);
}

void update_display(){
  if(currentScreen == 1) homepage();
  if(currentScreen == 2) graphpage();
  if(currentScreen == 3) tablescreen();
  if(currentScreen == 4) logs();
  if(currentScreen == 10) uploading_screen();
  else currentScreen = 1;

  LCD.hibernate();

}


void uploading_screen(){

}

void graphpage(){

}

void tablescreen(){

}

void logs(){

}

//indicative LEDs
uint8_t high = 26; // green
uint8_t medium = 27; //orange
uint8_t low   = 25;//red

void initialize_indicators(){
      pinMode(indicator, OUTPUT);  digitalWrite(indicator, HIGH);
      pinMode(low, OUTPUT); digitalWrite(low, HIGH); vTaskDelay(pdMS_TO_TICKS(500));
      pinMode(medium, OUTPUT); digitalWrite(medium, HIGH);  vTaskDelay(pdMS_TO_TICKS(500));
      pinMode(high, OUTPUT); digitalWrite(high, HIGH);  vTaskDelay(pdMS_TO_TICKS(500));
      Serial.println("Indicators initialized!"); 
}


#define ESPNOW_MAX_RETRIES 3
bool initializeWirelessCommunication() {
    WiFi.mode(WIFI_STA);
    
    // ESP-NOW initialization with retry logic
    for (uint8_t attempt = 1; attempt <= ESPNOW_MAX_RETRIES; attempt++) {
        if (esp_now_init() == ESP_OK) { 
            Serial.printf("✓ ESP-NOW initialized successfully (attempt %d/%d)\n", attempt, ESPNOW_MAX_RETRIES);
            
         // Register callback function
          esp_err_t result = esp_now_register_recv_cb(esp_now_recv_cb_t(OnSensorData_received)); // Serial.print("CALLBACK: ");Serial.println(result);
          if(result == ESP_OK) Serial.println("Call Back of Call Back successfully set!"); 
     
            memcpy(peerInfo.peer_addr, System.peer.broadcastAddress, 6);
            peerInfo.channel = 0;
            peerInfo.encrypt = false;
            
            if (esp_now_add_peer(&peerInfo) == ESP_OK) {
                Serial.println("✓ Peer registered successfully");
                can_send_wirelessly = true;
                return true;
            } else { can_send_wirelessly = false;
                Serial.println("!!! Peer registration failed");
            }
        } else {
           can_send_wirelessly = false;
           // print that Wireless Radio is faulty
            //return;
        }
        delay(100 * attempt); // Exponential backoff
    }
    
    Serial.println("!!! ESP-NOW initialization failed after retries");
    return false;
}


char dataPack[200] = ""; // receivable


void OnSensorData_received(const uint8_t * mac, const uint8_t *incomingData, int len){
     //   memcpy(&fetch, incomingData, min(len, sizeof(moiData))); // no matching function for call to 'min(int&, unsigned int)'
        // The compiler will complain because len is an int, while sizeof(moiData) is an unsigned int (size_t).
// The template deduction for std::min() fails when the two arguments are of different signedness.

        //Since memcpy length must be size_t, it’s best to cast explicitly: ... no template fights.
        size_t weight_of_packet = (len < (int)sizeof(moiData)) ? (size_t)len : sizeof(moiData);
        memcpy(&fetch, incomingData, weight_of_packet);

        uint32_t size_of_packet = sizeof(dataPack);

  //   memcpy(&fetch, incomingData, sizeof(moiData)); //If len < sizeof(moiData), memcpy will copy beyond valid incomingData.
 //    strcpy(dataPack, fetch.received_data_bundle); // If the buffer is not null-terminated, strcpy() will read past the end.
        strncpy(dataPack, fetch.received_data_bundle, size_of_packet-1);
        dataPack[size_of_packet-1] = '\0';
        Serial.print("Size of received BUFF: "); Serial.println(size_of_packet);

      Serial.print("Received => "); Serial.println(dataPack);  
      buzzer.beep(1, 50, 0);
      digitalWrite(indicator, HIGH); vTaskDelay(pdMS_TO_TICKS(50)); digitalWrite(indicator, LOW);
      extract_readings();
     
}


uint8_t reading_1_count = 0;
uint8_t reading_2_count = 0;

float moi_readings[6] = {0.0, 0.0, 0.0f, 0.0f, 0.0f, 0.0f};
uint32_t last_seen[6] = {0, 0, 0, 0, 0, 0}; // epoch seconds

//deserialize to assign char[] and floats accordingly
void extract_readings(){

    // serializeJson(JSON_data, dataPack); // serialize again
       deserializeJson(JSON_data, dataPack);
          
    const char * sensor_1_ID = JSON_data["Pos_1"] | "unknown";  // string
    const char * sensor_2_ID = JSON_data["Pos_2"] | "unknown";
    const char * sensor_3_ID = JSON_data["Pos_3"] | "unknown";
    const char * sensor_4_ID = JSON_data["Pos_4"] | "unknown";
    const char * sensor_5_ID = JSON_data["Pos_5"] | "unknown";
    const char * sensor_6_ID = JSON_data["Pos_6"] | "unknown";


    float  _1_moisture = JSON_data["Moi_1"]; // float
    float _1_voltage = JSON_data["Volt_1"]; // float
    uint8_t _1_PowerMode = JSON_data["PwrMde_1"]; //int or str
    uint64_t _1_receiveds = JSON_data["Sends_1"]; // long long

    float _2_moisture = JSON_data["Moi_2"];
    float _2_voltage = JSON_data["Volt_2"]; // float
    uint8_t _2_PowerMode = JSON_data["PwrMde_2"]; //int or str
    uint64_t _2_receiveds = JSON_data["Sends_2"]; // long long

    float sensor_3_moisture = JSON_data["Moi_3"];
    //uint32_t sensor_3_transmissions = JSON_data["Volt_1"];

    float sensor_4_moisture = JSON_data["Moi_4"];
    //uint32_t sensor_4_transmissions = JSON_data["Volt_1"];

    float sensor_5_moisture = JSON_data["Moi_5"];
    //uint32_t sensor_5_transmissions = JSON_data["Volt_1"];

    float sensor_6_moisture = JSON_data["Moi_6"];
    //uint32_t sensor_6_transmissions = JSON_data["Volt_1"];

    //// the temp and humidity readings are swapped
  if (_1_moisture > 1.00) {

        // --- 1️⃣ Update latest sensor reading ---
        sensor_1_moisture = _1_moisture;

        // Shift readings and timestamps right (keeping latest at index 0)
        for (int i = 29; i > 0; i--) {
            sensor_1_readings[i] = sensor_1_readings[i - 1];
            strcpy(sensor_1_time_stamps[i], sensor_1_time_stamps[i - 1]);
        }

        // Insert new reading at index 0
        sensor_1_readings[0] = sensor_1_moisture;
        strcpy(sensor_1_time_stamps[0], ShortTime);

        // Update reading count (max 30)
        if (reading_1_count < 30) reading_1_count++;

        // --- 2️⃣ Update node telemetry ---
        sensor_1_voltage       = _1_voltage;
        sensor_1_transmissions = _1_receiveds;
        sensor_1_PowerMode     = _1_PowerMode;

        // --- 3️⃣ Build power status string ---
        // Example output: "4.1V, Mode2, LIGHT SLP"
        dtostrf(sensor_1_voltage, 4, 1, sensor1_POWER_XTIX);  // e.g., "4.1"
        strncat(sensor1_POWER_XTIX, "V, ", sizeof(sensor1_POWER_XTIX) - strlen(sensor1_POWER_XTIX) - 1);

        char temp[8];
        snprintf(temp, sizeof(temp), "Mode%d", sensor_1_PowerMode);
        strncat(sensor1_POWER_XTIX, temp, sizeof(sensor1_POWER_XTIX) - strlen(sensor1_POWER_XTIX) - 1);

        switch (sensor_1_PowerMode) {
            case 0: strcat(sensor1_POWER_XTIX, ", DEEP SLP"); break;
            case 1:
            case 2: strcat(sensor1_POWER_XTIX, ", LIGHT SLP"); break;
            default: strcat(sensor1_POWER_XTIX, ", EXCELLENT"); break;
        }

        // --- 4️⃣ Update last seen timestamps ---
        if (!clock_is_working)
            strcpy(node_1_last_seen, time_str);
        else
            strcpy(node_1_last_seen, ShortTime);

        last_seen[0] = now_now; // numeric timestamp if tracking uptime
  }


  if (_2_moisture > 1.00) {

        // --- 1️⃣ Update latest sensor reading ---
        sensor_2_moisture = _2_moisture;

        // Shift readings and timestamps right
        for (int i = 29; i > 0; i--) {
          sensor_2_readings[i] = sensor_2_readings[i - 1];
          strcpy(sensor_2_time_stamps[i], sensor_2_time_stamps[i - 1]);
        }

    // Insert new reading at index 0
    sensor_2_readings[0] = sensor_2_moisture;
    strcpy(sensor_2_time_stamps[0], ShortTime);

    // Update reading count (max 30)
    if (reading_2_count < 30) reading_2_count++;

    // --- 2️⃣ Update node telemetry ---
      sensor_2_voltage      = _2_voltage;
      sensor_2_transmissions = _2_receiveds;
      sensor_2_PowerMode    = _2_PowerMode;

    // --- 3️⃣ Build power status string ---
    // Example output: "4.1V, Mode2, LIGHT SLP"
      dtostrf(sensor_2_voltage, 4, 1, sensor2_POWER_XTIX);  // e.g., "4.1"
      strncat(sensor2_POWER_XTIX, "V, ", sizeof(sensor2_POWER_XTIX) - strlen(sensor2_POWER_XTIX) - 1);

      char temp[8];
      snprintf(temp, sizeof(temp), "Mode%d", sensor_2_PowerMode);
      strncat(sensor2_POWER_XTIX, temp, sizeof(sensor2_POWER_XTIX) - strlen(sensor2_POWER_XTIX) - 1);

    switch(sensor_2_PowerMode) {
      case 0: strcat(sensor2_POWER_XTIX, ", DEEP SLP"); break;
      case 1:
      case 2: strcat(sensor2_POWER_XTIX, ", LIGHT SLP"); break;
      default: strcat(sensor2_POWER_XTIX, ", EXCELLENT"); break;
    }

    // --- 4️⃣ Update last seen timestamps ---
    if (!clock_is_working)
        strcpy(node_2_last_seen, time_str);
    else
        strcpy(node_2_last_seen, ShortTime);

     last_seen[1] = now_now; // numeric timestamp if tracking uptime
  }

   
    if(sensor_3_moisture > 2.00) {  last_seen[2] = now_now; moi_readings[2] =  sensor_3_moisture; }
    if(sensor_4_moisture > 2.00) {  last_seen[3] = now_now; moi_readings[3] =  sensor_4_moisture; }
    if(sensor_5_moisture > 2.00) {  last_seen[4] = now_now; moi_readings[4] =  sensor_5_moisture; }
    if(sensor_6_moisture > 2.00) {  last_seen[5] = now_now; moi_readings[5] =  sensor_6_moisture; }

}


bool battery_low_registered = false;
void beepLowBattery() {
  if(!battery_low_registered) {
      Serial.println("Battery Critically low");
      buzzer.beep(2, 800, 500);
      battery_low_registered = true;
  }
}

void BatteryRecovered() {
  if(battery_low_registered) {
      Serial.println("Battery recovered above 7.2V");
      battery_low_registered = false;
  }
}

 

char batt_str[12] = ".";
uint8_t batt_level = 0;

void MonitorBattery() {
    batt.update(); // get latest reading

    // plot 0,1,2,3 >> CRITICAL, LOW, MODERATE, EXCELLENT
      current_power = get_power_state(batt.getLevel());

    // Map power state → sleep mode
    switch(current_power) {
        case POWER_CRITICAL:  Serial.println("Discovery: CRITICAL POWER");  break;
        case POWER_LOW:       Serial.println("Discovery: LOW POWER");  break;
        case POWER_MODERATE:  Serial.println("Discovery: MODERATE");   break;
        case POWER_EXCELLENT: Serial.println("Discovery: EXCELLENT");  break;
    }

    if (batt.isLow()) {
        Serial.println("Battery LOW (with hysteresis) → forcing LOW POWER");
         // Disable OTA when battery is critical
          WiFi.mode(WIFI_STA);
          WiFi.disconnect();
      //  current_power = POWER_CRITICAL;
    }

     dtostrf(batt.getVoltage(), 4, 1, batt_str);   Serial.printf("Voltage: %s V\n", batt_str); Serial.println();
   
}

uint16_t box_width = 150;
uint16_t box_height = 90;


 
uint8_t temp = 0;
bool is_home = false;
void homepage(){

      // LCD.setPartialWindow(28, 70, 150, 120); // to update only thr readings and nothing else
  LCD.firstPage(); //FULLY CENTERED TEXTS
  do{
          //BOUNDARY AROUND THE 400 X 300px
      LCD.fillScreen(GxEPD_WHITE);    
      LCD.drawRect(0, 0, 400, 300, GxEPD_BLACK); LCD.drawRect(1, 1, 398, 298, GxEPD_BLACK);

     


       // ACTIVE MENU PANES
          LCD.fillRect(1, 1, 397, 27, (LCD.epd2.hasColor ? GxEPD_RED : GxEPD_BLACK)); 
          LCD.fillRoundRect(8, 2, 100, 30, 5, GxEPD_WHITE);

          LCD.setFont(&FreeSans12pt7b); // LCD.setFont(&FreeSans9pt7b);
          LCD.setTextColor(GxEPD_BLACK);
          LCD.setCursor(14, 22);  LCD.print("Sensors");
            
            //HOME TAB
            
          LCD.setFont(&FreeSans9pt7b);
          LCD.setTextColor(GxEPD_WHITE);
          LCD.setCursor(140, 19);  LCD.print("Data Table");
          LCD.fillRect(250, 5, 2, 20, GxEPD_WHITE); // divider
          LCD.setCursor(290, 19);  LCD.print("Settings");

           

 // VERTICAL SCROLL BAR
        LCD.drawRoundRect(370, 50, 20, 160, 5, LCD.epd2.hasColor ? GxEPD_RED : GxEPD_BLACK);
        LCD.fillRoundRect(375, 60, 10, 80, 12, GxEPD_BLACK);

        LCD.fillTriangle(370, 45, 380, 35, 390, 45, LCD.epd2.hasColor ? GxEPD_RED : GxEPD_BLACK);  // UPPER ARROW
        LCD.drawTriangle(370, 215, 380, 225, 390, 215, LCD.epd2.hasColor ? GxEPD_RED : GxEPD_BLACK);  // DOWN ARROE

    // SENSOR 1
          LCD.drawRoundRect(30, 40, box_width, box_height, 10, GxEPD_BLACK);

          LCD.fillRoundRect(30, 40, box_width, 20, 10, GxEPD_BLACK); //BLACK RIBBON
          LCD.fillRect(30, 50, box_width, 12, GxEPD_BLACK);

          //DYNAMICALLY RESIZABLE BUBBLE
          if(sensor_1_transmissions<99){ LCD.fillCircle((25+box_width), 42,  12, GxEPD_RED); }
          else { // should hold 3-5 digits: 100-99,999
                LCD.fillRoundRect((15+box_width), 30,  36, 20, 20, GxEPD_RED); // notification bubble
          } 

          LCD.setTextColor(GxEPD_WHITE); LCD.setFont(&FreeSans9pt7b);
          LCD.setCursor(46, 58); LCD.print("Sensor Node 1"); //AREA HEADING


          

        //DATA
         LCD.setFont(); LCD.setTextColor(GxEPD_BLACK);
     //    LCD.setCursor(35, 70); LCD.print(sensor_1_voltage); // "V"
     //    LCD.setCursor(170, 70); LCD.print(sensor_1_PowerMode);  //power mode 

         LCD.setCursor(45, 66); LCD.print(sensor1_POWER_XTIX);  //LCD.print("PERCENT VOLUME"); // wind speed log
     
         LCD.setFont(); LCD.setTextColor(GxEPD_WHITE); 
         LCD.setCursor((20+box_width), 37); LCD.print(sensor_1_transmissions);
         
         LCD.setFont(&FreeMono9pt7b); LCD.setTextColor(GxEPD_BLACK); //FreeMono9pt7b
         LCD.setCursor(70, 125); LCD.print(node_1_last_seen);

         LCD.setFont(&FreeSansBold24pt7b); LCD.setTextColor(GxEPD_BLACK);   
         LCD.setCursor(50, 110);      LCD.print(sensor_1_moisture);  



   // SENSOR 2
        LCD.drawRoundRect((25+(1.2*box_width)), 40, box_width, box_height, 10, GxEPD_BLACK); 
        LCD.fillRoundRect((25+(1.2*box_width)), 40, box_width, 20, 10, GxEPD_BLACK); //BLACK RIBBON
        LCD.fillRect((25+(1.2*box_width)), 50, box_width, 12, GxEPD_BLACK);
        
        if(sensor_2_transmissions<99){
          LCD.fillCircle((13+box_width+(1.2*box_width)), 42, 12, GxEPD_RED); // notification bubble
        }
        else { // should hold 3-5 digits: 100-99,999
          LCD.fillRoundRect((13+box_width+(1.2*box_width)), 30, 36, 20, 20, GxEPD_RED); // notification bubble
        }
        
        LCD.setTextColor(GxEPD_WHITE); LCD.setFont(&FreeSans9pt7b);
        LCD.setCursor(5+(30+(1.2*box_width)), 58); LCD.print("Sensor Node 2"); //AREA HEADING
       
        LCD.setFont(&FreeMono9pt7b);  LCD.setTextColor(GxEPD_BLACK); //FreeMono9pt7b
      //LCD.setCursor(85+(30+(1.2*box_width)), 85); LCD.print("%");

        LCD.setCursor(30+(30+(1.2*box_width)), 125); LCD.print(node_2_last_seen); 
      
        //DATA
         LCD.setFont(); LCD.setCursor((40+(1.2*box_width)), 66); 
         //LCD.print("PERCENT VOLUME");
         LCD.print(sensor2_POWER_XTIX);  //LCD.print("PERCENT VOLUME"); // wind speed log
          
         LCD.setCursor(((17+box_width)+(1.2*box_width)), 37); 
         LCD.setTextColor(GxEPD_WHITE); LCD.print(sensor_2_transmissions);

         LCD.setTextColor(GxEPD_BLACK);      LCD.setFont(&FreeSansBold24pt7b); 
         LCD.setCursor(40+((1.2*box_width)), 110);    LCD.print(sensor_2_moisture); 
         

       // FLOW METER
         LCD.drawRoundRect(30, 140, box_width, box_height, 10, (LCD.epd2.hasColor ? GxEPD_RED : GxEPD_BLACK));

         LCD.fillRoundRect(30, 140, box_width, 20, 10, (LCD.epd2.hasColor ? GxEPD_RED : GxEPD_BLACK)); //BLACK RIBBON
         LCD.fillRect(30, 150, box_width, 12, (LCD.epd2.hasColor ? GxEPD_RED : GxEPD_BLACK));

         LCD.setTextColor(GxEPD_WHITE); LCD.setFont(&FreeSans9pt7b);
         LCD.setCursor(60, 158); LCD.print("Flow Meter"); //AREA HEADING
        
         LCD.setFont(&FreeMonoBold12pt7b); LCD.setCursor(155, 155+45); LCD.print("litres");
         LCD.setFont(&FreeMono9pt7b); LCD.setTextColor((LCD.epd2.hasColor ? GxEPD_RED : GxEPD_BLACK)); //FreeMono9pt7b
         LCD.setCursor(50, (220));     LCD.print("time_stamp");
        //DATA
         LCD.setTextColor((LCD.epd2.hasColor ? GxEPD_RED : GxEPD_BLACK));
         LCD.setFont(&FreeSansBold24pt7b); 
         LCD.setCursor(60, (200));LCD.print("27.6");

         

    // SOLENOID 
         LCD.drawRoundRect((25+1.2*box_width), 140, box_width, box_height, 10, (LCD.epd2.hasColor ? GxEPD_RED : GxEPD_BLACK));

         LCD.fillRoundRect((25+1.2*box_width), 140, box_width, 20, 10, (LCD.epd2.hasColor ? GxEPD_RED : GxEPD_BLACK)); //BLACK RIBBON
         LCD.fillRect((25+1.2*box_width), 150, box_width, 12, (LCD.epd2.hasColor ? GxEPD_RED : GxEPD_BLACK));

         LCD.setTextColor(GxEPD_WHITE); LCD.setFont(&FreeSans9pt7b);
         LCD.setCursor((60+1.2*box_width), 158); LCD.print("Solenoid"); //AREA HEADING

         LCD.setFont(&FreeMono9pt7b); LCD.setTextColor((LCD.epd2.hasColor ? GxEPD_RED : GxEPD_BLACK)); //FreeMono9pt7b
         LCD.setCursor(80+(box_width), (220)); LCD.print("last_on"); 
      //DATA
         LCD.setTextColor((LCD.epd2.hasColor ? GxEPD_RED : GxEPD_BLACK));
         LCD.setFont(&FreeSansBold24pt7b); 
         LCD.setCursor(20+(60+box_width), (200));    LCD.print("OFF");
         

        

         // how many sensors
  //      LCD.setFont();
  //      LCD.setCursor(150,260); LCD.print(active_sensors); LCD.print(" sensors");
       
                // LOGS FOR NETWORK
         LCD.setFont();
         LCD.setTextColor(LCD.epd2.hasColor ? GxEPD_RED : GxEPD_BLACK);
         LCD.setCursor(15, 235); LCD.print("wifi_log");
         LCD.setCursor(15, 249); LCD.print("upload_log");

      footer();


      }  while (LCD.nextPage());
}


void footer(){     
  /*
            //upward curve
            LCD.fillRect(0, 260, 400, 40, GxEPD_BLACK); // GxEPD_BLACK
            LCD.fillRoundRect(2, 255, 396, 15, 8, GxEPD_WHITE);
            // LCD.fillCircle(0, 295, 2, GxEPD_BLACK);
            
            LCD.setFont(&FreeSans9pt7b); // LCD.setFont(&FreeSans9pt7b);
            LCD.setTextColor(GxEPD_WHITE);
    
            //HOME TAB
          //  LCD.fillRoundRect(5, 275, 62, 20, 10, GxEPD_WHITE); 
            //LCD.setCursor(12, 290);  LCD.print("Home"); 
            
            //DATE
            LCD.setCursor(2, 290);  LCD.print(SystemDate); 

            
            // WIFI & UPLOADS
            
            //NETWORK BARS
            LCD.fillRect(230, 293, 6, 5, GxEPD_WHITE); LCD.fillRect(240, 283, 6, 15, GxEPD_WHITE); LCD.fillRect(250, 274, 6, 25, GxEPD_WHITE);
            LCD.setFont();
            //LCD.setCursor(200, 290); LCD.print("WiFi: "); 
            LCD.setCursor(225, 280); LCD.print(wifi_state?"ON":"OFF");
            if(successful_uploads) {LCD.setCursor(265, 280); LCD.print("(");LCD.print(successful_uploads_c);LCD.print(")"); }
            
            //BATTERY
            LCD.drawRect(280, 275, 20, 10, GxEPD_WHITE);
            LCD.setCursor(310, 280); LCD.print(voltage_string); LCD.print("V");
            
            
            //SEPARATOR
           // LCD.fillRect(295, 275, 7, 20, GxEPD_WHITE);

            //TIME
            LCD.setFont(&FreeSans9pt7b); //FreeMono12pt7b
            LCD.setCursor(320, 290); LCD.print(ShortTime);

          //  LCD.setFont(&FreeMono9pt7b);
          //  LCD.setCursor(385, 292); LCD.print(DISP_MODE);
 */


   //footer with upward curve
            LCD.fillRect(0, 264, 400, 40, GxEPD_BLACK); // GxEPD_BLACK
            LCD.fillRoundRect(2, 259, 396, 0, 8, GxEPD_WHITE);
            // LCD.fillCircle(0, 295, 2, GxEPD_BLACK);


            LCD.setFont(&FreeSans9pt7b); // LCD.setFont(&FreeSans9pt7b);
            LCD.setTextColor(GxEPD_WHITE);
    
            //HOME TAB
          //  LCD.fillRoundRect(5, 275, 62, 20, 10, GxEPD_WHITE); 
            //LCD.setCursor(12, 290);  LCD.print("Home"); 
            
            //DATE
            LCD.setCursor(2, 290);  LCD.print(SystemDate); 

            
            // WIFI & UPLOADS
            
            //NETWORK BARS
            LCD.fillRect(210, 293, 6, 5, GxEPD_WHITE); LCD.fillRect(220, 283, 6, 15, GxEPD_WHITE); LCD.fillRect(230, 274, 6, 25, GxEPD_WHITE);
            LCD.setFont();
            //LCD.setCursor(200, 290); LCD.print("WiFi: "); 
        //    LCD.setCursor(224, 280); LCD.print(wifi_state?"ON":"OFF");
           // if(successful_uploads) {LCD.setCursor(265, 275); LCD.print("(");LCD.print(successful_uploads_c);LCD.print(")"); }
            
            //BATTERY
            LCD.fillRect(272, 282, 3, 6, GxEPD_WHITE); LCD.fillRoundRect(275, 277, 35, 16, 3, GxEPD_WHITE); 
            if(batt.getVoltage() >= 6.00) { LCD.setFont(); LCD.setTextColor(LCD.epd2.hasColor ? GxEPD_RED : GxEPD_BLACK);  LCD.setCursor(272, 281); LCD.print("voltage_string");  }
       
            //SEPARATOR
           // LCD.fillRect(295, 275, 7, 20, GxEPD_WHITE);

            //TIME
            LCD.setTextColor(GxEPD_WHITE);
            LCD.setFont(&FreeSans9pt7b); //FreeMono12pt7b
            LCD.setCursor(320, 290);
            if(clock_is_working) LCD.print(ShortTime);
            else LCD.print(time_str);

          //  LCD.setFont(&FreeMono9pt7b);
          //  LCD.setCursor(385, 292); LCD.print(DISP_MODE);

 }



const char Manufacturer[20] = "IntelliSys UG";
const char DeviceID[30] = "Irrigation Controller";
//const char Parameter[] = "Mountains of the Moon";
const char Parameter[30] = "Irri-Kit Ltd";
const char Location[40] = "Makerere University";



void BootScreen(){

  LCD.setRotation(2);
  LCD.setFont(&FreeSans12pt7b);
  LCD.setTextColor(GxEPD_BLACK);

  int16_t tbx, tby; uint16_t tbw, tbh;
 
  LCD.setFullWindow();

  LCD.firstPage(); //FULLY CENTERED TEXTS
  do{
  
        LCD.fillScreen(GxEPD_BLACK); delay(2000);
        LCD.fillScreen(GxEPD_RED); delay(2000);
        LCD.fillScreen(GxEPD_WHITE);
        LCD.setFont(&FreeSansBold24pt7b);

        LCD.setTextColor(LCD.epd2.hasColor ? GxEPD_RED : GxEPD_BLACK);
        LCD.getTextBounds(Manufacturer, 0, 0, &tbx, &tby, &tbw, &tbh);
          // center the bounding box by transposition of the origin:
          uint16_t x = ((LCD.width() - tbw) / 2) - tbx;
          uint16_t y = ((LCD.height() - tbh) / 2) - tby;
        LCD.setCursor(50, 100);    
        
        LCD.print(Manufacturer);

        LCD.setFont(&FreeMonoBold12pt7b);
        LCD.getTextBounds(DeviceID, 0, 0, &tbx, &tby, &tbw, &tbh);
        x = ((LCD.width() - tbw) / 2) - tbx;
        LCD.setCursor(x, y+tbh-20);    LCD.print(DeviceID);
        
        LCD.getTextBounds(Parameter, 0, 0, &tbx, &tby, &tbw, &tbh);
        x = ((LCD.width() - tbw) / 2) - tbx;
        LCD.setCursor(x, 200); LCD.print(Parameter);

        LCD.getTextBounds(Location, 0, 0, &tbx, &tby, &tbw, &tbh);
        x = ((LCD.width() - tbw) / 2) - tbx;
        LCD.setTextColor(GxEPD_BLACK);
        LCD.setCursor(x, 260); LCD.print(Location);

        delay(2000);

    }  while (LCD.nextPage());
}




// first instance of hr: to make hourly notifications....
float cabin_temperature = 0.0f;
uint32_t running_ticks = 0;
bool set_24_hour = false;

void query_rtc(){ //Serial.println("Time Check!"); Serial.println();
  running_ticks++;

  char root[5] = "th";
  uint8_t reminder = 0; // for correcting 24 hour clock

  uint8_t hr = 0, mint = 0, sec = 0, day_ = 0, mth = 0, yr = 0; uint16_t mwaka = 2000; uint64_t screen_timeout = 0; 
  uint8_t datey = 0;
  const char daysOfTheWeek[7][12] = {"Sun", "Mon", "Tue", "Wed", "Thur", "Fri", "Sat"};
  const char Moonth[12][12] = {"January", "February", "March", "April", "May", "June", "July", "August", "September", "October", "November", "December"};
  const char Mon_th[12][10] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sept", "Oct", "Nov", "Dec"};

  char hr_str[5] = ""; char min_str[5] = ""; char sec_str[5] = ""; 
  char day_str[15] = ""; char date_str[10] = ""; char mth_str[15] = ""; char yr_str[7] = ""; 
  char zero_holder[4] = "0"; char bucket[4] = ""; 

  // if(running_ticks%10 == 0){ //query for time once every 10 seconds

            DateTime time_now = real_time.now();

              hr = time_now.hour(); mint = time_now.minute();  sec = time_now.second();
              datey = time_now.day(); mth = time_now.month();  mwaka = time_now.year(); day_ = time_now.dayOfTheWeek();
              cabin_temperature = real_time.getTemperature();
            if(hr > 24){ 
                // DateTime now = real_time.now();
              // while(recursive_counter < 5){ query_rtc(); recursive_counter++;}

                hr = 0; mint = 0; sec = 0;  /*beep(2);*/ Serial.println("Time Chip Failed!");

              }

              if(!set_24_hour){ // if 12 hour is preferred
                    reminder = (hr%12);
                  if(hr<=12) itoa(hr, hr_str, 10);  // 00: - 12:
                  else   itoa(reminder, hr_str, 10); // 13: - 23:
              } itoa(hr, hr_str, 10); 

                itoa(mint, min_str, 10); 
                itoa(sec, sec_str, 10);  
                itoa(datey, date_str, 10); 
                strcpy(day_str, daysOfTheWeek[day_]);
                strcpy(mth_str, Moonth[mth-1]);  
                itoa(mwaka, yr_str, 10);

                  
              //construct the time sequences
        if(hr<=9){  strcpy(ShortTime, zero_holder); strcat(ShortTime, hr_str);} 
        else {
             strcpy(ShortTime, hr_str); 
        }
        strcat(ShortTime, ":"); 
        if(mint<=9){strcat(ShortTime, zero_holder);} 
                    strcat(ShortTime, min_str); // HH:MM

                    strcpy(SystemTime, ShortTime);  strcat(SystemTime, ":"); strcat(SystemTime, sec_str); // HH:MM:SS

          strcat(ShortTime, (hr<12)?" am":" pm");
          //construct the date sequence
        //      strcpy(SystemDate, day_str); strcat(SystemDate, " "); strcat(SystemDate, mth_str); strcat(SystemDate, ", "); strcat(SystemDate, yr_str);
         strcpy(SystemDate, day_str); strcat(SystemDate, " "); strcat(SystemDate, date_str);
         if(datey == 1 || datey == 21 || datey == 31) strcpy(root, "st");
         if(datey == 2 || datey == 22 ) strcpy(root, "nd");
         if(datey == 3 || datey == 23 ) strcpy(root, "rd");
         else strcpy(root, "th");
         strcat(SystemDate, root); strcat(SystemDate, " ");
         strcat(SystemDate, mth_str); strcat(SystemDate, ", "); strcat(SystemDate, yr_str);

                  Serial.println();
                  Serial.print("Internal Temperature: "); Serial.println(cabin_temperature);
                  Serial.print("Short Time: ");  Serial.println(ShortTime);
                  Serial.print("Full System Time: "); Serial.print(SystemTime); 
                  Serial.print("\tSystem Date: "); Serial.println(SystemDate); Serial.println();
                  Serial.println();
    //    }



}


char initializer[72] = "";
void initialize_RTC(){
  uint8_t trial = 0;

  while(trial < 5){
    clock_is_working = real_time.begin();
    if(clock_is_working) break;
    else trial++;
    delay(500); // retry delay
  }

  if(!clock_is_working) Serial.println("Clock not Found!");
  else { Serial.println("Clock Found");


    if (real_time.lostPower()) {
              strcpy(initializer, "RTC lost power, let's set the time!"); // When time needs to be set on a new device, or after a power loss, the
              
              real_time.adjust(DateTime(F(__DATE__), F(__TIME__)));  // This line sets the RTC with an explicit date & time, for example to set // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
              
              strcpy(initializer, "Clock Started and time set!");
    }
  }
}

