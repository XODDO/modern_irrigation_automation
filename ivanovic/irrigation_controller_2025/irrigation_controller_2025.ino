/*
  IVANOVIC IRRIGATION CONTROLLER
*/ 
const char *devicename = "Irrigation_Ctrl_CPU";
#define FW_VERSION "v1.3.5-ESP32-IrrigationController"

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
#include "WiFi_Manager.h"

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

// my own classes
#include "config.h"
#include "credentials.h"
#include "WiFi_Manager.h"

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
float MonitorBattery();

void update_display();
void BootScreen();
void homepage();
void footer();
void graphpage();
void tablescreen();
void logs();
void uploading_screen();

void log_irrigation_summary();
float measure_flow_rate(bool irrigating_status, uint64_t irrigation_duration_seconds);

bool switch_radio_to_wifi(); // turn OFF ESP-NOW, turn ON WiFi
bool bind_into_JSON(); // generate a data bundle
bool switch_radio_to_esp_now(); // 

void uptime(uint64_t loop_time);
void initialize_RTC();
void query_rtc();
void irrigate(float average_reading, float BatteryVoltage, uint64_t time_now_us);
float maker_of_averages(float reading_1, float reading_2);

enum class OperationMode {
    ESP_NOW,
    OTA_ACTIVE,
    OTA_FAILED
};

OperationMode currentMode = OperationMode::ESP_NOW;

bool otaModeActive = false;
bool wifi_connected = false;
uint64_t otaStartTime = 0;
const uint64_t OTA_TIMEOUT = 15ULL * 60ULL * 1000ULL;  // 15 minutes
uint64_t last_check = 0;
char LastOTAUpdate[60] = "27 Oct 2025 10:10";

// Function declarations
void initializeOTA();
bool read_button(uint8_t pin,uint64_t time_now);
bool switch_radio_to_wifi();
bool switch_radio_to_espnow();
void flash(uint64_t flash_time, uint8_t heartbeat,
           uint16_t ON_1 = 50, uint16_t OFF_1 = 75,
           uint16_t ON_2 = 50, uint16_t OFF_2 = 1500);

// --- OTA logging ---
char ota_log[150] = "...";


// EPD DISPLAY SETUP

// base class GxEPD2_GFX can be used to pass references or pointers to the LCD instance as parameter, uses ~1.2k more code
// enable or disable GxEPD2_GFX base class
#define ENABLE_GxEPD2_GFX 0
// pin definitions for the ESP32 Dev Board
// ESP32 CS(SS)=5,SCL(SCK)=18,SDA(MOSI)=23,
 // uint8_t CS_ = 5,  DC_ = 15, RES_ = 13, BUSY_ = 4;
    uint8_t CS_ = 5,  DC_ = 4, RES_ = 13, BUSY_ = 15;

// 4.2'' EPD Module
   GxEPD2_BW<GxEPD2_420_GDEY042T81, GxEPD2_420_GDEY042T81::HEIGHT> LCD(GxEPD2_420_GDEY042T81(CS_, DC_, RES_, BUSY_)); // 400x300, SSD1683
// GxEPD2_3C<GxEPD2_420c_GDEY042Z98, GxEPD2_420c_GDEY042Z98::HEIGHT> LCD(GxEPD2_420c_GDEY042Z98(CS_, DC_, RES_, BUSY_)); // 400x300, SSD1683

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

float SystemBatteryVoltage = 0.0f;

float average_soil_moisture = 0.0f;

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
char node_1_last_seen[32] = "HH:MM:SS";
char node_2_last_seen[32] = "HH:MM:SS";
float sensor_1_readings[30]; // up to 10 readings ago
char sensor_1_time_stamps[30][32] = {"10:10", "10:11", "10:12", "10:13"};

float sensor_2_readings[30]; // up to 30 readings
char sensor_2_time_stamps[30][32] = {"10:10", "10:11", "10:12", "10:13"};

void send_to_wifi_core();

const uint8_t wifi_led     = 25;
const uint8_t ota_button   = 2;
const uint8_t heartbeatLED = 27;

const uint8_t sensor_1_active_pin = 33; // problems with this pin?
const uint8_t sensor_2_active_pin = 26;
const int irrigation_valve_pin = 12; // GPIO pin controlling the irrigation valve
const int water_flow_sensor_pin = 39; // VN pin for the flow sensor
uint8_t active_sensors = 0;

Buzzer buzzer(14);   //uint8_t buzzer = 14;     // buzzer pin
Battery batt(36, 14.40, 1);         // V+ pin, scaling factor, num batteries
WiFi_Manager wifi_obj(wifi_led);

uint64_t now_now = 0;

int sent_bytes = 0;
float cabin_temperature = 0.0f;

char next_upload_schedule[10] = "XX:YY:ZZ";




// make OTA flags volatile
volatile bool otaFinished = false;
volatile bool otaStarted = false;
volatile bool otaError = false;


void setup() { delay(500);  // for OTAs to settle
      Serial.begin(115200); vTaskDelay(pdMS_TO_TICKS(100)); 
      Serial.println(); Serial.printf("------Firmware %s booting...\n", FW_VERSION); // Serial.print("Ready for OTA at IP: ");Serial.println(WiFi.localIP());

      Serial2.begin(115200, SERIAL_8N1, 16, 17);
     
      buzzer.begin(); // comes with pinMode(buzzer, OUTPUT);
         
     
  // CORE TASK 1
      xTaskCreatePinnedToCore(VeryFastTask, "BuzzingCheck", 2048, NULL, PRIORITY_HIGH, NULL, 1);
      Serial.println("\tBuzzer Task initialized!");
      
      buzzer.beep(1,100,0); // now this can go off in exactly 100ms before the loop comes in

  //WATER MATTERS: THE SOLENOID & THE FLOW SENSOR
      pinMode(irrigation_valve_pin, OUTPUT); // GPIO pin controlling the irrigation valve
      pinMode(water_flow_sensor_pin, INPUT); // GPIO pin for the flow sensor
  
      pinMode(ota_button, INPUT_PULLUP);

    //INDICATORS
      pinMode(heartbeatLED, OUTPUT);  digitalWrite(heartbeatLED, HIGH);  vTaskDelay(pdMS_TO_TICKS(100));
      pinMode(sensor_1_active_pin, OUTPUT); digitalWrite(sensor_1_active_pin, HIGH); vTaskDelay(pdMS_TO_TICKS(100));
      pinMode(sensor_2_active_pin, OUTPUT); digitalWrite(sensor_2_active_pin, HIGH); vTaskDelay(pdMS_TO_TICKS(100));

    //SOLENOID
      pinMode(irrigation_valve_pin, OUTPUT); digitalWrite(irrigation_valve_pin, HIGH); vTaskDelay(pdMS_TO_TICKS(100));

   //   xTaskCreatePinnedToCore(ScreenTask, "UpdateDisplay", 4096, NULL, PRIORITY_NORMAL, NULL, 1);

  // POWER MANAGEMENT   
      batt.begin(); //comes with pinMode(batteryPin, INPUT); and analogReadResolution(12) and analogSetAttenuation(ADC_11db);
      batt.onLow(beepLowBattery);
      batt.onRecovered(BatteryRecovered);

      Serial.println("[BATTERY] Performing initial voltage check...");
      SystemBatteryVoltage = MonitorBattery(); //       current_power = get_power_state(batt.getLevel());

     Serial.print("\tVoltage => "); Serial.println(SystemBatteryVoltage);

      Serial.println("Initializing RTC..."); vTaskDelay(pdMS_TO_TICKS(100));
       // But if the RTC is not available, do not keep everyone else waiting
        initialize_RTC();
     if(clock_is_working) query_rtc(); // query RTC but do not hang in there long if no response

   //  strcpy(next_upload_schedule, SystemTime);

  //UI MGT TASK

 // DISPLAY
      LCD.init(115200, true, 50, false); vTaskDelay(pdMS_TO_TICKS(500)); // for the hardware SPI to connect to start transactions
      BootScreen(); LCD.hibernate(); vTaskDelay(pdMS_TO_TICKS(500));
      Serial.println("Screen Boot Complete!");
  
 //after establishing power state
    //  initializeWirelessCommunication();
     // Serial.println("Wireless Communication Initialized!");

    

  // Initialize ESPNOW immediately
    if(switch_radio_to_espnow()) Serial.println("Started in ESPNOW MODE - NORMAL MODE");
    else Serial.println("ESPNOW MODE FAILED!!!");   
    vTaskDelay(pdMS_TO_TICKS(200));
   

      xTaskCreatePinnedToCore(ManageIrrigation, "IrrigationController", 8192, NULL, PRIORITY_NORMAL, NULL, 1);
        
      Serial.println("Irrigation  Task initialized!");

    digitalWrite(heartbeatLED, LOW); vTaskDelay(pdMS_TO_TICKS(200));
    digitalWrite(irrigation_valve_pin, LOW); vTaskDelay(pdMS_TO_TICKS(200));  // turn OFF after init
    digitalWrite(sensor_1_active_pin, LOW); vTaskDelay(pdMS_TO_TICKS(200));
    digitalWrite(sensor_2_active_pin, LOW); vTaskDelay(pdMS_TO_TICKS(200));
    // turn OFF after init

    currentScreen = 1; update_display(); vTaskDelay(pdMS_TO_TICKS(500));

    buzzer.beep(2,50,50); // now this can go off in exactly 50ms before the loop comes in

    Serial.println("SETUP COMPLETED SUCCESSFULLY!");
}


bool is_irrigating = false;
int irrigation_param_update_frequency = (5*60*1000); // default every 5 minute or every 5 seconds but not the screen
uint8_t update_frequency = 0;

void ManageIrrigation(void * pvParams) { 
  TickType_t prev_check_time = xTaskGetTickCount();
 // irrigation_param_update_frequency = is_irrigating? 5000 : 300000; // check more often when irrigating
     irrigation_param_update_frequency = is_irrigating? 2000 : 60000; // check more often when irrigating
   
    while (true) {
        SystemBatteryVoltage = MonitorBattery();

        average_soil_moisture = maker_of_averages(sensor_1_moisture, sensor_2_moisture);
        if(!clock_is_working) uptime(now_now);
            else query_rtc();

            bind_into_JSON();
            send_to_wifi_core();

            if(!is_irrigating) update_display();
            else { update_frequency++; // since an EPD takes 20 seconds to update, call it once every (5*30)secs
              if(update_frequency >= (30)) {update_display(); update_frequency = 0;}
            }
            query_rtc(); // current time in DATETIME
            irrigate(average_soil_moisture, SystemBatteryVoltage, now_now);
            vTaskDelayUntil(&prev_check_time, pdMS_TO_TICKS(irrigation_param_update_frequency));
    }
}

/*
    > irrigation logic based on average soil moisture
    > IF IRRIGATION HAS BEEN ON FOR max_irrigation_duration, FORCED TIMEOUT [EITHER TANK EMPTY OR SENSING FAULT]
    > LOGS INTO A CHAR[] the states: IRRIGATING / NOT IRRIGATING, last irrigation time, duration, cycles today, duration
    > if has not irrigated in a full day, reset cycles today to 0, report it
    > take care of impossible situations [avg_read > 100 or < 1.0]
    > measure batt voltage befopre and after irrigation cycle to see voltage drop
    > if voltage drop is significant, dont start irrigation, log["can't irrigate, low battery"]
*/

// IRRIGATION CONTROLLER 
char solenoid_issue[120] = "...";
char solenoid_fix[120] = "...";

const float wilting_point  = 18.0; // below which irrigation must be done
const float critical_point = 20.0; // threshold to trigger irrigation to start // hysteresis
const float field_capacity = 25.0; // above which irrigation is not needed

const uint8_t max_irrigation_cycles_per_day = 50; // Max cycles per day
const uint32_t irrigation_interval_seconds = 120; // 2 minute interval between irrigations
const uint32_t max_allowed_irrigation_duration_seconds = 30UL*60UL; // 30mins Maximum allowed irrigation duration

double discharge = 0.00; // flow rate in L/min
double water_usage_liters = 0.00;

char irrigation_status_log[256] = "...";
uint32_t elapsed_irrigation_seconds  = 0; // Duration to keep the valve open
char last_irrigation_date[32] = "00-00-0000";
int irrigation_cycles_today = 0;

int64_t irrigation_start_time = 0; // in microseconds
bool irrigation_completed = false;


char irrigation_start_time_str[32] = "10:10:25";
char irrigation_stop_time_str[32] = "10:10:55";

void irrigate(float average_reading, float BatteryVoltage, uint64_t time_now_us){
  bool should_irrigate = false;
  irrigation_status_log[0] = '\0';

  if (BatteryVoltage < 10.5) { // low battery guard
      if(is_irrigating){
          digitalWrite(irrigation_valve_pin, LOW); // Close valve
          is_irrigating = false;
          strcpy(irrigation_status_log, ">>> Low Battery, Skipping irrigation.");
        }
        return;
    }
    

    if (irrigation_cycles_today > max_irrigation_cycles_per_day) { // daily limit guard
        if(is_irrigating){
          digitalWrite(irrigation_valve_pin, LOW); // Close valve
          is_irrigating = false;
          strcpy(irrigation_status_log, ">>> Daily irrigation limit reached.");
        }
        
        return;
    }

    if ((time_now_us - irrigation_start_time)/1e6 < irrigation_interval_seconds) { // interval guard:
        strcpy(irrigation_status_log, ">>> Waiting for next irrigation interval.");
        return;
    }

    

    
    if (strcmp(SystemDate, last_irrigation_date) != 0) { // new day reset
        irrigation_cycles_today = 0;
        strcpy(last_irrigation_date, SystemDate);
    }


  if(average_reading < 3.0){ // impossibly low
    if(is_irrigating){
          digitalWrite(irrigation_valve_pin, LOW); // Close valve
          is_irrigating = false;
          strcpy(irrigation_status_log, ">>> Soil moisture out of range, check sensors.");
    }
  }

  if(average_reading < critical_point){ // a figure (20.0) above wilting point (18.0) and less than field capacity 25.0
    should_irrigate = true;
    irrigation_completed = false;
    elapsed_irrigation_seconds = 0;
    strcpy(irrigation_status_log, ">>> Soil moisture low, irrigation needed.");
  }
     
      //just in case it is returning from irrigating
 else if(average_reading >= field_capacity){ // above 25.0% Volumetric Water Content
      if(is_irrigating){

          digitalWrite(irrigation_valve_pin, LOW); // Close valve
          is_irrigating = false;
          should_irrigate = false; // just defensive... next cycle will land on the else 
          irrigation_cycles_today++;
          irrigation_completed = true;
          elapsed_irrigation_seconds = 0;
          strcpy(last_irrigation_date, SystemDate); // log the date

          strcpy(irrigation_stop_time_str, SystemTime); // log the time
          elapsed_irrigation_seconds = (time_now_us - irrigation_start_time)/1e6;
          snprintf(irrigation_status_log, sizeof(irrigation_status_log),
                   ">>>Field Capacity Reached! Irrigation stopped at %s", SystemTime);
          
      }
      else {
          strcpy(irrigation_status_log, ">>> Soil moisture adequate, no irrigation needed.");
          should_irrigate = false;
      }
    }
 

    else { // if average_reading = 22.0? (Between 20.0 and 25.0)
          // between critical_point and field_capacity
          should_irrigate = false;
          strcpy(irrigation_status_log, ">>> Moisture in acceptable range, no irrigation.");
    }


  if(should_irrigate && !is_irrigating){ // if the reading is low enough and not already irrigating
        Serial.print("Starting Irrigation Cycle..."); // for debugging
        digitalWrite(irrigation_valve_pin, LOW); // Close valve first to be sure
        digitalWrite(irrigation_valve_pin, HIGH); // Open valve
        is_irrigating = true;
        irrigation_start_time = time_now_us; // orirrigation_start_time = now_now // in microseconds
        strcpy(irrigation_start_time_str, SystemTime); // record the starting time
        snprintf(irrigation_status_log, sizeof(irrigation_status_log),
         ">>> Irrigation has started at at %s", irrigation_start_time_str);

      }
  else if(should_irrigate && is_irrigating){ // already irrigating
    // just keep looking at the time... checkin that max duration is not exceeded
    
          if(elapsed_irrigation_seconds  < max_allowed_irrigation_duration_seconds){
              elapsed_irrigation_seconds  = (time_now_us - irrigation_start_time)/1e6; // conv from microseconds
              char duration_str[32] = "";
              sprintf(duration_str, "Irrigation ongoing for %lu seconds", elapsed_irrigation_seconds );
          }

      else if(elapsed_irrigation_seconds  >= max_allowed_irrigation_duration_seconds){
              digitalWrite(irrigation_valve_pin, LOW); // Close valve
              is_irrigating = false;
              irrigation_cycles_today++;
              elapsed_irrigation_seconds = 0;
              strcpy(last_irrigation_date, SystemDate); // log the date

              strcpy(irrigation_stop_time_str, SystemTime); // log the time
              snprintf(irrigation_status_log, sizeof(irrigation_status_log),
              ">>> Irrigation has stopped at %s due to maximum allowable duration (30mins)", irrigation_stop_time_str);
             irrigation_completed = true;
            }  

  }
    else { // either shouldn't irrigate or already irrigating
      if(is_irrigating){
         strcpy(irrigation_status_log, ">>> System Irrigating...");

      }
      if(!should_irrigate){
        digitalWrite(irrigation_valve_pin, LOW); // Close valve
        is_irrigating = false;
        irrigation_cycles_today++;
        strcpy(irrigation_status_log, ">>> Irrigation Cycle Completed., Soil Moisture Adequate.");
      }
    }


    discharge = measure_flow_rate(is_irrigating, elapsed_irrigation_seconds);
    water_usage_liters = discharge * (elapsed_irrigation_seconds / 60.0); // L/min * minutes

    Serial.println(irrigation_status_log);
      if (irrigation_completed) {
      log_irrigation_summary();  // push to cloud or SD
      irrigation_completed = false;
    }
  
}
char flow_sensor_issue[120] = "...";
char flow_sensor_fix[120] = "...";

#define MAX_FLOW_RATE_LPM 10.0 // maximum flow rate in liters per minute

float measure_flow_rate(bool irrigating_status, uint64_t irrigation_duration_seconds){
  bool flow_meter_working = false;
  float flow_rate = 0.0;
  float cummulated_reading = 0.0;

  if(is_irrigating){
    // start measuring flow
    for(int i = 0; i<10; i++){
        int raw_reading = analogRead(water_flow_sensor_pin); // assuming readings go from 0 to 4095
        cummulated_reading += (float)raw_reading;
        vTaskDelay(pdMS_TO_TICKS(10)); // small delay between readings
    }
    cummulated_reading /= 10.0; // average reading
    
    flow_rate = ((cummulated_reading)/4096 )* MAX_FLOW_RATE_LPM;
      
    // implement a settling period  here
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


char irrigation_report[600] = "...";
void log_irrigation_summary(){ // VERBOSE LOG to be uploaded to dashboard
    // irrigation duration, start time, stop time, date, battery voltage before and after irrigation
  /*
 snprintf(irrigation_report, sizeof(irrigation_report),
  "Stopped at %s after %.1f s (%d cycles today) | Flow rate: %.2f L/min | Volume: %.2f Liters | Battery: %.2f V",
  irrigation_stop_time_str, elapsed_irrigation_seconds, irrigation_cycles_today, discharge, water_usage_liters, SystemBatteryVoltage);
  */
  snprintf(irrigation_report, sizeof(irrigation_report),
        "[Irrigation Summary]\n"
        "Date: %s\n"
        "Start: %s\n"
        "Stop: %s\n"
        "Duration: %.1f s\n"
        "Cycles today: %d\n"
        "Flow rate: %.2f L/min\n"
        "Water used: %.2f Liters\n"
        "Battery: %.2f V\n"
        "Valve: %s\n"
        "Flow Sensor: %s\n"
        "Remarks: %s\n",
        SystemDate,
        irrigation_start_time_str,
        irrigation_stop_time_str,
        elapsed_irrigation_seconds,
        irrigation_cycles_today,
        discharge,
        water_usage_liters,
        SystemBatteryVoltage,
        is_irrigating ? "OPEN" : "CLOSED",
        flow_sensor_issue,
        irrigation_status_log
    );

  Serial.println(irrigation_report);
}


char average_status_log[120] = "...";

float maker_of_averages(float reading_1, float reading_2) {
    float average_reading = 0.0f;
    const float DISPARITY_THRESHOLD = 10.0f;
    const float MIN_VALID = 1.0f;   // below this = disconnected/noise
    const float MAX_VALID = 100.0f; // physical upper limit (%)
    active_sensors = 0;

    // --- Sanitize inputs ---
    bool valid1 = !(isnan(reading_1) || reading_1 < MIN_VALID || reading_1 > MAX_VALID);
    bool valid2 = !(isnan(reading_2) || reading_2 < MIN_VALID || reading_2 > MAX_VALID);

    if (!valid1) reading_1 = 0.0f;
    if (!valid2) reading_2 = 0.0f;

    // --- Case 1: Both invalid ---
    if (!valid1 && !valid2) {
        active_sensors = 0;
        strcpy(average_status_log, "Both sensors disconnected or invalid!");
        return 0.0f;
    }

    // --- Case 2: Only one valid ---
    if (valid1 && !valid2) {
        active_sensors = 1;
        average_reading = reading_1;
        strcpy(average_status_log, "Sensor 2 invalid, using Sensor 1 reading only.");
        return average_reading;
    }
    if (!valid1 && valid2) {
        active_sensors = 1;
        average_reading = reading_2;
        strcpy(average_status_log, "Sensor 1 invalid, using Sensor 2 reading only.");
        return average_reading;
    }

    // --- Case 3: Both valid ---
    active_sensors = 2;
    float diff = fabs(reading_1 - reading_2);

    if (diff >= DISPARITY_THRESHOLD) {
        // large discrepancy: trust lower value
        average_reading = min(reading_1, reading_2);
        snprintf(average_status_log, sizeof(average_status_log),
                 "Disparity detected (%.1f vs %.1f). Using lower value: %.1f%%",
                 reading_1, reading_2, average_reading);
    } else {
        // normal condition
        average_reading = (reading_1 + reading_2) / 2.0f;
        snprintf(average_status_log, sizeof(average_status_log),
                 "Both sensors healthy. Average = %.1f%%", average_reading);
    }

    // --- Final clamp ---
    if (average_reading < MIN_VALID) average_reading = 0.0f;
    if (average_reading > MAX_VALID) average_reading = MAX_VALID;

    return average_reading;
}

/*
float maker_of_averages(float reading_1, float reading_2){
  float average_reading = 0.0f;
  if(reading_1 < 1.0 || isnan(reading_1)) { average_reading = reading_2; active_sensors = 1; }
  if(reading_2 < 1.0 || isnan(reading_2)) { average_reading = reading_1; active_sensors = 1; }

  if(reading_1 < 1.0 && reading_2 < 1.0) { average_reading = 0.0; active_sensors = 0; return average_reading; }

  if((reading_1 - reading_2) > 10.0 || (reading_2 - reading_1) > 10.0){
    // disparity too wide... take lower reading
    average_reading = min(reading_1, reading_2); active_sensors = 2;
  }
else {
  average_reading = (reading_1 + reading_2)/2.0;
  active_sensors = 2;
}
  

  return average_reading;
}
*/


char  sendable_to_wifi_core[4096];
char sendable_to_wifi_core2[4096];

// For debug logging
#define LOG(msg)  Serial.println(msg)
//JsonDocument JSON_sendable; // Static is deprecated 

bool bind_into_JSON(){
           LOG("[JSON] Starting JSON binding process...");
           bool bound_successfully = false;
           JSON_sendable.clear();

          // --- 1 System block ---
          JsonObject System      = JSON_sendable["System"].to<JsonObject>();
          System["FW_Version"]       = FW_VERSION;
          System["Time"]             = SystemTime;
          System["Uptime"]           = (now_now/1000); // in seconds
          System["Date"]             = SystemDate;
          System["Battery_Voltage"]  = SystemBatteryVoltage;
          System["Average_Moisture"] = average_soil_moisture;
          System["Active_Sensors"]   = active_sensors;
          System["Average_Status"]   = average_status_log;  
          System["Internal_Temperature"] = cabin_temperature;
          System["OTA_Status"] = ota_log;
          System["LastOTA_Update"] = LastOTAUpdate;

          LOG("[JSON] System data added.");

            // --- 2 Solenoid / Irrigation Section ---
          JsonObject Solenoid        = JSON_sendable["Solenoid"].to<JsonObject>();
          Solenoid["State"]           = is_irrigating; // either a string or boolead ("OPEN"/"CLOSED" or true/false)
          Solenoid["Last_Irrigation"] = irrigation_stop_time_str;
          Solenoid["Duration"]        = elapsed_irrigation_seconds;
          Solenoid["Cycles_Today"]    = irrigation_cycles_today;
          Solenoid["Status_Log"]      = irrigation_status_log;
          
          LOG("[JSON] Solenoid data added.");

          // Irrigation Summary
          JsonObject Irrigation  = JSON_sendable["Irrigation_Summary"].to<JsonObject>(); 
          Irrigation["Report"] = irrigation_report; // big string
          LOG("[JSON] Irrigation summary added.");

          // --- 3 Water usage ---
          JsonObject Flow_Sensor = JSON_sendable["Flow_Sensor"].to<JsonObject>();
          Flow_Sensor["Water_Usage_Liters"] = water_usage_liters; // water_usage_liters = flow_rate * (irrigation_duration_seconds / 60.0);
          Flow_Sensor["Flow_Rate_LPM"]      = discharge; // measure_flow_rate(is_irrigating, elapsed_irrigation_seconds);
          Flow_Sensor["Issue"] = flow_sensor_issue;
          Flow_Sensor["Fix"]   = flow_sensor_fix;
          
          LOG("[JSON] Water usage added.");


           // --- 4 Populate Sensor 1 Data ---
          JsonObject Sensor_1    = JSON_sendable["Sensor_1"].to<JsonObject>();
          Sensor_1["Position"]     = "SENSOR A1";
          Sensor_1["Moisture"]     = sensor_1_moisture;
          Sensor_1["Voltage"]      = sensor_1_voltage;
          Sensor_1["PwrMde"]       = sensor1_POWER_XTIX;
          Sensor_1["Last_Seen"]    = node_1_last_seen;
          Sensor_1["Sends"]        = sensor_1_transmissions;

            // time series readings
          JsonArray readings1 = Sensor_1["Readings"].to<JsonArray>();
          JsonArray times1    = Sensor_1["Timestamps"].to<JsonArray>();
          for (int i = 0; i < 30; i++) {
              readings1.add(sensor_1_readings[i]);
              times1.add(sensor_1_time_stamps[i]);
          }

          LOG("[JSON] Sensor 1 data added.");

          // --- 5 Populate Sensor 2 Data ---
          JsonObject Sensor_2    = JSON_sendable["Sensor_2"].to<JsonObject>();
          Sensor_2["Position"]     = "SENSOR A2";
          Sensor_2["Moisture"]     = sensor_2_moisture;
          Sensor_2["Voltage"]      = sensor_2_voltage;
          Sensor_2["PwrMde"]       = sensor2_POWER_XTIX;
          Sensor_2["Last_Seen"]    = node_2_last_seen;
          Sensor_2["Sends"]        = sensor_2_transmissions;

          // time series readings
          JsonArray readings2 = Sensor_2["Readings"].to<JsonArray>();
          JsonArray times2    = Sensor_2["Timestamps"].to<JsonArray>();
          for (int i = 0; i < 30; i++) {
              readings2.add(sensor_2_readings[i]);
              times2.add(sensor_2_time_stamps[i]);
          }

          LOG("[JSON] Sensor 2 data and nested arrays added.");


          // --- 7️⃣ Serialize into a buffer or string ---
          char payload[4096];
      //    size_t len = serializeJson(JSON_sendable, sendable_to_wifi_core, sizeof(payload));
           size_t len = serializeJsonPretty(JSON_sendable, sendable_to_wifi_core2);

    if (serializeJson(JSON_sendable, sendable_to_wifi_core, sizeof(sendable_to_wifi_core)) == 0) {
        Serial.println("Serialization failed");
    }

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
//ArduinoJson can easily exceed 1024 bytes with arrays.
void send_to_wifi_core(){ // Avoid Serial2 send buffer overflow
    //check if there is something on that serial line before sending data into Deep Space

   // Serial.println("JSON File to send: "); Serial.println(sendable_to_wifi_core);

    Serial.println("JSON File to send(Pretty): "); Serial.println(sendable_to_wifi_core2);

// But UART TX buffer is small (~256 bytes by default). 
//If your JSON >256 bytes and the other side isn’t reading fast, you’ll block.

  //  Serial2.write((uint8_t*)sendable_to_wifi_core, strlen(sendable_to_wifi_core));

      uint64_t starting_send = esp_timer_get_time();
    //int  sent_bytes = Serial2.print(sendable_to_wifi_core);  // this ~320B takes 20 milliseconds 
         sent_bytes = Serial2.print(sendable_to_wifi_core2); // this 490B takes about 40 milliseconds
      uint64_t stopped_sending = esp_timer_get_time();

      Serial.print("UART send Duration: "); Serial.print((stopped_sending - starting_send)/1000); Serial.println("milliseconds");
      Serial.print(sent_bytes); Serial.println(" Bytes pushed your into the UART TX ring buffer.");
    //Serial2.flush();   // blocks until UART TX buffer is empty

 /*
        if (sent_bytes > 0) {
            Serial.println("Data queued for transmission.");
        } else {
            Serial.println("Failed to queue data.");
        }
 */

      //Use print() for strings, JSON, numbers as text.
     // Use write() for raw binary data, byte arrays, or when efficiency really matters.
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

int uploading_interval = 10; // 10 minutes

bool uploaded = false;
int counter = 1;
uint64_t last_loop_time = 0;
void loop() {
  now_now = esp_timer_get_time()/1000ULL; // in milliseconds 
  
  /*
  // Check long press on button
  if (read_button(ota_button, now_now) && !otaModeActive) {
    esp_err_t result = esp_now_deinit();
    if (result == ESP_OK) {
        Serial.println("ESP-NOW deinitialized successfully");
            delay(500); // helping the PHY radio to reinitialize cleanly.
        switch_radio_to_wifi(); // Transition to WiFi mode
       } else {
           Serial.printf("ESP-NOW deinit failed: %d\n", result);
          return;
        }

      Serial.println("\n=== OTA Triggered ===");
      otaModeActive = true;
      otaStartTime = now_now;

    wifi_connected = wifi_obj.ensure_wifi();
    if (wifi_connected) {
      initializeOTA();
      Serial.println();
      Serial.printf("TIME: %llu OTA Mode started Active for %llu minutes...", now_now/1000, (OTA_TIMEOUT/60000));
    }
    else {
      Serial.println("WiFi connection failed - returning to ESP-NOW");
      otaModeActive = false;
      switch_radio_to_espnow();
    }
  }

  if (otaModeActive) {
    ArduinoOTA.handle();
    flash(now_now, heartbeatLED, 100, 100, 100, 100); // Rapid blink

    if (otaFinished) { // let it be seen and heard
            strcpy(ota_log, "Update complete!"); 
            snprintf(LastOTAUpdate, sizeof(LastOTAUpdate), "On Date: %s, At Time, %s", SystemDate, ShortTime);
            buzzer.beep(2, 100, 50);
            delay(50);
            flash(now_now, heartbeatLED, 1000, 100, 0, 0);
            otaFinished = false;
      }

     if(otaStarted){
        strcpy(ota_log, "Start updating...");
        buzzer.beep(1, 50, 0); 
        delay(50);
        otaStarted = false;
      }
    if(otaError){
      buzzer.beep(1, 500, 500);
      delay(500);
      otaError = false;
    }

    // Timeout after 5 minutes
    if ((now_now - otaStartTime) >= OTA_TIMEOUT) {
        Serial.println();
        Serial.printf("OTA timeout reached!!! Started at: %llu, Ended at: %llu ", otaStartTime/1000, now_now/1000);
        Serial.println("Returning to ESP-NOW mode...");
      
        otaModeActive = false;
        switch_radio_to_espnow();
    }
  } // !otaModeActive
  else {
      */    
    // Normal ESP-NOW operation + slow heartbeat
    flash(now_now, heartbeatLED, 50, 75, 50, 1500); // IDLE
     // TODO: your ESP-NOW logic here
    //  BUT ESP-NOW LOGIC IS HANDLED BY RTOS TASKS

  //}

    


}

// ------------------------------------------------------
// ------------- FUNCTION IMPLEMENTATIONS ---------------
// ------------------------------------------------------

void initializeOTA() {
  ArduinoOTA.setHostname(devicename);
  ArduinoOTA.setPassword("admin");

  ArduinoOTA
      .onStart([]() {  otaStarted = true; }) // starting
      .onEnd([]() {  otaFinished = true;}) // success // short tripple boot
      .onProgress([](unsigned int progress, unsigned int total) {
        snprintf(ota_log, sizeof(ota_log), "Progress: %u%%", (progress * 100) / total);
      })
      .onError([](ota_error_t error) { otaError = true; // failed... 2 long beeps
        snprintf(ota_log, sizeof(ota_log), "Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR)
          strcat(ota_log, "Auth Failed");
        else if (error == OTA_BEGIN_ERROR)
          strcat(ota_log, "Begin Failed");
        else if (error == OTA_CONNECT_ERROR)
          strcat(ota_log, "Connect Failed");
        else if (error == OTA_RECEIVE_ERROR)
          strcat(ota_log, "Receive Failed");
        else if (error == OTA_END_ERROR)
          strcat(ota_log, "End Failed");
      });
  delay(5);
  ArduinoOTA.begin();
  strcpy(ota_log, "OTA Ready!");
  Serial.println(ota_log);
}

// Simple long-press detector (>=3 seconds)

bool read_button(uint8_t pin, uint64_t time_now){
  static uint64_t pressStart = 0;
  static bool lastState = HIGH;
  static bool longPressHandled = false;
  bool pressed = false;

  bool state = digitalRead(pin);

  if (state == LOW && lastState == HIGH) {
    pressStart = time_now;
    longPressHandled = false;
  }

  if (state == LOW && !longPressHandled && (time_now - pressStart >= 3000ULL)) {
    pressed = true;
    longPressHandled = true;
    buzzer.beep(1, 50, 0);
  }

  if (state == HIGH && lastState == LOW) {
    longPressHandled = false;
  }

  lastState = state;
  return pressed;
}

// replace switch_radio_to_espnow() with the version from the "Fix B" snippet above
// and replace OnSensorData_received with the safer copy version above


// --- Radio mode switching ---
bool switch_radio_to_wifi() {
  Serial.println("Switching radio to Wi-Fi...");
  WiFi.mode(WIFI_STA); Serial.printf("WiFi channel: %d\n", WiFi.channel());

  delay(500);
  wifi_obj.initialize_ESP_WiFi(devicename);

  return true; // initializeWiFi successful
}

#define ESPNOW_MAX_RETRIES 3

bool switch_radio_to_espnow() {
  Serial.println("Switching radio to ESP-NOW...");

  /*
      // Proper cleanup
    if (esp_now_deinit() != ESP_OK) {
        Serial.println("Warning: ESP-NOW deinit had issues");
    }
  */  
    WiFi.disconnect(true);
    delay(100);
    WiFi.mode(WIFI_OFF);
    delay(100);
    
    WiFi.mode(WIFI_STA); Serial.printf("WiFi channel: %d\n", WiFi.channel());

  for (uint8_t attempt = 1; attempt <= ESPNOW_MAX_RETRIES; attempt++) {
      if (esp_now_init() == ESP_OK) { 
            Serial.printf("✓ ESP-NOW initialized successfully (attempt %d/%d)\n", attempt, ESPNOW_MAX_RETRIES);
      
   // Register callback function
          esp_err_t result = esp_now_register_recv_cb(esp_now_recv_cb_t(OnSensorData_received)); // Serial.print("CALLBACK: ");Serial.println(result);
          if(result == ESP_OK) Serial.println("Call Back of Call Back successfully set!"); 
           
        } else {
           // print that Wireless Radio is faulty
            //return;
            delay(100 * attempt); // Exponential backoff
        }
        
    }
    
    
    Serial.println("!!! ESP-NOW initialization failed after retries");
    return false;
}

// --- Flash pattern generator ---
uint64_t prev = 0;
uint8_t phase = 0;
void flash(uint64_t flash_time, uint8_t heartbeat,
           uint16_t ON_1, uint16_t OFF_1,
           uint16_t ON_2, uint16_t OFF_2) {
  uint64_t interval = 0;
  switch (phase) {
    case 0: interval = ON_1; break;
    case 1: interval = OFF_1; break;
    case 2: interval = ON_2; break;
    case 3: interval = OFF_2; break;
  }

  if ((flash_time - prev) >= interval) {
    prev = flash_time;
    phase = (phase + 1) % 4;
    bool ledState = (phase == 0 || phase == 2);
    digitalWrite(heartbeat, ledState ? HIGH : LOW);
  }
}


char time_str[32] = "";

void uptime(uint64_t loop_time){
        uint64_t seconds = loop_time;
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


void initialize_indicators(){
      //pinMode(indicator, OUTPUT);  digitalWrite(indicator, HIGH);
      pinMode(sensor_1_active_pin, OUTPUT); digitalWrite(sensor_1_active_pin, HIGH); vTaskDelay(pdMS_TO_TICKS(500));
      pinMode(sensor_2_active_pin, OUTPUT); digitalWrite(sensor_2_active_pin, HIGH);  vTaskDelay(pdMS_TO_TICKS(500));
      Serial.println("Indicators initialized!"); 
}

bool can_send_wirelessly = false;



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
      extract_readings();
     
}


uint8_t reading_1_count = 0;
uint8_t reading_2_count = 0;

float moi_readings[6] = {0.0, 0.0, 0.0f, 0.0f, 0.0f, 0.0f};
uint32_t last_seen[6] = {0, 0, 0, 0, 0, 0}; // epoch seconds

//deserialize to assign char[] and floats accordingly
void extract_readings(){ // accumulate average moisture according to sensors active

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
        digitalWrite(sensor_1_active_pin, HIGH); vTaskDelay(pdMS_TO_TICKS(50));

        // --- 1️⃣ Update latest sensor reading ---
        sensor_1_moisture = _1_moisture;
      //  average_soil_moisture += sensor_1_moisture; active_sensors++;

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
        digitalWrite(sensor_1_active_pin, LOW);
  }


  if (_2_moisture > 1.00) {
        digitalWrite(sensor_2_active_pin, HIGH); vTaskDelay(pdMS_TO_TICKS(50)); //  INDICATE ACTIVITY
        // --- 1️⃣ Update latest sensor reading ---
        sensor_2_moisture = _2_moisture;
        //average_soil_moisture += sensor_2_moisture; active_sensors++;
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
     digitalWrite(sensor_2_active_pin, LOW);
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

float MonitorBattery() {
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

   //  dtostrf(batt.getVoltage(), 4, 1, batt_str);   Serial.printf("Voltage: %s V\n", batt_str); Serial.println();
  
     return batt.getVoltage();
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
        LCD.drawTriangle(370, 215, 380, 225, 390, 215, LCD.epd2.hasColor ? GxEPD_RED : GxEPD_BLACK);  // DOWN ARROW

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
         LCD.setCursor(60, 125); LCD.print(node_1_last_seen);

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
          LCD.fillRoundRect((14+box_width+(1.2*box_width)), 30, 36, 20, 20, GxEPD_RED); // notification bubble
        }
        
        LCD.setTextColor(GxEPD_WHITE); LCD.setFont(&FreeSans9pt7b);
        LCD.setCursor(5+(30+(1.2*box_width)), 58); LCD.print("Sensor Node 2"); //AREA HEADING
       
        LCD.setFont(&FreeMono9pt7b);  LCD.setTextColor(GxEPD_BLACK); //FreeMono9pt7b
      //LCD.setCursor(85+(30+(1.2*box_width)), 85); LCD.print("%");

        LCD.setCursor(25+(30+(1.2*box_width)), 125); LCD.print(node_2_last_seen); 
      
        //DATA
         LCD.setFont(); LCD.setCursor((40+(1.2*box_width)), 66); 
         //LCD.print("PERCENT VOLUME");
         LCD.print(sensor2_POWER_XTIX);  //LCD.print("PERCENT VOLUME"); // wind speed log
          
         LCD.setTextColor(GxEPD_WHITE);
         LCD.setCursor(((10+box_width)+(1.2*box_width)), 37); 
         LCD.print(sensor_2_transmissions);

         LCD.setTextColor(GxEPD_BLACK);      LCD.setFont(&FreeSansBold24pt7b); 
         LCD.setCursor(40+((1.2*box_width)), 110);    LCD.print(sensor_2_moisture); 
         

       // FLOW METER
         LCD.drawRoundRect(30, 140, box_width, box_height, 10, (LCD.epd2.hasColor ? GxEPD_RED : GxEPD_BLACK));

         LCD.fillRoundRect(30, 140, box_width, 20, 10, (LCD.epd2.hasColor ? GxEPD_RED : GxEPD_BLACK)); //BLACK RIBBON
         LCD.fillRect(30, 150, box_width, 12, (LCD.epd2.hasColor ? GxEPD_RED : GxEPD_BLACK));

         LCD.setTextColor(GxEPD_WHITE); LCD.setFont(&FreeSans9pt7b);
         LCD.setCursor(60, 158); LCD.print("Flow Meter"); //AREA HEADING
        
         LCD.setFont(&FreeMono9pt7b); LCD.setTextColor((LCD.epd2.hasColor ? GxEPD_RED : GxEPD_BLACK)); //FreeMono9pt7b
         LCD.setCursor(40, (222));    /* LCD.print(elapsed_irrigation_seconds); */ LCD.print(" litres/min");
        //DATA
         LCD.setTextColor((LCD.epd2.hasColor ? GxEPD_RED : GxEPD_BLACK));
         LCD.setFont(&FreeSansBold24pt7b); 
         LCD.setCursor(60, (205));LCD.print(discharge); 

         

    // SOLENOID 
         LCD.drawRoundRect((25+1.2*box_width), 140, box_width, box_height, 10, (LCD.epd2.hasColor ? GxEPD_RED : GxEPD_BLACK));

         LCD.fillRoundRect((25+1.2*box_width), 140, box_width, 20, 10, (LCD.epd2.hasColor ? GxEPD_RED : GxEPD_BLACK)); //BLACK RIBBON
         LCD.fillRect((25+1.2*box_width), 150, box_width, 12, (LCD.epd2.hasColor ? GxEPD_RED : GxEPD_BLACK));

         LCD.setTextColor(GxEPD_WHITE); LCD.setFont(&FreeSans9pt7b);
         LCD.setCursor((60+1.2*box_width), 158); LCD.print("Solenoid"); //AREA HEADING

         
      //DATA
         LCD.setTextColor((LCD.epd2.hasColor ? GxEPD_RED : GxEPD_BLACK));
         LCD.setFont(&FreeSansBold24pt7b); 
         LCD.setCursor(20+(60+box_width), (205));    LCD.print(is_irrigating?"ON":"OFF");

         LCD.setFont(&FreeSans9pt7b); LCD.setTextColor((LCD.epd2.hasColor ? GxEPD_RED : GxEPD_BLACK)); //FreeMono9pt7b
         LCD.setCursor(70+(box_width), (222)); LCD.print("Last "); LCD.print(irrigation_stop_time_str);
        
        

         // how many sensors
  //      LCD.setFont();
        
       
                // LOGS FOR NETWORK
         LCD.setFont(); 
         LCD.setTextColor(LCD.epd2.hasColor ? GxEPD_RED : GxEPD_BLACK);
         LCD.setCursor(10, 235);  LCD.print(active_sensors); LCD.print(" Sensors Active:  "); LCD.print(average_status_log);
        // LCD.setCursor(250,235);
        
        
        LCD.setFont(&FreeMono9pt7b);
        LCD.setCursor(15, 260); LCD.print(sent_bytes); LCD.print(" Bytes.");
        LCD.setCursor(140, 260); LCD.print("Next Upload: "); LCD.print(next_upload_schedule);
        // LCD.setCursor(200, 249); LCD.print(uploaded?"✓ Uploaded":"✗ Not Sent");

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
            LCD.fillRect(0, 268, 400, 40, GxEPD_BLACK); // GxEPD_BLACK
          //  LCD.fillRoundRect(2, 259, 396, 0, 8, GxEPD_WHITE);
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
            if(SystemBatteryVoltage >= 6.00) { LCD.setFont(); LCD.setTextColor(LCD.epd2.hasColor ? GxEPD_RED : GxEPD_BLACK);  LCD.setCursor(285, 281); LCD.print(batt.getVoltage());  }
       
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

    }  while (LCD.nextPage());
}

uint32_t running_ticks = 0;
bool set_24_hour = false;

void query_rtc() {
    running_ticks++;

    // --- Fetch RTC values ---
    DateTime time_now = real_time.now();
    uint8_t hr = time_now.hour();
    uint8_t mint = time_now.minute();
    uint8_t sec = time_now.second();
    uint8_t datey = time_now.day();
    uint8_t mth = time_now.month();
    uint16_t mwaka = time_now.year();
    uint8_t day_ = time_now.dayOfTheWeek();
    cabin_temperature = real_time.getTemperature();

    // --- Sanity guard: detect invalid RTC values (e.g., 255) ---
    if (hr > 23 || mint > 59 || sec > 59) {
        Serial.println(" RTC time invalid — resetting to 00:00:00");
        hr = mint = sec = 0;
    }

    // --- Format arrays ---
    const char *daysOfTheWeek[7] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};
    const char *monthsFull[12]   = {"January", "February", "March", "April", "May", "June",
                                   "July", "August", "September", "October", "November", "December"};

    // --- Buffers ---
    char hr_str[4], min_str[4], sec_str[4];
    char date_str[4], yr_str[6];
    char ampm[3] = "";
    char suffix[3] = "th";

    // --- Convert numeric parts ---
    itoa(hr, hr_str, 10);
    itoa(mint, min_str, 10);
    itoa(sec, sec_str, 10);
    itoa(datey, date_str, 10);
    itoa(mwaka, yr_str, 10);

    // --- 12-hour or 24-hour format ---
    if (!set_24_hour) {
        if (hr == 0) hr = 12;
        else if (hr > 12) hr -= 12;
        strcpy(ampm, (time_now.hour() < 12) ? "am" : "pm");
    }

    // --- the zero before: 01, 08, 09, ---
    if (hr < 10) sprintf(hr_str, "0%d", hr);
    if (mint < 10) sprintf(min_str, "0%d", mint);
    if (sec < 10) sprintf(sec_str, "0%d", sec);

    // --- Composing the HH:MM:SS time strings ---
    snprintf(ShortTime, sizeof(ShortTime), "%s:%s %s", hr_str, min_str, set_24_hour ? "" : ampm);
    snprintf(SystemTime, sizeof(SystemTime), "%s:%s:%s", hr_str, min_str, sec_str);

    // --- the th Date suffix ---
    if (datey == 1 || datey == 21 || datey == 31) strcpy(suffix, "st");
    else if (datey == 2 || datey == 22) strcpy(suffix, "nd");
    else if (datey == 3 || datey == 23) strcpy(suffix, "rd");

    // --- Composing the lunaku mwezi mwaka date string ---
    snprintf(SystemDate, sizeof(SystemDate), "%s %d%s %s, %d",
             daysOfTheWeek[day_], datey, suffix, monthsFull[mth - 1], mwaka);

    // --- Hourly/periodic triggers ---
    if (mint == 0 && sec == 0) {
        Serial.println("Hourly trigger — could push log or sensor sync here.");
    }

    // --- Next upload scheduler (every 10 minutes) ---
    if (uploaded) {
        uint8_t next_hr = hr, next_min = mint + 10;
        if (next_min >= 60) {
            next_min -= 60;
            next_hr = (next_hr + 1) % 24;
        }
        snprintf(next_upload_schedule, sizeof(next_upload_schedule), "%02d:%02d", next_hr, next_min);
    }

    // --- Display ---
    Serial.println();
    Serial.printf("Internal Temp: %.2f °C\n", cabin_temperature);
    Serial.printf("Time: %s\nDate: %s\n", SystemTime, SystemDate);
    Serial.println();
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
     //   real_time.adjust(DateTime(2025, 10, 28, 9, 30, 0)); // comment afterwards

  }
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


/*
const TickType_t upload_interval = pdMS_TO_TICKS(10 * 60 * 1000);  // 10 min
char dataBundle[1024] = "S1:28.37";

// this is handled by the IoT core, as it is always connected to the WiFi
void upload_task(void *pvParams) {
    TickType_t prev_wake_time = xTaskGetTickCount();

    while (true) {
        currentScreen = 10;
        update_display(); // Indicate upload in progress

        bool wifi_enabled = switch_radio_to_wifi(); // Switch from ESP-NOW to WiFi

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

            switch_radio_to_esp_now(); // Return to ESP-NOW mode
        } else {
            Serial.println("WiFi unavailable. Retrying next cycle.");
            switch_radio_to_esp_now(); // Ensure radio state restored
        }

        UBaseType_t highWaterMark = uxTaskGetStackHighWaterMark(NULL);
        Serial.print("Upload Task Stack Remaining: ");  Serial.println(highWaterMark);
        Serial.println();

        // Sleep until the next upload cycle
        vTaskDelayUntil(&prev_wake_time, upload_interval);
    }
}

*/


/* RTC QUERY FUNCTION

// first instance of hr: to make hourly notifications....
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
            if(hr > 24){  // e.g returning 255
                // DateTime now = real_time.now();
              // while(recursive_counter < 5){ query_rtc(); recursive_counter++;}

                hr = 0; mint = 0; sec = 0;   Serial.println("Time Chip Failed!");
                
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

                    if(uploaded){
              //SETTING UPLOAD CYCLES
                    char temp_min[5] = ""; char temp_hr[5] = "";
                    if(mint < 50) {
                       itoa(mint+10, temp_min, 10);
                       snprintf(next_upload_schedule, sizeof(next_upload_schedule),
                         hr_str, ":", temp_min);
                    }
                    else { // HH:MM 10:55 > 11:05
                        itoa((hr+1), temp_hr, 10);
                        itoa(mint%10, temp_min, 10);
                         snprintf(next_upload_schedule, sizeof(next_upload_schedule),
                         temp_hr, ":", temp_min);
                    }
                  }
                     
                    

                    strcpy(SystemTime, ShortTime);  strcat(SystemTime, ":"); strcat(SystemTime, sec_str); // HH:MM:SS

          strcat(ShortTime, (hr<12)?" am":" pm");
          //construct the date sequence
        //      strcpy(SystemDate, day_str); strcat(SystemDate, " "); strcat(SystemDate, mth_str); strcat(SystemDate, ", "); strcat(SystemDate, yr_str);
         strcpy(SystemDate, day_str); strcat(SystemDate, " "); strcat(SystemDate, date_str);
         if(datey == 1 || datey == 21 || datey == 31) strcpy(root, "st");
         if(datey == 2 || datey == 22 ) strcpy(root, "nd");
         if(datey == 3 || datey == 23 ) strcpy(root, "rd");
         else strcpy(root, "th");
        // strcat(SystemDate, root); 
         strcat(SystemDate, " ");
         strcat(SystemDate, mth_str); strcat(SystemDate, ", "); strcat(SystemDate, yr_str);

                  Serial.println();
                  Serial.print("Internal Temperature: "); Serial.println(cabin_temperature);
                  Serial.print("Short Time: ");  Serial.println(ShortTime);
                  Serial.print("Full System Time: "); Serial.print(SystemTime); 
                  Serial.print("\tSystem Date: "); Serial.println(SystemDate); Serial.println();
                  Serial.println();
    //    }



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

*/

