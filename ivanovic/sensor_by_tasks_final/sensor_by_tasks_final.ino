#define FW_VERSION "v1.3.2-ESP32-SoilNode"

#include <Arduino.h>
#include <Wire.h> // for RTC
#include <SPI.h> // for EPD

#include <Adafruit_GFX.h> // for EPD
#include <GxEPD2_BW.h> // for EPD

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

#include <Fonts/FreeMonoBold9pt7b.h>
#include <Fonts/FreeMonoBold12pt7b.h>
#include <Fonts/FreeSansBold24pt7b.h> // FreeSerifBold24pt7b
#include <Fonts/FreeSansBold18pt7b.h>
#include <Fonts/FreeSans12pt7b.h>
#include <Fonts/FreeSansBold9pt7b.h>

#include <Fonts/FreeSans9pt7b.h>

// Peer info 
esp_now_peer_info_t peerInfo;


StaticJsonDocument<256> JSON_data; //100 bytes is very tight. With multiple floats + strings you’ll overflow.
//JsonDocument JSON_data;

Buzzer buzzer(14);   //uint8_t buzzer = 14;     // buzzer pin
Battery batt(32, 8.40, 1);         // pin, scaling factor, num batteries

//indicative LEDs
uint8_t high = 26; // green
uint8_t medium = 27; //orange
uint8_t low   = 25;//red
uint8_t indicator = 2;

//sensor etc
uint8_t sensorPin = 36;
uint8_t sensorRelay = 2;
uint8_t solarPin = 16;  // for waking up from deep slumber  //uint8_t batteryPin = 35;

// base class GxEPD2_GFX can be used to pass references or pointers to the display instance as parameter, uses ~1.2k more code
// enable or disable GxEPD2_GFX base class
#define ENABLE_GxEPD2_GFX 0

// 1.54'' EPD Module
   GxEPD2_BW<GxEPD2_154_D67, GxEPD2_154_D67::HEIGHT> LCD(GxEPD2_154_D67(/*CS=5*/ 5, /*DC=*/ 4, /*RES=*/ 13, /*BUSY=*/ 15)); // GDEH0154D67 200x200, SSD1681
// 1.54'' EPD Module (3-Color: B/W/R)
// GxEPD2_3C<GxEPD2_154_D67, GxEPD2_154_D67::HEIGHT> LCD(GxEPD2_154_D67(/*CS=*/5, /*DC=*/4, /*RES=*/13, /*BUSY=*/15)); // GDEW0154Z17 200x200, SSD1681
// ESP32 CS(SS)=5,SCL(SCK)=18,SDA(MOSI)=23,BUSY=15,RES(RST)=2,DC=0

#define SEND_TIMEOUT_MS 1500
#define SEND_MAX_RETRIES 3
const size_t MIN_STACK_SAFETY_MARGIN = 200;


// ===== FreeRTOS Tasks =====
void VeryFastTask(void *pvParams);
void CombinedTasks(void *pvParams);
//bool safe_send_data_task(uint8_t max_retries = SEND_MAX_RETRIES);


// ===== Callbacks =====
void beepLowBattery();
void BatteryRecovered();
void MonitorBattery();
void sleep_dynamically();
bool safe_send_data(uint8_t max_retries = 3);
bool prepareJSONfile(char* out_buf, size_t out_size, size_t &len);

void init_send_system();
void update_display();
void soilMoi_Screen();
void dataUpload_Screen();
void Power_Wifi_Screen();
void LowPower_Screen();
void BootScreen();


// Esp-Now Call Back
void OnDataSent(const wifi_tx_info_t *mac_addr, esp_now_send_status_t status);

// work horses
void sense_reading();
float soil_moisture_calibrator(uint16_t working_reading);
     
void initialize_indicators();
bool initializeWirelessCommunication();
void manage_wireless_mode();

typedef struct moistureData{ // all stringified
      char sendable_data_bundle[200] = "";
}  moistureData; 

// ADD: System state tracking
typedef enum {
    SYSTEM_BOOTING,
    SYSTEM_LOW_POWER,
    SYSTEM_OPERATIONAL,
    SYSTEM_ERROR
} system_state_t;

system_state_t currentSystemState = SYSTEM_BOOTING;

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
      return POWER_EXCELLENT; //  if(level  == 3)

}
power_state_t current_power;  // start low until measured shows high

uint64_t dynamic_interval = (10 * 1000); // ms DEFAULT: 10 secs // @Excellent Power

const uint64_t power_critical_sleep_duration = (2 * 3600ULL * 1000000ULL); // 2 hours
const uint64_t power_low_sleep_duration = (900ULL * 1000000ULL); // 15 minutes
const uint64_t power_mod_sleep_duration = (300ULL * 1000000ULL); // 5 minutes
const uint64_t power_excellent_delay = (10 * 1000); // 10 seconds

/*
power_state_t get_power_state(float voltage) {

    if(voltage  <= 7.2f) return POWER_CRITICAL; // < 3.6V
    if(voltage  <= 7.8f) return POWER_LOW;    //  < 3.9V
    if(voltage  <= 8.2f) return POWER_MODERATE; // < 4.1V
    else return POWER_EXCELLENT;
}
*/



SemaphoreHandle_t xDataMutex = xSemaphoreCreateMutex();

moistureData senderObject;
bool can_send_wirelessly = false;

#define PRIORITY_VERY_HIGH   4 // so that the buzzer is not kept waiting for LCD/send/heavy Serial
#define PRIORITY_NORMAL      1
int8_t currentScreen = -1; // negative for very low power

void setup(){  // to let the serial connection open fully. 
      Serial.begin(115200); delay(500); Serial.println(); Serial.printf("------Firmware %s booting...\n", FW_VERSION);
      
      buzzer.begin(); // comes with pinMode(buzzer, OUTPUT);
         
  // CORE TASK 1
      xTaskCreatePinnedToCore(VeryFastTask, "BuzzingCheck", 2048, NULL, PRIORITY_VERY_HIGH, NULL, 1);
      Serial.println("\tBuzzer Task initialized!");
      
      buzzer.beep(1,50,0); // now this can go off in exactly 50ms before the loop comes in
 
    //INDICATORS
     initialize_indicators();

      // DISPLAY
      LCD.init(115200, true, 50, false); vTaskDelay(pdMS_TO_TICKS(1000)); // for the hardware SPI to connect to start transactions
      BootScreen(); LCD.hibernate(); vTaskDelay(pdMS_TO_TICKS(2000));
      Serial.println("Screen Boot Complete!");
      
      //  POWER MANAGEMENT   
      batt.begin(); //comes with pinMode(batteryPin, INPUT); and analogReadResolution(12) and analogSetAttenuation(ADC_11db);
      batt.onLow(beepLowBattery);
      batt.onRecovered(BatteryRecovered);

      Serial.println("[BATTERY] Performing initial voltage check...");
      MonitorBattery(); //       current_power = get_power_state(batt.getLevel());

     // will the Scheduler return to the setup when it finds power critical? or will it just go to sleep?
      // first change things on the screen then call sleep_dynamically ... si kyo?
 
      if(current_power == POWER_CRITICAL) { // if power is infinitely low beep low power, 
        //batt.onLow(beepLowBattery);
          Serial.println("!!!SETUP FOUND POWER TOO LOW");
          
          LowPower_Screen(); LCD.hibernate(); vTaskDelay(pdMS_TO_TICKS(500)); // present voltage and low power prompt
          // currentScreen = -1; 
          // update_display();
          Serial.printf("[POWER STATE] %d → entering sleep routine...\n", current_power);
          sleep_dynamically(); // go to deep sleep for like 2 hours
          //delay(1000);    
          return; //so setup() exits cleanly before proceeding to the “high power” 
          //section — even though deep sleep never returns, this is good defensive coding.
      } // end point for every time that battery is too low


      else{ // if(current_power == POWER_LOW || current_power == POWER_MODERATE || current_power == POWER_EXCELLENT) { 
        // but to be ultra-clear, consider wrapping the rest to ensure deep sleep never accidentally runs scheduler creation code..
        
       Serial.println(">>>SETUP FOUND POWER HIGH ENOUGH");

     //THE KEY COMPONENT
      pinMode(sensorPin, INPUT);  Serial.println("Sensor initialized!"); vTaskDelay(pdMS_TO_TICKS(1000)); // give sensor some starting current
      pinMode(sensorRelay, OUTPUT); digitalWrite(sensorRelay, LOW); Serial.println("Sensor Relay Toggled!");
      pinMode(solarPin, INPUT); // TO READ SUNSHINE

          // Initialize wireless communication with retry logic
        if (!initializeWirelessCommunication()) {
            Serial.println("!!! WARNING: Wireless initialization failed - operating in local mode");
            // Continue without wireless capabilities
        }
        vTaskDelay(pdMS_TO_TICKS(50));
       // initOTA(); // Initialize OTA updates
        vTaskDelay(pdMS_TO_TICKS(50));

     // if power is sufficient, first sample readings before the first dynamic sleep duration reaches 
        Power_Wifi_Screen();  LCD.hibernate();  // present WiFi states, voltage state, sensor ready
        currentScreen = 0; //update_display();
  
      // spnd a bit more time on the Power_Wifi_Screen before calling the update_display in the CombinedTasks which will just present readings
          vTaskDelay(pdMS_TO_TICKS(5000)); currentScreen = 1;


          digitalWrite(sensorRelay, HIGH);
          buzzer.beep(2, 50, 50); 


   //  CORE TASK 2
      xTaskCreatePinnedToCore(CombinedTasks, "Scheduled_Stuff", 8192, NULL, PRIORITY_NORMAL, NULL, 1); // Rule of thumb: Don't exceed 60-70% of total available heap for all tasks combined.
     // this comes with MonitorBattery(); sense_reading(); prepareJSONfile(local_buf, sizeof(local_buf), len); safe_send_data_task(); update_display();

      init_send_system();
      // Add mutex for shared resources
    // SemaphoreHandle_t xDataMutex = xSemaphoreCreateMutex();
   

          
       } // else if(current_power == POWER_LOW || current_power == POWER_MODERATE || current_power == POWER_EXCELLENT) { 
          //That ensures deep sleep never accidentally runs scheduler creation code.
    // SUCCESSFUL End of setup with enuf batt power
}


void initOTA() {
  esp_now_deinit();                 // Stop ESP-NOW safely
  WiFi.mode(WIFI_AP);
  WiFi.softAP("SoilNode_OTA", "update123");

  ArduinoOTA.onStart([]() {
    Serial.println("OTA Update Start");
    // Suspend other tasks if needed
  });

  ArduinoOTA.onEnd([]() {
    Serial.println("OTA Update Complete");
  });

  ArduinoOTA.begin();
  Serial.println("OTA Ready. Connect to AP: SoilNode_OTA, then upload at 192.168.4.1");
}

/*
void initOTA() {
    // Start WiFi for OTA (can coexist with ESP-NOW)
    WiFi.mode(WIFI_AP_STA);
    WiFi.begin(System.OverTheAir.ssid, System.OverTheAir.password);
    
    // Wait for connection with timeout
    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < 10000) {
        delay(500);
        Serial.print(".");
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        ArduinoOTA.setHostname(System.OverTheAir.hostname);
        ArduinoOTA.setPort(System.OverTheAir.port);
        
        ArduinoOTA
            .onStart([]() {
                String type;
                if (ArduinoOTA.getCommand() == U_FLASH) {
                    type = "sketch";
                } else { // U_SPIFFS
                    type = "filesystem";
                }
                Serial.println("Start updating " + type);
                
                // Stop tasks and ESP-NOW during update
                can_send_wirelessly = false;
                vTaskSuspend(NULL); // Suspend current task
            })
            .onEnd([]() {
                Serial.println("\nOTA Update Complete");
            })
            .onProgress([](unsigned int progress, unsigned int total) {
                Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
            })
            .onError([](ota_error_t error) {
                Serial.printf("Error[%u]: ", error);
                if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
                else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
                else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
                else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
                else if (error == OTA_END_ERROR) Serial.println("End Failed");
                
                // Resume operations on error
                can_send_wirelessly = true;
            });

        ArduinoOTA.begin();
        Serial.println("OTA Ready");
        Serial.print("IP address: ");
        Serial.println(WiFi.localIP());
    } else {
        Serial.println("WiFi failed - OTA unavailable");
    }
}
*/

#define ESPNOW_MAX_RETRIES 3
bool initializeWirelessCommunication() {
    WiFi.mode(WIFI_STA);
    
    // ESP-NOW initialization with retry logic
    for (uint8_t attempt = 1; attempt <= ESPNOW_MAX_RETRIES; attempt++) {
        if (esp_now_init() == ESP_OK) { 
            Serial.printf("✓ ESP-NOW initialized successfully (attempt %d/%d)\n", attempt, ESPNOW_MAX_RETRIES);
            
            esp_now_register_send_cb(OnDataSent);
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

void initialize_indicators(){
      pinMode(indicator, OUTPUT);  digitalWrite(indicator, HIGH);
      pinMode(low, OUTPUT); digitalWrite(low, HIGH); vTaskDelay(pdMS_TO_TICKS(500));
      pinMode(medium, OUTPUT); digitalWrite(medium, HIGH);  vTaskDelay(pdMS_TO_TICKS(500));
      pinMode(high, OUTPUT); digitalWrite(high, HIGH);  vTaskDelay(pdMS_TO_TICKS(500));
      Serial.println("Indicators initialized!"); 
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
 float simulated_soil_moisture = 25.0; // Starting value

float soilPercent = 0.00f;  
uint64_t sends = 0; // 0 TO 4BILLION

// Data gonna be like = {SensorID1:"SensorA1", "Moisture1":25.87, Temperature1:32.87, Voltage: 8.2, Sends:256718};
const char sensor_position[10] = "Sensor A2";
bool prepareJSONfile(char* out_buf, size_t out_size, size_t &len) {
    JSON_data.clear();
    JSON_data["Pos_2"]  = sensor_position; // string
   // JSON_data["Moi_2"]  = soilPercent<1.0?simulated_soil_moisture:soilPercent; // keep as float // hopefully at 2dp
    JSON_data["Moi_2"]  = soilPercent; // keep as float // hopefully at 1-2dp
    JSON_data["Volt_2"]  = batt.getVoltage(); // float
    JSON_data["PwrMde_2"] = batt.getLevel(); // or string of current power 
    JSON_data["Sends_2"]    = sends; // very long integer of successful deliveries

   if(out_size < 200) return false; // Minimum safe size

    len = serializeJson(JSON_data, out_buf, out_size);
    return (len > 0 && len < out_size);

}


// -----------------------------
// Globals used by the snippet
// -----------------------------
// ======================
// Globals for sending
// ======================

SemaphoreHandle_t sendSemaphore = NULL;
//SemaphoreHandle_t xDataMutex = NULL;
volatile bool last_send_success = false;


// -----------------------------
// Safe send function: serializes to local buffer, sends and waits for callback
// returns true if acknowledged by peer
// -----------------------------
bool safe_send_data_task(uint8_t max_retries = SEND_MAX_RETRIES) {

   vTaskDelay(pdMS_TO_TICKS(500)); // give Wi-Fi time after wake
   if (!can_send_wirelessly) {
        Serial.println("safe_send_data: ESPNOW not ready, reinitializing...");
        initializeWirelessCommunication();
        vTaskDelay(pdMS_TO_TICKS(200)); // short wait
        return false;
    }
  if (!xDataMutex || !sendSemaphore) {Serial.println("Data Mutex and Send Semaphores Failed!"); return false; }

  // Allocate a local buffer on stack with a safe size
  // NOTE: senderObject.sendable_data_bundle is 200 bytes; we'll use a slightly larger local buffer
  char local_buf[256];
  size_t len = 0;
            
  // Lock the JSON source while serializing to ensure consistent snapshot
  if (xSemaphoreTake(xDataMutex, pdMS_TO_TICKS(500)) != pdTRUE) {
    Serial.println("safe_send_data: could not take data mutex");
    return false;
  }


  prepareJSONfile(local_buf, sizeof(local_buf), len);

  // Copy / serialize - using local_buf so we can release mutex quickly
  //len = serializeJson(JSON_data, local_buf, sizeof(local_buf));
  xSemaphoreGive(xDataMutex);

  if (len == 0 || len >= sizeof(local_buf)) {
    Serial.printf("safe_send_data: serialize failed or too large (%u)\n", (unsigned)len);
    return false;
  }
  if (len >= 250) {
    Serial.printf("safe_send_data: payload too large (%u)\n", (unsigned)len);
    return false;
  }

  // Try to send, wait for callback
  for (uint8_t attempt = 1; attempt <= max_retries; ++attempt) {
    Serial.printf("safe_send_data: attempt %u/%u, %u bytes\n", attempt, max_retries, (unsigned)len);

    esp_err_t res = esp_now_send(System.peer.broadcastAddress, (uint8_t*)local_buf, len);
    if (res != ESP_OK) {
      Serial.printf("esp_now_send returned %d\n", (int)res);
      // small backoff before retrying
      vTaskDelay(pdMS_TO_TICKS(200));
      continue;
    }

    // Wait for OnDataSent to give the semaphore
    if (xSemaphoreTake(sendSemaphore, pdMS_TO_TICKS(SEND_TIMEOUT_MS)) == pdTRUE) {
      if (last_send_success) { sends++;
        Serial.println("safe_send_data: sent successfully");
        return true;
      } else {
        Serial.println("safe_send_data: send reported failure");
      }
    } else {
      Serial.println("safe_send_data: send callback timeout");
    }

    // exponential backoff or small delay
    vTaskDelay(pdMS_TO_TICKS(200 * attempt));
  }

  Serial.println("safe_send_data: all retries failed");
  return false;
}



//modify sleep modes according to available battery power
void CombinedTasks(void *pvParams) { // the 4 in one
  
  UBaseType_t highWaterMark;
  const TickType_t return_to_task = pdMS_TO_TICKS(dynamic_interval); // dynamic interval: 3600ULL * 1000ULL / 900ULL * 1000ULL   each 15 mins or each hour when in deep sleep
  TickType_t xLastWake = xTaskGetTickCount();

  while (1) {// or vTaskdelayUntil...

            MonitorBattery();
            sense_reading();
            
            Serial.println("CombinedTasks resuming, allowing radio to settle...");
           
            // prepareJSONfile(local_buf, sizeof(local_buf), len);
            // manage_wireless_mode(); // ensure that either we are in OTA [WiFi] or ESPNOW 
            safe_send_data_task();
            
            update_display();
             
         // Check stack usage
            highWaterMark = uxTaskGetStackHighWaterMark(NULL);
            Serial.printf("SHW => Stack high water mark: %u bytes\n", (highWaterMark * sizeof(StackType_t)));
            
            // If this gets close to 0, increase stack size!
            if(highWaterMark < MIN_STACK_SAFETY_MARGIN) { // Less than 200 bytes remaining
                Serial.println("WARNING: Stack nearly exhausted!");
            }

            sleep_dynamically();   // vTaskDelayUntil(&xLastWake, return_to_task); // this is inside the sleep_dynamically
  }
}


char time_str[16];
uint8_t hours = 0;
uint64_t now_now = 0, prev = 0;
void loop() {
  // everything is handled by the scheduler in TASKS

  if((esp_timer_get_time() - now_now) >= 1e7){ // blink e.g. every 10 second // heart beat
        uint64_t seconds = now_now / 1000000ULL;
         hours   += (seconds / 3600) % 24;
        uint8_t minutes = (seconds % 3600) / 60;
        uint8_t secs    = 1+(seconds % 60);

        sprintf(time_str, "%02u:%02u:%02u", hours, minutes, secs);
        Serial.print("Uptime Check: ");Serial.println(time_str);

       // Serial.print("Time Check in (mins): "); Serial.println(now_now/60000000);
        // maybe we can have over the air upgrade here
        // Only check OTA when in excellent power mode or connected to power
        get_simulated_moisture(); // remove before deployment
        now_now = esp_timer_get_time(); 
  }
  
}

void manage_wireless_mode() {
    static bool ota_active = false;
    
    // Enable WiFi for OTA only when needed (excellent power or charging)
    if ((current_power == POWER_EXCELLENT || batt.getVoltage() > 8.5f) && !ota_active) {
        if (WiFi.status() != WL_CONNECTED) {
            esp_now_deinit();
            
            WiFi.mode(WIFI_AP_STA);
           
            WiFi.begin(System.OverTheAir.ssid, System.OverTheAir.password);
            ArduinoOTA.begin();
            ota_active = true;
           // vTaskSuspendAll(); //  freeze all other tasks

        }
    } else if (ota_active && current_power != POWER_EXCELLENT) {
        // Disable WiFi to save power when OTA not needed
        ArduinoOTA.end();         // stop OTA service
        WiFi.mode(WIFI_STA); // STA only for ESP-NOW
        esp_now_init();           // reinit ESPNOW
        WiFi.disconnect();
        ota_active = false;
    }
}

// -----------------------------
// Call this from setup() after esp_now_init()
// -----------------------------
void init_send_system() {
  // create in setup (not at global scope)
  sendSemaphore = xSemaphoreCreateBinary();
  if (!sendSemaphore) {
    Serial.println("ERROR: sendSemaphore create failed");
  }

  xDataMutex = xSemaphoreCreateMutex();
  if (!xDataMutex) {
    Serial.println("ERROR: xDataMutex create failed");
  }

      // ADD: Critical resource checks
    if (xDataMutex == NULL) {
        Serial.println("CRITICAL: Mutex creation failed!");
        // Handle error - perhaps enter safe mode
    }

    if (sendSemaphore == NULL) {
        Serial.println("WARNING: Send semaphore creation failed");
    }
}

// ======================
// ESP-NOW callback
// ======================
// -----------------------------
// Safe ISR callback (already correct style)
// -----------------------------
void OnDataSent(const wifi_tx_info_t *mac_addr, esp_now_send_status_t status) {
  last_send_success = (status == ESP_NOW_SEND_SUCCESS);
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  if (sendSemaphore) {
    xSemaphoreGiveFromISR(sendSemaphore, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) portYIELD_FROM_ISR();
  }
}

uint32_t delvs = 0;
bool successfully_delivered = false;
uint8_t send_attempts = 0;

    //if too low, go to sleep and display Shut down due to Low Power, onl;y awake after an hour, run set up and on reaching battery low, go back to sleep

    //if moderate, wake up and do all work once every 15 mins   

    // if excellent, never sleep, keep logging data once every 5 minutes


char SleepPrompt[200];

void sleep_dynamically() {
    switch (current_power) {
        case POWER_CRITICAL: { // 2 hours
              dynamic_interval = power_critical_sleep_duration;

              Serial.println("\n=== ENTERING CRITICAL POWER MODE ===");
              Serial.printf("Sampling Interval: %.1f sec (%.1f min)\n", 
                            dynamic_interval / 1e6, dynamic_interval / 6e7);

              // --- 1. Graceful shutdown sequence ---
              btStop();

          
         // adc_power_off(); // optional depending on ESP32 model

              // Stop high-current peripherals
               // --- 1. Immediate peripheral shutdown ---
              digitalWrite(sensorRelay, HIGH);      // turn off sensor power
              digitalWrite(low, LOW);
              digitalWrite(medium, LOW);
              digitalWrite(high, LOW);
              buzzer.stop();  // Ensure no tone active
              
              currentScreen = -1;  // update_display(); for UI
              LowPower_Screen(); // mid operation shut down
              LCD.hibernate();
              delay(100); // ensure SPI transactions complete
              
              //  // --- 2. Networking teardown (WiFi / ESP-NOW) ---
            if (can_send_wirelessly) {
                esp_err_t e = esp_now_deinit();
                Serial.printf("esp_now_deinit() -> %d\n", e);
      /*
                  if(!esp_now_deinit(){
                    Serial.println("ESP-NOW powering OFF Failed!");
                  }

                  else {
          */
                    can_send_wirelessly = false;
                    Serial.println("ESP-NOW powered OFF");

                 // stop WiFi fully
                if (esp_wifi_stop() == ESP_OK) Serial.println("esp_wifi_stop OK");
                if (esp_wifi_deinit() == ESP_OK) Serial.println("esp_wifi_deinit OK");

                WiFi.disconnect(true);
                WiFi.mode(WIFI_OFF);

                Serial.println("WiFi Powered down!");
            }
              //esp_wifi_stop(); alone can leave some chip subsystems powered

                    // Optional: power down ADC if supported
              // adc_power_off();  // (commented because ESP32-S3 deprecated this)

                    Serial.flush();  // Give Serial a moment to flush
                    vTaskDelay(pdMS_TO_TICKS(100));
      /*
                    // --- 3. Bluetooth teardown (if BT may be enabled) ---
                    // Use guarded calls; some IDFs require checking if BT is enabled
                    #if defined(CONFIG_BT_ENABLED) || defined(CONFIG_BLUEDROID_ENABLED)
                      // Try to safely disable blueooth controller & stack
                      esp_err_t bt_err = ESP_OK;
                      if (esp_bluedroid_get_status() == ESP_BLUEDROID_STATUS_ENABLED) {
                          esp_bluedroid_disable();
                          esp_bluedroid_deinit();
                      }
                      if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_ENABLED) {
                          esp_bt_controller_disable();
                          esp_bt_controller_deinit();
                      }
                      Serial.printf("Bluetooth shutdown: %d\n", bt_err);
                    #else
                      Serial.println("Bluetooth not enabled in build.");
                    #endif
        */
                     // Print debug info: reset reason and wake cause
                  esp_reset_reason_t rr = esp_reset_reason();
                  Serial.printf("Reset reason before sleep: %d\n", rr);
                  esp_sleep_wakeup_cause_t wck = esp_sleep_get_wakeup_cause();
                  Serial.printf("Wake cause before sleep: %d\n", wck);

                  // --- 5. Configure wake sources cleanly ---
                  // disable previously enabled wake sources first
                  /*
                  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);
                  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_EXT0);
                  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_EXT1);
                  */
                  // or
                  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);

                  // Enable single intentional wake source: timer (and optional ext0 for solarPin HIGH)
                  esp_sleep_enable_timer_wakeup(dynamic_interval);
                  // If you want to also allow solarPin to wake when it goes HIGH (e.g., sunrise),
                  // ensure solarPin is RTC-capable and uncomment:
                  // esp_sleep_enable_ext0_wakeup((gpio_num_t)solarPin, 1); // wake on HIGH

                    Serial.println("→ Entering deep sleep now...");
                    Serial.println("Expected wake in ~2 hours.");

                                        
                    Serial.flush();
                    vTaskDelay(pdMS_TO_TICKS(100));

              // --- 3. Enter deep sleep ---
                    esp_deep_sleep_start();

                     // Should never reach here
                      Serial.println("ERROR: deep sleep failed to start!");

                // }

            //  }
                            // Code never reaches here but break the case nonetheless
                            break;
        }

        case  POWER_LOW: { // 15 mins
              dynamic_interval = power_low_sleep_duration;
              Serial.printf("Sampling Interval = %.1f seconds. Entering LIGHT SLEEP.\n", dynamic_interval / 1e6);
              Serial.printf("Next Wake: %.1f minutes\n", (float)(dynamic_interval / 60000000.0));

              if (can_send_wirelessly) {
                  esp_now_deinit();
                  can_send_wirelessly = false;
              }

              // Turn off WiFi to save power
              WiFi.disconnect(true);
              WiFi.mode(WIFI_OFF);
              esp_wifi_stop();

              esp_sleep_enable_timer_wakeup(dynamic_interval);
              Serial.println("→ Sleeping...");
              esp_light_sleep_start();

             
              // Upon waking
              esp_sleep_wakeup_cause_t wakeCause = esp_sleep_get_wakeup_cause();
              Serial.printf("Woke up (cause=%d). Reinitializing radio...\n", wakeCause);

              // Restart radio for ESP-NOW or OTA
              if (!initializeWirelessCommunication()) {
                  Serial.println("Wireless reinit failed after wake.");
              } else {
                  Serial.println("Wireless reinit success.");
              }
        
          break;
        }

        
        case POWER_MODERATE: { // 5mins
            dynamic_interval = power_mod_sleep_duration;
              Serial.printf("Sampling Interval = %.1f seconds. Entering LIGHT SLEEP.\n", dynamic_interval / 1e6);
              Serial.printf("Next Wake: %.1f minutes\n", (float)(dynamic_interval / 60000000.0));

              if (can_send_wirelessly) {
                  esp_now_deinit();
                  can_send_wirelessly = false;
              }

              // Turn off WiFi to save power
              WiFi.disconnect(true);
              WiFi.mode(WIFI_OFF);
              esp_wifi_stop();

              esp_sleep_enable_timer_wakeup(dynamic_interval);
              Serial.println("→ Sleeping...");
              esp_light_sleep_start();

              // Upon waking
              esp_sleep_wakeup_cause_t wakeCause = esp_sleep_get_wakeup_cause();
              Serial.printf("Woke up (cause=%d). Reinitializing radio...\n", wakeCause);

              // Restart radio for ESP-NOW or OTA
              if (!initializeWirelessCommunication()) {
                  Serial.println("Wireless reinit failed after wake.");
              } else {
                  Serial.println("Wireless reinit success.");
              }
        
            break;
      }

        case POWER_EXCELLENT: { // 10 seconds
              
                  dynamic_interval = power_excellent_delay;

                  Serial.println("\n[POWER MODE] EXCELLENT — Full operational mode");
                  Serial.printf("Sampling Interval = %.0f seconds (%.1f minutes)\n", 
                                (float)dynamic_interval / 1000.0f, 
                                (float)dynamic_interval / 60000.0f);

                  /*

                  // Keep all systems awake
                  Serial.println("All peripherals active → continuous sampling and OTA ready.");

                  // Ensure WiFi is active for OTA and ESP-NOW
                  if (WiFi.getMode() != WIFI_AP_STA && WiFi.getMode() != WIFI_STA) {
                      Serial.println("WiFi inactive → restarting WiFi for OTA + ESP-NOW");
                      WiFi.mode(WIFI_AP_STA);
                      WiFi.begin(System.OverTheAir.ssid, System.OverTheAir.password);
                      delay(100);
                  }

                  // Handle OTA updates periodically (safe non-blocking)
                  if (WiFi.status() == WL_CONNECTED) {
                      ArduinoOTA.handle();
                      Serial.printf("OTA service alive @ %s\n", WiFi.localIP().toString().c_str());
                  } else {
                      Serial.println("WiFi disconnected — skipping OTA this cycle.");
                  }

                  */

                  // Optionally check radio state
                  if (!can_send_wirelessly) {
                    Serial.println("Woke up — allowing WiFi/ESP-NOW to reinit...");
                      vTaskDelay(pdMS_TO_TICKS(1000)); // give radio time to stabilize
                      if(initializeWirelessCommunication()) Serial.println("ESP-NOW READY!");
                      else {
                          Serial.println("ESP-NOW not ready → attempting reinit...");
                      }

                  }

                  Serial.printf("[WiFi Mode] %d | [ESPNOW Ready] %s\n", WiFi.getMode(), can_send_wirelessly ? "YES" : "NO");
                  // Wait before next cycle, ensuring no drift
                  vTaskDelay(pdMS_TO_TICKS(dynamic_interval));  // or use vTaskDelayUntil() for precise timing
                  break;
             }

    }
}

/*After esp_light_sleep_start(), 
  the program resumes after that call,
  but the Wi-Fi and ESP-NOW states are lost.
  So you must reinitialize radio after wake.*/
 

float get_simulated_moisture() {
    // Add small random changes (-2% to +2%)
    float change = (random(-200, 201)) / 100.0; // -2.00 to +2.00
    simulated_soil_moisture += change;
    
    // Keep within realistic bounds (5% to 45%)
    simulated_soil_moisture = constrain(simulated_soil_moisture, 5.0, 45.0);
    
    return simulated_soil_moisture;
}

// GLOBAL VARIABLES TO BE USED by 3 functions: sense_reading, update_display and send_as_JSON

// soil specific constants
const float wiltingPoint = 19.0;    
const float fieldCapacity = 60.0; // THE WETTEST SOIL  

/*
 * @brief Reads soil moisture sensor 100 times and averages result.
 * @return Updates global soilPercent variable.
 */
void sense_reading(){

     Serial.println("Now Scanning the Soil..."); Serial.println();
     flash_indicators();
     digitalWrite(sensorRelay, LOW); 
    vTaskDelay(pdMS_TO_TICKS(250));     //when powering sensorRelay set a small settle delay (e.g., 50–200 ms) before sampling.
 
      //char last_recording_time[12] = "21:45:00";    
      uint32_t movin_avg = 0;
      float newMax = 0.00, newMin = 0.00;
      float maxRange = 0.00;
      float variance = 0.00; 
        
      uint16_t lowest = 4090; // pull it so high so that it is always pushed downwards to the smallest soil moisture reading 
      uint16_t highest = 0;
      maxRange = 0.00;

      uint16_t readings = 0;

      uint16_t _12bit_val = 0; 
      movin_avg = 0;
     
      float readings_duration = 0.00; float reading_period = 0.0;

      
      uint64_t start_reading = esp_timer_get_time(); // returns microseconds

        for (uint16_t i = 0; i < 100; i++) { // 100 readings
            _12bit_val = analogRead(sensorPin);
            if (_12bit_val > highest) highest = _12bit_val;
            if (_12bit_val < lowest) lowest   = _12bit_val;
                 movin_avg += _12bit_val;
                 readings++;
            // Yield CPU every N samples to avoid blocking
            if (i % 50 == 0) {
                vTaskDelay(1); // yield @ ~1ms to Scheduler
            }
        }

              /*  
                  temp_average   = float(total) / float(reading);
                  temp_variance = temp_variance + ((sensorVal - temp_average)*(sensorVal - temp_average));
              */
            // Serial.print("Counter: "); Serial.print(readings); Serial.print(" Reading = "); Serial.println(_12bit_val);

          


     uint64_t stop_reading = esp_timer_get_time();
     readings_duration = float(stop_reading - start_reading)/1000.0; // by a million
     reading_period    = (readings_duration/float(readings));
            
       if(readings != 0) movin_avg /= (readings); 
        
      
       soilPercent = soil_moisture_calibrator(movin_avg); //the algorithm for soil moisture
   
        maxRange = (newMax - newMin); // spike identification

          update_indicators(soilPercent);    

     digitalWrite(sensorRelay, HIGH); buzzer.beep(1,50,0); // TO SIGNIFY END OF READING

     Serial.print("Reading Duration: ");    Serial.print(readings_duration, 4); Serial.println(" milliseconds");

//     Serial.print("Period of a reading: "); Serial.print(reading_period, 6);    Serial.println(" milliseconds");
   /* 
                  Serial.print("Last Direct: "); Serial.println(_12bit_val);
                  Serial.print("Readings: "); Serial.println(readings);
                  Serial.print("Highest Reading: "); Serial.println(highest);
                  Serial.print("Lowest Reading: ");  Serial.println(lowest);
                  Serial.print("Range: "); Serial.println(highest - lowest); 
   */
 if(highest > 0) { Serial.print("Percent Error: "); Serial.println(((highest - lowest)*100)/highest); }
                  Serial.print("Soil Moisture: "); Serial.print(soilPercent); Serial.println("%");
                  Serial.println();

}

  //   const float reading_range = 2800.0; // 50 - 2850
       const float reading_range = 4095.0; // 0 - 4095

float soil_moisture_calibrator(uint16_t working_reading){ Serial.println("Running Calibrator..."); vTaskDelay(pdMS_TO_TICKS(10));;

      float refined_val = (float)working_reading; Serial.print("Computed Average: "); Serial.println(working_reading);
            refined_val = refined_val / float(reading_range); Serial.print("Semi Refined:"); Serial.println(refined_val); // min => 5, max => 4000
        if(refined_val <= 0.80)  { refined_val = refined_val * fieldCapacity; } //making 40 to be the highest possible reading ... 
      else refined_val = refined_val * 100.0; //showing the 100% if in air
          
     return refined_val;
}

void flash_indicators(){
         digitalWrite(low, HIGH);   
         digitalWrite(medium, HIGH); 
         digitalWrite(high, HIGH);   
}

void update_indicators(float soilMoisture_val){
       if(soilMoisture_val <= wiltingPoint){

         digitalWrite(low, HIGH);   vTaskDelay(pdMS_TO_TICKS(10));
         digitalWrite(medium, LOW); vTaskDelay(pdMS_TO_TICKS(10));
         digitalWrite(high, LOW);   vTaskDelay(pdMS_TO_TICKS(10));
    
       }

       else if(soilMoisture_val <= 25.0){ // if 20 - 25
         digitalWrite(low, LOW);    vTaskDelay(pdMS_TO_TICKS(10));
         digitalWrite(medium, HIGH);vTaskDelay(pdMS_TO_TICKS(10));
         digitalWrite(high, LOW);   vTaskDelay(pdMS_TO_TICKS(10));

       }
       else { // if > 25
         digitalWrite(low, LOW);    vTaskDelay(pdMS_TO_TICKS(10));
         digitalWrite(medium, LOW); vTaskDelay(pdMS_TO_TICKS(10));
         digitalWrite(high, HIGH);   vTaskDelay(pdMS_TO_TICKS(10));

       }

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





void update_display(){
    if(currentScreen < 0) LowPower_Screen();
    if(currentScreen == 0) Power_Wifi_Screen();
    if(currentScreen == 1) soilMoi_Screen();
    if(currentScreen == 2) dataUpload_Screen();
    if(currentScreen == 3) ota_update();

    LCD.hibernate();
}

void ota_update(){

    LCD.firstPage();
    do {
        LCD.setRotation(2);
        LCD.fillScreen(GxEPD_WHITE);
        LCD.setFont(&FreeSans12pt7b);
        LCD.setCursor(20, 60);
        
        if (WiFi.status() == WL_CONNECTED) {
            LCD.print("OTA Ready");
            LCD.setCursor(20, 100);
            LCD.print(WiFi.localIP());
        } else {
            LCD.print("OTA Offline");
        }
        
        LCD.setCursor(20, 140);
        LCD.printf("Batt: %.1fV", batt.getVoltage());
    } while (LCD.nextPage());
    
}

void dataUpload_Screen(){

}

void LowPower_Screen(){
  
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

    //
    //LCD.setCursor(40, 190); 
    //LCD.printf("Waking in %.1f hr", power_critical_sleep_duration / 3.6e9);


        //BATTERY
    LCD.fillRoundRect(50, 144, 88, 40, 5, GxEPD_BLACK); LCD.fillRoundRect(54, 148, 80, 32, 5, GxEPD_WHITE); LCD.fillRoundRect((54+81), 155, 8, 20,2, GxEPD_BLACK);  
    LCD.setFont(&FreeSans9pt7b); LCD.setCursor(80, 168); LCD.print(batt.getVoltage()); 
    
  } while (LCD.nextPage());

  
}

void Power_Wifi_Screen() {
  LCD.firstPage();
  do {
    LCD.setRotation(2);
    LCD.fillScreen(GxEPD_WHITE);

    LCD.setFont(&FreeSans12pt7b);
    LCD.setTextColor(GxEPD_BLACK);

    LCD.setCursor(30, 100); LCD.print("Sensor: Ready");
    LCD.setCursor(30, 135);
    LCD.print(can_send_wirelessly ? "Wireless: ON" : "Wireless: OFF");

    // ON button icon
    LCD.fillRoundRect(30, 20, 140, 60, 30, GxEPD_BLACK);
    LCD.fillRoundRect(35, 25, 130, 50, 30, GxEPD_WHITE);
    LCD.fillCircle(60, 50, 22, GxEPD_BLACK);
    LCD.setFont(&FreeSansBold18pt7b);
    LCD.setCursor(96, 64);
    LCD.print("ON");

    // Battery
    drawLargeBatteryIcon(50, 144, batt.getVoltage());
   // drawBatteryIcon(50, 144, batt.getVoltage());

  } while (LCD.nextPage());
}

void drawLargeBatteryIcon(int x, int y, float voltage) {
    // Normalize fill between 7.4V (empty) and 8.4V (full)
    float fill = constrain((voltage - 7.4f) / (8.4f - 7.4f), 0.0f, 1.0f);

    // Outer shell (88×40) with rounded corners
    LCD.fillRoundRect(x, y, 88, 40, 5, GxEPD_BLACK);

    // Positive terminal
    LCD.fillRect((x+86), y + 10, 5, 20, GxEPD_BLACK);

    // Inner white background
    LCD.fillRoundRect(x + 4, y + 4, 80, 32, 4, GxEPD_WHITE);

    // Dynamic fill (black bar)
    int fill_width = (int)(80 * fill);
    LCD.fillRoundRect(x + 4, y + 4, fill_width, 32, 4, GxEPD_BLACK);

    // Voltage text — larger and well aligned
    LCD.setFont(&FreeSans12pt7b);
    LCD.setTextColor(GxEPD_BLACK);
    LCD.setTextSize(1);

    // Print voltage either *inside* if bright, or beside if low
    if (fill > 0.5f) {
        // White text inside battery
        LCD.setTextColor(GxEPD_WHITE);
        LCD.setCursor(x + 20, y + 28);
        LCD.printf("%.1fV", voltage);
    } else {
        // Black text beside battery
        LCD.setTextColor(GxEPD_BLACK);
        LCD.setCursor(x + 95, y + 28);
        LCD.printf("%.1fV", voltage);
    }
}

void drawBatteryIcon(int x, int y, float voltage) {
    float fill = constrain((voltage - 7.4) / (8.4 - 7.4), 0.0, 1.0);

    LCD.fillRoundRect(x, y, 38, 20, 3, GxEPD_BLACK);      // outer shell
    LCD.fillRect(x + 38, y + 5, 4, 10, GxEPD_BLACK);      // positive terminal
    LCD.fillRoundRect(x + 4, y + 4, 30, 12, 2, GxEPD_WHITE); // background
    LCD.fillRoundRect(x + 4, y + 4, (int)(30 * fill), 12, 2, GxEPD_BLACK);

    LCD.setFont(&FreeSans9pt7b);
    LCD.setTextColor(GxEPD_BLACK);
    LCD.setCursor(x - 35, y + 14);
    LCD.printf("%.1fV", voltage);
}


/*
void Power_Wifi_Screen(){

  LCD.firstPage();
  do {
    LCD.setRotation(2);
    LCD.fillScreen(GxEPD_WHITE);
    
    LCD.setTextColor(LCD.epd2.hasColor ? GxEPD_RED : GxEPD_BLACK);
    LCD.setFont(&FreeSans12pt7b);
    LCD.setCursor(30, 105); LCD.print("Sensor: Ready");

    LCD.setCursor(30, 135); LCD.print(can_send_wirelessly?"Wireless: ON":"Wireless: OFF");
    //    batt_str
    LCD.fillRoundRect(30, 20, 140, 60, 30, GxEPD_BLACK);
    LCD.fillRoundRect(35, 25, 130, 50, 30, GxEPD_WHITE);
    LCD.fillCircle(60, 50, 22, GxEPD_BLACK);

    LCD.setFont(&FreeSansBold18pt7b); LCD.setCursor(96, 64); LCD.print("ON"); //FreeSansBold18pt7b
    //LCD.fillCircle(100, 40, 25, GxEPD_BLACK);
    //LCD.fillCircle(100, 40, 18, GxEPD_WHITE);
    
        //BATTERY
    LCD.fillRoundRect(50, 144, 88, 40, 5, GxEPD_BLACK); LCD.fillRoundRect(54, 148, 80, 32, 5, GxEPD_WHITE); LCD.fillRect(45, 155, 5, 20, GxEPD_BLACK);  
    LCD.setCursor(60, 175); LCD.print(batt.getVoltage()); 
    
  } while (LCD.nextPage());

}
*/


void OTATask(void *pvParams) { // this is never creared as a task... do not want conflict with CombinedTasks
    const TickType_t xDelay = pdMS_TO_TICKS(10000); // Check for OTA every 10 seconds
    
    while (1) {
        if (WiFi.status() == WL_CONNECTED) {
            ArduinoOTA.handle();
        }
        
        // Only check OTA when in excellent power mode or connected to power
        if (current_power == POWER_EXCELLENT || batt.getVoltage() > 8.5f) {
            // Additional OTA maintenance can go here
        }
        
        vTaskDelay(xDelay);
    }
}


float recent_soil_data[6] = {0.00, 0.00, 0.00, 0.00, 0.00}; //6 slots ...but ... 5 readings
char  recent_soil_moistures[6][8] = {"XX", "YY", "ZZ"}; //6 slots for 5 most recent readings e.g. 10.4%
char  recent_soil_times[6][8] = {"xx:xx", "yy:yy", "10:35"}; //5 sets of seven each 10:35
char last_soil_moisture[10] = "x.x%";

char moi_char[10] = "23";

char operational_header[25] = "Soil Moisture (VWC)";

char ShortTime[32] = ".";


void soilMoi_Screen(){
    LCD.setRotation(2);
    LCD.setFont(&FreeMonoBold9pt7b);

  // do this outside of the loop
  int16_t tbx, tby; uint16_t tbw, tbh;

  // center the heading
  LCD.getTextBounds(operational_header, 0, 0, &tbx, &tby, &tbw, &tbh);
  uint16_t header_x = ((LCD.width() - tbw) / 2) - tbx;
  uint16_t header_y = ((LCD.height() / 4) - tbh / 2) - tby;


 //RECORD THE READING  & TIME
 
      char soil_moisture_char[7] = "x.x";

    /*
      //print only whole numbers on screen
      if(soilPercent <= 1.0) itoa((int)(simulated_soil_moisture+0.5), soil_moisture_char, 10);  // remove on deployment
      else itoa((int)(soilPercent+0.5), soil_moisture_char, 10); // strcat(last_soil_moisture, "%");
    */
      itoa((int)(soilPercent+0.5), soil_moisture_char, 10); 

  LCD.firstPage();
  do {
        LCD.fillScreen(GxEPD_WHITE);
        LCD.setTextColor(GxEPD_BLACK);
        LCD.setTextSize(1);

      //  HEADER
        LCD.setFont(); 
        LCD.getTextBounds(time_str, 0, 0, &tbx, &tby, &tbw, &tbh);
        LCD.setCursor(((LCD.width() - tbw) / 2), 5); LCD.print(time_str);

       //  LCD.setCursor(70, 5); LCD.print(time_str); // uptime
         LCD.setFont(&FreeSans9pt7b); LCD.setCursor(header_x+20, header_y-10); LCD.print(operational_header); 
        
          
      //  BODY
          //DROP OF WATER
          LCD.fillTriangle(20, 100, 30, 80, 40, 100, GxEPD_BLACK);
          LCD.fillCircle(30, 102, 10, GxEPD_BLACK);

          //the actual figure
          
          LCD.setFont(&FreeSansBold24pt7b);  LCD.setTextSize(2); 
          LCD.getTextBounds(soil_moisture_char, 0, 0, &tbx, &tby, &tbw, &tbh);
          uint16_t moi_x = (LCD.width() - tbw) / 2;
          uint16_t moi_y = (LCD.height() * 3 / 5);
          LCD.setCursor(moi_x, moi_y);
          LCD.print(soil_moisture_char);

         LCD.setFont(&FreeSansBold18pt7b);  LCD.setTextSize(1);  LCD.setCursor((moi_x+110), (moi_y-20));  LCD.print("%");
        
          // FreeSerifBold24pt7b // FreeSansBold24pt7b
       //   LCD.setTextSize(2);   LCD.setCursor(moi_x-40, moi_y);  LCD.println(soil_moisture_char);

     //   FOOTNOTE with Network Bar [ON or OFF], Uploads Icon[sents], and Battery Level [Voltage]
          footer();
  } while (LCD.nextPage());
 
}

void draw_mini_graph() {
    int baseY = 155, baseX = 20;
    for (int i = 1; i < 5; i++) {
        int y1 = baseY - (int)(recent_soil_data[i - 1] * 1.2);
        int y2 = baseY - (int)(recent_soil_data[i] * 1.2);
        LCD.drawLine(baseX + (i - 1) * 30, y1, baseX + i * 30, y2, GxEPD_BLACK);
    }
}


bool minute_count_passed = false; 

char bytes_char[10] = "";

void footer(){
              LCD.drawFastHLine(10, 165, 180, GxEPD_BLACK);
            // LCD.fillCircle(0, 295, 2, GxEPD_BLACK);
            
           
            //NETWORK BARS
            LCD.fillRect(10, 194, 6, 5, GxEPD_BLACK); LCD.fillRect(20, 184, 6, 15, GxEPD_BLACK); LCD.fillRect(30, 173, 6, 26, GxEPD_BLACK);
            
            LCD.setTextSize(1);
            LCD.setCursor(10, 180);  LCD.setFont(); LCD.print(can_send_wirelessly?"ON":"OFF"); 

          // DATA UPLOAD ICON
            LCD.fillRect(62, 180, 30, 20, GxEPD_BLACK); LCD.fillRect(66, 180, 22, 14, GxEPD_WHITE);
            LCD.fillRect(75, 176, 4, 15, GxEPD_BLACK);  LCD.fillTriangle(69, 176, 76, 170, 83, 176, GxEPD_BLACK);
            if(sends > 0)  {  LCD.setFont(&FreeSans9pt7b); LCD.setCursor(96, 195); LCD.print(sends);   }

                 
        // ==== DYNAMIC BATTERY ICON ====
          // Position: bottom-right (x=150, y=174)
          float v = batt.getVoltage();

          // Normalize 7.4V (empty) → 8.4V (full)
          float fill = constrain((v - 7.4) / (8.4 - 7.4), 0.0, 1.0);

          // Draw battery outline
          LCD.fillRoundRect(150, 174, 38, 20, 3, GxEPD_BLACK);  // outer shell
          LCD.fillRect(188, 179, 4, 10, GxEPD_BLACK);           // positive terminal

          // Inner empty space (white background)
          LCD.fillRoundRect(154, 178, 30, 12, 2, GxEPD_WHITE);

          // Fill proportionally
          int fill_width = (int)(30 * fill);
          LCD.fillRoundRect(154, 178, fill_width, 12, 2, GxEPD_BLACK);

          // Always print voltage in black, left of the battery icon
          LCD.setFont(); // &FreeSans9pt7b
          LCD.setTextSize(1);
          LCD.setTextColor(GxEPD_BLACK);
          LCD.setCursor(125, 180);  // Adjusted for baseline alignment
          LCD.printf("%.1fV", v);

      /*
            LCD.fillRoundRect(150, 174, 38, 20, 3, GxEPD_BLACK); LCD.fillRoundRect(154, 178, 30, 12, 3, GxEPD_WHITE); LCD.fillRect(146, 179, 5, 10, GxEPD_BLACK);  

            LCD.setFont(&FreeSans9pt7b); // LCD.setFont(&FreeSans9pt7b);
            LCD.setTextColor(GxEPD_BLACK); 
    */
          
           
        //  if(batt.getVoltage() >= 0.00) {  LCD.setFont(); LCD.setCursor(157, 180); LCD.print(batt.getVoltage()); }
               
}


const char ID[] = "IrriKit Node 1";
const char Device[] = "Soil Moisture";
const char Manufacturer[] = "IntelliSys Uganda";
const char Company[] = "Makerere University";

void BootScreen(){ Serial.println("Starting Display...");
  LCD.setRotation(2);
  int16_t tbx, tby; uint16_t tbw, tbh;

  // center the bounding box by transposition of the origin:
  uint16_t x = ((LCD.width() - tbw) / 2) - tbx;

  LCD.setFullWindow();
  LCD.firstPage();
  do
  {
    LCD.fillScreen(GxEPD_WHITE);
    LCD.fillRect(0,0,200,200,GxEPD_BLACK);
    LCD.fillRect(2,2,194,194,GxEPD_WHITE);

    //HEADER
    LCD.setFont(&FreeMonoBold12pt7b); // FreeSans12pt7b FreeMonoBold12pt7b
    LCD.setTextColor(GxEPD_BLACK);

    LCD.getTextBounds(ID, 0, 0, &tbx, &tby, &tbw, &tbh);
    x = ((LCD.width() - tbw) / 2) - tbx;
    LCD.setCursor(x, 30);
    LCD.print(ID);

  /* MY OWN 
    //LAND
    LCD.drawRect(70, 40, 60, 50, GxEPD_BLACK);
    for(int i = 70; i<130; i+=10)  {
      for(int j = 50; j<110; j+=5){
        LCD.drawFastHLine(i, 50, i+3, j, GxEPD_BLACK);
      }
    }
    
    //DROP OF WATER
    LCD.fillTriangle(90, 70, 100, 50, 110, 70, GxEPD_BLACK);
    LCD.fillCircle(100, 70, 10, GxEPD_BLACK);
 */
 /*
          // ENHANDED BY CHAT GPT
        // ==== LAND PATCH ====
      // Base soil patch
      LCD.fillRect(70, 80, 60, 20, GxEPD_BLACK);              // dark soil base
      LCD.fillRect(72, 82, 56, 16, GxEPD_WHITE);              // inner lighter soil

      // Soil texture: horizontal furrows
      for (int x = 72; x < 128; x += 6) {
          LCD.drawFastHLine(x, 90 + (x % 4), 4, GxEPD_BLACK);
      }

      // Soil cracks: irregular diagonal lines
      LCD.drawLine(75, 82, 85, 95, GxEPD_BLACK);
      LCD.drawLine(95, 82, 105, 96, GxEPD_BLACK);
      LCD.drawLine(120, 82, 115, 96, GxEPD_BLACK);
      LCD.drawLine(90, 95, 100, 98, GxEPD_BLACK);
      LCD.drawLine(105, 94, 120, 98, GxEPD_BLACK);

      // ==== WATER DROP ====
      // Teardrop body
      LCD.fillCircle(100, 70, 10, GxEPD_BLACK);
      LCD.fillTriangle(92, 60, 108, 60, 100, 45, GxEPD_BLACK);

      // Highlight for reflection
      LCD.fillCircle(96, 64, 3, GxEPD_WHITE);

      // Drop shadow (adds depth)
      LCD.fillEllipse(100, 83, 12, 3, GxEPD_BLACK);
  */
  // BY DEEP SEEK
   // draw_soil_moisture_icon(70, 40, 20);
     draw_simple_soil_icon(20, 35);
   //BY CHAT GPT... very scattered soil moisture // draw_soil_with_droplets(20, 35);


    LCD.setFont(&FreeMonoBold9pt7b);
    LCD.getTextBounds(Device, 0, 0, &tbx, &tby, &tbw, &tbh);
    x = ((LCD.width() - tbw) / 2) - tbx;
    LCD.setCursor(x, 110);
    LCD.print(Device);

    LCD.setFont(&FreeSansBold9pt7b); // FreeSans9pt7b  next besr  
    LCD.getTextBounds(Manufacturer, 0, 0, &tbx, &tby, &tbw, &tbh);
    x = ((LCD.width() - tbw) / 2) - tbx;
    LCD.setCursor(x, 150);
    LCD.print(Manufacturer);

    
    LCD.getTextBounds(Company, 0, 0, &tbx, &tby, &tbw, &tbh);
    x = ((LCD.width() - tbw) / 2) - tbx;
    LCD.setCursor(x, 180);
    LCD.print(Company);
  }
  while (LCD.nextPage());

}

void draw_simple_soil_icon(int x, int y) {
    // Clean wavy soil surface - EXTENDED TO 160px
    for(int i = 0; i < 160; i += 1) {
        float surface = sin(i * 0.08) * 3.0; // Adjusted frequency for wider area
        LCD.drawPixel(x + i, y + 8 + (int)surface, GxEPD_BLACK);
    }
    
    // Soil body with WAVY HORIZONTAL LINES - FEWER LINES, 160px WIDTH
    for(int i = 0; i < 50; i += 6) { // Reduced line density
        float phase = i * 0.15; // Adjusted for wider area
        
        // Wavy horizontal lines - each line has its own wave pattern!
        for(int j = 0; j < 140; j += 2) { // 140px wide lines with waves
            float line_wave = sin(j * 0.1 + phase * 2) * 1.5; // Wave along the line
            int pixel_x = x + 10 + j;
            int pixel_y = y + 10 + i + (int)line_wave;
            
            // Only draw if within the organic soil boundaries
            int left_boundary = sin(phase) * 8;
            int right_boundary = cos(phase * 0.6) * 8;
            
            if (j >= left_boundary && j <= (140 - right_boundary)) {
                LCD.drawPixel(pixel_x, pixel_y, GxEPD_BLACK);
            }
        }
    }
    
    // MORE DROPLETS - INCREASED TO 8 DROPLETS for 160px width
    // Random positions and sizes within logical ranges
    
    for(int i = 0; i < 8; i++) { // Increased from 5 to 8 droplets
        int drop_x, drop_y, drop_size;
        bool has_highlight;
        
        // Determine droplet characteristics based on "row" - adjusted for 160px
        switch(i) {
            case 0: // Top-left area
                drop_x = x + 15 + random(0, 20);
                drop_y = y + 10 + random(0, 6);
                drop_size = 7 + random(0, 2); // 7-8 (larger!)
                has_highlight = true;
                break;
            case 1: // Top-center area
                drop_x = x + 60 + random(0, 25);
                drop_y = y + 8 + random(0, 8);
                drop_size = 6 + random(0, 2); // 6-7
                has_highlight = true;
                break;
            case 2: // Top-right area
                drop_x = x + 110 + random(0, 25);
                drop_y = y + 12 + random(0, 6);
                drop_size = 7 + random(0, 2); // 7-8 (larger!)
                has_highlight = true;
                break;
            case 3: // Upper-middle left
                drop_x = x + 30 + random(0, 25);
                drop_y = y + 22 + random(0, 8);
                drop_size = 6 + random(0, 2); // 6-7
                has_highlight = random(0, 2); // Random highlight
                break;
            case 4: // Upper-middle right
                drop_x = x + 90 + random(0, 25);
                drop_y = y + 24 + random(0, 8);
                drop_size = 6 + random(0, 2); // 6-7
                has_highlight = random(0, 2); // Random highlight
                break;
            case 5: // Lower-middle center
                drop_x = x + 60 + random(0, 30);
                drop_y = y + 30 + random(0, 10);
                drop_size = 5 + random(0, 2); // 5-6
                has_highlight = false;
                break;
            case 6: // Bottom-left area
                drop_x = x + 40 + random(0, 25);
                drop_y = y + 38 + random(0, 12);
                drop_size = 4 + random(0, 2); // 4-5
                has_highlight = false;
                break;
            case 7: // Bottom-right area
                drop_x = x + 100 + random(0, 25);
                drop_y = y + 42 + random(0, 10);
                drop_size = 4 + random(0, 2); // 4-5
                has_highlight = false;
                break;
        }
        
        draw_water_droplet(drop_x, drop_y, drop_size, has_highlight);
    }
}

// Helper function to draw individual droplets
void draw_water_droplet(int center_x, int center_y, int size, bool has_highlight) {
    // Main drop body
    LCD.fillCircle(center_x, center_y, size, GxEPD_BLACK);
    
    // Drop tail (size-appropriate)
    int tail_length = size * 1.6;
    LCD.fillTriangle(center_x - (size-1), center_y - 1,
                    center_x, center_y - tail_length,
                    center_x + (size-1), center_y - 1, GxEPD_BLACK);
    
    // Highlight (only for fresh droplets)
    if (has_highlight) {
        LCD.fillCircle(center_x - (size/2), center_y - (size/2), max(1, size/3), GxEPD_WHITE);
    }
    
    // Absorption effect for deeper droplets
    if (center_y > 35) {
        LCD.drawCircle(center_x, center_y + 2, size/2, GxEPD_BLACK);
    }
}
/*
void draw_soil_with_droplets(int x, int y) {  
    // === Wavy surface line ===
    int width = 160;
    for (int i = 0; i < width; i++) {
        float surface = sin(i * 0.09) * 2.5 + cos(i * 0.04) * 1.8; // two harmonics for organic feel
        LCD.drawPixel(x + i, y + 10 + (int)surface, GxEPD_BLACK);
    }

    // === Soil body with gentle furrows ===
    // Only 5–6 layers of subtle wavy lines for texture
    for (int layer = 0; layer < 40; layer += 8) {
        float phase = layer * 0.25;
        for (int i = 0; i < width; i += 5) {
            float wave = sin((i * 0.07) + phase) * 3 + cos((i * 0.04) + phase * 0.5);
            int yy = y + 12 + layer + (int)wave;
            if (yy < y + 60) LCD.drawPixel(x + i, yy, GxEPD_BLACK);
        }
    }

    // === Small random cracks in soil ===
    for (int c = 0; c < 8; c++) {
        int cx = x + random(5, width - 5);
        int cy = y + random(18, 55);
        int len = random(8, 15);
        int dir = random(0, 2) ? 1 : -1;
        LCD.drawLine(cx, cy, cx + len * dir, cy + random(-4, 4), GxEPD_BLACK);
    }

    // === Randomized water droplets ===
    int numDrops = 6 + random(-1, 2);  // around 5–7 droplets
    for (int i = 0; i < numDrops; i++) {
        int drop_x = x + random(15, width - 15);
        int drop_y = y + random(15, 40);
        int drop_size = random(5, 9);  // larger droplets overall
        bool highlight = (random(0, 3) != 0); // ~2/3 chance to have highlight
        draw_water_droplet(drop_x, drop_y, drop_size, highlight);
    }

    // === Optional subtle ripple effect under one random large droplet ===
    if (random(0, 2)) {
        int ripple_x = x + random(40, width - 40);
        int ripple_y = y + random(30, 50);
        for (int r = 0; r < 3; r++) {
            LCD.drawCircle(ripple_x, ripple_y, 8 + r * 3, GxEPD_BLACK);
        }
    }
}

void draw_soil_with_droplets(int x, int y) {  
    // === Wavy surface line ===
    int width = 160;
    for (int i = 0; i < width; i++) {
        float surface = sin(i * 0.09) * 2.5 + cos(i * 0.04) * 1.8; // two harmonics for organic feel
        LCD.drawPixel(x + i, y + 10 + (int)surface, GxEPD_BLACK);
    }

    // === Soil body with gentle furrows ===
    // Only 5–6 layers of subtle wavy lines for texture
    for (int layer = 0; layer < 40; layer += 8) {
        float phase = layer * 0.25;
        for (int i = 0; i < width; i += 5) {
            float wave = sin((i * 0.07) + phase) * 3 + cos((i * 0.04) + phase * 0.5);
            int yy = y + 12 + layer + (int)wave;
            if (yy < y + 60) LCD.drawPixel(x + i, yy, GxEPD_BLACK);
        }
    }

    // === Small random cracks in soil ===
    for (int c = 0; c < 8; c++) {
        int cx = x + random(5, width - 5);
        int cy = y + random(18, 55);
        int len = random(8, 15);
        int dir = random(0, 2) ? 1 : -1;
        LCD.drawLine(cx, cy, cx + len * dir, cy + random(-4, 4), GxEPD_BLACK);
    }

    // === Randomized water droplets ===
    int numDrops = 6 + random(-1, 2);  // around 5–7 droplets
    for (int i = 0; i < numDrops; i++) {
        int drop_x = x + random(15, width - 15);
        int drop_y = y + random(15, 40);
        int drop_size = random(5, 9);  // larger droplets overall
        bool highlight = (random(0, 3) != 0); // ~2/3 chance to have highlight
        draw_water_droplet(drop_x, drop_y, drop_size, highlight);
    }

    // === Optional subtle ripple effect under one random large droplet ===
    if (random(0, 2)) {
        int ripple_x = x + random(40, width - 40);
        int ripple_y = y + random(30, 50);
        for (int r = 0; r < 3; r++) {
            LCD.drawCircle(ripple_x, ripple_y, 8 + r * 3, GxEPD_BLACK);
        }
    }
}

*/

void draw_soil_moisture_icon(int x, int y, int moisture_level) {
    // Soil container (pot/ground)
    LCD.drawRect(x, y, 60, 50, GxEPD_BLACK);
    
    // Soil texture with more natural pattern
    for(int i = 0; i < 60; i += 8) {
        for(int j = 0; j < 50; j += 6) {
            // Random soil particles - more organic look
            if(random(0, 3) > 0) { // 66% chance to draw a particle
                int particle_x = x + i + random(0, 5);
                int particle_y = y + j + random(0, 4);
                LCD.drawPixel(particle_x, particle_y, GxEPD_BLACK);
                
                // Occasionally draw larger soil clumps
                if(random(0, 5) == 0) {
                    LCD.drawPixel(particle_x + 1, particle_y, GxEPD_BLACK);
                    LCD.drawPixel(particle_x, particle_y + 1, GxEPD_BLACK);
                }
            }
        }
    }
    
    // Water level based on moisture (visual indicator)
    int water_height = map(constrain(moisture_level, 0, 100), 0, 100, 5, 45);
    LCD.fillRect(x + 2, y + 50 - water_height, 56, water_height, GxEPD_BLACK);
    
    // Realistic water drop with highlight
    int drop_x = x + 45;
    int drop_y = y + 25;
    
    // Water drop body (teardrop shape)
    LCD.fillCircle(drop_x, drop_y, 8, GxEPD_BLACK);
    LCD.fillTriangle(drop_x - 8, drop_y, drop_x, drop_y - 12, drop_x + 8, drop_y, GxEPD_BLACK);
    
    // Water drop highlight (white reflection)
    LCD.fillCircle(drop_x - 2, drop_y - 2, 2, GxEPD_WHITE);
    
    // Ripples around the drop (optional)
    LCD.drawCircle(drop_x, drop_y, 12, GxEPD_BLACK);
    LCD.drawCircle(drop_x, drop_y, 14, GxEPD_BLACK);
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
  LCD.getTextBounds(operational_header, 0, 0, &tbx, &tby, &tbw, &tbh);
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
        LCD.setCursor(header_x+20, header_y-20); LCD.print(operational_header); 
        
    //  BODY
        LCD.setTextColor(GxEPD_BLACK);  LCD.setCursor((moi_x+70), (moi_y-30));  LCD.println("%");
      
        LCD.setFont(&FreeSansBold24pt7b); // FreeSerifBold24pt7b // FreeSansBold24pt7b
        LCD.setTextSize(2);   LCD.setCursor(moi_x-40, moi_y);  LCD.println(moi_char);

    //  FOOTNOTE with Sent Messages and Battery Level
        footer();

  }
  while (LCD.nextPage());
  Serial.println("helloFullScreenPartialMode testing done");
}










