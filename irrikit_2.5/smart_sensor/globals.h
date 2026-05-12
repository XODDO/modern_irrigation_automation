//#pragma once
#ifndef GLOBALS_H
#define GLOBALS_H


#include <WiFi.h>       // For high-level Wi-Fi control (mode, STA/AP setup)
#include <esp_wifi.h>   // For ESP-NOW internal Wi-Fi APIs
#include <esp_now.h>    // The ESP-NOW API itself

#include <ArduinoJson.h>

#include "config.h" // for OTA


#include <SPI.h> // for EPD

#include <Adafruit_GFX.h> // for EPD
#include <GxEPD2_BW.h> // for EPD


//for OTA
#include "credentials.h"
#include "WiFi_Manager.h"
#include "ArduinoOTA.h"
#include <esp_wifi.h>   // For ESP-NOW internal Wi-Fi APIs
#include <Update.h> //FOR UPDATING OVER THE AIR
#include <ESPmDNS.h>
#include <WiFiUdp.h>


#include "esp_timer.h"
#include "esp_sleep.h"
#include "driver/gpio.h"


#include "sense.h"


#include <Preferences.h>
Preferences prefs;



#include "memory_monitor.h"
MemoryMonitor memMonitor;

#include "esp_task_wdt.h"
// WATCH DOG SHOULD THERE BE A HANG ESP IN CONNECTIVITY


#include "Battery.h"
#include "Buzzer.h"


#include <Fonts/FreeMonoBold9pt7b.h>
#include <Fonts/FreeMonoBold12pt7b.h>
#include <Fonts/FreeSansBold24pt7b.h> // FreeSerifBold24pt7b
#include <Fonts/FreeSansBold18pt7b.h>
#include <Fonts/FreeSans12pt7b.h>
#include <Fonts/FreeSansBold9pt7b.h>

#include <Fonts/FreeSans9pt7b.h>

// Peer info 
esp_now_peer_info_t peerInfo;



// Pin definitions

const float single_cell_max_voltage = 4.20;
const uint8_t number_of_cells = 1;
const uint8_t batteryPin = 39;

const uint8_t sensorPin = 36;

const uint8_t sensorRelay = 22;

const uint8_t lowPin = 25; // red
const uint8_t mediumPin = 33; // amber
const uint8_t highPin = 27; // green
const uint8_t indicator = 13;

const uint8_t solarPin = 16; // input pullup  // for waking up from deep slumber  //uint8_t batteryPin = 35;

const uint8_t ota_button = 26;  // Boot button, or a dedicated GPIO

const uint8_t buzzingPin = 14;

char moi_char[10] = "23";

char operational_header[25] = "Soil Moisture (VWC)";

char ShortTime[32] = ".";



char time_str[16];
uint8_t hours = 0;



// ===== Callbacks =====
void beepLowBattery();
void BatteryRecovered();
void MonitorBattery();


void update_display();
void soilMoi_Screen();
void dataUpload_Screen();
void Power_Wifi_Screen();
void LowPower_Screen();
void BootScreen();


// Esp-Now Call Back
//void OnDataSent(const wifi_tx_info_t *mac_addr, esp_now_send_status_t status);


// Old (before ESP-IDF v5)
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);

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
    POWER_CRITICAL = 0,  // < 7.2V/2
    POWER_LOW,           // (7.2 - 7.8)V/2  
    POWER_MODERATE,      // (7.8 - 8.2)V/2
    POWER_EXCELLENT      // > 8.2V/2
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
const uint64_t power_excellent_delay = (30 * 1000); // 10 seconds

/*
power_state_t get_power_state(float voltage) {

    if(voltage  <= 7.2f) return POWER_CRITICAL; // < 3.6V
    if(voltage  <= 7.8f) return POWER_LOW;    //  < 3.9V
    if(voltage  <= 8.2f) return POWER_MODERATE; // < 4.1V
    else return POWER_EXCELLENT;
}
*/


typedef struct moistureData{ // all stringified
      char sendable_data_bundle[200] = "";
}  moistureData; 




bool cpu_freq_set = false;
    char APB_Freq[36] = ".";
    char XTAL_Freq[36] = ".";
    char CPU_Freq[24] = ".";

volatile bool successfully_delivered = false;

#define WIFI_SWITCH_TIMEOUT_MS 10000
#define WIFI_RECONNECT_DELAY_MS 500
char wifi_connection_log[128] = "...";

uint64_t now_now_ms = 0, prev = 0;

// Add these enums and globals at the top with your other globals
typedef enum {
    OTA_STATE_IDLE,
    OTA_STATE_STARTING,
    OTA_STATE_PROGRESS,
    OTA_STATE_SUCCESS,
    OTA_STATE_ERROR,
    OTA_STATE_TIMEOUT
} OTAState;

OTAState current_ota_state = OTA_STATE_IDLE;
uint64_t ota_success_time = 0;
uint64_t ota_error_time = 0;
uint64_t ota_timeout_time = 0;
uint64_t ota_start_time = 0;

uint64_t otaStartTime = 0; 
const uint64_t OTA_TIMEOUT = 15ULL * 60ULL * 1000ULL;  // 15 minutes
char LastOTAUpdate[60] = "27 Oct 2025 10:10";

// These are accessed from multiple tasks (loop + OTA task)
volatile bool otaTriggered = false;
volatile bool otaModeActive = false;
volatile bool otaStarted = false;
volatile bool otaFinished = false;
volatile bool otaError = false;
volatile int otaProgress = 0;


char ota_log[150] = "...";

uint32_t WiFi_Strength = 0;








#endif