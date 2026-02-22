/*
  WiFi_Manager.cpp / WiFi_Manager.h
  Reusable class for ESP32 WiFi management.
  - Scans and connects to strongest known network.
  - Maintains connection, auto-reconnects if lost.
  - Keeps LED and log states consistent.
  - Uses esp_timer_get_time() instead of millis().
*/

#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include <Arduino.h>
#include <WiFi.h>
#include "credentials.h"  // Defines knownNetworks[], knownCount, etc.

// External configuration
 // int knownCount = 3;

class WiFi_Manager {
public:
  WiFi_Manager(uint8_t wifi_led_pin = 2)
      : wifi_led(wifi_led_pin), currentNetworkIndex(-1),
        wifi_connect_attempts(0), lastReconnectAttempt(0) {}

  uint8_t initialize_ESP_WiFi(const char *device_name);
  bool ensure_wifi();  // Safe to call periodically
  bool is_connected() const { return WiFi.status() == WL_CONNECTED; }

  // Optional helper
  String get_ip() const {
    return (WiFi.status() == WL_CONNECTED) ? WiFi.localIP().toString() : "0.0.0.0";
  }

private:
  // === Internal State ===
  const char *devicename;
  int currentNetworkIndex;
  int wifi_connect_attempts;
  uint8_t wifi_led;
  uint64_t lastReconnectAttempt;
  #define MAX_WIFI_ATTEMPTS 10



// Log buffer
 char wifi_log[250] = "No Connection Yet"; 


  // === Internal helpers ===
  void log_message(const char *msg) {
    Serial.print("[WiFi] ");
    Serial.println(msg);
  }

  void clear_log() {
    // Optionally clear Serial or external log buffer
  }

  bool try_connect(const char *ssid, const char *pass, int timeout_ms = 10000);
};

// === Implementation ===

uint8_t WiFi_Manager::initialize_ESP_WiFi(const char *device_name) {
  devicename = device_name;
  uint8_t success_level = 4; // nothing done yet
  
  WiFi.mode(WIFI_STA);
  pinMode(wifi_led, OUTPUT);
  digitalWrite(wifi_led, HIGH);  // HIGH = disconnected

  clear_log();
  //log_message("Scanning for available WiFi networks...");
  success_level = 5; //  Scanning for available WiFi networks...

  int n = WiFi.scanNetworks(/*async=*/false, /*hidden=*/true);
  if (n <= 0) {
    success_level = 5; //No networks found. Connection failed.
    //log_message("No networks found. Connection failed.");
    WiFi.scanDelete();
    return success_level;
  }

  // Track best match (strongest RSSI)
  int bestIndex = -1;
  int bestNetwork = -1000;

  for (int i = 0; i < n; i++) {
    String foundSSID = WiFi.SSID(i);
    for (int k = 0; k < knownCount; k++) {
      if (foundSSID == knownNetworks[k].SECRET_SSID && WiFi.RSSI(i) > bestNetwork) {
        bestIndex = k;
        bestNetwork = WiFi.RSSI(i);
      }
    }
  }

  if (bestIndex < 0) {
    log_message("No known networks available. Connection failed.");
    success_level = 6;
    WiFi.scanDelete();
    return success_level;
  }

  //clear_log();
  //log_message("Found known network. Connecting...");
  success_level = 7; // Found known network. Connecting...
   
  if (try_connect(knownNetworks[bestIndex].SECRET_SSID, knownNetworks[bestIndex].SECRET_PASS)) {
    currentNetworkIndex = bestIndex;
    //log_message("WiFi connected successfully.");
    success_level = 8; // WiFi connected successfully.
    log_message(get_ip().c_str());
    digitalWrite(wifi_led, LOW);  // LOW = connected
  } else {
    //log_message("Connection attempt failed.");
    success_level = 9; //  Connection attempt failed.
    digitalWrite(wifi_led, HIGH);
  }
 return success_level; // for successfully found and connected to WuFi
  WiFi.scanDelete();  // Clean up memory
}

// === Safe non-blocking connection maintainer ===
bool WiFi_Manager::ensure_wifi() {
  // Already connected → nothing to do
  if (WiFi.status() == WL_CONNECTED) return true;

  uint64_t now_us = esp_timer_get_time();
  if (now_us - lastReconnectAttempt < 5ULL * 1000000ULL)
    return false; // throttle attempts

  lastReconnectAttempt = now_us;

  log_message("WiFi lost. Attempting reconnection...");
  digitalWrite(wifi_led, HIGH);

  bool reconnected = false;

  // Step 1: quick reconnect to previous network if RSSI is still good
  if (currentNetworkIndex >= 0) {
    int n = WiFi.scanNetworks(false, true);
    bool foundPrev = false;
    int prevRSSI = -1000;

    for (int i = 0; i < n; i++) {
      if (WiFi.SSID(i) == knownNetworks[currentNetworkIndex].SECRET_SSID) {
        foundPrev = true;
        prevRSSI = WiFi.RSSI(i);
        break;
      }
    }

    // only retry same network if still visible and decent RSSI (> -75dBm)
    if (foundPrev && prevRSSI > -75) {
      log_message("Previous network still visible. Reconnecting...");
      reconnected = try_connect(knownNetworks[currentNetworkIndex].SECRET_SSID,
                                knownNetworks[currentNetworkIndex].SECRET_PASS);
      WiFi.scanDelete();
      if (reconnected) {
        digitalWrite(wifi_led, LOW);
        log_message("Reconnected to previous network.");
        return true;
      }
    } else {
      log_message("Previous network weak or gone. Searching alternatives...");
      WiFi.scanDelete();
    }
  }

  // Step 2: fallback — scan for all known networks and choose strongest
  int n = WiFi.scanNetworks(false, true);
  int bestIndex = -1;
  int bestRSSI = -1000;

  for (int i = 0; i < n; i++) {
    String foundSSID = WiFi.SSID(i);
    for (int k = 0; k < knownCount; k++) {
      if (foundSSID == knownNetworks[k].SECRET_SSID && WiFi.RSSI(i) > bestRSSI) {
        bestIndex = k;
        bestRSSI = WiFi.RSSI(i);
      }
    }
  }

  if (bestIndex >= 0) {
    log_message("Connecting to strongest known network...");
    if (try_connect(knownNetworks[bestIndex].SECRET_SSID,
                    knownNetworks[bestIndex].SECRET_PASS)) {
      currentNetworkIndex = bestIndex;
      digitalWrite(wifi_led, LOW);
      log_message("Connected to new network successfully.");
      WiFi.scanDelete();
      return true;
    }
  }

  WiFi.scanDelete();
  log_message("WiFi reconnect failed. Will retry later.");
  return false;
}


// === Blocking connect helper ===
bool WiFi_Manager::try_connect(const char *ssid, const char *pass, int timeout_ms) {
  WiFi.begin(ssid, pass);
  uint64_t start = esp_timer_get_time();

  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    if ((esp_timer_get_time() - start) / 1000 > timeout_ms) {
      log_message("Connection timeout.");
      return false;
    }
  }

  return true;
}

#endif
