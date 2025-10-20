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

class WiFi_Manager {
public:
  WiFi_Manager(uint8_t wifi_led_pin = 2)
      : wifi_led(wifi_led_pin), currentNetworkIndex(-1),
        wifi_connect_attempts(0), lastReconnectAttempt(0) {}

  void initialize_ESP_WiFi(const char *device_name);
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

// External configuration
extern const int knownCount;


// Log buffer
static char wifi_log[250] = "No Connection Yet"; 


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

void WiFi_Manager::initialize_ESP_WiFi(const char *device_name) {
  devicename = device_name;
  WiFi.mode(WIFI_STA);
  pinMode(wifi_led, OUTPUT);
  digitalWrite(wifi_led, HIGH);  // HIGH = disconnected

  clear_log();
  log_message("Scanning for available WiFi networks...");

  int n = WiFi.scanNetworks(/*async=*/false, /*hidden=*/true);
  if (n <= 0) {
    log_message("No networks found. Connection failed.");
    WiFi.scanDelete();
    return;
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
    WiFi.scanDelete();
    return;
  }

  clear_log();
  log_message("Found known network. Connecting...");
  if (try_connect(knownNetworks[bestIndex].SECRET_SSID, knownNetworks[bestIndex].SECRET_PASS)) {
    currentNetworkIndex = bestIndex;
    log_message("WiFi connected successfully.");
    log_message(get_ip().c_str());
    digitalWrite(wifi_led, LOW);  // LOW = connected
  } else {
    log_message("Connection attempt failed.");
    digitalWrite(wifi_led, HIGH);
  }

  WiFi.scanDelete();  // Clean up memory
}

// === Safe non-blocking connection maintainer ===
bool WiFi_Manager::ensure_wifi() {
  if (WiFi.status() == WL_CONNECTED) return true;

  uint64_t now_us = esp_timer_get_time();
  if (now_us - lastReconnectAttempt < 5ULL * 1000000ULL) {
    // Only attempt every 5 seconds
    return false;
  }
  lastReconnectAttempt = now_us;

  log_message("WiFi lost. Attempting reconnection...");
  digitalWrite(wifi_led, HIGH);

  if (currentNetworkIndex >= 0) {
    // Try reconnecting to previous network first
    if (try_connect(knownNetworks[currentNetworkIndex].SECRET_SSID,
                    knownNetworks[currentNetworkIndex].SECRET_PASS)) {
      digitalWrite(wifi_led, LOW);
      log_message("Reconnected to previous network.");
      return true;
    }
  }

  // Otherwise, full rescan
  initialize_ESP_WiFi(devicename);
  return (WiFi.status() == WL_CONNECTED);
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
