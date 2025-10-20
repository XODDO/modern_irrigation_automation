
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


#endif