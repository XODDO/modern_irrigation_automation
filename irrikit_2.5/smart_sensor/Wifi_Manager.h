#pragma once
#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include <Arduino.h>
#include <WiFi.h>
#include "credentials.h"  // Defines knownNetworks[], knownCount, etc.
#include "time.h" // to get ntp time

/**
 * @brief Return codes for WiFi initializations.
 */
enum WiFiMgrStatus : uint8_t {
    WIFI_MGR_SUCCESS = 0,
    WIFI_MGR_NO_NETWORKS = 1,
    WIFI_MGR_NO_KNOWN = 2,
    WIFI_MGR_CONN_FAIL = 3,
    WIFI_MGR_SCAN_ERR = 4
};

/**
 * @brief ESP32 WiFi management class. Connects to strongest known network, auto-reconnect, status LED.
 */
class WiFi_Manager {
public:
    explicit WiFi_Manager(uint8_t wifi_led_pin);
        bool connect_to_ssid(const char* ssid, const char* password);
        bool is_connected_to_ssid(const char* ssid);
    
        struct tm ntp_timeinfo;
        bool ntp_synced = false;

                bool sync_ntp(long gmtOffset_sec = 10800, int daylightOffset_sec = 0,
                    uint8_t timeoutSeconds = 10);



    /**
     * @brief Initialize WiFi, scan, and connect to strongest known network.
     * @param device_name Hostname for ESP32.
     * @return WiFiMgrStatus enum value.
     */
    WiFiMgrStatus initialize_ESP_WiFi(const char* device_name);

    /**
     * @brief Ensure WiFi connection. Attempts reconnect periodically.
     * @return true if connected, false otherwise.
     */
    bool ensure_wifi();

    /**
     * @brief Get current connection status.
     * @return true if connected.
     */
    bool is_connected() const;

    /**
     * @brief Get device's IP address as string.
     * @return IP as string.
     */
    String get_ip() const;

    void print_connection_stats() const;


private:

   unsigned long backoff_delay_ms;
    int consecutive_failures;
    
    // Internal state
    const char* devicename;
    volatile int currentNetworkIndex;
    volatile int wifi_connect_attempts;
    uint8_t wifi_led;
    volatile uint64_t lastReconnectAttempt;

    // Internal configuration
    static constexpr int MAX_WIFI_ATTEMPTS = 10;
    static constexpr int RSSI_THRESHOLD = -75;
    static constexpr uint64_t RECONNECT_INTERVAL_US = 15ULL * 1000000ULL; // 15 seconds

    // Internal helpers
    void log_message(const char* msg) const;
    void set_led(bool connected) const;
    bool try_connect(const char* ssid, const char* pass, int timeout_ms = 10000);
};

#endif // WIFI_MANAGER_H