#include "WiFi_Manager.h"

WiFi_Manager::WiFi_Manager(uint8_t wifi_led_pin)
    : devicename(nullptr), currentNetworkIndex(-1),
      wifi_connect_attempts(0), wifi_led(wifi_led_pin), lastReconnectAttempt(0),
      backoff_delay_ms(1000), consecutive_failures(0), ntp_synced(false) {}

WiFiMgrStatus WiFi_Manager::initialize_ESP_WiFi(const char* device_name) {
    devicename = device_name;
    WiFi.mode(WIFI_STA);
    pinMode(wifi_led, OUTPUT);
    set_led(false); // Assume disconnected

    Serial.println("\n[WiFi] ========== INITIAL SCAN ==========");
    log_message("Scanning for available WiFi networks...");
    
    int n = WiFi.scanNetworks(false, true);
    if (n < 0) {
        log_message("WiFi scan error.");
        WiFi.scanDelete();
        return WIFI_MGR_SCAN_ERR;
    }
    if (n == 0) {
        log_message("No WiFi networks found.");
        WiFi.scanDelete();
        return WIFI_MGR_NO_NETWORKS;
    }

    // Print all found networks
    Serial.printf("[WiFi] Found %d network(s):\n", n);
    for (int i = 0; i < n; i++) {
        Serial.printf("  %d. %s (RSSI: %d dBm)\n", i + 1, WiFi.SSID(i).c_str(), WiFi.RSSI(i));
    }

    // Find strongest known network
    int bestIndex = -1, bestRSSI = -1000;
    String bestSSID = "";
    for (int i = 0; i < n; i++) {
        String foundSSID = WiFi.SSID(i);
        for (int k = 0; k < knownCount; k++) {
            if (foundSSID == knownNetworks[k].SECRET_SSID && WiFi.RSSI(i) > bestRSSI) {
                bestIndex = k;
                bestRSSI = WiFi.RSSI(i);
                bestSSID = foundSSID;
            }
        }
    }
    WiFi.scanDelete();
    
    if (bestIndex < 0) {
        Serial.println("[WiFi] Known networks list:");
        for (int k = 0; k < knownCount; k++) {
            Serial.printf("  - %s\n", knownNetworks[k].SECRET_SSID);
        }
        log_message("No known networks available.");
        return WIFI_MGR_NO_KNOWN;
    }

    Serial.printf("[WiFi] Best known network: %s (RSSI: %d dBm)\n", bestSSID.c_str(), bestRSSI);
    log_message("Found known network. Connecting...");
    
    if (try_connect(knownNetworks[bestIndex].SECRET_SSID, knownNetworks[bestIndex].SECRET_PASS)) {
        currentNetworkIndex = bestIndex;
        set_led(true);
        log_message("WiFi connected successfully.");
        Serial.printf("[WiFi] SSID: %s\n", knownNetworks[bestIndex].SECRET_SSID);
        Serial.printf("[WiFi] IP Address: %s\n", get_ip().c_str());
        Serial.printf("[WiFi] RSSI: %d dBm\n", WiFi.RSSI());
        Serial.printf("[WiFi] Channel: %d\n", WiFi.channel());
        
        consecutive_failures = 0;  // Reset failure counter
        backoff_delay_ms = 1000;   // Reset backoff
        return WIFI_MGR_SUCCESS;
    } else {
        set_led(false);
        log_message("Connection attempt failed.");
        consecutive_failures++;
        return WIFI_MGR_CONN_FAIL;
    }
}

bool WiFi_Manager::ensure_wifi() {
    // If already connected, DO NOT scan, do NOT reconnect
    if (WiFi.status() == WL_CONNECTED) {
        set_led(true);
        if (consecutive_failures > 0) {
            consecutive_failures = 0;
            backoff_delay_ms = 1000;
            Serial.println("[WiFi] Connection restored, resetting backoff counter.");
        }
        return true;
    }

    uint64_t now_us = esp_timer_get_time();
    uint64_t now_ms = now_us / 1000;
    
    // Check if we're in backoff period
    if (consecutive_failures >= 5 && (now_ms - lastReconnectAttempt) < backoff_delay_ms) {
        static uint32_t last_backoff_print = 0;
        if ((now_ms - last_backoff_print) > 10000) { // Print every 10 seconds
            Serial.printf("[WiFi] Exponential backoff: Waiting %lu ms before next attempt (failures: %d)\n", 
                         backoff_delay_ms, consecutive_failures);
            last_backoff_print = now_ms;
        }
        return false; // Still in backoff period
    }
    
    if (now_us - lastReconnectAttempt < RECONNECT_INTERVAL_US)
        return false; // throttle attempts
    lastReconnectAttempt = now_us;

    Serial.println("\n[WiFi] ========== RECONNECTION ATTEMPT ==========");
    log_message("WiFi lost. Attempting reconnection...");
    Serial.printf("[WiFi] Status code: %d\n", WiFi.status());
    set_led(false);

    // Step 1: Try previous network if still visible and decent RSSI
    if (currentNetworkIndex >= 0) {
        Serial.printf("[WiFi] Checking previous network: %s\n", 
                     knownNetworks[currentNetworkIndex].SECRET_SSID);
        
        int n = WiFi.scanNetworks(false, true);
        bool foundPrev = false;
        int prevRSSI = -1000;
        
        if (n > 0) {
            for (int i = 0; i < n; i++) {
                String foundSSID = WiFi.SSID(i);
                if (foundSSID == knownNetworks[currentNetworkIndex].SECRET_SSID) {
                    foundPrev = true;
                    prevRSSI = WiFi.RSSI(i);
                    Serial.printf("[WiFi] Previous network found! RSSI: %d dBm\n", prevRSSI);
                    break;
                }
            }
        }
        WiFi.scanDelete();
        
        if (foundPrev && prevRSSI > RSSI_THRESHOLD) {
            log_message("Previous network still visible. Reconnecting...");
            if (try_connect(knownNetworks[currentNetworkIndex].SECRET_SSID,
                            knownNetworks[currentNetworkIndex].SECRET_PASS)) {
                set_led(true);
                log_message("Reconnected to previous network.");
                Serial.printf("[WiFi] SSID: %s\n", knownNetworks[currentNetworkIndex].SECRET_SSID);
                Serial.printf("[WiFi] IP: %s\n", get_ip().c_str());
                consecutive_failures = 0;
                backoff_delay_ms = 1000;
                return true;
            }
        } else {
            if (!foundPrev) {
                log_message("Previous network not found in scan.");
            } else {
                Serial.printf("[WiFi] Previous network RSSI %d dBm below threshold %d\n", 
                             prevRSSI, RSSI_THRESHOLD);
            }
            log_message("Searching alternatives...");
        }
    }

    // Step 2: Try strongest of all known
    Serial.println("[WiFi] Scanning for alternative networks...");
    int n = WiFi.scanNetworks(false, true);
    int bestIndex = -1, bestRSSI = -1000;
    String bestSSID = "";
    
    if (n > 0) {
        Serial.printf("[WiFi] Found %d network(s) in scan:\n", n);
        for (int i = 0; i < n && i < 10; i++) { // Limit to first 10 for readability
            Serial.printf("  %d. %s (RSSI: %d dBm)\n", i + 1, WiFi.SSID(i).c_str(), WiFi.RSSI(i));
        }
        
        for (int i = 0; i < n; i++) {
            String foundSSID = WiFi.SSID(i);
            for (int k = 0; k < knownCount; k++) {
                if (foundSSID == knownNetworks[k].SECRET_SSID && WiFi.RSSI(i) > bestRSSI) {
                    bestIndex = k;
                    bestRSSI = WiFi.RSSI(i);
                    bestSSID = foundSSID;
                }
            }
        }
    }
    WiFi.scanDelete();

    if (bestIndex >= 0) {
        Serial.printf("[WiFi] Best alternative: %s (RSSI: %d dBm)\n", bestSSID.c_str(), bestRSSI);
        log_message("Connecting to strongest known network...");
        if (try_connect(knownNetworks[bestIndex].SECRET_SSID,
                        knownNetworks[bestIndex].SECRET_PASS)) {
            currentNetworkIndex = bestIndex;
            set_led(true);
            log_message("Connected to new network successfully.");
            Serial.printf("[WiFi] New SSID: %s\n", knownNetworks[bestIndex].SECRET_SSID);
            Serial.printf("[WiFi] New IP: %s\n", get_ip().c_str());
            consecutive_failures = 0;
            backoff_delay_ms = 1000;
            return true;
        }
    }

    // Connection failed - update backoff
    consecutive_failures++;
    
    // Exponential backoff: 1s, 2s, 4s, 8s, 16s, 32s, 64s... capped at 5 minutes
    if (consecutive_failures >= 5) {
        unsigned long new_backoff = backoff_delay_ms * 2;
        backoff_delay_ms = (new_backoff < 300000) ? new_backoff : 300000; // Cap at 5 minutes
        Serial.printf("[WiFi] Exponential backoff increased to %lu ms (failures: %d)\n", 
                     backoff_delay_ms, consecutive_failures);
    }
    
    // Print WDT warning if needed
    if (consecutive_failures >= 10) {
        Serial.println("[WiFi] WARNING: Many consecutive failures! Watchdog may trigger soon.");
        Serial.printf("[WiFi] Last successful connection: %d attempts ago\n", consecutive_failures);
    }
    
    log_message("WiFi reconnect failed. Will retry later.");
    set_led(false);
    return false;
}

bool WiFi_Manager::is_connected() const {
    bool connected = (WiFi.status() == WL_CONNECTED);
    if (!connected && wifi_connect_attempts > 0) {
        // Don't spam, just occasional debug
        static uint32_t last_status_print = 0;
        uint64_t now_ms = esp_timer_get_time() / 1000;
        if ((now_ms - last_status_print) > 30000) { // Every 30 seconds
            Serial.printf("[WiFi] Status: Disconnected (code: %d)\n", WiFi.status());
            last_status_print = now_ms;
        }
    }
    return connected;
}

String WiFi_Manager::get_ip() const {
    return (WiFi.status() == WL_CONNECTED) ? WiFi.localIP().toString() : "0.0.0.0";
}

bool WiFi_Manager::try_connect(const char* ssid, const char* pass, int timeout_ms) {
    Serial.printf("[WiFi] Connecting to '%s'...\n", ssid);
    WiFi.begin(ssid, pass);
    
    uint64_t start_us = esp_timer_get_time();
    uint64_t timeout_us = timeout_ms * 1000ULL;
    int last_status = -1;
    
    while (WiFi.status() != WL_CONNECTED) {
        delay(250);
        
        // Print status every 2 seconds during connection attempt
        int current_status = WiFi.status();
        if (current_status != last_status) {
            const char* status_str;
            switch(current_status) {
                case WL_NO_SSID_AVAIL: status_str = "NO_SSID_AVAIL"; break;
                case WL_CONNECT_FAILED: status_str = "CONNECT_FAILED"; break;
                case WL_IDLE_STATUS: status_str = "IDLE"; break;
                case WL_DISCONNECTED: status_str = "DISCONNECTED"; break;
                default: status_str = "UNKNOWN";
            }
            Serial.printf("[WiFi] Connection status: %s (%d)\n", status_str, current_status);
            last_status = current_status;
        }
        
        if ((esp_timer_get_time() - start_us) > timeout_us) {
            log_message("Connection timeout.");
            Serial.printf("[WiFi] Failed to connect to '%s' after %d ms\n", ssid, timeout_ms);
            return false;
        }
    }
    
    Serial.printf("[WiFi] Successfully connected to '%s'\n", ssid);
    return true;
}

void WiFi_Manager::log_message(const char* msg) const {
    Serial.print("[WiFi] ");
    Serial.println(msg);
}

void WiFi_Manager::set_led(bool connected) const {
    digitalWrite(wifi_led, connected ? LOW : HIGH);
}

bool WiFi_Manager::sync_ntp(long gmtOffset_sec, int daylightOffset_sec, uint8_t timeoutSeconds) {
    if (WiFi.status() != WL_CONNECTED) {
        log_message("Cannot sync NTP - WiFi not connected.");
        ntp_synced = false;
        return false;
    }

    log_message("Starting NTP sync...");
    Serial.println("[WiFi] NTP Servers: pool.ntp.org, time.nist.gov");
    configTime(gmtOffset_sec, daylightOffset_sec, "pool.ntp.org", "time.nist.gov");

    uint8_t attempts = 0;
    uint8_t maxAttempts = timeoutSeconds * 2;  // check every 500ms
    struct tm test_time;

    while (!getLocalTime(&test_time) && attempts < maxAttempts) {
        delay(500);
        attempts++;
        if (attempts % 4 == 0) { // Print every 2 seconds
            Serial.printf("[WiFi] Waiting for NTP... (%d/%d attempts)\n", attempts, maxAttempts);
        }
    }

    if (!getLocalTime(&ntp_timeinfo)) {
        log_message("NTP sync timed out.");
        Serial.printf("[WiFi] NTP failed after %d attempts\n", maxAttempts);
        ntp_synced = false;
        return false;
    }

    char buf[128];
    snprintf(buf, sizeof(buf), "NTP synced: %04d-%02d-%02d %02d:%02d:%02d (UTC%+ld)",
             ntp_timeinfo.tm_year + 1900,
             ntp_timeinfo.tm_mon  + 1,
             ntp_timeinfo.tm_mday,
             ntp_timeinfo.tm_hour,
             ntp_timeinfo.tm_min,
             ntp_timeinfo.tm_sec,
             gmtOffset_sec / 3600);
    log_message(buf);
    
    Serial.printf("[WiFi] NTP Server: %s\n", ntp_synced ? "Synced" : "Failed");
    Serial.printf("[WiFi] Time source: %s\n", ntp_synced ? "Network" : "RTC fallback");

    ntp_synced = true;
    return true;
}

// Helper method to get connection statistics
void WiFi_Manager::print_connection_stats() const {
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\n[WiFi] ========== CONNECTION STATS ==========");
        Serial.printf("  SSID: %s\n", WiFi.SSID().c_str());
        Serial.printf("  IP Address: %s\n", WiFi.localIP().toString().c_str());
        Serial.printf("  Gateway: %s\n", WiFi.gatewayIP().toString().c_str());
        Serial.printf("  Subnet Mask: %s\n", WiFi.subnetMask().toString().c_str());
        Serial.printf("  DNS: %s\n", WiFi.dnsIP().toString().c_str());
        Serial.printf("  RSSI: %d dBm\n", WiFi.RSSI());
        Serial.printf("  Channel: %d\n", WiFi.channel());
        Serial.printf("  BSSID: %s\n", WiFi.BSSIDstr().c_str());
        Serial.printf("  Failures: %d\n", consecutive_failures);
        if (consecutive_failures >= 5) {
            Serial.printf("  Backoff delay: %lu ms\n", backoff_delay_ms);
        }
        Serial.println("=========================================");
    } else {
        Serial.println("[WiFi] Not connected - cannot print stats");
    }
}

// ====== Wi-Fi Connect Cycle ======
bool connectKnownWiFi() {
  bool connected_state = false;
  
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true);
  delay(100);

  int n = WiFi.scanNetworks();
  if (n <= 0) return false;

  int bestIndex = -1, bestRSSI = -999;
  for (int i = 0; i < n; i++) {
    String foundSSID = WiFi.SSID(i);
    int32_t rssi = WiFi.RSSI(i);
    for (int j = 0; j < knownCount; j++) {
      if (foundSSID == knownNetworks[j].SECRET_SSID) {
        if (rssi > bestRSSI) { bestRSSI = rssi; bestIndex = j; }
      }
    }
  }

  if (bestIndex == -1) return false;

  WiFi.begin(knownNetworks[bestIndex].SECRET_SSID, knownNetworks[bestIndex].SECRET_PASS);
  unsigned long startAttempt = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startAttempt < 10000) {
    delay(100);
    yield();
  }

  if (WiFi.status() == WL_CONNECTED) {
    //err = 14;
    //error_log(err);
    connected_state = true;
   // return true;
  } else { // WiFi connection failed
    //err = 15;
    //error_log(err);
    connected_state = false;
   // return false;
  }

  return connected_state;
}

bool WiFi_Manager::connect_to_ssid(const char* ssid, const char* password) {
    Serial.printf("[WiFi] Connecting to specific SSID: %s\n", ssid);
    WiFi.begin(ssid, password);
    
    uint64_t start_us = esp_timer_get_time();
    uint64_t timeout_us = 20000ULL * 1000ULL; // 20 seconds timeout
    
    while (WiFi.status() != WL_CONNECTED) {
        delay(250);
        if ((esp_timer_get_time() - start_us) > timeout_us) {
            Serial.println("[WiFi] Connection timeout");
            return false;
        }
    }
    
    Serial.printf("[WiFi] Connected to %s\n", ssid);
    Serial.printf("[WiFi] IP: %s\n", WiFi.localIP().toString().c_str());
    return true;
}

bool WiFi_Manager::is_connected_to_ssid(const char* ssid) {
    if (WiFi.status() != WL_CONNECTED) return false;
    return (WiFi.SSID() == String(ssid));
}

