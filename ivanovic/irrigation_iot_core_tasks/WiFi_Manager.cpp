#include "WiFi_Manager.h"

WiFi_Manager::WiFi_Manager(uint8_t wifi_led_pin)
    : devicename(nullptr), currentNetworkIndex(-1),
      wifi_connect_attempts(0), wifi_led(wifi_led_pin), lastReconnectAttempt(0) {}

WiFiMgrStatus WiFi_Manager::initialize_ESP_WiFi(const char* device_name) {
    devicename = device_name;
    WiFi.mode(WIFI_STA);
    pinMode(wifi_led, OUTPUT);
    set_led(false); // Assume disconnected

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

    // Find strongest known network
    int bestIndex = -1, bestRSSI = -1000;
    for (int i = 0; i < n; i++) {
        String foundSSID = WiFi.SSID(i);
        for (int k = 0; k < knownCount; k++) {
            if (foundSSID == knownNetworks[k].SECRET_SSID && WiFi.RSSI(i) > bestRSSI) {
                bestIndex = k;
                bestRSSI = WiFi.RSSI(i);
            }
        }
    }
    WiFi.scanDelete();
    if (bestIndex < 0) {
        log_message("No known networks available.");
        return WIFI_MGR_NO_KNOWN;
    }

    log_message("Found known network. Connecting...");
    if (try_connect(knownNetworks[bestIndex].SECRET_SSID, knownNetworks[bestIndex].SECRET_PASS)) {
        currentNetworkIndex = bestIndex;
        set_led(true);
        log_message("WiFi connected successfully.");
        log_message(get_ip().c_str());
        return WIFI_MGR_SUCCESS;
    } else {
        set_led(false);
        log_message("Connection attempt failed.");
        return WIFI_MGR_CONN_FAIL;
    }
}

bool WiFi_Manager::ensure_wifi() {
    if (WiFi.status() == WL_CONNECTED) return true;

    uint64_t now_us = esp_timer_get_time();
    if (now_us - lastReconnectAttempt < RECONNECT_INTERVAL_US)
        return false; // throttle attempts
    lastReconnectAttempt = now_us;

    log_message("WiFi lost. Attempting reconnection...");
    set_led(false);

    // Step 1: Try previous network if still visible and decent RSSI
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
        WiFi.scanDelete();
        if (foundPrev && prevRSSI > RSSI_THRESHOLD) {
            log_message("Previous network still visible. Reconnecting...");
            if (try_connect(knownNetworks[currentNetworkIndex].SECRET_SSID,
                            knownNetworks[currentNetworkIndex].SECRET_PASS)) {
                set_led(true);
                log_message("Reconnected to previous network.");
                return true;
            }
        } else {
            log_message("Previous network weak or gone. Searching alternatives...");
        }
    }

    // Step 2: Try strongest of all known
    int n = WiFi.scanNetworks(false, true);
    int bestIndex = -1, bestRSSI = -1000;
    for (int i = 0; i < n; i++) {
        String foundSSID = WiFi.SSID(i);
        for (int k = 0; k < knownCount; k++) {
            if (foundSSID == knownNetworks[k].SECRET_SSID && WiFi.RSSI(i) > bestRSSI) {
                bestIndex = k;
                bestRSSI = WiFi.RSSI(i);
            }
        }
    }
    WiFi.scanDelete();

    if (bestIndex >= 0) {
        log_message("Connecting to strongest known network...");
        if (try_connect(knownNetworks[bestIndex].SECRET_SSID,
                        knownNetworks[bestIndex].SECRET_PASS)) {
            currentNetworkIndex = bestIndex;
            set_led(true);
            log_message("Connected to new network successfully.");
            return true;
        }
    }

    log_message("WiFi reconnect failed. Will retry later.");
    set_led(false);
    return false;
}

bool WiFi_Manager::is_connected() const {
    return WiFi.status() == WL_CONNECTED;
}

String WiFi_Manager::get_ip() const {
    return (WiFi.status() == WL_CONNECTED) ? WiFi.localIP().toString() : "0.0.0.0";
}

bool WiFi_Manager::try_connect(const char* ssid, const char* pass, int timeout_ms) {
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

void WiFi_Manager::log_message(const char* msg) const {
    Serial.print("[WiFi] ");
    Serial.println(msg);
}

void WiFi_Manager::set_led(bool connected) const {
    digitalWrite(wifi_led, connected ? LOW : HIGH);
}