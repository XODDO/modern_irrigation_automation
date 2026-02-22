#include "upload.h"

upload::upload(uint8_t server_led_pin, const char* server_url)
    : server_led(server_led_pin), last_upload(0), serverName(server_url), keyIndex(0)
{
    error_code[0] = '\0';
    upload_report[0] = '\0';
}

void upload::begin() {
    pinMode(server_led, OUTPUT);
    set_led(false); // HIGH = idle/off
    Serial.println("[Uploader] Initialized.");
    update_report("Uploader initialized.");
}

UploadStatus upload::upload_to_web_of_iot(const char* JSON_Bundle) {
    Serial.println();
    Serial.println("=== [Uploader] Begin HTTP Upload ===");

    size_t payload_len = strlen(JSON_Bundle);
    if (payload_len > MAX_PAYLOAD_SIZE) {
        Serial.println("[Uploader] ❌ Payload too large!");
        update_error("Payload too large");
        update_report("Upload failed: Payload too large.");
        return UPLOAD_TOO_LARGE;
    }

    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("[Uploader] ❌ WiFi not connected. Upload skipped.");
        update_error("WiFi disconnected");
        update_report("Upload failed: WiFi not connected.");
        return UPLOAD_WIFI_DISCONNECTED;
    }

    set_led(true);

    WiFiClientSecure client;
    client.setInsecure(); // NOTE: Insecure, but practical for many ESP32 apps

    HTTPClient https;
    if (!https.begin(client, serverName)) {
        Serial.println("[Uploader] ❌ HTTPS initialization failed! Check URL or memory.");
        update_error("HTTPS init failed");
        update_report("Upload failed: HTTPS init failed.");
        set_led(false);
        return UPLOAD_HTTPS_INIT_FAILED;
    }

    https.setTimeout(15000);
    https.addHeader("Content-Type", "application/json");

    Serial.printf("[Uploader] ➜ POST to: %s\n", serverName);
    Serial.printf("[Uploader] ➜ Payload length: %d bytes\n", int(payload_len));

    int httpResponse = https.POST((uint8_t*)JSON_Bundle, payload_len);
    String httpStr = https.errorToString(httpResponse);

    snprintf(upload_report, sizeof(upload_report),
        "HTTP POST: %s, Resp=%d (%s), Len=%d",
        serverName, httpResponse, httpStr.c_str(), int(payload_len));

    Serial.printf("[Uploader] HTTP Response: %d (%s)\n", httpResponse, httpStr.c_str());
    Serial.println();

    UploadStatus status = UPLOAD_NET_TLS_FAILURE;

    if (httpResponse > 0) {
        if (httpResponse == HTTP_CODE_OK) {
            Serial.println("[Uploader] ✅ Uploaded Successfully!");
            update_error("OK");
            strncat(upload_report, " | Success", sizeof(upload_report) - strlen(upload_report) - 1);
            status = UPLOAD_HTTP_OK;
        }
        else if (httpResponse >= 400 && httpResponse < 500) {
            Serial.println("[Uploader] ❌ Client Error (4xx)");
            update_error("4xx client error");
            strncat(upload_report, " | Client Error", sizeof(upload_report) - strlen(upload_report) - 1);
            status = UPLOAD_HTTP_CLIENT_ERROR;
        }
        else if (httpResponse >= 500) {
            Serial.println("[Uploader] ❌ Server Error (5xx)");
            update_error("5xx server error");
            strncat(upload_report, " | Server Error", sizeof(upload_report) - strlen(upload_report) - 1);
            status = UPLOAD_HTTP_SERVER_ERROR;
        }
        else {
            Serial.println("[Uploader] ⚠️ Unexpected response code");
            update_error("Unexpected");
            strncat(upload_report, " | Unexpected", sizeof(upload_report) - strlen(upload_report) - 1);
            status = UPLOAD_HTTP_UNEXPECTED;
        }
    } else {
        Serial.println("[Uploader] ❌ Network-level failure");
        update_error("Network/TLS failure");
        strncat(upload_report, " | Network/TLS failure", sizeof(upload_report) - strlen(upload_report) - 1);
        status = UPLOAD_NET_TLS_FAILURE;
    }

    https.end();
    set_led(false);
    last_upload = esp_timer_get_time();

    char ts_msg[50];
    snprintf(ts_msg, sizeof(ts_msg), " | Timestamp: %llu µs", last_upload);
    strncat(upload_report, ts_msg, sizeof(upload_report) - strlen(upload_report) - 1);

    Serial.printf("[Uploader] Finished. Timestamp: %llu µs\n", last_upload);
    Serial.printf("[Uploader] Report: %s\n", upload_report);
    Serial.println("========================================");

    return status;
}

uint64_t upload::get_last_upload() const {
    return last_upload;
}

const char* upload::get_error_code() const {
    return error_code;
}

const char* upload::get_upload_report() const {
    return upload_report;
}

void upload::set_server_url(const char* url) {
    serverName = url;
}

void upload::update_report(const char* report) {
    strncpy(upload_report, report, sizeof(upload_report) - 1);
    upload_report[sizeof(upload_report) - 1] = '\0';
}

void upload::update_error(const char* err) {
    strncpy(error_code, err, sizeof(error_code) - 1);
    error_code[sizeof(error_code) - 1] = '\0';
}

void upload::set_led(bool active) const {
    digitalWrite(server_led, active ? LOW : HIGH);
}