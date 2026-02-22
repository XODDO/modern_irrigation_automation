#pragma once
#ifndef UPLOAD_H
#define UPLOAD_H

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <cstdint>

/**
 * @brief Status codes for upload results.
 */
enum UploadStatus : uint8_t {
    UPLOAD_SUCCESS = 0,
    UPLOAD_WIFI_DISCONNECTED,
    UPLOAD_HTTPS_INIT_FAILED,
    UPLOAD_HTTP_OK,
    UPLOAD_HTTP_CLIENT_ERROR,
    UPLOAD_HTTP_SERVER_ERROR,
    UPLOAD_HTTP_UNEXPECTED,
    UPLOAD_NET_TLS_FAILURE,
    UPLOAD_TIMEOUT,
    UPLOAD_TOO_LARGE
};

/**
 * @brief ESP32 HTTPS uploader class.
 */
class upload {
public:
    explicit upload(uint8_t server_led_pin = 27, const char* server_url = "https://www.webofiot.com/irrigation/muarik_irrikit/server.php");

    /**
     * @brief Initialize uploader hardware (LED etc).
     */
    void begin();

    /**
     * @brief Upload JSON data to server.
     * @param JSON_Bundle Payload to upload (up to 4096 bytes).
     * @return UploadStatus code.
     */
    UploadStatus upload_to_web_of_iot(const char* JSON_Bundle);

    /**
     * @brief Get last upload timestamp (µs).
     */
    uint64_t get_last_upload() const;

    /**
     * @brief Get last error code string.
     */
    const char* get_error_code() const;

    /**
     * @brief Get upload report log.
     */
    const char* get_upload_report() const;

    /**
     * @brief Set server URL.
     */
    void set_server_url(const char* url);

private:
    uint8_t server_led;
    uint64_t last_upload;
    char error_code[50];
    char upload_report[150];
    const char* serverName;
    int keyIndex; // reserved

    static constexpr size_t MAX_PAYLOAD_SIZE = 4096;

    /**
     * @brief Helper to update report buffer (safe).
     */
    void update_report(const char* report);

    /**
     * @brief Helper to update error code buffer (safe).
     */
    void update_error(const char* err);

    /**
     * @brief LED state setter.
     */
    void set_led(bool active) const;
};

#endif // UPLOAD_H