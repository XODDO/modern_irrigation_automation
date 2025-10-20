#ifndef UPLOADER_H
#define UPLOADER_H

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>

/*
uploader.h — Class for securely posting JSON data to a remote web server.

Features:
• Uses WiFiClientSecure (setInsecure) for shared hosting compatibility
• Clean resource management (no heap leaks)
• Timestamp tracking via esp_timer_get_time()
• Detailed failure logging and human-readable error codes
• Modular class — ready for use inside RTOS tasks or other projects
*/

class uploader {
public:
//uint8_t server_led = 14; // Status LED for server activity
uint64_t last_upload = 0; // Timestamp (µs) of last upload
char error_code[50] = ""; // Stores reason for last failure/success
int keyIndex = 0; // Reserved for future use (WEP legacy)

// Initialize I/O and internal state
void begin(uint8_t server_led);

// Primary upload routine (HTTP POST JSON)
void upload_to_web_of_iot();

// Deprecated function placeholder for legacy system
void upload_to_thingsPeak() { Serial.println("[Uploader] Deprecated: upload_to_thingsPeak()"); }


};

#endif // UPLOADER_H