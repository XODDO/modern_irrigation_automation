#include "uploader.h"
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>

extern const char* dataPack; // defined elsewhere (JSON payload)
const char* serverName = "https://www.webofiot.com/irrigation/muarik_irrikit/server.php"; // replace with your endpoint

void uploader::begin(server_led) {
pinMode(server_led, OUTPUT);
digitalWrite(server_led, HIGH); // HIGH = idle
Serial.println("[Uploader] Initialized.");
}

void uploader::upload_to_web_of_iot() {
Serial.println();
Serial.println("=== [Uploader] Begin HTTP Upload ===");

if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[Uploader] ❌ WiFi not connected. Upload skipped.");
    strcpy(error_code, "WiFi disconnected");
    return;
}

digitalWrite(server_led, LOW);  // LOW = active

WiFiClientSecure client;
client.setInsecure(); // Disable certificate verification (shared hosting friendly)

HTTPClient https;
if (!https.begin(client, serverName)) {
    Serial.println("[Uploader] ❌ HTTPS initialization failed! Check URL or memory.");
    strcpy(error_code, "HTTPS init failed");
    digitalWrite(server_led, HIGH);
    return;
}

https.setTimeout(15000); // 15-second network timeout
https.addHeader("Content-Type", "application/json");

Serial.printf("[Uploader] ➜ POST to: %s\n", serverName);
Serial.printf("[Uploader] ➜ Payload length: %d bytes\n", strlen(dataPack));

int httpResponse = https.POST((uint8_t*)dataPack, strlen(dataPack));

Serial.printf("[Uploader] HTTP Response: %d (%s)\n",
              httpResponse, https.errorToString(httpResponse).c_str());
Serial.println();

// --- Interpret Response Codes ---
if (httpResponse > 0) {
    if (httpResponse == HTTP_CODE_OK) {
        Serial.println("[Uploader] ✅ Uploaded Successfully! YeeeeY!");
        strcpy(error_code, "OK");
    }
    else if (httpResponse >= 100 && httpResponse < 200) {
        Serial.println("[Uploader] ⚠️ Informational response received — unusual for POST.");
        strcpy(error_code, "Informational response");
    }
    else if (httpResponse >= 200 && httpResponse < 300) {
        Serial.println("[Uploader] ✅ Success (2xx) — data accepted by server.");
        strcpy(error_code, "2xx success");
    }
    else if (httpResponse >= 300 && httpResponse < 400) {
        Serial.println("[Uploader] ⚠️ Redirection (3xx) — check endpoint or SSL redirect.");
        strcpy(error_code, "3xx redirect");
    }
    else if (httpResponse >= 400 && httpResponse < 500) {
        Serial.println("[Uploader] ❌ Client Error (4xx). Possible causes:");
        Serial.println("   → Wrong URL or API path");
        Serial.println("   → Missing headers or invalid JSON format");
        Serial.println("   → Authentication/authorization failure");
        strcpy(error_code, "4xx client error");
    }
    else if (httpResponse >= 500 && httpResponse < 600) {
        Serial.println("[Uploader] ❌ Server Error (5xx). Possible causes:");
        Serial.println("   → Server temporarily unavailable");
        Serial.println("   → Backend error or misconfigured PHP handler");
        strcpy(error_code, "5xx server error");
    }
    else if (httpResponse >= 600) {
        Serial.println("[Uploader] ❌ Undefined HTTP status code (>=600). Server bug?");
        strcpy(error_code, "600+ undefined");
    }
    else {
        Serial.println("[Uploader] ⚠️ Unexpected response code — investigate manually.");
        strcpy(error_code, "unknown response");
    }
}
else {
    // Negative responses are connection-level failures, not HTTP responses
    Serial.println("[Uploader] ❌ Transmission failed. Possible causes:");
    Serial.println("   → TLS handshake timeout");
    Serial.println("   → DNS lookup failed");
    Serial.println("   → No route to host / broken socket");
    Serial.println("   → Server offline");
    strcpy(error_code, "network or TLS failure");
}

https.end();  // free resources
digitalWrite(server_led, HIGH); // return to idle
last_upload = esp_timer_get_time(); // microsecond precision

Serial.printf("[Uploader] Finished. Timestamp: %llu µs\n", last_upload);
Serial.println("========================================");


}