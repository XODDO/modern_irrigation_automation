/*
   XODDOCODE 2025
   RECEIVE READINGS FROM READINGS BY UART SERIAL
   SEND AS IS TO WEB OF IOT
   ONLY EXTRACT TEMPERATURE TO DRIVE THE INCABINET FAN

   Refactored:
   - Uses FreeRTOS mutex (SemaphoreHandle_t) for safe double-buffering between UART and upload tasks
   - receive_serial_data() writes to receiveBuffer, then copies to uploadBuffer under mutex
   - upload_task() copies uploadBuffer to local buffer under mutex before upload
   - All other system logic (WiFi, OTA, fan, logging) retained as before
*/

#include <stdlib.h>
#include <Arduino.h>
#include <WiFi.h>
#include <math.h>
#include <ArduinoJson.h>
#include <ArduinoOTA.h>
#include <SPIFFS.h>
#include "WiFi_Manager.h"
#include "upload.h"

const char *server_address = "https://www.webofiot.com/irrigation/muarik_irrikit/server.php";
const char *devicename = "IRRIGATION_CONTROLLER_GATEWAY";
const char *OTA_PASS = "password";

enum SystemStatus : uint8_t {
    STATUS_BOOT = 1,
    STATUS_SERIAL_INIT,
    STATUS_BUFFER_READY,
    STATUS_WIFI_INIT,
    STATUS_RUNNING,
    STATUS_WIFI_FAIL = 10
};

volatile bool otaFinished = false;
volatile bool otaStarted = false;
volatile bool otaError = false;

const uint8_t server_led = 27;
const uint8_t heartbeat = 2;
const uint8_t flow_meter_indicator = 23;
const uint8_t internal_fan = 5;
constexpr uint8_t wifi_led = 25;

WiFi_Manager wifi_obj(wifi_led);
upload Uploader(server_led, server_address);

constexpr size_t JSON_BUFFER_SIZE = 8192; // 4096; UART frame exceeded ~4095 bytes before a closing brace } was seen.

// Reserve enough JSON heap for parsing incoming payloads. 4KB is large, but ok here.
static StaticJsonDocument<JSON_BUFFER_SIZE> JSON_received;


// --- MUTEX and BUFFERS for double-buffered UART/upload ---
SemaphoreHandle_t xDataMutex = NULL;
char receiveBuffer[JSON_BUFFER_SIZE];
char uploadBuffer[JSON_BUFFER_SIZE];

constexpr int32_t wiot_upload_frequency = int32_t(10 * 60 * 1000); // 10 min

void upload_task(void *pvParams);
void fanning_task(void *pvParams);
void Serial_RX_Task(void *pvParams);

void receive_serial_data(uint64_t call_time);
void monitor_internal_temp();
void log_status(SystemStatus state);
void printSystemDiagnostics(uint64_t now_diag_time);

void initializeOTA();

void flash(uint64_t time_now, int heartbeat,
           uint16_t ON_1, uint16_t OFF_1,
           uint16_t ON_2, uint16_t OFF_2);

constexpr size_t UPLOAD_LOG_CAPACITY = 16;
char upload_logs[UPLOAD_LOG_CAPACITY][150];
uint8_t upload_log_index = 0;

uint8_t status_code = STATUS_BOOT;
uint64_t now_now = 0, most_recent_packet_time = 0;

// --- OTA logging ---
char ota_log[150] = "...";
bool wifi_connected = false;
uint8_t PRIORITY_HIGH = 4; // Ensures upload runs deterministically even under serial bursts
uint8_t PRIORITY_AVG = 2;
uint8_t PRIORITY_LOW = 1;
void setup() {
    delay(500); // For OTA RO STABILIZE

    Serial.begin(115200); 
    delay(500); 
    status_code = STATUS_BOOT; 
    log_status(static_cast<SystemStatus>(status_code));

    Serial2.begin(115200, SERIAL_8N1, 16, 17); 
    status_code = STATUS_SERIAL_INIT;
    Serial2.setRxBufferSize(JSON_BUFFER_SIZE);  
    status_code = STATUS_BUFFER_READY;      
    log_status(static_cast<SystemStatus>(status_code));

    pinMode(heartbeat, OUTPUT); digitalWrite(heartbeat, HIGH); delay(500);
    pinMode(internal_fan, OUTPUT); digitalWrite(internal_fan,  HIGH); delay(500);
    pinMode(flow_meter_indicator, OUTPUT); digitalWrite(flow_meter_indicator,  HIGH); delay(500);
    pinMode(server_led, OUTPUT); digitalWrite(server_led, HIGH); delay(500);

    status_code = static_cast<uint8_t>(wifi_obj.initialize_ESP_WiFi(devicename));
    log_status(static_cast<SystemStatus>(status_code));
    
    Uploader.begin();
    initializeOTA();

    Serial.println("OTA Ready!");
    Serial.print("Hostname: ");
    Serial.println(devicename);

    digitalWrite(heartbeat, LOW); delay(500);
    digitalWrite(wifi_led, LOW);  delay(500);
    digitalWrite(server_led, LOW); delay(500);
    digitalWrite(internal_fan, LOW); delay(500);
    digitalWrite(flow_meter_indicator, LOW); delay(500);

    xTaskCreatePinnedToCore(fanning_task, "InternalTEMP_MGT", 6240, NULL, PRIORITY_LOW, NULL, 1);

    xTaskCreatePinnedToCore(Serial_RX_Task, "Update_RECV_BUFF", 8192, NULL, PRIORITY_AVG, NULL, 1);
    // The uploader can stay deterministic, while serial RX can flexibly pause/resume. 
    // Uploads (Core 0) and serial parsing (Core 1) no longer block each other.

    xDataMutex = xSemaphoreCreateMutex();
    if (xDataMutex == NULL) {
        Serial.println("[BOOT] Failed to create data mutex!");
    }

    // Clear buffers so we don't attempt uploads with garbage
    receiveBuffer[0] = '\0';
    uploadBuffer[0] = '\0';


    Serial.printf("Heap before upload task: %u B\n", (ESP.getFreeHeap()));
    //size_t upload_stack_size = ESP.getFreeHeap() > 12288 ? 12288 : 8192; 
    // for 8192 360 bytes means you’re close to the limit — there’s a risk of overflow if the HTTP library uses more stack in a future transaction 
    //(e.g. during TLS renegotiation, long JSON strings, or error handling).

        // SIGNIFICANTLY increase stack size - WiFiClientSecure needs lots of stack
    size_t upload_stack_size = 16384; // 16KB minimum for HTTPS
    
    // If you have enough heap, use even more
    if (ESP.getFreeHeap() > 40000) {
        upload_stack_size = 20480; // 20KB for safety
    }
    
    BaseType_t RC = xTaskCreatePinnedToCore(upload_task, "Upload to the Internet", upload_stack_size, NULL, PRIORITY_HIGH, NULL, 0); // core 0

    Serial.printf("xTaskCreatePinnedToCore(upload_task) -> rc=%d\n", (int)RC);
    if (RC != pdPASS) {
        Serial.println("[BOOT] ERROR: upload_task creation failed!");
        // Add recovery logic here
    }

    Serial.printf("Heap after upload task: %u B\n", (ESP.getFreeHeap()));
    Serial.printf("Upload task stack size: %u B\n", (upload_stack_size));

    Serial.println("Done Booting!");
}

void Serial_RX_Task(void *pvParams) {
    const size_t task_stack_size = 8192;  // matches xTaskCreatePinnedToCore allocation
    UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);

    Serial.printf("[Serial_RX_Task] Started. Stack size = %u bytes. Initial high water mark = %u bytes\n",
                  task_stack_size, uxHighWaterMark);

    for (;;) {
        receive_serial_data(now_now);  // keep listening as frequently as possible...
        vTaskDelay(pdMS_TO_TICKS(5));  // every 5 ms

        // monitor stack usage occasionally
        static uint32_t lastCheck = 0;
        uint32_t now = millis();
        if (now - lastCheck >= 15000) { // every 15 seconds
            lastCheck = now;
            UBaseType_t uxNow = uxTaskGetStackHighWaterMark(NULL);
            size_t used = task_stack_size - uxNow;
            float usage_percent = (100.0f * used) / task_stack_size;

            Serial.printf("[Serial_RX_Task] Stack used: %u bytes (%.1f%%), Free: %u bytes\n",
                          (unsigned)used, usage_percent, (unsigned)uxNow);

            if (uxNow < 512) {
                Serial.printf("[Serial_RX_Task][WARN] Stack critically low: %u bytes remaining!\n",
                              (unsigned)uxNow);
            }
        }
    }
}

void upload_task(void *pvParams) {
    TickType_t prev_wake_time = xTaskGetTickCount();
    char localJson[JSON_BUFFER_SIZE];

    UBaseType_t uxHighWaterMark;
    uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL); // Monitor for Stack overflow in upload task
    Serial.printf("[upload_task] Initial stack high water mark before: %u\n", uxHighWaterMark);

    while (true) {
        // Clear local buffer first
        localJson[0] = '\0';
            wifi_connected = wifi_obj.ensure_wifi(); // record the most recent state of WiFi
            if (!wifi_connected) { // Extra debug: WiFi status before trying upload
                Serial.println("[Upload Task] WiFi not connected, skipping this cycle.");
                status_code = STATUS_WIFI_FAIL;
                log_status(static_cast<SystemStatus>(status_code));
                vTaskDelay(pdMS_TO_TICKS(10000)); // wait 10s, then retry next cycle
                continue; // keep task alive
            }


          // Copy uploadBuffer to localJson under mutex protection (short critical section)
        if (xSemaphoreTake(xDataMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
            // ensure null termination
            strncpy(localJson, uploadBuffer, sizeof(localJson) - 1);
            localJson[sizeof(localJson) - 1] = '\0';
            xSemaphoreGive(xDataMutex);
            Serial.println("[upload_task] Copied from uploadBuffer to local buffer.");
            uploadBuffer[0] = '\0'; // CLEAR UPLOAD BUFFER SO AS TO NOT UPLOAD STALE DATA

        } else {
            localJson[0] = '\0'; // failed to take mutex
            Serial.println("[upload_task] WARN: failed to take mutex for upload buffer copy.");
        }
                size_t payload_len = strlen(localJson);
        Serial.printf("[Upload Task] payload_len=%u\n", (unsigned)payload_len);

        // Skip if no fresh payload
        if (payload_len == 0) {
            Serial.println("[Upload Task] no payload, skipping upload cycle.");
        } else {
            

            UploadStatus st = Uploader.upload_to_web_of_iot(localJson);
            Serial.printf("[Upload Task] upload status=%d; report=%s\n", (int)st, Uploader.get_upload_report());

            uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
            Serial.printf("[upload_task] Stack high water mark after: %u\n", uxHighWaterMark);

        }

        // wait until next scheduled upload (non-drifting)
        vTaskDelayUntil(&prev_wake_time, pdMS_TO_TICKS(wiot_upload_frequency));
    }
    
}

void fanning_task(void *pvParams) {
    // Capture initial high-water mark right after task start
    UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
    const size_t task_stack_size = 4096; // same as you allocated

    Serial.printf("[fanning_task] Started. Stack size = %u bytes. Initial high water mark = %u bytes\n",
                  task_stack_size, uxHighWaterMark);

    for (;;) {
        monitor_internal_temp();

        // Recheck remaining stack margin occasionally
        UBaseType_t uxNow = uxTaskGetStackHighWaterMark(NULL);
        size_t used = task_stack_size - uxNow;
        float usage_percent = (100.0f * used) / task_stack_size;

        Serial.printf("[fanning_task] Stack used: %u bytes (%.1f%%), Free: %u bytes\n",
                      (unsigned)used, usage_percent, (unsigned)uxNow);

        vTaskDelay(pdMS_TO_TICKS(60000)); // every MINUTE
    }
}



bool receiving_UART = false;

void loop() {
    ArduinoOTA.handle();
    now_now = esp_timer_get_time() / 1000ULL; 
   
    // receive_serial_data(now_now);  // keep listening as frequently as possible... 

    if (!receiving_UART) {
            flash(now_now, heartbeat, 50, 75, 50, 1500); 
            wifi_connected = wifi_obj.ensure_wifi();
         // UPLOAD IS HANDLED BY RTOS TASK
          //  printSystemDiagnostics(now_now); // only print when connected to PC

    }

    else { // if receiving_UART
             flash(now_now, heartbeat, 200, 1000, 0, 0);        //          flash(now_now, wifi_led, 100, 100, 100, 900);

    if ((now_now - most_recent_packet_time) >= 5000) { // 5 seconds timeout
            receiving_UART = false;
        }
       
    }


    
}


void initializeOTA() {
    ArduinoOTA.setHostname(devicename);
    ArduinoOTA.setPassword(OTA_PASS);

    ArduinoOTA
        .onStart([]() { otaStarted = true; }) 
        .onEnd([]() { otaFinished = true;}) 
        .onProgress([](unsigned int progress, unsigned int total) {
            snprintf(ota_log, sizeof(ota_log), "Progress: %u%%", (progress * 100) / total);
        })
        .onError([](ota_error_t error) { otaError = true;
            snprintf(ota_log, sizeof(ota_log), "Error[%u]: ", error);
            if (error == OTA_AUTH_ERROR)
                strcat(ota_log, "Auth Failed");
            else if (error == OTA_BEGIN_ERROR)
                strcat(ota_log, "Begin Failed");
            else if (error == OTA_CONNECT_ERROR)
                strcat(ota_log, "Connect Failed");
            else if (error == OTA_RECEIVE_ERROR)
                strcat(ota_log, "Receive Failed");
            else if (error == OTA_END_ERROR)
                strcat(ota_log, "End Failed");
        });

    ArduinoOTA.begin();
    strcpy(ota_log, "OTA Ready!");
    Serial.println(ota_log);
}

void log_status(SystemStatus state) {
    char details[200] = ".";
    switch (state) {
        case STATUS_BOOT:
            snprintf(details, sizeof(details), "%s Booting...", devicename);
            break;
        case STATUS_SERIAL_INIT:
            snprintf(details, sizeof(details), "Serial2 initialized");
            break;
        case STATUS_BUFFER_READY:
            snprintf(details, sizeof(details), "Serial2 buffer set to %d bytes", JSON_BUFFER_SIZE);
            break;
        case STATUS_WIFI_INIT:
            snprintf(details, sizeof(details), "WiFi initialized");
            break;
        case STATUS_RUNNING:
            snprintf(details, sizeof(details), "System running");
            break;
        case STATUS_WIFI_FAIL:
            snprintf(details, sizeof(details), "WiFi connection failed");
            break;
        default:
            snprintf(details, sizeof(details), "Unknown status %u", state);
            break;
    }
    Serial.println(details);
}


uint64_t prev = 0;
uint8_t phase = 0;
void flash(uint64_t time_now, int heartbeat,
           uint16_t ON_1 = 50, uint16_t OFF_1 = 75,
           uint16_t ON_2 = 50, uint16_t OFF_2 = 1500) {
    uint64_t interval = 0;
    switch (phase) {
        case 0: interval = ON_1; break;   
        case 1: interval = OFF_1; break;  
        case 2: interval = ON_2; break;   
        case 3: interval = OFF_2; break;  
    }
    if ((time_now - prev) >= interval) {
        prev = time_now;
        phase = (phase + 1) % 4;
        bool ledState = (phase == 0 || phase == 2);
        digitalWrite(heartbeat, ledState ? HIGH : LOW);
    }
}

float cabin_temp = 0.0;
bool inner_fan_status = false;

void monitor_internal_temp() {
    Serial.print("MCU TEMP: "); Serial.println(cabin_temp);
    if (cabin_temp >= 33.0 && !inner_fan_status) {
        inner_fan_status = true; digitalWrite(internal_fan, HIGH);
    } else if (cabin_temp <= 30.0 && inner_fan_status) {
        inner_fan_status = false; digitalWrite(internal_fan, LOW);
    }
    Serial.print("Cabin Fan: "); Serial.println(inner_fan_status ? "ON" : "OFF"); Serial.println();
}


void printSystemDiagnostics(uint64_t now_diag_time) {
    static uint64_t lastDiagTime = 0;

    if (now_diag_time - lastDiagTime > 30000) { // Every 30 seconds
        Serial.println("\n=== SYSTEM DIAGNOSTICS ===");
        Serial.printf("Heap Free: %u bytes\n", ESP.getFreeHeap());
        Serial.printf("WiFi Status: %d\n", WiFi.status());
        Serial.printf("OTA STARTED: %s\n", otaStarted ? "YES" : "NO");
        
        // small preview buffer to avoid stack overflow
    char previewBuf[128];

    if (xSemaphoreTake(xDataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            // copy only up to preview length to avoid using tons of stack
            size_t copyLen = strnlen(uploadBuffer, JSON_BUFFER_SIZE);
            Serial.printf("uploadBuffer length: %u\n", (unsigned)copyLen);

            // copy a preview (first N bytes)
            size_t n = (copyLen < sizeof(previewBuf)-1) ? copyLen : (sizeof(previewBuf)-1);
            memcpy(previewBuf, uploadBuffer, n);
            previewBuf[n] = '\0';
            Serial.printf("uploadBuffer starts with: %.120s\n", previewBuf); // safe print
            xSemaphoreGive(xDataMutex);
         }
        Serial.printf("Last cabin temp: %.1f\n", cabin_temp);
        Serial.println("==========================\n");
        lastDiagTime = now_diag_time;
    }
}


bool hasFreshJson = false;
void receive_serial_data(uint64_t call_time) {
    static uint16_t bufferIndex = 0;
    static int braceLevel = 0;

    while (Serial2.available()) {
        receiving_UART = true;
        most_recent_packet_time = call_time;

        char c = Serial2.read();

        if (c == '{') {
            if (braceLevel == 0) bufferIndex = 0;  
            braceLevel++;
        }
        if (braceLevel > 0) {
            if (bufferIndex < JSON_BUFFER_SIZE - 1)
                receiveBuffer[bufferIndex++] = c;
            else {
                Serial.println(" Buffer overflow! JSON too large. Frame dropped.");
                bufferIndex = 0; braceLevel = 0;
                continue;
            }
        }
        if (c == '}' && braceLevel > 0) {
            braceLevel--;
            if (braceLevel == 0 && bufferIndex > 0) {
                receiveBuffer[bufferIndex] = '\0';

                DeserializationError error = deserializeJson(JSON_received, receiveBuffer);
                if (!error) {
                    Serial.printf("✅ JSON received at: %llu\n", (now_now/1000)); 
                  //  Serial.println(receiveBuffer); // print to screen only for diagnostics

                    cabin_temp = JSON_received["Internal_Temperature"] | 0.0;
                    Serial.printf("cabin temp=%.1f\n", cabin_temp);

                    hasFreshJson = true;

                    // --- Mutex-protected copy to uploadBuffer ---
                    if (xSemaphoreTake(xDataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                        strncpy(uploadBuffer, receiveBuffer, JSON_BUFFER_SIZE);
                        uploadBuffer[JSON_BUFFER_SIZE-1] = '\0'; // // Ensure null termination
                        xSemaphoreGive(xDataMutex);
                    }
                } else {
                    Serial.printf("❌ JSON Error: %s\n", error.c_str());
                }
                bufferIndex = 0;
            }
        }
    }
    vTaskDelay(pdMS_TO_TICKS(10));
}




// --- Legacy/Expansion commented code remains below ---
/*
void receive_serial_data(){
    //  while (Serial2.available()) {  // Serial.println("Receiving JSON Data...");
  while (Serial2.available() && Serial2.read() != '}') { // drain until the next {, so you can desync if noise appears.
    char c = Serial2.read();

    // Track opening and closing braces and Handle nested braces correctly if your JSON contains objects inside objects.
    // Start collecting once '{' is seen
        if (c == '{') {
            if (braceLevel == 0) bufferIndex = 0; // reset buffer at start
            braceLevel++;
        }

        if (braceLevel > 0) {
            if (bufferIndex < JSON_BUFFER_SIZE - 1) { // leave space for null terminator
                dataPack[bufferIndex++] = c;
            } else {
                // buffer overflow, reset
                bufferIndex = 0;
                braceLevel = 0;
                Serial.println("Buffer overflow! JSON too large.");
            }
        }

        if (c == '}') {
            braceLevel--;
            if (braceLevel == 0 && bufferIndex > 0) {
                dataPack[bufferIndex] = '\0'; // null terminate

                // Deserialize JSON
                DeserializationError error = deserializeJson(JSON_received, dataPack);
                if (!error) {
                    // Access fields example
                    Serial.println("RECEIVED:");
                    Serial.println(dataPack);
                    
                      const char * SystemDate = JSON_received["Date"];
                      const char * Systemtime = JSON_received["Time"];
                       active_sensors = JSON_received["Active_Sensors"] | 0;
                       average_temp = JSON_received["Average_Temperature"] | 0.00;
                       average_humi = JSON_received["Average_Humidity"] | 0.00;
                       voltage = JSON_received["Voltage"] | 0.00;
                       cabin_temp = JSON_received["Internal_Temp"] | 0.00;

                  
                    Serial.println(); Serial.printf("average_temp=%.1f, average_humi=%.1f\n", average_temp, average_humi); Serial.println();
                } else {
                    Serial.print("JSON Error: ");
                    Serial.println(error.c_str());
                }
            }
        }
    }
}
*/