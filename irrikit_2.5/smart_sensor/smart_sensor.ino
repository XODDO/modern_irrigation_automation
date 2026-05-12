/* XODDOCODE 2026
  * IRRIKIT SOIL SENSING NODE
  * 25 APRIL 2026
*/
#include <Arduino.h>
#include "globals.h" // all other directives and declarations

#define FW_VERSION "v1.3.2-ESP32-SoilNode"


const char SensorID[15] = "Hoima_Node_2"; 
const char sensor_position[10] = "BLK 1";

OTA_Credentials get;
const char* OTA_PASS = get.secure_ota_password;

float soil_moisture_percent = 0.0f;


typedef struct sensorData{ // all stringified
      char sendable_data_bundle[240] = "";
   }  sensorData; 

sensorData senderObj;



// Static JSON document (internal to this compilation unit)
static StaticJsonDocument<240> JSON_data; //<250B the largest payload to be carried at once by espnow

Buzzer buzzer(buzzingPin);   //uint8_t buzzer = 14;     // buzzer pin
Battery batt(batteryPin, single_cell_max_voltage, number_of_cells);         // pin, scaling factor, num batteries
WiFi_Manager wifi_obj(27); // pin 27
sense sensor;


// base class GxEPD2_GFX can be used to pass references or pointers to the display instance as parameter, uses ~1.2k more code
// enable or disable GxEPD2_GFX base class
#define ENABLE_GxEPD2_GFX 0

// 1.54'' EPD Module
   GxEPD2_BW<GxEPD2_154_D67, GxEPD2_154_D67::HEIGHT> LCD(GxEPD2_154_D67(/*CS=5*/ 5, /*DC=*/ 4, /*RES=*/ 13, /*BUSY=*/ 15)); // GDEH0154D67 200x200, SSD1681
// 1.54'' EPD Module (3-Color: B/W/R)
// GxEPD2_3C<GxEPD2_154_D67, GxEPD2_154_D67::HEIGHT> LCD(GxEPD2_154_D67(/*CS=*/5, /*DC=*/4, /*RES=*/13, /*BUSY=*/15)); // GDEW0154Z17 200x200, SSD1681
// ESP32 CS(SS)=5,SCL(SCK)=18,SDA(MOSI)=23,BUSY=15,RES(RST)=2,DC=0

const size_t MIN_STACK_SAFETY_MARGIN = 200;

const uint32_t  SENSING_TASK_STACK_SIZE = 12288; // 12kB
const uint32_t  OTA_TASK_STACK_SIZE  = 4096; // 4kB
const uint32_t  BUZZING_TASK_STACK_SIZE= 4096; // 4kB
const uint32_t  OTA_BTN_STACK_SIZE = 4096;  // 4kB

uint8_t SENSING_TASK_CORE = 0;
uint8_t OTA_SERVICE_TASK_CORE  =  0;
uint8_t OTA_BTN_TASK_CORE = 1;
uint8_t BUZZING_TASK_CORE = 1;

#define BUZZING_TASK_PRIORITY 4 // so that the buzzer is not kept waiting for LCD/send/heavy Serial
#define OTA_TASK_PRIORITY     3
#define OTA_BTN_PRIORITY      2
#define SENSING_TASK_PRIORITY 1


TaskHandle_t buzzingTaskHandle = NULL;
TaskHandle_t otaTaskHandle = NULL;
TaskHandle_t otaBtnHandle = NULL;
TaskHandle_t sensingTaskHandle = NULL;
volatile bool sensingTaskSuspended = false;

SemaphoreHandle_t otaMutex = NULL;
SemaphoreHandle_t xDataMutex;
SemaphoreHandle_t sendSemaphore;  // You mentioned this but didn't create it
SemaphoreHandle_t serialMutex;

void Read_display_send_task(void *pvParams);
void OtaButtonTask(void *pvParameters);
void BuzzingTask(void * pvParams);
void OtaServiceTask(void *pv);

void handle_ota_exit_conditions(uint64_t running_time);
void process_ota_state_transitions(uint64_t running_time);
void sleep_dynamically(uint8_t state_of_power);

void initializeOTA();
void toggle_ota();

bool switch_radio_to_espnow(); 
bool switch_radio_to_wifi();
bool is_radio_hardware_ok();
bool is_radio_off();
void radio_hardware_reset();

#define SEND_TIMEOUT_MS 1500
#define SEND_MAX_RETRIES 3

bool can_send_over_espnow = false;

bool esp_now_initialized = false;

bool ON  = true;
bool OFF = false;

float effective_voltage = 0.0f;
float voltage_before = 0.0f; 
float voltage_during = 0.0f;
float voltage_after  = 0.0f;
float voltage_change = 0.0f;

// In your globals:
float voltage_drop_history[5] = {0.0};
uint8_t drop_index = 0;
float avg_voltage_drop = 0;


moistureData senderObject;

int8_t currentScreen = -1; // negative for very low power

bool battery_low_registered = false;


char soil_moisture_char[8] = "x.x";


char batt_str[12] = ".";
uint8_t batt_level = 0;


uint32_t send_timeout = 0;     uint8_t retry = 0;
uint64_t sends = 0; //uint64_t sends = 0; // 0 TO 4BILLION
char transmissions[12];

bool waiting_for_delivery = false;


bool wifi_connected = false;
uint64_t last_wifi_check = 0;

uint64_t ota_running_time = 0;


void setup(){    // The principle: nothing that draws current should run until confirmed we can afford it.
  
    cpu_freq_set = setCpuFrequencyMhz(40); 
    delay(1000);
    Serial.begin(115200); 
   // Serial.setTimeout(100);
    Serial.flush();   delay(500);     // Clear any garbage in serial buffer

    if (!cpu_freq_set) {
        Serial.println("Warning: Failed to set CPU frequency!");
        delay(1000); 
        setCpuFrequencyMhz(40); 
        Serial.flush();
        delay(1000);
        Serial.begin(115200);   // reinitialize UART after freq change
        // retry in 3 or 5 seconds
    }  
        uint32_t cpu_freq_int = getCpuFrequencyMhz();
        snprintf(CPU_Freq, sizeof(CPU_Freq), "%luMHz", cpu_freq_int);

        Serial.println("\n\n");  // Clear any garbage
        Serial.print(SensorID);  Serial.print(" booting... with frequency: "); Serial.println(CPU_Freq);
        Serial.flush();  delay(200); // Ensure it's sent
       
        Serial.printf("------Firmware %s version.\n", FW_VERSION);

        Serial.printf("ESP32 Arduino Core: %d.%d.%d\n",
                        ESP_ARDUINO_VERSION_MAJOR,
                        ESP_ARDUINO_VERSION_MINOR,
                        ESP_ARDUINO_VERSION_PATCH);
        Serial.flush(); delay(200);

      //  POWER MANAGEMENT   
      batt.begin(); //comes with pinMode(batteryPin, INPUT); and analogReadResolution(12) and analogSetAttenuation(ADC_11db);
      Serial.println("[BATTERY] Performing initial voltage check...");
      MonitorBattery(); //       current_power = get_power_state(batt.getLevel());
      delay(200);
      
      Serial.print("First Battery Voltage: "); Serial.println(batt_str); 
      Serial.print("First Battery Level: "); Serial.print(current_power); Serial.println("(0-3)");
      delay(200);
     // will the Scheduler return to the setup when it finds power critical? or will it just go to sleep?
      // first change things on the screen then call sleep_dynamically ... si kyo?
 
      if(current_power == POWER_CRITICAL) { // if power is <3.6 for a single Li batt, 
        //batt.onLow(beepLowBattery); // the buzzer will consume an empty battery
              Serial.println("!!!SETUP FOUND POWER TOO LOW"); // get to sleep straight away
              
               //only init stuff that consumes extremely low power
              // E-PAPER DISPLAY, THIS WILL PRESENT THE LOW POWER SHUT DOWN!
              LCD.init(115200, true, 50, false); vTaskDelay(pdMS_TO_TICKS(200)); // for the hardware SPI to connect to start transactions
            
              // currentScreen = -1; // update_display();
               // present voltage and low power prompt
              LowPower_Screen();  vTaskDelay(pdMS_TO_TICKS(100)); // Small delay to ensure display driver finishes
              LCD.hibernate(); vTaskDelay(pdMS_TO_TICKS(300));  // Force display to deep sleep (some e-paper drivers keep SPI awake)

              
              Serial.printf("[POWER STATE] %d → entering sleep routine...\n", current_power);

              Serial.flush();  vTaskDelay(pdMS_TO_TICKS(200)); // for serial to flush out garbage

              esp_sleep_enable_timer_wakeup(power_critical_sleep_duration); // (2 * 3600ULL * 1000000ULL); // 2 hours

              esp_deep_sleep_start();

          return; // this line is never reached, but added defensively to quit setup

      } // end point for every time that battery is too low

  else { // battery is sufficient
    Serial.println("!!!SETUP FOUND POWER SUFFICIENT");
    Serial.flush();
    delay(100);
    
    // CRITICAL: Proper frequency change with UART reinit
    Serial.end();  // Stop UART completely
    delay(50);
    
    if (!setCpuFrequencyMhz(80)) {
        // Fallback - stay at 40MHz but warn
        Serial.begin(115200);
        delay(100);
        Serial.println("WARNING: Failed to set 80MHz, staying at 40MHz");
    } else {
        delay(200);  // Let PLL stabilize
        
        // Restart UART at new frequency
        Serial.begin(115200);
        delay(200);
        
        // Verify and print
        uint32_t actual_freq = getCpuFrequencyMhz();
        Serial.printf("✓ CPU Frequency changed to: %luMHz\n", actual_freq);
        Serial.flush();
    }
    
    // Now safe to continue with other initializations
    // Get frequencies
    uint32_t xtal_freq_int = getXtalFrequencyMhz();
    uint32_t apb_freq_int = getApbFrequency() / 1000000;
    
    snprintf(XTAL_Freq, sizeof(XTAL_Freq), "%luMHz XTAL", xtal_freq_int); 
    Serial.println(XTAL_Freq);
    snprintf(APB_Freq, sizeof(APB_Freq), "%luMHz APB", apb_freq_int); 
    Serial.println(APB_Freq);
    
    Serial.println(">>>SETUP FOUND POWER HIGH ENOUGH");
    Serial.flush();
    

    // NOW START SPINNING TASKS AND ACTIVATE GPIOz
       buzzer.begin(); // comes with pinMode(buzzer, OUTPUT);
         
  // CORE TASK 1
      xTaskCreatePinnedToCore(BuzzingTask, "BuzzingCheck", BUZZING_TASK_STACK_SIZE, NULL, BUZZING_TASK_PRIORITY, &buzzingTaskHandle, BUZZING_TASK_CORE); // ATTACH TO CORE 1
      Serial.println("\tBuzzer Task initialized!");
      
      buzzer.beep(1,50,0); // now this can go off in exactly 50ms before the loop comes in

     batt.onLow(beepLowBattery);
     batt.onRecovered(BatteryRecovered);

     xTaskCreatePinnedToCore(OtaButtonTask,  "OTA_Button", OTA_BTN_STACK_SIZE,   NULL,   OTA_BTN_PRIORITY,   &otaBtnHandle,  OTA_BTN_TASK_CORE); // polls every 100ms

     Serial.println("Initializing Sensor");
     sensor.begin(sensorPin, sensorRelay);
     sensor.initialize_indicators(lowPin, mediumPin, highPin);

     Serial.println("Initializing display");
     LCD.init(115200, true, 50, false); vTaskDelay(pdMS_TO_TICKS(1000)); // for the hardware SPI to connect to start transactions


     BootScreen(); LCD.hibernate(); vTaskDelay(pdMS_TO_TICKS(1000));
     Serial.println("Screen Boot Complete!");
      
     Serial.println("Turning ON ESPNOW");

          // Initialize wireless communication with retry logic
      if (!switch_radio_to_espnow()) {
            Serial.println("!!! WARNING: Wireless initialization failed - operating in local mode");
            // Continue without wireless capabilities
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
      
     // if power is sufficient, first sample readings before the first dynamic sleep duration reaches 
        Power_Wifi_Screen();  LCD.hibernate();  // present WiFi states, voltage state, sensor ready
        currentScreen = 0; //update_display();
  
      // spnd a bit more time on the Power_Wifi_Screen before calling the update_display in the CombinedTasks which will just present readings
         vTaskDelay(pdMS_TO_TICKS(2000)); currentScreen = 1;


  /*  
        SemaphoreHandle_t xDataMutex = NULL;
        SemaphoreHandle_t sendSemaphore = NULL;

  */

     // Create mutexes BEFORE creating tasks that will need them
    xDataMutex = xSemaphoreCreateMutex();
    sendSemaphore = xSemaphoreCreateBinary();  // Binary semaphore for send completion
    serialMutex = xSemaphoreCreateMutex(); // for serial protection
    

    if (xDataMutex == NULL || sendSemaphore == NULL) {
        Serial.println("Failed to create mutex/semaphore!");
      //  while(1) { delay(100); }  // Halt
    }



   //  CORE TASK 2
    
    Serial.printf("  - Free heap before task: %u bytes\n", esp_get_free_heap_size());
    Serial.printf("  - Minimum free heap: %u bytes\n", esp_get_minimum_free_heap_size());
     Serial.printf("Heap on entry: free=%u, largest=%u\n",
                  esp_get_free_heap_size(),
                  heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));

    // Create sensing task with handle
    BaseType_t result_task = xTaskCreatePinnedToCore(
                Read_display_send_task, 
                "Scheduled_Stuff", 
                SENSING_TASK_STACK_SIZE,
                NULL, 
                SENSING_TASK_PRIORITY, 
                &sensingTaskHandle,  // for suspending this task as and when needed
                SENSING_TASK_CORE
            );
    
    if (result_task != pdPASS) {
        Serial.printf("ERROR: Failed to create task! Reason: %d\n", result_task);
        
        switch(result_task) {
            case errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY:
                Serial.println("  - Could not allocate required memory");
                Serial.printf("  - Try increasing PACKET_STACK_SIZE from %d\n", SENSING_TASK_STACK_SIZE);
                break;
            default:
                Serial.printf("  - Unknown error: %d\n", result_task);
                break;
        }
        
     //   return false;
    }
    
    Serial.println("  - Task created successfully");
    Serial.printf("  - Free heap after task: %u bytes\n", esp_get_free_heap_size());
    

    delay(2000);

            otaMutex = xSemaphoreCreateMutex(); // Create mutex for OTA coordination


        Serial.println("\n=== Creating OTA Service Task ===");

        Serial.printf("  - Free heap before task: %u bytes\n",
                    esp_get_free_heap_size());

        Serial.printf("  - Minimum free heap: %u bytes\n",
                    esp_get_minimum_free_heap_size());

        Serial.printf("  - Largest free block: %u bytes\n",
                    heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));

        BaseType_t ota_result = xTaskCreatePinnedToCore(
            OtaServiceTask,
            "OTA_Service",
            OTA_TASK_STACK_SIZE,
            NULL,
            OTA_TASK_PRIORITY,
            &otaTaskHandle,
            OTA_SERVICE_TASK_CORE
        );

        if (ota_result != pdPASS)
        {
            Serial.printf("ERROR: Failed to create OTA task! Reason: %d\n",
                        ota_result);

            switch(ota_result)
            {
                case errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY:
                    Serial.println("  - Could not allocate required memory");
                    Serial.printf("  - Try increasing OTA_TASK_STACK_SIZE from %d\n",
                                OTA_TASK_STACK_SIZE);
                    break;

                default:
                    Serial.printf("  - Unknown error: %d\n", ota_result);
                    break;
            }
        }
        else
        {
            Serial.println("  - OTA task created successfully");

            Serial.printf("  - Free heap after task: %u bytes\n",
                        esp_get_free_heap_size());

            Serial.printf("  - Minimum free heap after task: %u bytes\n",
                        esp_get_minimum_free_heap_size());

            Serial.printf("  - Largest free block after task: %u bytes\n",
                        heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
        }

          //That ensures deep sleep never accidentally runs scheduler creation code.
    // SUCCESSFUL End of setup with enuf batt power

    pinMode(ota_button, INPUT_PULLUP);
    buzzer.beep(2, 50, 50); 

    }  // else if(current_power == POWER_LOW || current_power == POWER_MODERATE || current_power == POWER_EXCELLENT)

  // Enable watchdog (timeout after 6mins)
    esp_task_wdt_init(360, true);
    esp_task_wdt_add(NULL);

    Serial.println("End of Setup!");
}



void BuzzingTask(void * pvParams) { 
  uint8_t checkin_frequency = 10; 
  const TickType_t xDelay = pdMS_TO_TICKS(checkin_frequency);
  while (true) {
    buzzer.update();
    vTaskDelay(xDelay); // yield to la skedula
    //esp_task_wdt_reset();  

  }
}

void OtaButtonTask(void *pvParameters){

    bool lastState = HIGH;
    bool longPressHandled = false;

    uint64_t pressStart = 0;

    while(true){
    
        bool state = digitalRead(ota_button);

        uint64_t now = esp_timer_get_time();

        // Button pressed
        if(state == LOW && lastState == HIGH)
        {
            pressStart = now;
            longPressHandled = false;
        }

        // Long press detected (2.2s)
        if(state == LOW &&  !longPressHandled && (now - pressStart >= 2000000ULL)){
        
            longPressHandled = true;
   
            Serial.println("OTA button activated");
            if(!otaModeActive)  toggle_ota(); //  buzzer.beep(2, 500, 500); and  initializeOTA(); happen in toggle ota

           // handle_ota(now, true); //  on true calls toggle_ota() which itself calls initializeOTA();

        }

        // Button released
        if(state == HIGH && lastState == LOW)
        {
            longPressHandled = false;
        }

        lastState = state;

        // sleep 100ms
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void OtaServiceTask(void *pv){
    UBaseType_t ota_watermark =  uxTaskGetStackHighWaterMark(NULL);

    while(true){
    
        if(otaModeActive){
        
            ArduinoOTA.handle();

             ota_running_time = esp_timer_get_time();
             // Manage WiFi connection (every 2 seconds)
            if ((ota_running_time - last_wifi_check) >= (2ULL * 1000ULL)) {
                wifi_connected = wifi_obj.ensure_wifi();
               // process_ota_state_transitions(ota_running_time);

                handle_ota_exit_conditions(ota_running_time);

                last_wifi_check = ota_running_time;
            }

            

            
            // small delay during OTA
            vTaskDelay(pdMS_TO_TICKS(10)); // VERY IMPORTANT:
        }
        else
        {   
            
            Serial.printf("OTA Task Stack High Water: %u bytes\n", ota_watermark * sizeof(StackType_t));

            // just sleep when inactive
            vTaskDelay(pdMS_TO_TICKS(5000));
            
        }
    }
}


//modify sleep modes according to available battery power
void Read_display_send_task(void *pvParams) { // the 4 in one


   // Get initial stack high water mark
    UBaseType_t initial_watermark = uxTaskGetStackHighWaterMark(NULL);
    Serial.printf("[Stack] Initial high water: %u bytes\n", initial_watermark);
    
    uint32_t packets_sent = sends;
    uint32_t last_print = 0;


  const TickType_t return_to_task = pdMS_TO_TICKS(dynamic_interval); // dynamic interval: 3600ULL * 1000ULL / 900ULL * 1000ULL   each 15 mins or each hour when in deep sleep
  TickType_t xLastWake = xTaskGetTickCount();

  while (1) {// or vTaskdelayUntil...
  // Check if OTA is active - if yes, suspend self
        if (otaModeActive) {
            vTaskSuspend(NULL);  // Suspend this task
            // Will resume when OTA is done
            continue;
        }

            esp_task_wdt_reset();  
            Serial.println("Measuring the battery before turining on sensor...");

            MonitorBattery(); // measure voltage before sensor is powered
            voltage_before =  batt.getVoltage();

             // 1. READ SENSORS
             Serial.println("Turning ON sensor");
            sensor.toggle_sensor(ON);
            delay(50); // give the battery time to settle

            Serial.println("Measuring the battery after turning ON sensor...");

            MonitorBattery(); // measure voltage after sensor is powered
            voltage_during = batt.getVoltage();
            
            voltage_change = voltage_before - voltage_during;

            soil_moisture_percent = sensor.sense_reading();
            
            sensor.toggle_sensor(OFF);

            delay(100);
            Serial.println("Measuring the battery after turning OFF sensor...");
            MonitorBattery(); // measure voltage after sensor is powered
            float voltage_after = batt.getVoltage();

            effective_voltage = (voltage_after + voltage_before)/2.0;
            current_power = get_power_state(batt.getLevel());

            Serial.printf("\n[SENSOR POWER CHECK]\nVoltage B4 ON: %.2f\tVoltage After ON:%.2f\tVoltage after Sensor OFF:%.2f,\tVoltage Drop: %.2f\n", voltage_before, voltage_during, voltage_after, voltage_change);

            //buzzer.beep(1,50,0);

            bool serialized = serialize_to_JSON();

           if (!serialized) { 
                    buzzer.alertBeep(); 
                    Serial.println("Serialization Failed!");
            } else {
                    Serial.println("Serialization Done. Now Sending...");
                    // Store timestamp and flag for async delivery check
                    waiting_for_delivery = true;
                     
                     //  random delay to avoid collision with another one transmitting
                    delay(random(10, 200));  // 10-200ms jitter for transmission assurance random delay
    
                    // Send asynchronously
                   // send_d_JSON();

                    send_with_retry();

                }
            
            
            update_display();

            // Check stack usage every 10 packets
            if (packets_sent % 10 == 0) {
                UBaseType_t watermark = uxTaskGetStackHighWaterMark(NULL);
                uint32_t used = SENSING_TASK_STACK_SIZE - watermark;
                uint32_t percent = (used * 100) / SENSING_TASK_STACK_SIZE;
                
                Serial.printf("[Stack] After %lu packets: Used %u/%u bytes (%u%%)\n",
                             packets_sent, used, SENSING_TASK_STACK_SIZE, percent);
                last_print = packets_sent;
                 Serial.printf("SHW => Stack high water mark: %u bytes\n", (watermark * sizeof(StackType_t)));
            
            // If this gets close to 0, increase stack size!
            if(watermark < MIN_STACK_SAFETY_MARGIN) { // Less than 200 bytes remaining
                Serial.println("WARNING: Stack nearly exhausted!");
            }
            // Optional: Print task stats
            Serial.printf("Sensor task ran at %llu, free heap: %dkB\n", now_now_ms, esp_get_free_heap_size()/1024);
    
            }
                         
           
            sleep_dynamically(current_power);   // vTaskDelayUntil(&xLastWake, return_to_task); 
  }

}








bool pressed = false;

// if((esp_timer_get_time() - now_now_ms) > 10000){ // 10 milliseconds

uint64_t timer_check = 0;
uint32_t delay_duration = 300000; //(5*60*1000) // 5 min; 
void loop() { //   everything is handled by the scheduler in TASKS
 
     // BLOCK TASK HERE
        vTaskDelay(pdMS_TO_TICKS(delay_duration)); // BLOCK FOR 10SECS OR 5MINS
    
        now_now_ms = esp_timer_get_time();

        esp_task_wdt_reset();  // Reset watchdog timer
        update_time();
         // get_simulated_moisture(); // remove before deployment

    //OTA 
       // pressed = read_button(ota_button, now_now_ms);

      //  handle_ota(now_now_ms, pressed); // when activated, disable the task
       
}



                    /*

                    // Only save to flash every 100 transmissions
                    // or else if per send, 10 years × 365 days × 24 hours × 180 transmissions/hour = 15,768,000 writes
                    if (sends - last_save_counter >= 1000) { // = 15,768,000 transmissions/10 years
                        save_sends_count();
                        last_save_counter = sends;
                        Serial.printf("Saved sends counter: %u\n", sends);
                    }
                    */


void toggle_ota() {
    // Enter OTA mode - COMPLETELY stop the sensing task
    if (otaMutex != NULL) {
        xSemaphoreTake(otaMutex, portMAX_DELAY);
    }
        
    // 1. Stop the sensing task
    if (sensingTaskHandle != NULL) {
        vTaskSuspend(sensingTaskHandle);
        sensingTaskSuspended = true;
        Serial.println("Sensing task suspended for OTA");
    }
    
     Serial.println("\n=== OTA Triggered ===");
     buzzer.beep(1, 300, 0);
    
    esp_err_t result = esp_now_deinit();
    if (result != ESP_OK) {
        Serial.printf("ESP-NOW deinit failed: %d\n", result);
        return;
    }
    
    delay(300);
    
    if (!switch_radio_to_wifi()) {
        Serial.println("WiFi failed. Returning to ESP-NOW.");
        switch_radio_to_espnow(); 
        return;
    }
    
    initializeOTA();
    
    
    
    if (otaMutex != NULL) {
        xSemaphoreGive(otaMutex);
    }
    
    // 4. Set OTA start time
     if (!otaModeActive) otaModeActive = true;
    otaStartTime = esp_timer_get_time() / 1000ULL;
    
    Serial.printf("OTA Mode Active for %llu minutes\n", (OTA_TIMEOUT / 60000));
 
    
    Serial.println("\n=== OTA Mode Activated ===");
    buzzer.beep(1, 1000, 0); // now active
    
    // Update display to show OTA screen
    currentScreen = 10;
    update_display();
}


void process_ota_state_transitions(uint64_t running_time) {
    // Handle OTA started state
    if (otaStarted) {
        strcpy(ota_log, "Starting OTA update...");
        buzzer.beep(2, 300, 200);
        ota_start_time = running_time;
        otaStarted = false;
        current_ota_state = OTA_STATE_STARTING;
    }
    
    // Handle OTA progress state
    if (otaProgress > 0 && otaProgress < 100) {
        snprintf(ota_log, sizeof(ota_log), "Updating... %d%%", otaProgress);
        current_ota_state = OTA_STATE_PROGRESS;
    }
    
    // Handle OTA finished state
    if (otaFinished) {
        strcpy(ota_log, "Update complete!");
        buzzer.beep(2, 500, 200);
        otaFinished = false;
        current_ota_state = OTA_STATE_SUCCESS;
        ota_success_time = running_time;
    }
    
    // Handle OTA error state
    if (otaError) {
        strcpy(ota_log, "Update failed!");
        buzzer.beep(1, 500, 500);
        otaError = false;
        current_ota_state = OTA_STATE_ERROR;
        ota_error_time = running_time;
    }
}

void handle_ota_exit_conditions(uint64_t running_time) {
    // Exit on timeout
    if (current_ota_state == OTA_STATE_TIMEOUT && 
        (running_time - ota_timeout_time) >= 5000) {
        exit_ota_mode();
        switch_radio_to_espnow();
    }
    
    // Exit on success
    if (current_ota_state == OTA_STATE_SUCCESS && 
        (running_time - ota_success_time) >= 5000) {
        exit_ota_mode();
        switch_radio_to_espnow();
    }
    
    // Exit on error
    if (current_ota_state == OTA_STATE_ERROR && 
        (running_time - ota_error_time) >= 5000) {
        exit_ota_mode();
        switch_radio_to_espnow();
    }
}

void exit_ota_mode() {
    Serial.println("Exiting OTA mode, resuming normal operation");
    
    // Stop OTA
    ArduinoOTA.end();
 
    // Resume sensing task
    if (sensingTaskSuspended && sensingTaskHandle != NULL) {
        vTaskResume(sensingTaskHandle);
        sensingTaskSuspended = false;
    }
    
    // Reset OTA state
    otaModeActive = false;
    current_ota_state = OTA_STATE_IDLE;
    otaProgress = 0;
}



// Old (before ESP-IDF v5)
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status){

        // New (ESP-IDF v5+)
      if (status == ESP_NOW_SEND_SUCCESS) {
        successfully_delivered = true;
        sends++;
        
        // Convert sends to string if needed elsewhere
        ltoa(sends, transmissions, 10);
        
        char macStr[18];
        snprintf(macStr, sizeof(macStr),
                "%02X:%02X:%02X:%02X:%02X:%02X",
                mac_addr[0], mac_addr[1], mac_addr[2],
                mac_addr[3], mac_addr[4], mac_addr[5]);
        
        Serial.printf("✅ Delivered to %s (total: %d)\n", macStr, sends);
    } else {
        successfully_delivered = false;
        Serial.println("❌ Delivery failed");
        // When a critical error occurs:
        //if (fatal_error) {
       // buzzer.windowsErrorBeep();  // Plays the complex tone sequence
        //}
    }
}


//SemaphoreHandle_t xDataMutex = NULL;
volatile bool last_send_success = false;
// Sender with retry logic
bool send_with_retry() {

    for (retry = 0; retry < 3; retry++) {
        if (send_d_JSON()) {
            // Wait for acknowledgment callback
              send_timeout = (millis() + 100);
            while (!successfully_delivered && millis() < send_timeout) {
                delay(1); // hang in here for 100 millisecs
            }
            if (successfully_delivered) return true;
        }
        delay(random(50, 200)); // Exponential backoff
    }
    if(retry>=2)         buzzer.errorBeep(); // Error beep on delivery failure
    retry = 0;
    return false;
}


// Helper function to round to 2 decimal places
float round2(float value) {
    return roundf(value * 100.0f) / 100.0f;
}

bool serialize_to_JSON() {
    JSON_data.clear();
    
    // Populate ALL fields FIRST
    JSON_data["S_ID"] = SensorID;
    JSON_data["POS"] = sensor_position;
    JSON_data["READS"] = sensor.SAMPLE_COUNT;
    JSON_data["DUR"] =   sensor.readings_duration;
   // JSON_data["MEAN"] = sensor.mean;
   // JSON_data["VAR"] = sensor.variance; //to be computed by receiver
    JSON_data["DEV"] = sensor.stddev; 
    JSON_data["MEDIAN"] = sensor.median;
    JSON_data["RANGE"] = sensor.range;
    JSON_data["MOI"] = sensor.soilPercent;

    JSON_data["VOLT"] = batt.getVoltage(); // float e.g 4.21
    JSON_data["VOLT_DIFF"] = voltage_change;
    JSON_data["VOLT_AFT"] = voltage_after;
    JSON_data["PWR_LVL"] = batt.getLevel(); // uint8_t 1 or 2 or 3
    JSON_data["UpT"] = now_now_ms; // uint64_t
    
    // Optional constants (add if needed by receiver)
    JSON_data["WILT"] = sensor.wiltingPoint;
    JSON_data["FC"] = sensor.fieldCapacity;
    JSON_data["CPU"] = CPU_Freq; //     char CPU_Freq[12]; "240 MHz"
    
    // Calculate size BEFORE adding PKT
    size_t estimated_size = measureJson(JSON_data);  // Better than serializing first
    
    if (estimated_size >= sizeof(senderObj.sendable_data_bundle)) {
        Serial.printf("Buffer overflow! Need %zu bytes, have %u\n", 
                      estimated_size, sizeof(senderObj.sendable_data_bundle));
        return false;
    }
    
    // Now add PKT field
    //JSON_data["PKT"] = estimated_size;  // Or final size after adding PKT
    
    // Single serialization
    size_t written = serializeJson(JSON_data, senderObj.sendable_data_bundle);
    
    if (written == 0) {
        return false;
    }
    
    Serial.printf("JSON size: %zu bytes\n", written);
    
    can_send_over_espnow = true;
    return true;
}

    // SEND TO 2 RECEIVERS
// SEND TO 2 RECEIVERS
bool send_d_JSON() { // bundle of JOY ... limit to ~ 150bytes and AVOID PARSING NULLz or NANz
    
    // Validate mutex exists
    if (xDataMutex == NULL) {
        Serial.println("Data Mutex not initialized!");
        return false;
    }
    
    // Take mutex to protect reading the JSON buffer
    if (xSemaphoreTake(xDataMutex, pdMS_TO_TICKS(500)) != pdTRUE) {
        Serial.println("send_d_JSON: could not take data mutex");
        return false;
    }
    
    // CRITICAL FIX: Get the JSON string length (NOT sizeof struct!)
    size_t payload_len = strlen(senderObj.sendable_data_bundle);
    
    // Safety check
    if (payload_len == 0) {
        Serial.println("send_d_JSON: Empty JSON buffer!");
        xSemaphoreGive(xDataMutex);
        return false;
    }
    
    if (payload_len > 247) {
        Serial.printf("send_d_JSON: Payload too large: %zu bytes!\n", payload_len);
        xSemaphoreGive(xDataMutex);
        return false;
    }
    
    // Copy to local buffer while protected (optional but safer)
    char local_buf[250];
    memcpy(local_buf, senderObj.sendable_data_bundle, payload_len + 1);
    
    // Release mutex BEFORE sending (send doesn't need protection)
    xSemaphoreGive(xDataMutex);
    
    // Send to both receivers - USING JSON STRING, NOT STRUCT!
    esp_err_t result1 = esp_now_send(System.peer.broadcastAddress, 
                                     (uint8_t*)local_buf, 
                                     payload_len);
    /*
    delay(10);  // Small gap between sends
    
    esp_err_t result2 = esp_now_send(System.peer.broadcastAddress2, 
                                     (uint8_t*)local_buf, 
                                     payload_len);
    */
    
    if (result1 == ESP_OK) {
        Serial.print("\t");
        Serial.print(local_buf);
        Serial.println(" Sent (awaiting delivery confirmation)!");
        return true;
    } else {
        Serial.println("Send Failed!");
        //buzzer.errorBeep();
        return false;
    }
}


// Enhanced flash function with pattern reset capability
void flash(uint64_t flash_time, uint8_t heartbeat,
           uint16_t ON_1, uint16_t OFF_1,
           uint16_t ON_2, uint16_t OFF_2) {
    
    static uint64_t previe = 0;
    static uint8_t phase = 0;
    static uint16_t last_pattern[4] = {0};
    
    // Check if pattern changed (for reset_blink_phase to work)
    uint16_t current_pattern[4] = {ON_1, OFF_1, ON_2, OFF_2};
    if (memcmp(last_pattern, current_pattern, sizeof(current_pattern)) != 0) {
        phase = 0;
        previe = flash_time;
        memcpy(last_pattern, current_pattern, sizeof(current_pattern));
    }
    
    uint64_t interval = 0;
    switch (phase) {
        case 0: interval = ON_1; break;
        case 1: interval = OFF_1; break;
        case 2: interval = ON_2; break;
        case 3: interval = OFF_2; break;
    }

    if ((flash_time - previe) >= interval) {
        previe = flash_time;
        phase = (phase + 1) % 4;
        bool ledState = (phase == 0 || phase == 2);
        digitalWrite(heartbeat, ledState ? HIGH : LOW);
    }
}

// Helper to reset blink pattern
void reset_blink_phase(void) {
    // This will be detected in flash() by pattern change
    // Just need to call this before changing patterns
}




void read_ota_button(){
    //read button state
    //toggle ota mode if button low for >2 secs

}

struct ButtonState {
    uint64_t pressStart;
    bool lastState;
    bool longPressHandled;
    uint64_t lastDebounceTime;  // Add debounce
    bool lastStableState;
};

/*
// In read_button():
bool state = digitalRead(pin);
uint64_t now = time_now;

// Debounce: wait 50ms for stable state
if (now - btn->lastDebounceTime > 50) {
    if (state != btn->lastStableState) {
        btn->lastDebounceTime = now;
        btn->lastStableState = state;
    }
    state = btn->lastStableState;
}
*/

// Map to store state for each button pin (up to 10 buttons)
static ButtonState buttonStates[10] = {0};
static uint8_t buttonPins[10] = {0};
static uint8_t buttonCount = 0;

// Helper to find or create button state index
int getButtonIndex(uint8_t pin) {
    // Look for existing pin
    for (int i = 0; i < buttonCount; i++) {
        if (buttonPins[i] == pin) return i;
    }
    
    // Add new pin if space available
    if (buttonCount < 10) {
        buttonPins[buttonCount] = pin;
        buttonStates[buttonCount].lastState = HIGH;
        buttonStates[buttonCount].pressStart = 0;
        buttonStates[buttonCount].longPressHandled = false;
        return buttonCount++;
    }
    
    return -1; // No space
}

bool read_button(uint8_t pin, uint64_t time_now) {
    int index = getButtonIndex(pin);
    if (index == -1) return false; // Too many buttons
    
    ButtonState* btn = &buttonStates[index];
    
    bool pressed = false;
    bool state = digitalRead(pin);   // LOW = pressed (active LOW)

    // Button pressed (HIGH → LOW transition)
    if (state == LOW && btn->lastState == HIGH) {
        btn->pressStart = time_now;
        btn->longPressHandled = false;
    }

    // Long press detected (2.2 seconds)
    if (state == LOW &&
        !btn->longPressHandled &&
        (time_now - btn->pressStart >= 2200ULL))
    {
        pressed = true;
        btn->longPressHandled = true;
        buzzer.beep(1, 50, 0);
        
        // Optional: Add pin-specific debug
        Serial.print("Button on pin ");
        Serial.print(pin);
        Serial.println(" activated!");
    }

    // Button released (LOW → HIGH transition)
    if (state == HIGH && btn->lastState == LOW) {
        btn->longPressHandled = false;
    }

    btn->lastState = state;
    return pressed;
}

#define ESPNOW_MAX_RETRIES 3


void update_time(){
        uint64_t seconds = now_now_ms / 1000000ULL;
         hours   += (seconds / 3600) % 24;
        uint8_t minutes = (seconds % 3600) / 60;
        uint8_t secs    = 1+(seconds % 60);

        sprintf(time_str, "%02u:%02u:%02u", hours, minutes, secs);
        Serial.print("Uptime Check: ");Serial.println(time_str);
}

void initializeOTA() {
    ArduinoOTA.setHostname(SensorID);
    ArduinoOTA.setPassword(OTA_PASS);

    ArduinoOTA
        .onStart([]() { otaStarted = true; otaProgress = false;}) 
        .onEnd([]() { otaFinished = true; otaProgress = false; }) 
        .onProgress([](unsigned int progress, unsigned int total) { otaProgress = true;
           // snprintf(ota_log, sizeof(ota_log), "Progress: %u%%", (progress * 100) / total);
           // flash(now_now_ms, blinker, 50, 50, 0, 0);
        })
        .onError([](ota_error_t error) { otaError = true; otaProgress = false;
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
    //webLog("OTA Ready!");
}


bool switch_radio_to_wifi(){
  
    // Clean shutdown of PacketHandler
   
    esp_now_register_send_cb(NULL);

     if (esp_now_deinit() == ESP_OK) {
        Serial.println("  - ESP-NOW deinitialized");
        delay(500);
    } else {
        Serial.println("  - ESP-NOW deinit warning");
    }
    

  
  WiFi.disconnect(true, true); // Full cleanup
  delay(WIFI_RECONNECT_DELAY_MS);
  WiFi.mode(WIFI_OFF);
  delay(WIFI_RECONNECT_DELAY_MS);
  
  // Set WiFi mode before initialization
  WiFi.mode(WIFI_STA);

 //  snprintf(wifi_log, sizeof(wifi_log), " WiFi channel: %d\n", WiFi.channel());

  //Serial.println(wifi_log);
    unsigned long startTime = now_now_ms;

  // Initialize WiFi with timeout protection
  bool wifiSuccess = false;

  /*              WiFi.begin(System.OverTheAir.ssid, System.OverTheAir.password);
  */
 
    while (now_now_ms - startTime < WIFI_SWITCH_TIMEOUT_MS) {
        if (wifi_obj.initialize_ESP_WiFi(SensorID) == WIFI_MGR_SUCCESS) {
            wifiSuccess = true;
            break;
        }
    
    Serial.printf("⏳ WiFi connection failed, retrying... (%lu ms elapsed)\n", 
                 now_now_ms - startTime);
    delay(1000);
    
    Serial.printf("⏳ WiFi connection failed, retrying... (%lu ms elapsed)\n", 
                 now_now_ms - startTime);
    delay(1000);
  }
  
  if (!wifiSuccess) {
    Serial.println("💥 Failed to initialize WiFi within timeout period");
    
    // Fallback attempt - try with different approach
    Serial.println("🔄 Attempting fallback WiFi initialization...");
    WiFi.begin(); // Try simple connection
    
    /*
     WiFi.mode(WIFI_AP);
     WiFi.softAP("irri_Node_OTA", "update123");

    */
    unsigned long fallbackStart = now_now_ms;
    while ((WiFi.status() != WL_CONNECTED) && ((now_now_ms - fallbackStart) < 5000)) {
      delay(500);
      Serial.print(".");
    }
    
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("\n✅ Fallback WiFi connection successful");
      wifiSuccess = true;
    } else {
      Serial.println("\n💥 Fallback WiFi also failed");
      return false;
    }
  }
  
  // Initialize OTA if WiFi is successful
  if (wifiSuccess) {
    Serial.println("🔌 Initialize OTA updates as well, just in case");
    initializeOTA();
    
    // Verify OTA is ready
    Serial.printf("🎯 WiFi Mode Activated - IP: %s, RSSI: %d dBm\n", 
                 WiFi.localIP().toString().c_str(), WiFi.RSSI());
    return true;
  }
  
  return false;
}


bool switch_radio_to_espnow(){

   // 1. Always start from a clean state
    esp_now_deinit();        // safe if not initialized
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    delay(100);

    WiFi.mode(WIFI_STA);
    delay(100);              // let the radio settle in STA mode

    /*
    // First, check if radio is still functional
    if(!is_radio_hardware_ok()) {
        Serial.println("Radio hardware unresponsive - forcing full reset");
        WiFi.persistent(false);  // Don't save to NVS
        WiFi.mode(WIFI_OFF);
        delay(500);
        // Hardware reset through ESP32 register
        REG_SET_BIT(0x3FF00088, BIT(0)); // Reset WiFi MAC
        delay(100);
        REG_CLR_BIT(0x3FF00088, BIT(0));
        delay(500);
    }

    */

    /*
     static bool last_state = false; // check if radio was still on during sleep.
    
    if (last_state == can_send_over_espnow && can_send_over_espnow) {
        return true;  // Already initialized
    }
    */


        // Explicitly start WiFi
    if(esp_wifi_start() != ESP_OK) {
        Serial.println("Failed to start WiFi");
       // last_state = false;
        return false;
    }
    /*
    // Wait for WiFi to be ready
    int attempts = 0;
    while(WiFi.status() == WL_NO_SHIELD && attempts++ < 20) {
        delay(50);
    }
    */
    
    
    // ESP-NOW initialization with retry logic
    for (uint8_t attempt = 1; attempt <= ESPNOW_MAX_RETRIES; attempt++) {
        if (esp_now_init() == ESP_OK) { esp_now_initialized = true;
            Serial.printf("✓ ESP-NOW initialized successfully (attempt %d/%d)\n", attempt, ESPNOW_MAX_RETRIES);
            
            // Initialize peerInfo first
            memcpy(peerInfo.peer_addr, System.peer.broadcastAddress, 6);
            peerInfo.channel = 0;
            peerInfo.encrypt = false;

            // Now try to delete (in case it exists from previous session)
            esp_now_del_peer(peerInfo.peer_addr);  // Safe even if not present

            // Then add
            if (esp_now_add_peer(&peerInfo) == ESP_OK) {
                Serial.println("✓ Peer registered successfully");
                can_send_over_espnow = true;
                return true;
            }
            
            esp_now_register_send_cb(OnDataSent);
            

            /*
                memcpy(peerInfo.peer_addr, System.peer.broadcastAddress2, 6);
                peerInfo.channel = 0;
                peerInfo.encrypt = false;
            
            */
            
            if (esp_now_add_peer(&peerInfo) == ESP_OK) {
                Serial.println("✓ Peer registered successfully");
                can_send_over_espnow = true;
               // last_state = true;
                return true;
            } 
            else { can_send_over_espnow = false;
                     Serial.println("!!! Peer registration failed");
            }

        } else {
           can_send_over_espnow = false;
           // print that Wireless Radio is faulty
            //return;
        }
        //if it reaches here, it has failed to init
         // 3. Deinit before retrying — critical
        esp_now_deinit();
        delay(100 * attempt); // Exponential backoff
    }
    
    Serial.println("!!! ESP-NOW initialization failed after retries");
   // last_state = false;
    return false;
}

uint8_t radio_failure_count = 0;

void toggle_radio(bool state){

    if(state == false){
        Serial.println("=== RADIO DE-INITIALIZATION SEQUENCE ===");

              // First, remove all peers before deinit
       // 1. Stop all radio operations
      //  btStop();
        can_send_over_espnow = false;
        
        // 2. Proper ESP-NOW cleanup
       // if(esp_now_initialized) {  //esp_now_is_init()
           esp_now_initialized  = false;
            
            // Remove all peers
            // Correct way to iterate through peers:
        esp_now_peer_num_t peer_num;
        esp_now_get_peer_num(&peer_num);

        if(peer_num.total_num > 0) {
            Serial.printf("Removing %d peers\n", peer_num.total_num);
            
            // ESP-NOW doesn't have a direct iteration API
            // Either delete specific known peer or let deinit handle it
            if(esp_now_del_peer(System.peer.broadcastAddress) != ESP_OK) {
                Serial.println("Failed to remove peer (may not exist)");
            }
        }
      delay(200);

        // Unregister callbacks
        esp_now_register_send_cb(NULL);
        esp_now_register_recv_cb(NULL);
        
        delay(100); // Allow callbacks to complete
        
        // Now deinit
        esp_err_t e = esp_now_deinit();
        Serial.printf("esp_now_deinit() -> %d\n", e);
              //  // --- 2. Networking teardown (WiFi / ESP-NOW) ---

              // --- 1. Graceful shutdown sequence ---
           //   btStop(); ... aint no bluetooth here

            
          
                    can_send_over_espnow = false;
                    Serial.println("ESP-NOW powered OFF");

                 // stop WiFi fully
                  WiFi.disconnect(true); delay(50);
                if (esp_wifi_stop() == ESP_OK) Serial.println("esp_wifi_stopped successfully");
                else {
                        Serial.println("WiFi stop failed - forcing reset");
                        esp_wifi_restore();  // Force hardware reset
                     }
                    delay(100);

                if (esp_wifi_deinit() == ESP_OK) Serial.println("esp_wifi_deinit successfully");
               
                WiFi.mode(WIFI_OFF);   // Critical: allow hardware to settle

                

                            // 7. Verify state
                if(WiFi.getMode() != WIFI_OFF) {
                    Serial.println("WARNING: WiFi still active");
                }
                else Serial.println("WiFi Powered down successfully!");
                     Serial.println("Radio shutdown complete");
                     radio_failure_count = 0;
                      //esp_wifi_stop(); alone can leave some chip subsystems powered
       // }
       
    }

    else {  delay(200);
            Serial.println("=== RADIO INITIALIZATION SEQUENCE ===");

             // Check if we're waking from deep sleep (needs extra time)
                esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
                if(cause == ESP_SLEEP_WAKEUP_TIMER) {
                    // Coming from deep sleep - power was completely off
                    delay(200);  // Allow 32kHz crystal and RF PLL to stabilize
                }
                
                // Reset radio subsystem if it's in bad state
                if(radio_failure_count > 2) {
                    Serial.println("Multiple radio failures - performing hardware reset");
                    radio_hardware_reset();
                    delay(500);
                    radio_failure_count = 0;
                }
        

             // Restart radio for ESP-NOW or OTA
             bool switched_on = switch_radio_to_espnow();

              if (!switched_on) {             radio_failure_count++;
                  Serial.println("Wireless reinit failed after wake.");

                    if(radio_failure_count >= 3) {
                        // Severe failure - schedule watchdog reset
                        Serial.println("CRITICAL: Radio failure - resetting in 5 seconds");
                        delay(5000);
                        ESP.restart();
                    }

              } else {
                 // esp_now_register_send_cb(OnDataSent); // if not called by the switch radio function
                  Serial.println("Wireless reinit success.");
                   radio_failure_count = 0;
              }
    }
      
}



bool is_radio_off() {
    return (!can_send_over_espnow && WiFi.getMode() == WIFI_OFF);
}


void radio_hardware_reset() {
    esp_wifi_stop();
    esp_wifi_deinit();
    delay(100);
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
}


void shutdown_radio_with_confirmation() {
    toggle_radio(OFF);
    unsigned long start = millis();
    while (!is_radio_off() && (millis() - start) < 1000) {
        delay(10);
    }
    if (!is_radio_off()) Serial.println("WARNING: Radio failed to power down");
}

/*
bool is_radio_hardware_ok() {
    // Check if WiFi/BT are in a known good state
    wifi_mode_t mode = WiFi.getMode();
    
    if(mode == WIFI_MODE_NULL || mode == WIFI_OFF) {
        // Need full reinit
        return false;
    }
    
    // Try a simple register read to verify hardware
    uint32_t mac_low = READ_PERI_REG(0x3FF0008C); // MAC address low register
    if(mac_low == 0xFFFFFFFF || mac_low == 0) {
        return false; // Hardware not responding
    }
    
    return true;
}
*/

void beepLowBattery() {
  if(!battery_low_registered) {
      Serial.println("Battery Critically low");
      //buzzer.beep(2, 800, 500);
      battery_low_registered = true;
  }
}

void BatteryRecovered() {
  if(battery_low_registered) {
      Serial.println("Battery recovered above 3.7");
      battery_low_registered = false;
  }
}


void MonitorBattery() {
     batt.update(); // get latest reading
     effective_voltage = batt.getVoltage();
    // plot 0,1,2,3 >> CRITICAL, LOW, MODERATE, EXCELLENT
      current_power = get_power_state(batt.getLevel());
      snprintf(batt_str, sizeof(batt_str), "%.1fV", effective_voltage); 
}

void compute_sleep_parameters(uint64_t &duration_us, bool &use_deep_sleep) {

    switch (current_power) {

        case POWER_CRITICAL:
            duration_us = power_critical_sleep_duration;
            use_deep_sleep = true;
            break;

        case POWER_LOW:
            duration_us = power_low_sleep_duration;
            use_deep_sleep = true;   // 🔥 change this
            break;

        case POWER_MODERATE:
            duration_us = power_mod_sleep_duration;
            use_deep_sleep = false;   // 🔥 change this
            break;

        case POWER_EXCELLENT:
            duration_us = power_excellent_delay * 1000;
            use_deep_sleep = false;  // light sleep here
            break;
    }
}

char SleepPrompt[64];
char Power_MD[128];

void sleep_dynamically(uint8_t state_of_power) {
    Serial.println("SLEEP MODULATOR");
    /*    // Map power state → sleep mode
    
    switch(current_power) {
        case POWER_CRITICAL:  Serial.println("Discovery: CRITICAL POWER");  break;
        case POWER_LOW:       Serial.println("Discovery: LOW POWER");  break;
        case POWER_MODERATE:  Serial.println("Discovery: MODERATE");   break;
        case POWER_EXCELLENT: Serial.println("Discovery: EXCELLENT");  break;
    }

    if (current_power == POWER_LOW || current_power == POWER_CRITICAL) {
        Serial.println("Battery LOW (with hysteresis) → forcing Wireless OFF, in case was ON");
         // Disable OTA when battery is critical
         if(wifi_connected) {
            WiFi.mode(WIFI_STA);
            WiFi.disconnect();
         }
         else Serial.println("Wireless Radio wasn't ON. continuing with ultralow power ish");
      //  current_power = POWER_CRITICAL;
    }
 */
    switch (state_of_power) {
        case POWER_CRITICAL: { // 2 hours
              dynamic_interval = power_critical_sleep_duration; // this comes in microseconds
            
              Serial.println("\n=== NOW IN CRITICAL POWER MODE ===");
              snprintf(Power_MD, sizeof(Power_MD), "Sampling Interval: %.1f sec (%.1f min)\n", 
                            dynamic_interval / 1e6, dynamic_interval / 6e7);
              Serial.println("Turning OFF Radio, Sensor etc...");

              // Stop high-current peripherals
              toggle_radio(OFF);
              sensor.toggle_sensor(OFF); // adc_power_off();  // (commented because ESP32-S3 deprecated this)
              sensor.flash_indicators(OFF);
              
              //currentScreen = -1;  // update_display(); for UI
              LowPower_Screen(); // mid operation shut down
              LCD.hibernate();
              delay(100); // ensure SPI transactions complete
              
            
                     // Print debug info: reset reason and wake cause
                esp_reset_reason_t rr = esp_reset_reason();
                esp_sleep_wakeup_cause_t wck = esp_sleep_get_wakeup_cause();

                  snprintf(SleepPrompt, sizeof(SleepPrompt), "RST REASON: %d\t WAKE CAUSE: %d\n", rr, wck);

                  // --- 5. Configure wake sources cleanly ---
                  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);

                  // Enable single intentional wake source: timer (and optional ext0 for solarPin HIGH)
                  esp_sleep_enable_timer_wakeup(dynamic_interval);
                  // ensure solarPin is RTC-capable to also allow solarPin to wake when it goes HIGH:
                  // esp_sleep_enable_ext0_wakeup((gpio_num_t)solarPin, 1); // wake on HIGH

                    Serial.println(SleepPrompt);
                    Serial.println(Power_MD);
                    Serial.println("\n\n→ Entering deep sleep now...\tExpected wake in ~2 hours.");
                                        
                    Serial.flush();
                    vTaskDelay(pdMS_TO_TICKS(200)); // TIME FOR SERIAL TO FLUSH
                    buzzer.stop();  // Ensure no tone active
              
              // --- 3. Enter deep sleep ---
                    esp_deep_sleep_start();

                    // If we reach here, deep sleep failed
                    Serial.println("CRITICAL: deep sleep failed - entering emergency light sleep");
                    esp_light_sleep_start();  // Fallback


                           // Code never reaches here but break the case nonetheless
                            break;
        }

        case POWER_LOW: { // 15 mins
             dynamic_interval = power_low_sleep_duration;
             snprintf(Power_MD, sizeof(Power_MD), "\nEntering LIGHT SLEEP Sampling Interval = %.1f mins .\n", dynamic_interval / (60*1e6));
              
             Serial.println(Power_MD);
             Serial.printf("\nNext Wake: %.1f minutes\n", (float)(dynamic_interval / 60000000.0));

             toggle_radio(OFF);
             delay(500);
             Serial.println("\n\n→ Entering light sleep now...\tExpected wake in ~15mins.");
             buzzer.stop();  // Ensure no tone active
             
             esp_sleep_enable_timer_wakeup(dynamic_interval);
              
             esp_light_sleep_start();

              // Upon waking
               esp_sleep_wakeup_cause_t wck = esp_sleep_get_wakeup_cause();
               snprintf(SleepPrompt, sizeof(SleepPrompt), "WAKE UP CAUSE: %d\n", wck);
               Serial.println(SleepPrompt);
               delay(500); // Critical: Wait for power rails to stabilize // Minimum 100ms for 32kHz crystal
               toggle_radio(ON);
              
          break;
        }

        
        case POWER_MODERATE: { // 5mins
            dynamic_interval = power_mod_sleep_duration;
              snprintf(Power_MD, sizeof(Power_MD), "\nEntering LIGHT SLEEP Sampling Interval = %.1f mins.\n", dynamic_interval / (60*1e6));
              Serial.println(Power_MD);

              Serial.printf("Next Wake: %.1f minutes\n", (float)(dynamic_interval / 60000000.0));
              toggle_radio(OFF);
              delay(500);
              buzzer.stop();  // Ensure no tone active
              Serial.println("\n\n→ Entering light sleep now...\tExpected wake in ~5mins.");
             

              esp_sleep_enable_timer_wakeup(dynamic_interval);
              esp_light_sleep_start();

              // Upon waking
                esp_sleep_wakeup_cause_t wck = esp_sleep_get_wakeup_cause();
                snprintf(SleepPrompt, sizeof(SleepPrompt), "WAKE UP CAUSE: %d\n", wck);
                Serial.println(SleepPrompt);
                delay(500);
                toggle_radio(ON);

              
            break;
      }

        case POWER_EXCELLENT: { // 10 seconds
                                // Keep all systems awake

                  dynamic_interval = power_excellent_delay; // in milliseconds

                  Serial.println("\n[POWER MODE] EXCELLENT — Full operational mode");
                  Serial.printf("Sampling Interval = %.0f seconds (%.1f minutes)\n", 
                                (float)dynamic_interval / 1000.0f, 
                                (float)dynamic_interval / 60000.0f);

                 
                  // Optionally check radio state to be sure it is ON
                  if (!can_send_over_espnow) { toggle_radio(ON); can_send_over_espnow = true; }

                  snprintf(SleepPrompt, sizeof(SleepPrompt),"Woke up — allowing WiFi/ESP-NOW to reinit...");
                     

                  Serial.printf("[WiFi Mode] %d | [ESPNOW Ready] %s\n", WiFi.getMode(), can_send_over_espnow ? "YES" : "NO");
                 
                  // Wait before next cycle, ensuring no drift
                  vTaskDelay(pdMS_TO_TICKS(dynamic_interval));  // or use vTaskDelayUntil() for precise timing
                 
                  break;
             }

    }
}

// wifi ota and espnow co-existence
 /*

                  Serial.println("All peripherals active → continuous sampling and OTA ready.");

                  // Ensure WiFi is active for OTA and ESP-NOW
                  if (WiFi.getMode() != WIFI_AP_STA && WiFi.getMode() != WIFI_STA) {
                      Serial.println("WiFi inactive → restarting WiFi for OTA + ESP-NOW");
                      WiFi.mode(WIFI_AP_STA);
                      WiFi.begin(System.OverTheAir.ssid, System.OverTheAir.password);
                      delay(100);
                  }

                  // Handle OTA updates periodically (safe non-blocking)
                  if (WiFi.status() == WL_CONNECTED) {
                      ArduinoOTA.handle();
                      Serial.printf("OTA service alive @ %s\n", WiFi.localIP().toString().c_str());
                  } else {
                      Serial.println("WiFi disconnected — skipping OTA this cycle.");
                  }

                  */








void update_display(){
    if(currentScreen < 0) LowPower_Screen();
    if(currentScreen == 0) Power_Wifi_Screen();
    if(currentScreen == 1) soilMoi_Screen();
    if(currentScreen == 2) dataUpload_Screen();
    if(currentScreen == 3) ota_update();

    LCD.hibernate();
}

void ota_update(){

    LCD.firstPage();
    do {
        LCD.setRotation(2);
        LCD.fillScreen(GxEPD_WHITE);
        LCD.setFont(&FreeSans12pt7b);
        LCD.setCursor(20, 60);
        
        if (WiFi.status() == WL_CONNECTED) {
            LCD.print("OTA Ready");
            LCD.setCursor(20, 100);
            LCD.print(WiFi.localIP());
        } else {
            LCD.print("OTA Offline");
        }
        
        LCD.setCursor(20, 140);
        LCD.printf("Batt: %.1fV", batt.getVoltage());
    } while (LCD.nextPage());
    
}

void dataUpload_Screen(){

}

void LowPower_Screen(){
  
  LCD.firstPage();
  do {
    LCD.setRotation(2);
    LCD.fillScreen(GxEPD_WHITE);
    
    LCD.setTextColor(LCD.epd2.hasColor ? GxEPD_RED : GxEPD_BLACK);
    LCD.setFont(&FreeSans12pt7b);
    LCD.setCursor(50, 120); LCD.print("Shutdown");
    //    batt_str
    LCD.fillRoundRect(30, 20, 140, 60, 30, GxEPD_BLACK);
    LCD.fillRoundRect(35, 25, 130, 50, 30, GxEPD_WHITE);
    LCD.fillCircle(60, 50, 22, GxEPD_BLACK);

    LCD.setFont(&FreeSansBold18pt7b); LCD.setCursor(86, 64); LCD.print("OFF"); //FreeSansBold18pt7b
    //LCD.fillCircle(100, 40, 25, GxEPD_BLACK);
    //LCD.fillCircle(100, 40, 18, GxEPD_WHITE);

    //
    //LCD.setCursor(40, 190); 
    //LCD.printf("Waking in %.1f hr", power_critical_sleep_duration / 3.6e9);


        //BATTERY
    LCD.fillRoundRect(50, 144, 88, 40, 5, GxEPD_BLACK); LCD.fillRoundRect(54, 148, 80, 32, 5, GxEPD_WHITE); LCD.fillRoundRect((54+81), 155, 8, 20,2, GxEPD_BLACK);  
    LCD.setFont(&FreeSans9pt7b); LCD.setCursor(80, 168); LCD.print(batt.getVoltage()); 
    
  } while (LCD.nextPage());

  
}

void Power_Wifi_Screen() {
  LCD.firstPage();
  do {
    LCD.setRotation(2);
    LCD.fillScreen(GxEPD_WHITE);

    LCD.setFont(&FreeSans12pt7b);
    LCD.setTextColor(GxEPD_BLACK);

    LCD.setCursor(30, 100); LCD.print("Sensor: Ready");
    LCD.setCursor(30, 135);
    LCD.print(can_send_over_espnow ? "Wireless: ON" : "Wireless: OFF");

    // ON button icon
    LCD.fillRoundRect(30, 20, 140, 60, 30, GxEPD_BLACK);
    LCD.fillRoundRect(35, 25, 130, 50, 30, GxEPD_WHITE);
    LCD.fillCircle(60, 50, 22, GxEPD_BLACK);
    LCD.setFont(&FreeSansBold18pt7b);
    LCD.setCursor(96, 64);
    LCD.print("ON");

    // Battery
    drawLargeBatteryIcon(50, 144, batt.getVoltage());
   // drawBatteryIcon(50, 144, batt.getVoltage());

  } while (LCD.nextPage());
}

void drawLargeBatteryIcon(int x, int y, float voltage) {
    // Normalize fill between 7.4V (empty) and 8.4V (full)
    float fill = constrain((voltage - 7.4f) / (8.4f - 7.4f), 0.0f, 1.0f);

    // Outer shell (88×40) with rounded corners
    LCD.fillRoundRect(x, y, 88, 40, 5, GxEPD_BLACK);

    // Positive terminal
    LCD.fillRect((x+86), y + 10, 5, 20, GxEPD_BLACK);

    // Inner white background
    LCD.fillRoundRect(x + 4, y + 4, 80, 32, 4, GxEPD_WHITE);

    // Dynamic fill (black bar)
    int fill_width = (int)(80 * fill);
    LCD.fillRoundRect(x + 4, y + 4, fill_width, 32, 4, GxEPD_BLACK);

    // Voltage text — larger and well aligned
    LCD.setFont(&FreeSans12pt7b);
    LCD.setTextColor(GxEPD_BLACK);
    LCD.setTextSize(1);

    // Print voltage either *inside* if bright, or beside if low
    if (fill > 0.5f) {
        // White text inside battery
        LCD.setTextColor(GxEPD_WHITE);
        LCD.setCursor(x + 20, y + 28);
        LCD.printf("%.1fV", voltage);
    } else {
        // Black text beside battery
        LCD.setTextColor(GxEPD_BLACK);
        LCD.setCursor(x + 95, y + 28);
        LCD.printf("%.1fV", voltage);
    }
}

void drawBatteryIcon(int x, int y, float voltage) {
    float fill = constrain((voltage - 7.4) / (8.4 - 7.4), 0.0, 1.0);

    LCD.fillRoundRect(x, y, 38, 20, 3, GxEPD_BLACK);      // outer shell
    LCD.fillRect(x + 38, y + 5, 4, 10, GxEPD_BLACK);      // positive terminal
    LCD.fillRoundRect(x + 4, y + 4, 30, 12, 2, GxEPD_WHITE); // background
    LCD.fillRoundRect(x + 4, y + 4, (int)(30 * fill), 12, 2, GxEPD_BLACK);

    LCD.setFont(&FreeSans9pt7b);
    LCD.setTextColor(GxEPD_BLACK);
    LCD.setCursor(x - 35, y + 14);
    LCD.printf("%.1fV", voltage);
}


/*
void Power_Wifi_Screen(){

  LCD.firstPage();
  do {
    LCD.setRotation(2);
    LCD.fillScreen(GxEPD_WHITE);
    
    LCD.setTextColor(LCD.epd2.hasColor ? GxEPD_RED : GxEPD_BLACK);
    LCD.setFont(&FreeSans12pt7b);
    LCD.setCursor(30, 105); LCD.print("Sensor: Ready");

    LCD.setCursor(30, 135); LCD.print(can_send_over_espnow?"Wireless: ON":"Wireless: OFF");
    //    batt_str
    LCD.fillRoundRect(30, 20, 140, 60, 30, GxEPD_BLACK);
    LCD.fillRoundRect(35, 25, 130, 50, 30, GxEPD_WHITE);
    LCD.fillCircle(60, 50, 22, GxEPD_BLACK);

    LCD.setFont(&FreeSansBold18pt7b); LCD.setCursor(96, 64); LCD.print("ON"); //FreeSansBold18pt7b
    //LCD.fillCircle(100, 40, 25, GxEPD_BLACK);
    //LCD.fillCircle(100, 40, 18, GxEPD_WHITE);
    
        //BATTERY
    LCD.fillRoundRect(50, 144, 88, 40, 5, GxEPD_BLACK); LCD.fillRoundRect(54, 148, 80, 32, 5, GxEPD_WHITE); LCD.fillRect(45, 155, 5, 20, GxEPD_BLACK);  
    LCD.setCursor(60, 175); LCD.print(batt.getVoltage()); 
    
  } while (LCD.nextPage());

}
*/



const char ID[] = "IrriKit Node 1";
const char Device[] = "Soil Moisture";
const char Manufacturer[] = "IntelliSys Uganda";
const char Company[] = "Makerere University";

void soilMoi_Screen(){
    LCD.setRotation(2);
    LCD.setFont(&FreeMonoBold9pt7b);

  // do this outside of the loop
  int16_t tbx, tby; uint16_t tbw, tbh;

  // center the heading
  LCD.getTextBounds(operational_header, 0, 0, &tbx, &tby, &tbw, &tbh);
  uint16_t header_x = ((LCD.width() - tbw) / 2) - tbx;
  uint16_t header_y = ((LCD.height() / 4) - tbh / 2) - tby;


 //RECORD THE READING  & TIME
 

    /*
      //print only whole numbers on screen
      if(soil_moisture_percent <= 1.0) itoa((int)(simulated_soil_moisture+0.5), soil_moisture_char, 10);  // remove on deployment
      else itoa((int)(soil_moisture_percent+0.5), soil_moisture_char, 10); // strcat(last_soil_moisture, "%");
    */
      itoa((int)(soil_moisture_percent+0.5), soil_moisture_char, 10); 

  LCD.firstPage();
  do {
        LCD.fillScreen(GxEPD_WHITE);
        LCD.setTextColor(GxEPD_BLACK);
        LCD.setTextSize(1);

      //  HEADER
        LCD.setFont(); 
        LCD.getTextBounds(time_str, 0, 0, &tbx, &tby, &tbw, &tbh);
        LCD.setCursor(((LCD.width() - tbw) / 2), 5); LCD.print(time_str);

       //  LCD.setCursor(70, 5); LCD.print(time_str); // uptime
         LCD.setFont(&FreeSans9pt7b); LCD.setCursor(header_x+20, header_y-10); LCD.print(operational_header); 
        
          
      //  BODY
          //DROP OF WATER
          LCD.fillTriangle(20, 100, 30, 80, 40, 100, GxEPD_BLACK);
          LCD.fillCircle(30, 102, 10, GxEPD_BLACK);

          //the actual figure
          
          LCD.setFont(&FreeSansBold24pt7b);  LCD.setTextSize(2); 
          LCD.getTextBounds(soil_moisture_char, 0, 0, &tbx, &tby, &tbw, &tbh);
          uint16_t moi_x = (LCD.width() - tbw) / 2;
          uint16_t moi_y = (LCD.height() * 3 / 5);
          LCD.setCursor(moi_x, moi_y);
          LCD.print(soil_moisture_char);

         LCD.setFont(&FreeSansBold18pt7b);  LCD.setTextSize(1);  LCD.setCursor((moi_x+110), (moi_y-20));  LCD.print("%");
        
          // FreeSerifBold24pt7b // FreeSansBold24pt7b
       //   LCD.setTextSize(2);   LCD.setCursor(moi_x-40, moi_y);  LCD.println(soil_moisture_char);

     //   FOOTNOTE with Network Bar [ON or OFF], Uploads Icon[sents], and Battery Level [Voltage]
          footer();
  } while (LCD.nextPage());
 
}

void draw_mini_graph() {
    int baseY = 155, baseX = 20;
    for (int i = 1; i < 5; i++) {
        int y1 = baseY - (int)(sensor.recent_soil_data[i - 1] * 1.2);
        int y2 = baseY - (int)(sensor.recent_soil_data[i] * 1.2);
        LCD.drawLine(baseX + (i - 1) * 30, y1, baseX + i * 30, y2, GxEPD_BLACK);
    }
}


bool minute_count_passed = false; 

char bytes_char[10] = "";

void footer(){
              LCD.drawFastHLine(10, 165, 180, GxEPD_BLACK);
            // LCD.fillCircle(0, 295, 2, GxEPD_BLACK);
            
           
            //NETWORK BARS
            LCD.fillRect(10, 194, 6, 5, GxEPD_BLACK); LCD.fillRect(20, 184, 6, 15, GxEPD_BLACK); LCD.fillRect(30, 173, 6, 26, GxEPD_BLACK);
            
            LCD.setTextSize(1);
            LCD.setCursor(10, 180);  LCD.setFont(); LCD.print(can_send_over_espnow?"ON":"OFF"); 

          // DATA UPLOAD ICON
            LCD.fillRect(62, 180, 30, 20, GxEPD_BLACK); LCD.fillRect(66, 180, 22, 14, GxEPD_WHITE);
            LCD.fillRect(75, 176, 4, 15, GxEPD_BLACK);  LCD.fillTriangle(69, 176, 76, 170, 83, 176, GxEPD_BLACK);
            if(sends > 0)  {  LCD.setFont(&FreeSans9pt7b); LCD.setCursor(96, 195); LCD.print(sends);   }

                 
        // ==== DYNAMIC BATTERY ICON ====
          // Position: bottom-right (x=150, y=174)
          float v = batt.getVoltage();

          // Normalize 7.4V (empty) → 8.4V (full)
          float fill = constrain((v - 7.4) / (8.4 - 7.4), 0.0, 1.0);

          // Draw battery outline
          LCD.fillRoundRect(150, 174, 38, 20, 3, GxEPD_BLACK);  // outer shell
          LCD.fillRect(188, 179, 4, 10, GxEPD_BLACK);           // positive terminal

          // Inner empty space (white background)
          LCD.fillRoundRect(154, 178, 30, 12, 2, GxEPD_WHITE);

          // Fill proportionally
          int fill_width = (int)(30 * fill);
          LCD.fillRoundRect(154, 178, fill_width, 12, 2, GxEPD_BLACK);

          // Always print voltage in black, left of the battery icon
          LCD.setFont(); // &FreeSans9pt7b
          LCD.setTextSize(1);
          LCD.setTextColor(GxEPD_BLACK);
          LCD.setCursor(125, 180);  // Adjusted for baseline alignment
          LCD.printf("%.1fV", v);

      /*
            LCD.fillRoundRect(150, 174, 38, 20, 3, GxEPD_BLACK); LCD.fillRoundRect(154, 178, 30, 12, 3, GxEPD_WHITE); LCD.fillRect(146, 179, 5, 10, GxEPD_BLACK);  

            LCD.setFont(&FreeSans9pt7b); // LCD.setFont(&FreeSans9pt7b);
            LCD.setTextColor(GxEPD_BLACK); 
    */
          
           
        //  if(batt.getVoltage() >= 0.00) {  LCD.setFont(); LCD.setCursor(157, 180); LCD.print(batt.getVoltage()); }
               
}



void BootScreen(){ Serial.println("Starting Display...");
  LCD.setRotation(2);
  int16_t tbx, tby; uint16_t tbw, tbh;

  // center the bounding box by transposition of the origin:
  uint16_t x = ((LCD.width() - tbw) / 2) - tbx;

  LCD.setFullWindow();
  LCD.firstPage();
  do
  {
    LCD.fillScreen(GxEPD_WHITE);
    LCD.fillRect(0,0,200,200,GxEPD_BLACK);
    LCD.fillRect(2,2,194,194,GxEPD_WHITE);

    //HEADER
    LCD.setFont(&FreeMonoBold12pt7b); // FreeSans12pt7b FreeMonoBold12pt7b
    LCD.setTextColor(GxEPD_BLACK);

    LCD.getTextBounds(ID, 0, 0, &tbx, &tby, &tbw, &tbh);
    x = ((LCD.width() - tbw) / 2) - tbx;
    LCD.setCursor(x, 30);
    LCD.print(ID);

  /* MY OWN 
    //LAND
    LCD.drawRect(70, 40, 60, 50, GxEPD_BLACK);
    for(int i = 70; i<130; i+=10)  {
      for(int j = 50; j<110; j+=5){
        LCD.drawFastHLine(i, 50, i+3, j, GxEPD_BLACK);
      }
    }
    
    //DROP OF WATER
    LCD.fillTriangle(90, 70, 100, 50, 110, 70, GxEPD_BLACK);
    LCD.fillCircle(100, 70, 10, GxEPD_BLACK);
 */
 /*
          // ENHANDED BY CHAT GPT
        // ==== LAND PATCH ====
      // Base soil patch
      LCD.fillRect(70, 80, 60, 20, GxEPD_BLACK);              // dark soil base
      LCD.fillRect(72, 82, 56, 16, GxEPD_WHITE);              // inner lighter soil

      // Soil texture: horizontal furrows
      for (int x = 72; x < 128; x += 6) {
          LCD.drawFastHLine(x, 90 + (x % 4), 4, GxEPD_BLACK);
      }

      // Soil cracks: irregular diagonal lines
      LCD.drawLine(75, 82, 85, 95, GxEPD_BLACK);
      LCD.drawLine(95, 82, 105, 96, GxEPD_BLACK);
      LCD.drawLine(120, 82, 115, 96, GxEPD_BLACK);
      LCD.drawLine(90, 95, 100, 98, GxEPD_BLACK);
      LCD.drawLine(105, 94, 120, 98, GxEPD_BLACK);

      // ==== WATER DROP ====
      // Teardrop body
      LCD.fillCircle(100, 70, 10, GxEPD_BLACK);
      LCD.fillTriangle(92, 60, 108, 60, 100, 45, GxEPD_BLACK);

      // Highlight for reflection
      LCD.fillCircle(96, 64, 3, GxEPD_WHITE);

      // Drop shadow (adds depth)
      LCD.fillEllipse(100, 83, 12, 3, GxEPD_BLACK);
  */
  // BY DEEP SEEK
   // draw_soil_moisture_icon(70, 40, 20);
     draw_simple_soil_icon(20, 35);
   //BY CHAT GPT... very scattered soil moisture // draw_soil_with_droplets(20, 35);


    LCD.setFont(&FreeMonoBold9pt7b);
    LCD.getTextBounds(Device, 0, 0, &tbx, &tby, &tbw, &tbh);
    x = ((LCD.width() - tbw) / 2) - tbx;
    LCD.setCursor(x, 110);
    LCD.print(Device);

    LCD.setFont(&FreeSansBold9pt7b); // FreeSans9pt7b  next besr  
    LCD.getTextBounds(Manufacturer, 0, 0, &tbx, &tby, &tbw, &tbh);
    x = ((LCD.width() - tbw) / 2) - tbx;
    LCD.setCursor(x, 150);
    LCD.print(Manufacturer);

    
    LCD.getTextBounds(Company, 0, 0, &tbx, &tby, &tbw, &tbh);
    x = ((LCD.width() - tbw) / 2) - tbx;
    LCD.setCursor(x, 180);
    LCD.print(Company);
  }
  while (LCD.nextPage());

}

void draw_simple_soil_icon(int x, int y) {
    // Clean wavy soil surface - EXTENDED TO 160px
    for(int i = 0; i < 160; i += 1) {
        float surface = sin(i * 0.08) * 3.0; // Adjusted frequency for wider area
        LCD.drawPixel(x + i, y + 8 + (int)surface, GxEPD_BLACK);
    }
    
    // Soil body with WAVY HORIZONTAL LINES - FEWER LINES, 160px WIDTH
    for(int i = 0; i < 50; i += 6) { // Reduced line density
        float phase = i * 0.15; // Adjusted for wider area
        
        // Wavy horizontal lines - each line has its own wave pattern!
        for(int j = 0; j < 140; j += 2) { // 140px wide lines with waves
            float line_wave = sin(j * 0.1 + phase * 2) * 1.5; // Wave along the line
            int pixel_x = x + 10 + j;
            int pixel_y = y + 10 + i + (int)line_wave;
            
            // Only draw if within the organic soil boundaries
            int left_boundary = sin(phase) * 8;
            int right_boundary = cos(phase * 0.6) * 8;
            
            if (j >= left_boundary && j <= (140 - right_boundary)) {
                LCD.drawPixel(pixel_x, pixel_y, GxEPD_BLACK);
            }
        }
    }
    
    // MORE DROPLETS - INCREASED TO 8 DROPLETS for 160px width
    // Random positions and sizes within logical ranges
    
    for(int i = 0; i < 8; i++) { // Increased from 5 to 8 droplets
        int drop_x, drop_y, drop_size;
        bool has_highlight;
        
        // Determine droplet characteristics based on "row" - adjusted for 160px
        switch(i) {
            case 0: // Top-left area
                drop_x = x + 15 + random(0, 20);
                drop_y = y + 10 + random(0, 6);
                drop_size = 7 + random(0, 2); // 7-8 (larger!)
                has_highlight = true;
                break;
            case 1: // Top-center area
                drop_x = x + 60 + random(0, 25);
                drop_y = y + 8 + random(0, 8);
                drop_size = 6 + random(0, 2); // 6-7
                has_highlight = true;
                break;
            case 2: // Top-right area
                drop_x = x + 110 + random(0, 25);
                drop_y = y + 12 + random(0, 6);
                drop_size = 7 + random(0, 2); // 7-8 (larger!)
                has_highlight = true;
                break;
            case 3: // Upper-middle left
                drop_x = x + 30 + random(0, 25);
                drop_y = y + 22 + random(0, 8);
                drop_size = 6 + random(0, 2); // 6-7
                has_highlight = random(0, 2); // Random highlight
                break;
            case 4: // Upper-middle right
                drop_x = x + 90 + random(0, 25);
                drop_y = y + 24 + random(0, 8);
                drop_size = 6 + random(0, 2); // 6-7
                has_highlight = random(0, 2); // Random highlight
                break;
            case 5: // Lower-middle center
                drop_x = x + 60 + random(0, 30);
                drop_y = y + 30 + random(0, 10);
                drop_size = 5 + random(0, 2); // 5-6
                has_highlight = false;
                break;
            case 6: // Bottom-left area
                drop_x = x + 40 + random(0, 25);
                drop_y = y + 38 + random(0, 12);
                drop_size = 4 + random(0, 2); // 4-5
                has_highlight = false;
                break;
            case 7: // Bottom-right area
                drop_x = x + 100 + random(0, 25);
                drop_y = y + 42 + random(0, 10);
                drop_size = 4 + random(0, 2); // 4-5
                has_highlight = false;
                break;
        }
        
        draw_water_droplet(drop_x, drop_y, drop_size, has_highlight);
    }
}

// Helper function to draw individual droplets
void draw_water_droplet(int center_x, int center_y, int size, bool has_highlight) {
    // Main drop body
    LCD.fillCircle(center_x, center_y, size, GxEPD_BLACK);
    
    // Drop tail (size-appropriate)
    int tail_length = size * 1.6;
    LCD.fillTriangle(center_x - (size-1), center_y - 1,
                    center_x, center_y - tail_length,
                    center_x + (size-1), center_y - 1, GxEPD_BLACK);
    
    // Highlight (only for fresh droplets)
    if (has_highlight) {
        LCD.fillCircle(center_x - (size/2), center_y - (size/2), max(1, size/3), GxEPD_WHITE);
    }
    
    // Absorption effect for deeper droplets
    if (center_y > 35) {
        LCD.drawCircle(center_x, center_y + 2, size/2, GxEPD_BLACK);
    }
}
/*
void draw_soil_with_droplets(int x, int y) {  
    // === Wavy surface line ===
    int width = 160;
    for (int i = 0; i < width; i++) {
        float surface = sin(i * 0.09) * 2.5 + cos(i * 0.04) * 1.8; // two harmonics for organic feel
        LCD.drawPixel(x + i, y + 10 + (int)surface, GxEPD_BLACK);
    }

    // === Soil body with gentle furrows ===
    // Only 5–6 layers of subtle wavy lines for texture
    for (int layer = 0; layer < 40; layer += 8) {
        float phase = layer * 0.25;
        for (int i = 0; i < width; i += 5) {
            float wave = sin((i * 0.07) + phase) * 3 + cos((i * 0.04) + phase * 0.5);
            int yy = y + 12 + layer + (int)wave;
            if (yy < y + 60) LCD.drawPixel(x + i, yy, GxEPD_BLACK);
        }
    }

    // === Small random cracks in soil ===
    for (int c = 0; c < 8; c++) {
        int cx = x + random(5, width - 5);
        int cy = y + random(18, 55);
        int len = random(8, 15);
        int dir = random(0, 2) ? 1 : -1;
        LCD.drawLine(cx, cy, cx + len * dir, cy + random(-4, 4), GxEPD_BLACK);
    }

    // === Randomized water droplets ===
    int numDrops = 6 + random(-1, 2);  // around 5–7 droplets
    for (int i = 0; i < numDrops; i++) {
        int drop_x = x + random(15, width - 15);
        int drop_y = y + random(15, 40);
        int drop_size = random(5, 9);  // larger droplets overall
        bool highlight = (random(0, 3) != 0); // ~2/3 chance to have highlight
        draw_water_droplet(drop_x, drop_y, drop_size, highlight);
    }

    // === Optional subtle ripple effect under one random large droplet ===
    if (random(0, 2)) {
        int ripple_x = x + random(40, width - 40);
        int ripple_y = y + random(30, 50);
        for (int r = 0; r < 3; r++) {
            LCD.drawCircle(ripple_x, ripple_y, 8 + r * 3, GxEPD_BLACK);
        }
    }
}

void draw_soil_with_droplets(int x, int y) {  
    // === Wavy surface line ===
    int width = 160;
    for (int i = 0; i < width; i++) {
        float surface = sin(i * 0.09) * 2.5 + cos(i * 0.04) * 1.8; // two harmonics for organic feel
        LCD.drawPixel(x + i, y + 10 + (int)surface, GxEPD_BLACK);
    }

    // === Soil body with gentle furrows ===
    // Only 5–6 layers of subtle wavy lines for texture
    for (int layer = 0; layer < 40; layer += 8) {
        float phase = layer * 0.25;
        for (int i = 0; i < width; i += 5) {
            float wave = sin((i * 0.07) + phase) * 3 + cos((i * 0.04) + phase * 0.5);
            int yy = y + 12 + layer + (int)wave;
            if (yy < y + 60) LCD.drawPixel(x + i, yy, GxEPD_BLACK);
        }
    }

    // === Small random cracks in soil ===
    for (int c = 0; c < 8; c++) {
        int cx = x + random(5, width - 5);
        int cy = y + random(18, 55);
        int len = random(8, 15);
        int dir = random(0, 2) ? 1 : -1;
        LCD.drawLine(cx, cy, cx + len * dir, cy + random(-4, 4), GxEPD_BLACK);
    }

    // === Randomized water droplets ===
    int numDrops = 6 + random(-1, 2);  // around 5–7 droplets
    for (int i = 0; i < numDrops; i++) {
        int drop_x = x + random(15, width - 15);
        int drop_y = y + random(15, 40);
        int drop_size = random(5, 9);  // larger droplets overall
        bool highlight = (random(0, 3) != 0); // ~2/3 chance to have highlight
        draw_water_droplet(drop_x, drop_y, drop_size, highlight);
    }

    // === Optional subtle ripple effect under one random large droplet ===
    if (random(0, 2)) {
        int ripple_x = x + random(40, width - 40);
        int ripple_y = y + random(30, 50);
        for (int r = 0; r < 3; r++) {
            LCD.drawCircle(ripple_x, ripple_y, 8 + r * 3, GxEPD_BLACK);
        }
    }
}

*/

void draw_soil_moisture_icon(int x, int y, int moisture_level) {
    // Soil container (pot/ground)
    LCD.drawRect(x, y, 60, 50, GxEPD_BLACK);
    
    // Soil texture with more natural pattern
    for(int i = 0; i < 60; i += 8) {
        for(int j = 0; j < 50; j += 6) {
            // Random soil particles - more organic look
            if(random(0, 3) > 0) { // 66% chance to draw a particle
                int particle_x = x + i + random(0, 5);
                int particle_y = y + j + random(0, 4);
                LCD.drawPixel(particle_x, particle_y, GxEPD_BLACK);
                
                // Occasionally draw larger soil clumps
                if(random(0, 5) == 0) {
                    LCD.drawPixel(particle_x + 1, particle_y, GxEPD_BLACK);
                    LCD.drawPixel(particle_x, particle_y + 1, GxEPD_BLACK);
                }
            }
        }
    }
    
    // Water level based on moisture (visual indicator)
    int water_height = map(constrain(moisture_level, 0, 100), 0, 100, 5, 45);
    LCD.fillRect(x + 2, y + 50 - water_height, 56, water_height, GxEPD_BLACK);
    
    // Realistic water drop with highlight
    int drop_x = x + 45;
    int drop_y = y + 25;
    
    // Water drop body (teardrop shape)
    LCD.fillCircle(drop_x, drop_y, 8, GxEPD_BLACK);
    LCD.fillTriangle(drop_x - 8, drop_y, drop_x, drop_y - 12, drop_x + 8, drop_y, GxEPD_BLACK);
    
    // Water drop highlight (white reflection)
    LCD.fillCircle(drop_x - 2, drop_y - 2, 2, GxEPD_WHITE);
    
    // Ripples around the drop (optional)
    LCD.drawCircle(drop_x, drop_y, 12, GxEPD_BLACK);
    LCD.drawCircle(drop_x, drop_y, 14, GxEPD_BLACK);
}

void partial_update_Screen(){
  Serial.println("testing for helloFullScreenPartialMode");
  const char fullscreen[] = "full screen update";
  const char fpm[] = "fast partial mode";
  const char spm[] = "slow partial mode";
  const char npm[] = "no partial mode";
  LCD.setPartialWindow(0, 0, LCD.width(), LCD.height());
  
  LCD.setRotation(2);
  LCD.setFont(&FreeMonoBold9pt7b);
  if (LCD.epd2.WIDTH < 104) LCD.setFont(0);
  LCD.setTextColor(GxEPD_BLACK);
  const char* updatemode;
  if (LCD.epd2.hasFastPartialUpdate)
  {
    updatemode = fpm;
    Serial.println("Screen Has Fast PArtial Update");
  }
  else if (LCD.epd2.hasPartialUpdate)
  {
    updatemode = spm;
    Serial.println("Screen Has  Partial Update");
  }
  else
  {
    updatemode = npm;
    Serial.println("Screen has NO Partial Update");
  }


  // do this outside of the loop
  int16_t tbx, tby; uint16_t tbw, tbh;

  // center the heading
  LCD.getTextBounds(operational_header, 0, 0, &tbx, &tby, &tbw, &tbh);
  uint16_t header_x = ((LCD.width() - tbw) / 2) - tbx;
  uint16_t header_y = ((LCD.height() / 4) - tbh / 2) - tby;

  // center the reading
  LCD.getTextBounds(moi_char, 0, 0, &tbx, &tby, &tbw, &tbh);
  uint16_t moi_x = ((LCD.width() - tbw) / 2) - tbx;
  uint16_t moi_y = ((LCD.height() * 3 / 5) - tbh / 2) - tby;




  LCD.firstPage();
  do
  {
        LCD.fillScreen(GxEPD_WHITE);
        LCD.setTextSize(1);

    //  HEADER
        LCD.setFont(&FreeSans9pt7b);
        LCD.setCursor(header_x+20, header_y-20); LCD.print(operational_header); 
        
    //  BODY
        LCD.setTextColor(GxEPD_BLACK);  LCD.setCursor((moi_x+70), (moi_y-30));  LCD.println("%");
      
        LCD.setFont(&FreeSansBold24pt7b); // FreeSerifBold24pt7b // FreeSansBold24pt7b
        LCD.setTextSize(2);   LCD.setCursor(moi_x-40, moi_y);  LCD.println(moi_char);

    //  FOOTNOTE with Sent Messages and Battery Level
        footer();

  }
  while (LCD.nextPage());
  Serial.println("helloFullScreenPartialMode testing done");
}



/*
void handle_ota(uint64_t now, bool button_pressed) {
    static uint64_t last_ota_check = 0;
    
    if (button_pressed && !ota_active) {
        Serial.println("OTA mode activated!");
        ota_active = true;
        
        // 1. Suspend sensor task immediately
        if (sensor_task_handle != NULL) {
            vTaskSuspend(sensor_task_handle);
            Serial.println("Sensor task suspended");
        }
        
        // 2. Power down sensors
        sensor.toggle_sensor(OFF);
        sensor.flash_indicators(false);
        
        // 3. Deinit ESP-NOW
        esp_now_deinit();
        esp_wifi_stop();
        
        // 4. Start OTA in normal WiFi mode
        start_ota_mode();  // Your OTA initialization
        
        last_ota_check = now;
    }
    
    // Handle OTA updates
    if (ota_active) {
        handle_ota_updates();  // Your OTA loop
        
        // Check if OTA timed out (e.g., 5 minutes no activity)
        if ((now - last_ota_check) > (300 * 1000000ULL)) {
            Serial.println("OTA timeout, resuming normal operation");
            exit_ota_mode();
        }
    }
}

void exit_ota_mode() {
    ota_active = false;
    
    // 1. Stop OTA
    stop_ota();
    
    // 2. Reinitialize ESP-NOW
    init_esp_now();  // Your ESP-NOW init function
    
    // 3. Resume sensor task
    if (sensor_task_handle != NULL) {
        vTaskResume(sensor_task_handle);
        Serial.println("Sensor task resumed");
    }
}

void handle_ota(uint64_t running_time, bool btn_state) {

    if(btn_state) otaModeActive = true;

    if(!otaModeActive) return;

    // WiFi check every 2 seconds
    if ((running_time - last_wifi_check) >= (2ULL * 1000ULL)) { // wifi_check_interval
        wifi_connected = wifi_obj.ensure_wifi();
        last_wifi_check = running_time;
    }
    
    // Handle OTA state transitions with appropriate LED patterns
    if (otaStarted) {
        strcpy(ota_log, "Starting OTA update...");
        buzzer.beep(2, 300, 200);
        ota_start_time = running_time;  // Track when OTA actually started
        otaStarted = false;
        current_ota_state = OTA_STATE_STARTING;
    }
    
    if (otaProgress > 0 && otaProgress < 100) {
        snprintf(ota_log, sizeof(ota_log), "Updating... %d%%", otaProgress);
        current_ota_state = OTA_STATE_PROGRESS;
    }
    
    if (otaFinished) {
        strcpy(ota_log, "Update complete!");
        //snprintf(LastOTAUpdate, sizeof(LastOTAUpdate), "On Date: %s, at %s", sawa.SystemDate, sawa.ShortTime);
        buzzer.beep(2, 500, 200);
        otaFinished = false;
        current_ota_state = OTA_STATE_SUCCESS;
        ota_success_time = running_time;  // Track for success display duration
    }
    
    if (otaError) {
        strcpy(ota_log, "Update failed!");
        buzzer.beep(1, 500, 500);
        otaError = false;
        current_ota_state = OTA_STATE_ERROR;
        ota_error_time = running_time;
    }
    
    // Check for timeout
    if ((running_time - otaStartTime) >= OTA_TIMEOUT && otaModeActive) {
        snprintf(ota_log, sizeof(ota_log), "OTA timeout after %llu seconds", 
                 (running_time - otaStartTime) / 1000);
        current_ota_state = OTA_STATE_TIMEOUT;
        ota_timeout_time = running_time;
    }
    
    // Apply appropriate blink pattern based on current state
   // apply_ota_blink_pattern(running_time);
    
    // Exit conditions
    if (current_ota_state == OTA_STATE_TIMEOUT) {
        if ((running_time - ota_timeout_time) >= 5000) {  // Show timeout for 5 seconds
            otaModeActive = false;
            switch_radio_to_espnow();
        }
    }
    
    if (current_ota_state == OTA_STATE_SUCCESS) {
        if ((running_time - ota_success_time) >= 5000) {  // Show success for 5 seconds
            otaModeActive = false;
            switch_radio_to_espnow();
        }
    }
    
    if (current_ota_state == OTA_STATE_ERROR) {
        if ((running_time - ota_error_time) >= 5000) {  // Show error for 5 seconds
            otaModeActive = false;
            switch_radio_to_espnow();
        }
    }
}
*/




// -----------------------------
// Call this from setup() after esp_now_init()
// -----------------------------
void init_send_system() {
  // create in setup (not at global scope)
  sendSemaphore = xSemaphoreCreateBinary();
  if (!sendSemaphore) {
    Serial.println("ERROR: sendSemaphore create failed");
  }

  xDataMutex = xSemaphoreCreateMutex();
  if (!xDataMutex) {
    Serial.println("ERROR: xDataMutex create failed");
  }

      // ADD: Critical resource checks
    if (xDataMutex == NULL) {
        Serial.println("CRITICAL: Mutex creation failed!");
        // Handle error - perhaps enter safe mode
    }

    if (sendSemaphore == NULL) {
        Serial.println("WARNING: Send semaphore creation failed");
    }
}

// ======================
// ESP-NOW callback
// ======================
// -----------------------------
// Safe ISR callback (already correct style)
// -----------------------------

/*
// Old (before ESP-IDF v5)
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status){

//void OnDataSent(const wifi_tx_info_t *mac_addr, esp_now_send_status_t status) {
  last_send_success = (status == ESP_NOW_SEND_SUCCESS);
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  if (sendSemaphore) {
    xSemaphoreGiveFromISR(sendSemaphore, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) portYIELD_FROM_ISR();
  }
}

uint32_t delvs = 0;
bool successfully_delivered = false;
uint8_t send_attempts = 0;

    //if too low, go to sleep and display Shut down due to Low Power, onl;y awake after an hour, run set up and on reaching battery low, go back to sleep

    //if moderate, wake up and do all work once every 15 mins   

    // if excellent, never sleep, keep logging data once every 5 minutes

*/


// -----------------------------
// Globals used by the snippet
// -----------------------------
// ======================
// Globals for sending
// ======================





/*
void initOTA() {
    // Start WiFi for OTA (can coexist with ESP-NOW)
    WiFi.mode(WIFI_AP_STA);
    WiFi.begin(System.OverTheAir.ssid, System.OverTheAir.password);
    
    // Wait for connection with timeout
    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < 10000) {
        delay(500);
        Serial.print(".");
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        ArduinoOTA.setHostname(System.OverTheAir.hostname);
        ArduinoOTA.setPort(System.OverTheAir.port);
        
        ArduinoOTA
            .onStart([]() {
                String type;
                if (ArduinoOTA.getCommand() == U_FLASH) {
                    type = "sketch";
                } else { // U_SPIFFS
                    type = "filesystem";
                }
                Serial.println("Start updating " + type);
                
                // Stop tasks and ESP-NOW during update
                can_send_over_espnow = false;
                vTaskSuspend(NULL); // Suspend current task
            })
            .onEnd([]() {
                Serial.println("\nOTA Update Complete");
            })
            .onProgress([](unsigned int progress, unsigned int total) {
                Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
            })
            .onError([](ota_error_t error) {
                Serial.printf("Error[%u]: ", error);
                if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
                else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
                else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
                else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
                else if (error == OTA_END_ERROR) Serial.println("End Failed");
                
                // Resume operations on error
                can_send_over_espnow = true;
            });

        ArduinoOTA.begin();
        Serial.println("OTA Ready");
        Serial.print("IP address: ");
        Serial.println(WiFi.localIP());
    } else {
        Serial.println("WiFi failed - OTA unavailable");
    }
}
*/




/*
void handle_ota(uint64_t running_time, bool btn_state) {
    static bool ota_initialized = false;
    
    // Enter OTA mode when button is pressed and not already in OTA
    if (btn_state && !otaModeActive) {
        enter_ota_mode();
        ota_initialized = false;
        return;
    }
    
    // Exit if OTA mode not active
    if (!otaModeActive) return;
    
    // Initialize OTA only once when mode becomes active
    if (!ota_initialized) {
        toggle_ota();
       // initialize_ota_mode();
        ota_initialized = true;
    }
    
    // Handle OTA updates
 //   ArduinoOTA.handle();
    
    // Manage WiFi connection (every 2 seconds)
    if ((running_time - last_wifi_check) >= (2ULL * 1000ULL)) {
        wifi_connected = wifi_obj.ensure_wifi();
        last_wifi_check = running_time;
    }
    
    // Process OTA state transitions
    process_ota_state_transitions(running_time);
    
    // Check for OTA timeout
    if ((running_time - otaStartTime) >= OTA_TIMEOUT && otaModeActive) {
        snprintf(ota_log, sizeof(ota_log), "OTA timeout after %llu seconds", 
                 (running_time - otaStartTime) / 1000);
        current_ota_state = OTA_STATE_TIMEOUT;
        ota_timeout_time = running_time;
    }
    
    // Handle exit conditions
    handle_ota_exit_conditions(running_time);
}
*/


/*
void initialize_ota_mode() {
   
    // Start WiFi for OTA
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);  // You'll need to provide ssid/password variables
    
    // Configure OTA callbacks
    ArduinoOTA.setPassword(OTA_PASS);
    
    ArduinoOTA.onStart([]() {
        Serial.println("OTA Start");
        otaStarted = true;
    });
    
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        otaProgress = (progress / (total / 100));
    });
    
    ArduinoOTA.onEnd([]() {
        Serial.println("OTA End");
        otaFinished = true;
    });
    
    ArduinoOTA.onError([](ota_error_t error) {
        otaError = true;
    });
    
    ArduinoOTA.begin();
    
    Serial.println("OTA Mode Active - Waiting for update");
    Serial.printf("OTA Timeout: %llu minutes\n", (OTA_TIMEOUT / 60000));
    buzzer.beep(2, 200, 100);
}

*/




