#include "sense.h"
#include "Arduino.h"
#include <math.h>

// Static JSON document (uncomment if you need JSON serialization)
// static StaticJsonDocument<240> JSON_data;

void sense::begin(uint8_t sensor_pin, uint8_t toggle_pin) {
    sensorPin = sensor_pin;
    sensorRelay = toggle_pin;
    pinMode(sensorPin, INPUT);
    pinMode(sensorRelay, OUTPUT);
}

void sense::initialize_indicators(uint8_t low, uint8_t medium, uint8_t high) {
    lowPin = low;
    mediumPin = medium;
    highPin = high;
    
    pinMode(lowPin, OUTPUT); 
    digitalWrite(lowPin, HIGH); 
    vTaskDelay(pdMS_TO_TICKS(500));
    
    pinMode(mediumPin, OUTPUT); 
    digitalWrite(mediumPin, HIGH);  
    vTaskDelay(pdMS_TO_TICKS(500));
    
    pinMode(highPin, OUTPUT); 
    digitalWrite(highPin, HIGH);  
    vTaskDelay(pdMS_TO_TICKS(500));
    
    Serial.println("Indicators initialized!");
}

void sense::toggle_sensor(bool state) {
    if(state == true) 
        digitalWrite(sensorRelay, HIGH);
    else 
        digitalWrite(sensorRelay, LOW);
    
    vTaskDelay(pdMS_TO_TICKS(settling_time));
}

float sense::get_simulated_moisture() {
    float change = (random(-200, 201)) / 100.0;
    simulated_soil_moisture += change;
    simulated_soil_moisture = constrain(simulated_soil_moisture, 5.0, 45.0);
    return simulated_soil_moisture;
}

// Main Orchestrator
float sense::sense_reading() {
    Serial.println("Now Scanning the Soil...\n");
    flash_indicators(true);
    
    uint16_t raw_buffer[SAMPLE_COUNT];
    float refined_buffer[SAMPLE_COUNT];  // NEW: Store calibrated values
    
    uint32_t raw_sum = 0;
    uint16_t lowest = 4095, highest = 0;
    uint16_t readings = 0;
    
    
    uint64_t start_reading = esp_timer_get_time();
    
    // --- Sampling & Immediate Calibration ---
    for (uint16_t i = 0; i < SAMPLE_COUNT; i++) {
        uint16_t raw_val = analogRead(sensorPin);
        raw_buffer[i] = raw_val;
        
        // Track raw min/max for range calculation
        if (raw_val > highest) highest = raw_val;
        if (raw_val < lowest)  lowest = raw_val;
        
        raw_sum += raw_val;
        
        // IMMEDIATELY calibrate each reading
        refined_buffer[i] = soil_moisture_calibrator(raw_val);
        
        readings++;
        vTaskDelay(1);  // Small delay between samples
    }
    
    uint64_t stop_reading = esp_timer_get_time();
    readings_duration = float(stop_reading - start_reading) / 1000.0;
    
    // --- Statistics on CALIBRATED values (NOT raw) ---
    float calibrated_sum = 0.0;
    for (uint16_t i = 0; i < SAMPLE_COUNT; i++) {
        calibrated_sum += refined_buffer[i];
    }
    mean = calibrated_sum / SAMPLE_COUNT;  // Mean of calibrated values
    
    variance = compute_variance(refined_buffer, SAMPLE_COUNT, mean);
    stddev = compute_stddev(variance);
    median = compute_median(refined_buffer, SAMPLE_COUNT);
    range = compute_range(highest, lowest);  // Range still on raw values
    
    // --- Final soil moisture (already calibrated from mean of calibrated values)
    soilPercent = mean;  // Because mean is already calibrated!
    
    /*
    // --- Output ---
    update_indicators(soilPercent);
    
    
    Serial.print("Reading Duration: ");
    Serial.print(readings_duration, 2);
    Serial.println(" ms");
    
    Serial.print("Mean (calibrated): "); Serial.println(mean);
    Serial.print("Median (calibrated): "); Serial.println(median);
    Serial.print("Std Dev (calibrated): "); Serial.println(stddev);
   // Serial.print("Raw Range: "); Serial.println(range);

   */
    
    if (highest > 0) {
        Serial.print("Raw Signal Variation (%): ");
        Serial.println(((highest - lowest) * 100.0) / highest);
    }
    
    Serial.print("Soil Moisture: ");
    Serial.print(soilPercent);
    Serial.println("%\n");
    
    flash_indicators(false);
    return soilPercent;
}

// STATISTICS ENGINE (UPDATED for float arrays)
float sense::compute_mean(uint32_t sum, uint16_t readings) {
    if (readings == 0) return 0;
    return (float)sum / readings;
}

float sense::compute_variance(float *buffer, uint16_t readings, float mean) {
    if (readings == 0) return 0;
    
    float variance = 0.0;
    for (uint16_t i = 0; i < readings; i++) {
        float diff = buffer[i] - mean;
        variance += diff * diff;
    }
    return variance / readings;
}

float sense::compute_stddev(float variance) {
    return sqrt(variance);
}

float sense::compute_median(float *buffer, uint16_t readings) {  // FIXED: returns float, not uint16_t
    if (readings == 0) return 0;
    
    // Copy to temporary array to avoid modifying original
    float temp[readings];
    for (uint16_t i = 0; i < readings; i++) {
        temp[i] = buffer[i];
    }
    
    // Sort the copy
    for (uint16_t i = 0; i < readings - 1; i++) {
        for (uint16_t j = 0; j < readings - i - 1; j++) {
            if (temp[j] > temp[j + 1]) {
                float temp_val = temp[j];
                temp[j] = temp[j + 1];
                temp[j + 1] = temp_val;
            }
        }
    }
    
    if (readings % 2 == 0) {
        return (temp[readings / 2] + temp[(readings / 2) - 1]) / 2.0f;
    } else {
        return temp[readings / 2];
    }
}

uint16_t sense::compute_range(uint16_t highest, uint16_t lowest) {
    return (highest - lowest);
}

void sense::read_sensor_samples(uint16_t *buffer, uint16_t sample_count,
                                uint16_t &lowest, uint16_t &highest,
                                uint32_t &sum, uint16_t &readings) {
    lowest = 4095;
    highest = 0;
    sum = 0;
    readings = 0;
    
    for (uint16_t i = 0; i < sample_count; i++) {
        uint16_t val = analogRead(sensorPin);
        buffer[i] = val;
        
        if (val > highest) highest = val;
        if (val < lowest)  lowest = val;
        
        sum += val;
        readings++;
        vTaskDelay(1);
    }
}

float sense::soil_moisture_calibrator(uint16_t working_reading) {
    float refined_val = (float)working_reading;
    
    refined_val = refined_val / reading_range;
    
    if (refined_val <= 0.80) {
        refined_val = refined_val * fieldCapacity;
    } else {
        refined_val = refined_val * 100.0;
    }
    
    return refined_val;
}

void sense::flash_indicators(bool state) {
    if(state == true) {
        digitalWrite(lowPin, HIGH);   
        digitalWrite(mediumPin, HIGH); 
        digitalWrite(highPin, HIGH); 
    } else {
        digitalWrite(lowPin, LOW);
        digitalWrite(mediumPin, LOW);
        digitalWrite(highPin, LOW);
    }
}

void sense::update_indicators(float soilMoisture_val) {
    if(soilMoisture_val <= wiltingPoint) {
        digitalWrite(lowPin, HIGH);   
        digitalWrite(mediumPin, LOW); 
        digitalWrite(highPin, LOW);   
    } else if(soilMoisture_val <= 25.0) {
        digitalWrite(lowPin, LOW);    
        digitalWrite(mediumPin, HIGH);
        digitalWrite(highPin, LOW);   
    } else {
        digitalWrite(lowPin, LOW);    
        digitalWrite(mediumPin, LOW);
        digitalWrite(highPin, HIGH);  
    }
    vTaskDelay(pdMS_TO_TICKS(10));
}