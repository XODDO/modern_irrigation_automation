#ifndef SENSE_H
#define SENSE_H

#include <Arduino.h>
//#include <ArduinoJson.h>
#include "Battery.h"

// Forward declarations for external variables (defined in main)
extern uint64_t now_now_ms;
extern float voltage_change;

// External objects (defined in main)
// External objects with YOUR actual class names
 class Battery;         // Your Battery class instance
//extern Battery batt(32, 4.20, 1);

// External objects - DECLARE only (no constructor parameters!)
extern Battery batt;


class sense {
private:
    // Variables that will never be called directly by main
    
    // Internal pin variables
    uint8_t sensorPin;
    uint8_t sensorRelay;
    uint8_t lowPin;
    uint8_t mediumPin;
    uint8_t highPin;
    
    uint32_t settling_time = 500; // half a second to settle
    
    // Internal methods
    void update_indicators(float soilMoisture_val);
    void read_sensor_samples(uint16_t *buffer, uint16_t sample_count,
                            uint16_t &lowest, uint16_t &highest,
                            uint32_t &sum, uint16_t &readings);
    float compute_mean(uint32_t sum, uint16_t readings);
    float compute_variance(float *buffer, uint16_t readings, float mean);
    float compute_stddev(float variance);
    float compute_median(float *buffer, uint16_t readings); 
    uint16_t compute_range(uint16_t highest, uint16_t lowest);

public:

    const uint16_t SAMPLE_COUNT = 500;
    float mean = 0.0f, variance = 0.0f, stddev = 0.0f;
    uint16_t median = 0, range = 0;
    float readings_duration = 0.0;

   
    // Passed directly from main via methods
    void begin(uint8_t sensor_pin, uint8_t toggle_pin);
    void initialize_indicators(uint8_t low, uint8_t medium, uint8_t high);
    void toggle_sensor(bool state);
    void flash_indicators(bool state);

    // Work horses
    float sense_reading();
    float soil_moisture_calibrator(uint16_t working_reading);
    //bool serialize_to_JSON();

    // Public variables
    float soilPercent = 0.00f;
    
    // Soil specific constants
    const float wiltingPoint = 19.0;     // the THE DRYEST SOIL
    const float fieldCapacity = 40.0;    // THE WETTEST SOIL
    
    // GLOBAL VARIABLES TO BE USED by 3 functions: sense_reading, update_display and send_as_JSON
    //const float reading_range = 2800.0; // 50 - 2850
    const float reading_range = 4095.0;   // 0 - 4095
    float simulated_soil_moisture = 25.0; // Starting value
    
    float recent_soil_data[6] = {0.00, 0.00, 0.00, 0.00, 0.00}; //6 slots ...but ... 5 readings
    char  recent_soil_moistures[6][8] = {"XX", "YY", "ZZ"}; //6 slots for 5 most recent readings e.g. 10.4%
    char  recent_soil_times[6][8] = {"xx:xx", "yy:yy", "10:35"}; //5 sets of seven each 10:35
    char last_soil_moisture[10] = "x.x%";
    
    // Simulation helper
    float get_simulated_moisture();
};

#endif