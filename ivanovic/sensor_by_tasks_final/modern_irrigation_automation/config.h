// Instead of hard-coded values
typedef struct {
    struct {
        uint8_t broadcastAddress[6] = {0xF4, 0x65, 0x0B, 0x54, 0x60, 0x94};
        const char* location = "IRRI-KIT, MUARIK, SENSOR 1";
    } peer;
    
    struct {
        uint16_t critical_power_interval = (2*3600); // 2 HOURS
        uint16_t low_power_interval = 1800; // 30 MINS seconds
        uint16_t moderate_interval = 300;   // 30 seconds
        uint16_t excellent_interval = 10;  // 10 seconds
    } intervals;

    struct {
    const char* ssid = "IntelliSys Pro Max";
    const char* password = "intel_cool@2025";
    const char* hostname = "soil-sensor-1";
    uint16_t port = 3232;
    } OverTheAir;
} Config;

Config System;
