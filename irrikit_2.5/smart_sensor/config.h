// Instead of hard-coded values
typedef struct {
    struct {
        //uint8_t broadcastAddress[6] = {0xF4, 0x65, 0x0B, 0x54, 0x60, 0x94};
        uint8_t broadcastAddress[6] = {0x1C, 0x69, 0x20, 0xA3, 0xDD, 0xC4}; // 
        //09:42:06.778 -> ESP Board MAC Address:  1C:69:20:A3:DD:C4

        uint8_t broadcastAddress2[6] = {0xEC, 0x62, 0x60, 0x2C, 0xA6, 0xD4}; //  FANS CONTROLLER - AHAMADIAN ESP Board MAC Address:  EC:62:60:2C:A6:D4

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
