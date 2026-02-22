#pragma once
#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <cstdint>
#include <cstddef>

// ============================================================================
// TASK CONFIGURATION
// ============================================================================
namespace TaskPriority {
    constexpr uint8_t PRIORITY_HIGH = 4;    // For time-critical operations (uploads)
    constexpr uint8_t AVG  = 2;    // For normal operations (serial processing)  
    constexpr uint8_t PRIORITY_LOW  = 1;    // For background tasks (fan control)
}

namespace TaskStack {
    constexpr size_t FANNING  = 4096;   // Reduced from 6240 - your logs showed only 48% usage
    constexpr size_t SERIAL_RX = 8192;  // Matches your working configuration
    constexpr size_t UPLOAD    = 20480; // 20KB for HTTPS security
}

namespace TaskCore {
    constexpr BaseType_t CORE_0 = 0;    // WiFi/Upload operations
    constexpr BaseType_t CORE_1 = 1;    // Sensor/Control operations
}

// ============================================================================
// NETWORK & SERVER CONFIGURATION
// ============================================================================
namespace Netwaka {
    const char* const SERVER_URL = "https://www.webofiot.com/irrigation/muarik_irrikit/server.php";
    const char* const DEVICE_NAME = "IRRIGATION_CONTROLLER_GATEWAY";
    const char* const OTA_PASSWORD = "password";
    
    constexpr uint32_t UPLOAD_FREQUENCY_MS = 10 * 60 * 1000; // 10 minutes
    constexpr uint32_t WIFI_RECONNECT_DELAY_MS = 10000;      // 10 seconds
    constexpr uint32_t HTTP_TIMEOUT_MS = 15000;              // 15 seconds
}

// ============================================================================
// HARDWARE PIN ASSIGNMENTS
// ============================================================================
namespace Pins {
    constexpr uint8_t SERVER_LED        = 27;
    constexpr uint8_t HEARTBEAT_LED     = 25;
    constexpr uint8_t FLOW_METER_LED    = 23; 
    constexpr uint8_t INTERNAL_FAN      = 5;
    constexpr uint8_t WIFI_LED          = 2;
    
    // UART Pins
    constexpr uint8_t SERIAL2_RX        = 16;
    constexpr uint8_t SERIAL2_TX        = 17;
    
    // Future expansion
    constexpr uint8_t BUZZER            = 14; // Reserved
    constexpr uint8_t OTA_BUTTON        = 2;  // Reserved
}

// ============================================================================
// BUFFER & MEMORY CONFIGURATION
// ============================================================================
namespace Memory {
    constexpr size_t JSON_BUFFER_SIZE        = 8192;  // UART frame support
    constexpr size_t UPLOAD_LOG_CAPACITY     = 16;    // Circular buffer size
    constexpr size_t UPLOAD_REPORT_SIZE      = 150;   // Upload status messages
    constexpr size_t OTA_LOG_SIZE            = 150;   // OTA status messages
    constexpr size_t MAX_PAYLOAD_SIZE        = 4096;  // Matches upload.h
}

// ============================================================================
// TIMING & INTERVALS
// ============================================================================
namespace Timing {
    // Task intervals
    constexpr uint32_t FAN_CHECK_MS          = 10000;   // 10 seconds
    constexpr uint32_t SERIAL_RX_DELAY_MS    = 5;       // 5ms between UART reads
    constexpr uint32_t UART_TIMEOUT_MS       = 5000;    // 5 seconds UART inactivity
    
    // LED flash patterns (milliseconds)
    namespace Flash {
        constexpr uint16_t STANDBY_ON_1      = 50;
        constexpr uint16_t STANDBY_OFF_1     = 75; 
        constexpr uint16_t STANDBY_ON_2      = 50;
        constexpr uint16_t STANDBY_OFF_2     = 1500;
        
        constexpr uint16_t RECEIVING_ON      = 200;
        constexpr uint16_t RECEIVING_OFF     = 1000;
        
        constexpr uint16_t ERROR_ON          = 100;
        constexpr uint16_t ERROR_OFF         = 100;
        constexpr uint16_t ERROR_CYCLE_OFF   = 900;
    }
    
    // Diagnostic intervals
    constexpr uint32_t DIAGNOSTIC_INTERVAL_MS = 30000;  // 30 seconds
    constexpr uint32_t STACK_CHECK_INTERVAL_MS = 15000; // 15 seconds
}

// ============================================================================
// SYSTEM PARAMETERS
// ============================================================================
namespace System {
    // Temperature control (Celsius)
    constexpr float FAN_ON_TEMPERATURE   = 33.0f;
    constexpr float FAN_OFF_TEMPERATURE  = 30.0f;
    constexpr float TEMP_INVALID         = -999.0f; // Sentinel value
    
    // JSON field names (for extraction)
    namespace JsonFields {
        const char* const INTERNAL_TEMP = "Internal_Temperature";
        const char* const INTERNAL_TEMP_ALT = "Internal_Temp"; // Fallback
        const char* const TEMPERATURE = "temperature";         // Fallback
    }
    
    // OTA Settings
    constexpr uint64_t OTA_TIMEOUT_MS    = 5 * 60 * 1000; // 5 minutes
}

// ============================================================================
// ERROR CODES & STATUS
// ============================================================================
enum class SystemStatus : uint8_t {
    BOOT = 1,
    SERIAL_INIT,
    BUFFER_READY, 
    WIFI_INIT,
    RUNNING,
    WIFI_FAIL = 10
};

enum class Upload_Status : uint8_t {
    SUCCESS = 0,
    WIFI_DISCONNECTED,
    HTTPS_INIT_FAILED,
    HTTP_OK,
    HTTP_CLIENT_ERROR,
    HTTP_SERVER_ERROR, 
    HTTP_UNEXPECTED,
    NET_TLS_FAILURE,
    TIMEOUT,
    TOO_LARGE
};

#endif // CONSTANTS_H