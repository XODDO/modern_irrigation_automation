// memory_monitor.h

#ifndef MEMORY_MONITOR_H
#define MEMORY_MONITOR_H

#include <esp_heap_caps.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

class MemoryMonitor {
private:
    uint32_t last_free_heap;
    uint32_t last_largest_block;
    uint32_t last_min_free;
    uint32_t last_psram_free;
    
    // Buffer for formatted output
    char memory_report_buffer[1024];
    char task_info_buffer[512];
    
public:
    MemoryMonitor() {
        last_free_heap = 0;
        last_largest_block = 0;
        last_min_free = 0;
        last_psram_free = 0;
        memory_report_buffer[0] = '\0';
        task_info_buffer[0] = '\0';
    }
    
    // Get memory report as string (returns pointer to buffer)
    const char* get_memory_report(const char* tag) {
        uint32_t free_heap = ESP.getFreeHeap();
        uint32_t largest_block = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
        uint32_t min_free = ESP.getMinFreeHeap();
        uint32_t free_psram = ESP.getFreePsram();
        uint32_t total_psram = ESP.getPsramSize();
        
        // Calculate fragmentation percentage
        float fragmentation = 100.0f - (largest_block * 100.0f / free_heap);
        
        int len = snprintf(memory_report_buffer, sizeof(memory_report_buffer),
            "\n========== MEMORY REPORT [%s] ==========\n"
            "Free Heap:      %6u bytes (%.1f KB)\n"
            "Largest Block:  %6u bytes (%.1f KB)\n"
            "Min Free Heap:  %6u bytes (%.1f KB)\n"
            "Fragmentation:  %.1f%%\n",
            tag, free_heap, free_heap/1024.0,
            largest_block, largest_block/1024.0,
            min_free, min_free/1024.0,
            fragmentation);
        
        if (free_psram > 0) {
            len += snprintf(memory_report_buffer + len, sizeof(memory_report_buffer) - len,
                "Free PSRAM:     %6u bytes (%.1f KB)\n"
                "Total PSRAM:    %6u bytes (%.1f KB)\n",
                free_psram, free_psram/1024.0,
                total_psram, total_psram/1024.0);
        }
        
        // Detect leaks
        if (last_free_heap > 0 && free_heap < last_free_heap) {
            int32_t leak = last_free_heap - free_heap;
            len += snprintf(memory_report_buffer + len, sizeof(memory_report_buffer) - len,
                "⚠️ Memory LEAK: -%d bytes since last report\n", leak);
        }
        
        // Detect fragmentation increase
        if (last_largest_block > 0 && largest_block < last_largest_block) {
            int32_t frag_increase = last_largest_block - largest_block;
            len += snprintf(memory_report_buffer + len, sizeof(memory_report_buffer) - len,
                "⚠️ Fragmentation increased: largest block shrank by %d bytes\n", frag_increase);
        }
        
        snprintf(memory_report_buffer + len, sizeof(memory_report_buffer) - len,
            "========================================\n");
        
        last_free_heap = free_heap;
        last_largest_block = largest_block;
        
        return memory_report_buffer;
    }
    
    // Get task info as string (returns pointer to buffer)
    const char* get_task_info() {
        int len = snprintf(task_info_buffer, sizeof(task_info_buffer),
            "\n========== TASK INFO ==========\n"
            "Total tasks: %d\n",
            uxTaskGetNumberOfTasks());
        
        // Get current task handle
        TaskHandle_t current_task = xTaskGetCurrentTaskHandle();
        char* task_name = pcTaskGetName(current_task);
        len += snprintf(task_info_buffer + len, sizeof(task_info_buffer) - len,
            "Current task: %s\n", task_name);
        
        // Get idle task info
        TaskHandle_t idle_task = xTaskGetIdleTaskHandle();
        if (idle_task) {
            char* idle_name = pcTaskGetName(idle_task);
            len += snprintf(task_info_buffer + len, sizeof(task_info_buffer) - len,
                "Idle task: %s\n", idle_name);
        }
        
        snprintf(task_info_buffer + len, sizeof(task_info_buffer) - len,
            "================================\n");
        
        return task_info_buffer;
    }
    
    // Get combined memory and task report (for system log)
    const char* get_system_memory_report(const char* tag) {
        static char combined_buffer[2048];
        const char* mem_report = get_memory_report(tag);
        const char* task_info = get_task_info();
        
        snprintf(combined_buffer, sizeof(combined_buffer), "%s%s", mem_report, task_info);
        return combined_buffer;
    }
    
    // Print to serial (original functionality preserved)
    void print_memory_report(const char* tag) {
        Serial.print(get_memory_report(tag));
    }
    
    void print_task_info() {
        Serial.print(get_task_info());
    }
    
    // Print combined report
    void print_system_memory_report(const char* tag) {
        Serial.print(get_system_memory_report(tag));
    }
    
    // Get stack high water mark for current task
    const char* get_current_task_stack() {
        static char stack_buffer[128];
        UBaseType_t stack_high_water = uxTaskGetStackHighWaterMark(NULL);
        snprintf(stack_buffer, sizeof(stack_buffer),
                 "Current task stack high water: %u bytes\n", stack_high_water);
        return stack_buffer;
    }
    
    void print_current_task_stack() {
        Serial.print(get_current_task_stack());
    }
    
    // Get stack for a specific task by name
    const char* get_task_stack(const char* task_name) {
        static char stack_buffer[128];
        TaskHandle_t task = xTaskGetHandle(task_name);
        if (task) {
            UBaseType_t stack_high_water = uxTaskGetStackHighWaterMark(task);
            snprintf(stack_buffer, sizeof(stack_buffer),
                     "Task '%s' stack high water: %u bytes\n", task_name, stack_high_water);
        } else {
            snprintf(stack_buffer, sizeof(stack_buffer),
                     "Task '%s' not found\n", task_name);
        }
        return stack_buffer;
    }
    
    void print_task_stack(const char* task_name) {
        Serial.print(get_task_stack(task_name));
    }
    
    // List all tasks (returns formatted string)
    const char* list_all_tasks() {
        static char tasks_buffer[1024];
        int len = snprintf(tasks_buffer, sizeof(tasks_buffer),
            "\n========== ALL TASKS ==========\n");
        
        const char* known_tasks[] = {
            "loopTask",
            "PacketHandler",
            "esp_timer",
            "IDLE0",
            "IDLE1",
            "Tmr Svc",
            "wifi",
            "event",
            "ipc0",
            "ipc1",
            "rtTp",
            NULL
        };
        
        for (int i = 0; known_tasks[i] != NULL; i++) {
            TaskHandle_t task = xTaskGetHandle(known_tasks[i]);
            if (task) {
                UBaseType_t stack_water = uxTaskGetStackHighWaterMark(task);
                len += snprintf(tasks_buffer + len, sizeof(tasks_buffer) - len,
                    "Task: %-15s Stack High Water: %5u bytes\n",
                    known_tasks[i], stack_water);
            }
        }
        
        snprintf(tasks_buffer + len, sizeof(tasks_buffer) - len,
            "================================\n");
        
        return tasks_buffer;
    }
    
    void list_all_tasks_serial() {
        Serial.print(list_all_tasks());
    }
    
    void check_heap_corruption() {
        if (heap_caps_check_integrity_all(true)) {
            Serial.println("✅ Heap integrity: OK");
        } else {
            Serial.println("❌ HEAP CORRUPTION DETECTED!");
        }
    }
    
    // Get heap corruption status as string
    const char* get_heap_corruption_status() {
        static char status_buffer[64];
        if (heap_caps_check_integrity_all(true)) {
            snprintf(status_buffer, sizeof(status_buffer), "✅ Heap integrity: OK");
        } else {
            snprintf(status_buffer, sizeof(status_buffer), "❌ HEAP CORRUPTION DETECTED!");
        }
        return status_buffer;
    }
    
    // Simple memory summary as string
    const char* get_summary() {
        static char summary_buffer[256];
        uint32_t free_heap = ESP.getFreeHeap();
        uint32_t largest_block = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
        
        snprintf(summary_buffer, sizeof(summary_buffer),
                 "[MEM] Free: %u B (%.1f KB), Largest: %u B (%.1f KB), Tasks: %d\n",
                 free_heap, free_heap/1024.0,
                 largest_block, largest_block/1024.0,
                 uxTaskGetNumberOfTasks());
        
        return summary_buffer;
    }
    
    void print_summary() {
        Serial.print(get_summary());
    }
};

// Global instance
extern MemoryMonitor memMonitor;

#endif