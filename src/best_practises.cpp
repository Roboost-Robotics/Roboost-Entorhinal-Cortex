#include <Arduino.h>
#include <esp_task_wdt.h>
#include <freertos/semphr.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>

#include <std_msgs/msg/int32.h>

#include "rcl_checks.h"

// Constants and Definitions
const size_t RING_BUFFER_SIZE = 1024;
const int SENSOR_TASK_PRIORITY = 2;
const int ROS_TASK_PRIORITY = 1;
const int RECONNECT_INTERVAL_MS =
    5000; // Changed to 5s for reconnection attempts
const int LED_PIN = 32;

// Updated State Enumeration for clarity
enum ConnectionState
{
    STATE_WAITING_AGENT,
    STATE_AGENT_AVAILABLE,
    STATE_AGENT_CONNECTED,
    STATE_AGENT_DISCONNECTED,
    STATE_LOW_POWER_MODE // New state for low power mode
};

// Shared Data Structures and Variables
volatile unsigned int ringBuffer[RING_BUFFER_SIZE];
unsigned int ringBufferIndex = 0;
SemaphoreHandle_t ringBufferSemaphore;
ConnectionState connectionState = STATE_WAITING_AGENT;

// ROS Entities
rcl_publisher_t int_publisher;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
std_msgs__msg__Int32 msg;

// Function Declarations
inline void writeToRingBuffer(unsigned int value);
inline unsigned int readFromRingBuffer();
void sensorTask(void* parameter);
void rosTask(void* parameter);
void attemptReconnection();

void setup()
{
    // ... (setup code remains unchanged)
}

inline void writeToRingBuffer(unsigned int value)
{
    // ... (function implementation remains unchanged)
}

inline unsigned int readFromRingBuffer()
{
    // ... (function implementation remains unchanged)
}

void sensorTask(void* parameter)
{
    // ... (function implementation remains unchanged)
}

void rosTask(void* parameter)
{
    for (;;)
    {
        switch (connectionState)
        {
            case STATE_WAITING_AGENT:
                // Logic for waiting agent state
                break;
            case STATE_AGENT_AVAILABLE:
                // Logic for agent available state
                break;
            case STATE_AGENT_CONNECTED:
                // Regular publishing logic
                unsigned int receivedValue = readFromRingBuffer();
                msg.data = receivedValue;
                rcl_publish(&int_publisher, &msg, NULL);
                // Check connection status
                if (rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)) !=
                    RCL_RET_OK)
                {
                    connectionState = STATE_AGENT_DISCONNECTED;
                }
                break;
            case STATE_AGENT_DISCONNECTED:
                // Transition to low power mode
                connectionState = STATE_LOW_POWER_MODE;
                break;
            case STATE_LOW_POWER_MODE:
                // Low power mode logic, attempt reconnection every 5s
                attemptReconnection();
                break;
        }
        esp_task_wdt_reset();                  // Reset watchdog timer
        vTaskDelay(1000 / portTICK_PERIOD_MS); // Delay for task frequency
    }
}

void attemptReconnection()
{
    static unsigned long lastAttemptTime = 0;
    if (millis() - lastAttemptTime > RECONNECT_INTERVAL_MS)
    {
        // Attempt to reconnect to the micro-ROS agent
        // Reconnection logic implementation
        lastAttemptTime = millis();
        connectionState =
            STATE_WAITING_AGENT; // Transition back to initial state
    }
}

void loop()
{
    // Since all logic is handled in tasks, loop can be empty or removed
}