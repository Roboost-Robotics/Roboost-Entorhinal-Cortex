/**
 * @file core.cpp
 * @author Friedl Jakob (friedl.jak@gmail.com)
 * @brief Core file handling ROS communication and sensor processing. Currently
 * supports the Roboost-V2 sensorshield hardware.
 * @version 0.1
 * @date 2023-07-06
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <Arduino.h>
#include <micro_ros_platformio.h>

#include "rcl_checks.h"
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>

#include <diagnostic_msgs/msg/diagnostic_status.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/laser_scan.h>
#include <std_msgs/msg/float32.h>

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <RPLidar.h>
#include <Wire.h>

#include "conf_hardware.h"
// #include "conf_network.h"

HardwareSerial RPLidarSerial(2);
RPLidar lidar;

Adafruit_MPU6050 mpu;

sensors_event_t a, g, temp;

rcl_publisher_t scan_publisher;
rcl_publisher_t battery_publisher;
rcl_publisher_t diagnostic_publisher;
rcl_publisher_t imu_publisher;
rcl_publisher_t temperature_publisher;
rcl_publisher_t delta_time_publisher;

sensor_msgs__msg__LaserScan scan_msg;
std_msgs__msg__Float32 battery_msg;
diagnostic_msgs__msg__DiagnosticStatus diagnostic_msg;
sensor_msgs__msg__Imu imu_msg;
std_msgs__msg__Float32 temperature_msg;
std_msgs__msg__Float32 delta_time_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// Variables for publishing battery level and temperature
unsigned long last_infrequent_publish_time = 0;
const unsigned long infrequent_publish_interval = 2000;

// Variables for calculating scan time
unsigned long scan_start_time = 0;

// Define global constants for maximum measurements and measurement buffers
const size_t max_measurements = 500;
float ranges_buffer[max_measurements];
float intensities_buffer[max_measurements];
size_t num_measurements = 0;

// Time synchronization variables
unsigned long last_time_sync_ms = 0;
unsigned long last_time_sync_ns = 0;
unsigned long time_sync_interval = 1000; // Sync timeout
const int timeout_ms = 500;
int64_t synced_time_ms = 0;
int64_t synced_time_ns = 0;

unsigned long last_time = 0;

void publishDiagnosticMessage(const char* message)
{
    diagnostic_msg.level = diagnostic_msgs__msg__DiagnosticStatus__WARN;
    diagnostic_msg.message.data = (char*)message;
    diagnostic_msg.message.size = strlen(diagnostic_msg.message.data);
    diagnostic_msg.message.capacity = diagnostic_msg.message.size + 1;

    RCSOFTCHECK(rcl_publish(&diagnostic_publisher, &diagnostic_msg, NULL));
}

void setup()
{
    // Initialize serial and lidar
    Serial.begin(115200);
    RPLidarSerial.begin(115200, SERIAL_8N1, RPLIDAR_RX, RPLIDAR_TX);

    // Initialize the lidar's motor control pin
    pinMode(RPLIDAR_MOTOR, OUTPUT);

    // Initialize battery voltage measurement pin and LED
    pinMode(PWR_IN, INPUT);
    pinMode(PWR_LED, OUTPUT);

    // Initialize the status LED
    pinMode(LED_BUILTIN, OUTPUT);

    // Initialize the lidar
    lidar.begin(RPLidarSerial);
    lidar.startScan();

    // Initialize the IMU
    while (!mpu.begin())
    {
        // Serial.println("Failed to find MPU6050 chip");
        digitalWrite(LED_BUILTIN, HIGH);
        delay(100);
        digitalWrite(LED_BUILTIN, LOW);
        delay(100);
    }
    mpu.enableSleep(false);
    mpu.enableCycle(false);

    // Configure micro-ROS
    // IPAddress agent_ip(AGENT_IP);
    // uint16_t agent_port = AGENT_PORT;

    set_microros_serial_transports(Serial);

    // set_microros_wifi_transports((char*)SSID, (char*)SSID_PW, agent_ip,
    //                              agent_port);
    delay(2000);

    allocator = rcl_get_default_allocator();

    // clang-format off
    RCCHECK(rclc_support_init(
        &support,
        0,
        NULL,
        &allocator));
    RCCHECK(rclc_node_init_default(
        &node,
        "lidar_node",
        "",
        &support));
    RCCHECK(rclc_publisher_init_default(
        &scan_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan),
        "scan"));
    RCCHECK(rclc_publisher_init_default(
        &battery_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "battery_level"));
    RCCHECK(rclc_publisher_init_default(
        &diagnostic_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(diagnostic_msgs, msg, DiagnosticStatus),
        "diagnostics"));
    RCCHECK(rclc_publisher_init_default(
        &imu_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "imu"));
    RCCHECK(rclc_publisher_init_default(
        &temperature_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "temperature"));
    RCCHECK(rclc_publisher_init_default(
        &delta_time_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "delta_time/EC"));
    RCCHECK(rclc_executor_init(
        &executor,
        &support.context,
        1,
        &allocator));
    // clang-format on

    delay(500);
    digitalWrite(LED_BUILTIN, HIGH);
    digitalWrite(RPLIDAR_MOTOR, HIGH);

    // Initialize scan message
    // Set frame_id
    scan_msg.header.frame_id.data = (char*)"lidar_link";
    scan_msg.header.frame_id.size = strlen(scan_msg.header.frame_id.data);
    scan_msg.header.frame_id.capacity = scan_msg.header.frame_id.size + 1;

    scan_msg.range_min = 0.15;
    scan_msg.range_max = 16.0;

    // Allocate memory for scan message data
    scan_msg.ranges.data = ranges_buffer;
    scan_msg.ranges.size = 0;
    scan_msg.ranges.capacity = max_measurements;
    scan_msg.intensities.data = intensities_buffer;
    scan_msg.intensities.size = 0;
    scan_msg.intensities.capacity = max_measurements;

    // Initialize IMU message
    imu_msg.header.frame_id.data = (char*)"imu_link";
    imu_msg.header.frame_id.size = strlen(imu_msg.header.frame_id.data);
    imu_msg.header.frame_id.capacity = imu_msg.header.frame_id.size + 1;

    double* temp_covariance = (double*)malloc(9 * sizeof(double));

    for (int i = 0; i < 9; i++)
    {
        if (i % 4 != 0)
        {
            temp_covariance[i] = 0.0;
        }
    }

    temp_covariance[0] = 0.0025;
    temp_covariance[4] = 0.0025;
    temp_covariance[8] = 0.0025;

    memcpy(imu_msg.orientation_covariance, temp_covariance, 9 * sizeof(double));

    temp_covariance[0] = 0.02;
    temp_covariance[4] = 0.02;
    temp_covariance[8] = 0.02;

    memcpy(imu_msg.angular_velocity_covariance, temp_covariance,
           9 * sizeof(double));

    temp_covariance[0] = 0.04;
    temp_covariance[4] = 0.04;
    temp_covariance[8] = 0.04;

    memcpy(imu_msg.linear_acceleration_covariance, temp_covariance,
           9 * sizeof(double));

    free(temp_covariance);
}

void loop()
{
    // Calculate the delta time
    unsigned long now = millis();
    double dt = (now - last_time) / 1000.0;
    last_time = now;

    // Publish the delta time
    delta_time_msg.data = dt;
    RCSOFTCHECK(rcl_publish(&delta_time_publisher, &delta_time_msg, NULL));

    // Time synchronization
    if (millis() - last_time_sync_ms > time_sync_interval)
    {
        // Synchronize time with the agent
        rmw_uros_sync_session(timeout_ms);

        if (rmw_uros_epoch_synchronized())
        {
            // Get time in milliseconds or nanoseconds
            synced_time_ms = rmw_uros_epoch_millis();
            synced_time_ns = rmw_uros_epoch_nanos();
            last_time_sync_ms = millis();
            last_time_sync_ns = micros() * 1000;
        }
    }

    // ------------------- IMU -------------------

    mpu.getEvent(&a, &g, &temp);

    imu_msg.header.stamp.sec =
        (synced_time_ms + millis() - last_time_sync_ms) / 1000;
    imu_msg.header.stamp.nanosec =
        synced_time_ns + (micros() * 1000 - last_time_sync_ns);
    imu_msg.header.stamp.nanosec %= 1000000000;

    imu_msg.orientation.x = 0.0;
    imu_msg.orientation.y = 0.0;
    imu_msg.orientation.z = 0.0;
    imu_msg.orientation.w = 1.0;

    imu_msg.angular_velocity.x = g.gyro.x;
    imu_msg.angular_velocity.y = g.gyro.y;
    imu_msg.angular_velocity.z = g.gyro.z;

    imu_msg.linear_acceleration.x = a.acceleration.x;
    imu_msg.linear_acceleration.y = a.acceleration.y;
    imu_msg.linear_acceleration.z = a.acceleration.z;

    RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));

    // Publish battery level and temperature every 2 seconds
    unsigned long current_time = millis();
    if (current_time - last_infrequent_publish_time >=
        infrequent_publish_interval)
    {

        // ------------------- BATTERY -------------------

        float battery_voltage =
            analogRead(PWR_IN) * (3.3 * PWR_FACTOR / 4095.0);

        if (battery_voltage < 0.85 * 12.4) // 12.4 * 0.85 = 10.54 V
        {
            digitalWrite(PWR_LED, HIGH);
            publishDiagnosticMessage("Low battery");
        }
        else
        {
            digitalWrite(PWR_LED, LOW);
        }

        battery_msg.data = battery_voltage;
        RCSOFTCHECK(rcl_publish(&battery_publisher, &battery_msg, NULL));
        last_infrequent_publish_time = current_time; // Update last publish time

        // ------------------- TEMPERATURE -------------------

        temperature_msg.data = temp.temperature;
        RCSOFTCHECK(
            rcl_publish(&temperature_publisher, &temperature_msg, NULL));
    }

    // ------------------- LIDAR -------------------

    if (lidar.isOpen())
    {
        std::vector<RPLidarMeasurement> measurements;

        // Read lidar measurements
        while (true)
        {
            lidar.waitPoint();
            if (lidar.getCurrentPoint().startBit)
            {
                scan_start_time = millis();
                break; // Start bit detected
            }
        }

        do
        {
            measurements.push_back(lidar.getCurrentPoint());
            lidar.waitPoint();
        } while (!lidar.getCurrentPoint().startBit);

        if (measurements.size() >= 200)
        {
            unsigned long scan_end_time = millis();

            // Order the measurements by angle.
            std::sort(
                measurements.begin(), measurements.end(),
                [](const RPLidarMeasurement& a, const RPLidarMeasurement& b)
                { return a.angle < b.angle; });

            // Update scan message data
            num_measurements = measurements.size();
            scan_msg.angle_min = measurements.front().angle * DEG_TO_RAD;
            scan_msg.angle_max = measurements.back().angle * DEG_TO_RAD;
            scan_msg.angle_increment =
                (scan_msg.angle_max - scan_msg.angle_min) /
                (num_measurements - 1);

            // Populate scan message data
            uint32_t num_partially_invalid_measurements = 0;
            uint32_t num_invalid_measurements = 0;
            for (size_t i = 0; i < num_measurements; i++)
            {
                if (measurements[i].quality == 0)
                    num_invalid_measurements++;
                else if (measurements[i].quality < 14)
                    num_partially_invalid_measurements++;

                scan_msg.ranges.data[i] = measurements[i].distance / 1000.0;
                scan_msg.intensities.data[i] = measurements[i].quality;
            }

            u_int32_t num_valid_measurements =
                num_measurements - num_invalid_measurements;

            // Publish number of different measurement types
            String msg =
                "Valid measurements: " + String(num_valid_measurements) + "/" +
                String(num_measurements);
            publishDiagnosticMessage(msg.c_str());

            // Update scan message size
            scan_msg.ranges.size = num_measurements;
            scan_msg.intensities.size = num_measurements;

            if (num_partially_invalid_measurements / num_valid_measurements <
                0.1) // TODO: Make this a parameter
            {
                scan_msg.header.stamp.sec =
                    (synced_time_ms + millis() - last_time_sync_ms) / 1000;
                scan_msg.header.stamp.nanosec =
                    synced_time_ns + (micros() * 1000 - last_time_sync_ns);
                scan_msg.header.stamp.nanosec %= 1000000000;

                scan_msg.scan_time = (scan_end_time - scan_start_time) / 1000.0;
                scan_msg.time_increment = scan_msg.scan_time / num_measurements;

                RCSOFTCHECK(rcl_publish(&scan_publisher, &scan_msg, NULL));
            }
            else
            {
                String msg = "Too many invalid measurements: " +
                             String(num_invalid_measurements) + "/" +
                             String(num_measurements);
                publishDiagnosticMessage(msg.c_str());
            }
        }
    }
    else
    {
        publishDiagnosticMessage("Lidar not connected");
    }

    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}