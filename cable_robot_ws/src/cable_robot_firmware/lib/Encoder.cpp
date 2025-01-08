//File for holding the a temporary version of an alternate main file 

#include <WiFi.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <micro_ros_platformio.h>
#include "credentials.h"

// Encoder pins - connect A (white) to GPIO 25 and B (green) to GPIO 26
const int ENCODER_PIN_A = 25;
const int ENCODER_PIN_B = 26;

// Encoder variables
volatile long encoder_count = 0;
volatile int lastEncoded = 0;
const int PPR = 1000;  // Pulses per revolution

// ROS objects
rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t executor;

void IRAM_ATTR handleEncoder() {
    int MSB = digitalRead(ENCODER_PIN_A);  // MSB = most significant bit
    int LSB = digitalRead(ENCODER_PIN_B);  // LSB = least significant bit

    int encoded = (MSB << 1) | LSB;  // Converting the 2 pin values to single number
    int sum = (lastEncoded << 2) | encoded;

    if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoder_count++;
    if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoder_count--;

    lastEncoded = encoded;
}

bool micro_ros_init() {
    Serial.println("Initializing micro-ROS...");
    
    IPAddress agent_ip(AGENT_IP);
    const uint16_t agent_port = AGENT_PORT;

    char ssid[] = WIFI_SSID;
    char psk[] = WIFI_PASSWORD;

    Serial.print("Connecting to WiFi");
    set_microros_wifi_transports(ssid, psk, agent_ip, agent_port);

    allocator = rcl_get_default_allocator();

    rcl_ret_t ret = rclc_support_init(&support, 0, NULL, &allocator);
    if (ret != RCL_RET_OK) {
        Serial.println("Failed to initialize support");
        return false;
    }

    ret = rclc_node_init_default(&node, "encoder_node", "", &support);
    if (ret != RCL_RET_OK) {
        Serial.println("Failed to initialize node");
        return false;
    }

    ret = rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "encoder_ticks"
    );
    if (ret != RCL_RET_OK) {
        Serial.println("Failed to initialize publisher");
        return false;
    }

    ret = rclc_executor_init(&executor, &support.context, 1, &allocator);
    if (ret != RCL_RET_OK) {
        Serial.println("Failed to initialize executor");
        return false;
    }

    Serial.println("micro-ROS initialized successfully");
    return true;
}

void setup() {
    Serial.begin(115200);
    while (!Serial) {
        delay(100);
    }
    
    // Initialize encoder pins
    pinMode(ENCODER_PIN_A, INPUT_PULLUP);
    pinMode(ENCODER_PIN_B, INPUT_PULLUP);
    
    // Attach interrupts
    attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), handleEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_B), handleEncoder, CHANGE);
    
    // Initialize micro-ROS
    while (!micro_ros_init()) {
        Serial.println("micro-ROS init failed! Retrying in 1 second...");
        delay(1000);
    }
}

void loop() {
    // Update message and publish
    msg.data = encoder_count;
    rcl_ret_t ret = rcl_publish(&publisher, &msg, NULL);
    
    if (ret == RCL_RET_OK) {
        // Debug output
        Serial.print("Encoder ticks: ");
        Serial.print(encoder_count);
        Serial.print("\tAngle (degrees): ");
        Serial.println(360.0 * encoder_count / (4 * PPR), 2);
    } else {
        Serial.println("Publishing failed!");
    }
    
    // Spin executor
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    delay(100);  // 10Hz publishing rate
}