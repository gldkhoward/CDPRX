#include <Arduino.h>
#include "ros_manager.h"
#include <std_msgs/msg/string.h>


// ROS message for testing
std_msgs__msg__String test_msg;
rcl_publisher_t test_publisher;

// ROS manager instance
ROSManager* ros_manager;

void setup() {
    Serial.begin(115200);
    while (!Serial) {
        delay(100);
    }
    Serial.println("Starting ROS Test Program...");

    // Configure ROS Manager
    ROSManager::Config config;
    config.node_name = "test_node";
    config.namespace_name = "";
    config.wifi_ssid = "Irish Lad";
    config.wifi_password = "UtsLads23";
    config.agent_ip = IPAddress(192, 168, 1, 119);  // Replace with your ROS2 machine IP
    config.agent_port = 8888;
    config.max_publishers = 1;
    config.max_subscribers = 0;

    // Initialize ROS Manager
    ros_manager = new ROSManager(config);
    
    if (!ros_manager->begin()) {
        Serial.println("Failed to initialize ROS Manager!");
        return;
    }
    Serial.println("ROS Manager initialized successfully");

    // Create a test publisher
    test_msg.data.data = (char*)"Hello from ESP32!";
    test_msg.data.size = strlen((char*)test_msg.data.data);
    test_msg.data.capacity = test_msg.data.size + 1;
    test_publisher = *ros_manager->createPublisher<std_msgs__msg__String>(
        "test_topic", 
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        10
    );
    if (!&test_publisher) {
        Serial.println("Failed to create publisher!");
        return;
    }
    Serial.println("Publisher created successfully");
}

void loop() {
    static unsigned long last_pub_time = 0;
    const unsigned long PUB_PERIOD = 1000;  // Publish every second

    if (millis() - last_pub_time >= PUB_PERIOD) {
        rcl_ret_t ret = rcl_publish(&test_publisher, &test_msg, NULL);
        if (ret == RCL_RET_OK) {
            Serial.println("Published message successfully");
        } else {
            Serial.printf("Failed to publish message: %d\n", ret);
        }
        last_pub_time = millis();
    }

    // Let ROS manager handle its tasks
    ros_manager->spinOnce();
    delay(10);
}