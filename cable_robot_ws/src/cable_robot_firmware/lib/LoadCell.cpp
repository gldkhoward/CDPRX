#include <WiFi.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>
#include <micro_ros_platformio.h>
#include "HX711.h"
#include "credentials.h"

// HX711 circuit wiring
const int LOADCELL_DOUT_PIN = 26;
const int LOADCELL_SCK_PIN = 25;

// Load cell variables
HX711 scale;
float calibration_factor = 1;  // Adjust based on your calibration
float zero_factor = 0;

// ROS objects
rcl_publisher_t publisher;
std_msgs__msg__Float32 msg;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t executor;

// Function to convert raw value to Newtons
float convertToNewtons(long raw_value) {
  return (raw_value - zero_factor) / calibration_factor;
}

bool micro_ros_init() {
  Serial.println("Initializing micro-ROS...");
  
  IPAddress agent_ip(AGENT_IP);  // Define this in credentials.h
  const uint16_t agent_port = AGENT_PORT;  // Define this in credentials.h

  char ssid[] = WIFI_SSID;  // Define this in credentials.h
  char psk[] = WIFI_PASSWORD;  // Define this in credentials.h

  Serial.print("Connecting to WiFi");
  set_microros_wifi_transports(ssid, psk, agent_ip, agent_port);

  allocator = rcl_get_default_allocator();

  // Create init_options and support object
  rcl_ret_t ret = rclc_support_init(&support, 0, NULL, &allocator);
  if (ret != RCL_RET_OK) {
    Serial.println("Failed to initialize support");
    return false;
  }

  // Create node
  ret = rclc_node_init_default(&node, "load_cell_node", "", &support);
  if (ret != RCL_RET_OK) {
    Serial.println("Failed to initialize node");
    return false;
  }

  // Create publisher
  ret = rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "force_readings"
  );
  if (ret != RCL_RET_OK) {
    Serial.println("Failed to initialize publisher");
    return false;
  }

  // Initialize executor
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
  
  // Initialize load cell
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  Serial.println("Initializing scale...");
  scale.set_scale();
  scale.tare();
  zero_factor = scale.read_average();
  
  // Initialize micro-ROS
  while (!micro_ros_init()) {
    Serial.println("micro-ROS init failed! Retrying in 1 second...");
    delay(1000);
  }
}

void loop() {
  if (scale.is_ready()) {
    long raw_value = scale.get_value(5);  // Average of 5 readings
    float force_n = convertToNewtons(raw_value);
    
    // Update message and publish
    msg.data = force_n;
    rcl_ret_t ret = rcl_publish(&publisher, &msg, NULL);
    
    if (ret == RCL_RET_OK) {
      // Debug output
      Serial.print("Raw: ");
      Serial.print(raw_value);
      Serial.print("\tForce (N): ");
      Serial.println(force_n, 3);
    } else {
      Serial.println("Publishing failed!");
    }
  }
  
  // Spin executor
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
  delay(100);  // 10Hz publishing rate
}