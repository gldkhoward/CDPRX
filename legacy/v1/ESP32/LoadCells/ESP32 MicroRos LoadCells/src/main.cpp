#include <micro_ros_arduino.h>
#include <WiFi.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>
#include "HX711.h"
#include "credentials.h"

// HX711 circuit wiring
const int LOADCELL_DOUT_PIN = 21;
const int LOADCELL_SCK_PIN = 19;

// Load cell variables
HX711 scale;
float calibration_factor = -1;  // Adjust based on your calibration
float zero_factor = 0;

// ROS objects
rcl_publisher_t publisher;
std_msgs__msg__Float32 msg;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// Function to convert raw value to Newtons
float convertToNewtons(long raw_value) {
  return (raw_value - zero_factor) / calibration_factor;
}

void setup() {
  Serial.begin(115200);
  
  // Initialize load cell
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  Serial.println("Initializing scale...");
  scale.set_scale();
  scale.tare();
  zero_factor = scale.read_average();
  
  // Connect to WiFi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi Connected!");

  // Initialize micro-ROS
  set_microros_wifi_transports(
    WIFI_SSID,
    WIFI_PASSWORD,
    AGENT_IP_ADDR,
    AGENT_PORT
  );
  
  allocator = rcl_get_default_allocator();
  
  // Create init_options and support object
  rclc_support_init(&support, 0, NULL, &allocator);

  // Create node
  rclc_node_init_default(&node, "load_cell_node", "", &support);

  // Create publisher
  rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "force_readings");

  Serial.println("ROS2 publisher initialized");
}

void loop() {
  if (scale.is_ready()) {
    long raw_value = scale.get_value(5);  // Average of 5 readings
    float force_n = convertToNewtons(raw_value);
    
    // Update message and publish
    msg.data = force_n;
    rcl_ret_t ret = rcl_publish(&publisher, &msg, NULL);
    if (ret != RCL_RET_OK) {
      Serial.println("Publishing failed!");
    }
    
    // Debug output
    Serial.print("Raw: ");
    Serial.print(raw_value);
    Serial.print("\tForce (N): ");
    Serial.println(force_n, 3);
  }
  
  delay(100);  // 10Hz publishing rate
}