// ros_manager.cpp
#include "ros_manager.h"
#include <Arduino.h>

ROSManager::ROSManager(const Config& config) : config_(config) {
    allocator_ = rcl_get_default_allocator();
    publishers_.reserve(config.max_publishers);
    subscriptions_.reserve(config.max_subscribers);
}

ROSManager::~ROSManager() {
    shutdown();
}

bool ROSManager::begin() {
    if (is_initialized_) {
        return true;
    }

    if (!initializeMicroROS()) {
        return false;
    }

    // Initialize ROS node
    rcl_ret_t ret = rclc_node_init_default(
        &node_,
        config_.node_name,
        config_.namespace_name,
        &support_
    );
    
    if (ret != RCL_RET_OK) {
        handleError(ret, "Failed to initialize ROS node");
        return false;
    }

    // Initialize executor
    ret = rclc_executor_init(
        &executor_,
        &support_.context,
        config_.max_subscribers + config_.max_services,  // Number of handles
        &allocator_
    );
    
    if (ret != RCL_RET_OK) {
        handleError(ret, "Failed to initialize executor");
        return false;
    }

    is_initialized_ = true;
    Serial.printf("ROS: Node '%s' initialized successfully\n", config_.node_name);
    return true;
}

bool ROSManager::initializeMicroROS() {
    Serial.println("ROS: Initializing micro-ROS...");
    
    if (!config_.wifi_ssid || !config_.wifi_password) {
        setError("WiFi credentials not provided");
        return false;
    }

    // Create non-const char buffers
    char ssid[32];  // Typical max SSID length
    char password[64];  // Typical max password length
    
    // Copy the strings
    strlcpy(ssid, config_.wifi_ssid, sizeof(ssid));
    strlcpy(password, config_.wifi_password, sizeof(password));
    
    set_microros_wifi_transports(
        ssid,
        password,
        config_.agent_ip,
        config_.agent_port
    );

    delay(2000);  // Give some time for transport to initialize

    rcl_ret_t ret = rclc_support_init(&support_, 0, NULL, &allocator_);
    if (ret != RCL_RET_OK) {
        handleError(ret, "Failed to initialize ROS support");
        return false;
    }

    return true;
}

void ROSManager::shutdown() {
    if (!is_initialized_) {
        return;
    }

    // Destroy all publishers
    for (auto& publisher : publishers_) {
        rcl_publisher_fini(&publisher, &node_);
    }
    publishers_.clear();

    // Destroy all subscriptions
    for (auto& subscription : subscriptions_) {
        rcl_subscription_fini(&subscription, &node_);
    }
    subscriptions_.clear();

    // Cleanup executor and node
    rclc_executor_fini(&executor_);
    rcl_node_fini(&node_);
    rclc_support_fini(&support_);

    is_initialized_ = false;
    Serial.println("ROS: Shutdown complete");
}

void ROSManager::spinOnce() {
    if (!is_initialized_) {
        return;
    }

    rcl_ret_t ret = rclc_executor_spin_some(&executor_, RCL_MS_TO_NS(config_.spin_timeout_ms));
    if (ret != RCL_RET_OK) {
        handleError(ret, "Executor spin failed");
    }
}

void ROSManager::spin(uint32_t timeout_ms) {
    if (!is_initialized_) {
        return;
    }

    unsigned long start_time = millis();
    while (millis() - start_time < timeout_ms) {
        spinOnce();
        delay(1);  // Prevent watchdog triggers
    }
}

bool ROSManager::checkConnection() const {
    if (!is_initialized_) {
        return false;
    }

    // Try to ping the ROS agent
    return rmw_uros_ping_agent(100, 3) == RMW_RET_OK;
}
const char* ROSManager::getError() const {
    return last_error_.c_str();
}

void ROSManager::handleError(rcl_ret_t ret, const char* context) {
    char error_buffer[128];
    snprintf(error_buffer, sizeof(error_buffer), "%s (error code: %d)", context, ret);
    last_error_ = error_buffer;
    Serial.println(error_buffer);
}

bool ROSManager::isInitialized() const {
    return is_initialized_;
}

void ROSManager::setError(const char* error) {
    last_error_ = error;
    Serial.println(error);
}
