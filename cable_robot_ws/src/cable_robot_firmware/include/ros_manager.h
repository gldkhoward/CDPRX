// ros_manager.h
#pragma once

#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <string>
#include <vector>

class ROSManager {
public:
    struct Config {
        const char* node_name;
        const char* namespace_name = "";
        const char* wifi_ssid;      // Add this
        const char* wifi_password;  // Add this
        IPAddress agent_ip;
        uint16_t agent_port;
        size_t max_publishers = 10;
        size_t max_subscribers = 10;
        size_t max_services = 5;
        uint32_t spin_timeout_ms = 100;
    };

    ROSManager() = default;
    explicit ROSManager(const Config& config);
    ~ROSManager();

    bool begin();
    void shutdown();
    bool isInitialized() const;
    
template<typename T>
rcl_publisher_t* createPublisher(const char* topic_name, const rosidl_message_type_support_t* type_support, size_t queue_size = 10) {
    if (!is_initialized_) {
        setError("Cannot create publisher - ROS not initialized");
        return nullptr;
    }

    if (publishers_.size() >= config_.max_publishers) {
        setError("Maximum number of publishers reached");
        return nullptr;
    }

    publishers_.emplace_back();
    rcl_publisher_t* publisher = &publishers_.back();

    rcl_ret_t ret = rclc_publisher_init_default(
        publisher,
        &node_,
        type_support,
        topic_name
    );

    if (ret != RCL_RET_OK) {
        publishers_.pop_back();
        handleError(ret, "Failed to create publisher");
        return nullptr;
    }

    return publisher;
}

template<typename T>
rcl_subscription_t* createSubscription(
    const char* topic_name,
    const rosidl_message_type_support_t* type_support,
    void (*callback)(const void*, void*),
    size_t queue_size = 10
) {
    if (!is_initialized_) {
        setError("Cannot create subscription - ROS not initialized");
        return nullptr;
    }

    if (subscriptions_.size() >= config_.max_subscribers) {
        setError("Maximum number of subscribers reached");
        return nullptr;
    }

    subscriptions_.emplace_back();
    rcl_subscription_t* subscription = &subscriptions_.back();

    rcl_ret_t ret = rclc_subscription_init_default(
        subscription,
        &node_,
        type_support,
        topic_name
    );

    if (ret != RCL_RET_OK) {
        subscriptions_.pop_back();
        handleError(ret, "Failed to create subscription");
        return nullptr;
    }

    return subscription;
}

template<typename T>
rcl_service_t* createService(
    const char* service_name,
    const rosidl_service_type_support_t* type_support,
    rclc_service_callback_t callback,  // Update callback type
    void* request_msg,                 // Add request message
    void* response_msg                 // Add response message
) {
    if (!is_initialized_) {
        setError("Cannot create service - ROS not initialized");
        return nullptr;
    }

    if (services_.size() >= config_.max_services) {
        setError("Maximum number of services reached");
        return nullptr;
    }

    services_.emplace_back();
    rcl_service_t* service = &services_.back();

    rcl_ret_t ret = rclc_service_init_default(
        service,
        &node_,
        type_support,
        service_name
    );

    if (ret != RCL_RET_OK) {
        services_.pop_back();
        handleError(ret, "Failed to create service");
        return nullptr;
    }

    // Add the service callback to the executor with all required parameters
    ret = rclc_executor_add_service(
        &executor_,
        service,
        request_msg,
        response_msg,
        callback
    );

    if (ret != RCL_RET_OK) {
        services_.pop_back();
        handleError(ret, "Failed to add service to executor");
        return nullptr;
    }

    return service;
}

    
    // Executor management
    void spinOnce();
    void spin(uint32_t timeout_ms = 100);
    
    // Status and diagnostics
    bool checkConnection() const;
    const char* getError() const;
    
private:
    Config config_;
    bool is_initialized_ = false;
    std::string last_error_;
    
    rcl_allocator_t allocator_;
    rclc_support_t support_;
    rcl_node_t node_;
    rclc_executor_t executor_;

    
    std::vector<rcl_publisher_t> publishers_;
    std::vector<rcl_subscription_t> subscriptions_;
    std::vector<rcl_service_t> services_;

    // Message storage for subscriptions
    std::vector<void*> subscription_msgs_;
    
    bool initializeMicroROS();
    void handleError(rcl_ret_t ret, const char* context);
    void setError(const char* error);
};