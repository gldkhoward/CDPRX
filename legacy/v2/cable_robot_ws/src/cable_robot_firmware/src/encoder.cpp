#include "encoder.h"
#include <Arduino.h>
#include "board_base.h"
#include <std_srvs/srv/trigger.h>
#include <std_srvs/srv/detail/trigger__struct.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <std_msgs/msg/float32.h>
#include <std_srvs/srv/trigger.h>
#include <Arduino.h>

// Initialize static instance pointer
Encoder* Encoder::instance_ = nullptr;

Encoder::Encoder() : ComponentBase() {
    type_ = "encoder";
    instance_ = this;
}

bool Encoder::configure(const ComponentConfig& base_config) {
    const auto& config = static_cast<const EncoderConfig&>(base_config);
    config_ = config;
    id_ = config.id;
    
    pinMode(config_.pin_a, INPUT_PULLUP);
    pinMode(config_.pin_b, INPUT_PULLUP);
    
    return true;
}

bool Encoder::init() {
    auto& ros_manager = board_->getROSManager();
    std::string base_topic = std::string(getId());
    
    abs_position_publisher_ = ros_manager.createPublisher<std_msgs__msg__Float32>(
        (base_topic + "/absolute_position").c_str(),
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        10
    );
    
    rel_position_publisher_ = ros_manager.createPublisher<std_msgs__msg__Float32>(
        (base_topic + "/relative_position").c_str(),
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        10
    );
    
    total_ticks_publisher_ = ros_manager.createPublisher<std_msgs__msg__Float32>(
        (base_topic + "/total_ticks").c_str(),
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        10
    );
    
    rel_ticks_publisher_ = ros_manager.createPublisher<std_msgs__msg__Float32>(
        (base_topic + "/relative_ticks").c_str(),
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        10
    );

    // Initialize all messages
    abs_position_msg_.data = 0.0f;
    rel_position_msg_.data = 0.0f;
    total_ticks_msg_.data = 0.0f;
    rel_ticks_msg_.data = 0.0f;
    
    // Create zero service
   zero_service_ = ros_manager.createService<std_srvs__srv__Trigger_Request>(
    (base_topic + "/zero").c_str(),
    ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger),
    reinterpret_cast<rclc_service_callback_t>(handleZeroServiceRequest),
    static_cast<void*>(&trigger_request_),
    static_cast<void*>(&trigger_response_)
);
    
    
    // Attach interrupt handlers
    attachInterrupt(digitalPinToInterrupt(config_.pin_a), handleInterruptA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(config_.pin_b), handleInterruptB, CHANGE);
    
    last_publish_time_ = millis();
    status_ = "OK";
    return true;
}

void Encoder::update() {
    if (millis() - last_publish_time_ >= (1000 / config_.publish_rate_hz)) {
        publishPosition();
        last_publish_time_ = millis();
    }
}

void Encoder::shutdown() {
    detachInterrupt(digitalPinToInterrupt(config_.pin_a));
    detachInterrupt(digitalPinToInterrupt(config_.pin_b));
}

float Encoder::getAngleDegrees() const {
    // Calculate angle considering quadrature encoding (4x PPR)
    float angle = (360.0f * relative_ticks_) / (4 * PPR);
    // Normalize to 0-360 range
    while (angle >= 360.0f) angle -= 360.0f;
    while (angle < 0.0f) angle += 360.0f;
    return angle;
}

void Encoder::zeroPosition() {
    relative_ticks_ = 0;
}

void IRAM_ATTR Encoder::handleInterruptA() {
    if (instance_) {
        instance_->handleA();
    }
}

void IRAM_ATTR Encoder::handleInterruptB() {
    if (instance_) {
        instance_->handleB();
    }
}

void Encoder::handleA() {
    // Single interrupt handler for both pins
    int msb = digitalRead(config_.pin_a);
    int lsb = digitalRead(config_.pin_b);

    int encoded = (msb << 1) | lsb;
    int sum = (last_encoded_ << 2) | encoded;

    // State machine for quadrature decoding
    if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
        raw_ticks_++;
        relative_ticks_++;
    }
    if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
        raw_ticks_--;
        relative_ticks_--;
    }

    last_encoded_ = encoded;
}

void Encoder::handleB() {
    // Use same handler as A for consistency
    handleA();
}



void Encoder::publishPosition() {
    if (!abs_position_publisher_ || !rel_position_publisher_ || 
        !total_ticks_publisher_ || !rel_ticks_publisher_) {
        return;
    }

    // Calculate positions
    float abs_angle = (360.0f * raw_ticks_) / (4 * config_.counts_per_revolution);
    while (abs_angle >= 360.0f) abs_angle -= 360.0f;
    while (abs_angle < 0.0f) abs_angle += 360.0f;

    float rel_angle = (360.0f * relative_ticks_) / (4 * config_.counts_per_revolution);

    // Update messages
    abs_position_msg_.data = abs_angle;
    rel_position_msg_.data = rel_angle;
    total_ticks_msg_.data = static_cast<float>(raw_ticks_);
    rel_ticks_msg_.data = static_cast<float>(relative_ticks_);

    // Publish all messages
    rcl_publish(abs_position_publisher_, &abs_position_msg_, NULL);
    rcl_publish(rel_position_publisher_, &rel_position_msg_, NULL);
    rcl_publish(total_ticks_publisher_, &total_ticks_msg_, NULL);
    rcl_publish(rel_ticks_publisher_, &rel_ticks_msg_, NULL);
}

void Encoder::handleZeroServiceRequest(
    const std_srvs__srv__Trigger_Request* req,
    std_srvs__srv__Trigger_Response* res
) {
    if (instance_) {
        instance_->zeroPosition();
        res->success = true;
        res->message.data = const_cast<char*>("Position zeroed successfully");
        res->message.size = strlen(res->message.data);
    } else {
        res->success = false;
        res->message.data = const_cast<char*>("Encoder instance not available");
        res->message.size = strlen(res->message.data);
    }
}

const char* Encoder::getId() const {
    return id_;
}

std::string Encoder::getStatus() const {
    return status_;
}

void Encoder::publishDiagnostics() {
    // TODO: Implement diagnostic message publishing
}

void Encoder::getType() {
    // Return the component type
}