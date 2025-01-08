/**
 * @file encoder.h
 * @brief Header file for quadrature encoder component
 */

#pragma once

#include "component_base.h"
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <std_msgs/msg/float32.h>
#include <std_srvs/srv/trigger.h>
#include <Arduino.h>

class Encoder : public ComponentBase {
public:
    struct EncoderConfig : ComponentConfig {
        int pin_a;                          // First encoder pin
        int pin_b;                          // Second encoder pin
        bool reverse_direction = false;      // Reverse counting direction if true
        int counts_per_revolution = 360;     // Encoder counts per full revolution
        int publish_rate_hz = 10;           // Rate to publish encoder ticks
    };

    Encoder();
    virtual ~Encoder() = default;

    // ComponentBase interface implementation
    bool configure(const ComponentConfig& config) override;
    bool init() override;
    void update() override;
    void shutdown() override;
    const char* getId() const override;
    std::string getStatus() const override;
    void publishDiagnostics() override;
    void getType() override;

    // Encoder-specific methods
    long getRawTicks() const { return raw_ticks_; }
    long getRelativeTicks() const { return relative_ticks_; }
    void zeroPosition();
    float getAngleDegrees() const;

    // Static instance pointer for interrupt handling
    static Encoder* instance_;

private:
    EncoderConfig config_;
    std::string status_;
    volatile long raw_ticks_ = 0;      // Total ticks from start
    volatile long relative_ticks_ = 0;  // Ticks since last zero
    volatile int last_a_ = 0;
    volatile int last_b_ = 0;
    unsigned long last_publish_time_ = 0; 

    rcl_service_t* zero_service_ = nullptr;

    rcl_publisher_t* abs_position_publisher_ = nullptr;
    rcl_publisher_t* rel_position_publisher_ = nullptr;
    rcl_publisher_t* total_ticks_publisher_ = nullptr;
    rcl_publisher_t* rel_ticks_publisher_ = nullptr;
    
    std_msgs__msg__Float32 abs_position_msg_;
    std_msgs__msg__Float32 rel_position_msg_;
    std_msgs__msg__Float32 total_ticks_msg_;
    std_msgs__msg__Float32 rel_ticks_msg_;
    

    // Static interrupt handlers
    static void IRAM_ATTR handleInterruptA();
    static void IRAM_ATTR handleInterruptB();
    
    // Instance methods for actual interrupt handling
    void handleA();
    void handleB();
    void publishPosition();

    static constexpr unsigned long DEBOUNCE_TIME = 1; // 1ms debounce
    unsigned long last_interrupt_time_a_ = 0;
    unsigned long last_interrupt_time_b_ = 0;

    volatile int last_encoded_ = 0;
    static constexpr int PPR = 1000;  // Pulses per revolution

    std_srvs__srv__Trigger_Request trigger_request_;
    std_srvs__srv__Trigger_Response trigger_response_;

    // Service callback
    static void handleZeroServiceRequest(
    const std_srvs__srv__Trigger_Request* req,
    std_srvs__srv__Trigger_Response* res
    
);
};