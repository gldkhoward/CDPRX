#include <Arduino.h>
#include "board_base.h"
#include "encoder.h"
#include <memory>

// Global board instance
std::unique_ptr<BoardBase> board;

template<typename T, typename... Args>
std::unique_ptr<T> make_unique(Args&&... args) {
    return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
}

void setup() {
    Serial.begin(115200);
    while (!Serial) {
        delay(100);
    }
    Serial.println("Starting Encoder Test Program...");

    // Configure ROS Manager
    ROSManager::Config ros_config;
    ros_config.node_name = "encoder_node";
    ros_config.namespace_name = "";
    ros_config.wifi_ssid = "Irish Lad";
    ros_config.wifi_password = "UtsLads23";
    ros_config.agent_ip = IPAddress(192, 168, 1, 119);  // Replace with your ROS2 machine IP
    ros_config.agent_port = 8888;
    ros_config.max_publishers = 4;
    ros_config.max_subscribers = 4;

    // Configure board
    BoardBase::Config board_config;
    board_config.board_name = "encoder_board";
    board_config.ros_config = ros_config;
    board_config.watchdog_timeout_ms = 5000;

    // Create board
    board = make_unique<BoardBase>(board_config);

    // Create and configure encoder
    auto encoder = std::make_shared<Encoder>();
    Encoder::EncoderConfig encoder_config;
    encoder_config.id = "encoder1";
    encoder_config.type = "encoder";
    encoder_config.pin_a = 26;                    // Replace with your encoder pins
    encoder_config.pin_b = 27;                    // Replace with your encoder pins
    encoder_config.reverse_direction = false;
    encoder_config.counts_per_revolution = 1000;
    encoder_config.publish_rate_hz = 50;
    encoder_config.enabled = true;

    // Configure encoder and add to board
    if (!encoder->configure(encoder_config)) {
        Serial.println("Failed to configure encoder!");
        return;
    }

    if (!board->addComponent(encoder)) {
        Serial.println("Failed to add encoder to board!");
        return;
    }

    // Initialize board
    if (!board->begin()) {
        Serial.println("Failed to initialize board!");
        return;
    }

    Serial.println("Board and encoder initialized successfully!");
}

void loop() {
    if (board) {
        board->update();
        
        // Print status every second
        static unsigned long last_status_time = 0;
        if (millis() - last_status_time >= 1000) {
            auto encoder = board->getComponent("encoder1");
            if (encoder) {
                Serial.printf("Board Status: %s\n", board->getStatus().c_str());
                Serial.printf("Encoder Status: %s\n", encoder->getStatus().c_str());
            }
            last_status_time = millis();
        }
    }
    
    delay(10);  // Small delay to prevent overwhelming the system
}