#include "board_base.h"
#include "component_base.h"
#include <algorithm>

BoardBase::BoardBase(const Config& config)
    : config_(config)
    , ros_manager_(config.ros_config) {
}

BoardBase::~BoardBase() {
    shutdown();
}

bool BoardBase::begin() {
    if (is_initialized_) {
        return true;
    }

    // Initialize ROS manager first
    if (!ros_manager_.begin()) {
        setError("Failed to initialize ROS manager");
        return false;
    }

    // Setup watchdog
    setupWatchdog();

    // Initialize all components
    bool all_components_initialized = true;
    for (auto& component : components_) {
    if (!component->init()) {
        std::string error_msg = "Failed to initialize component: " + std::string(component->getId());
        setError(error_msg.c_str());
        all_components_initialized = false;
        break;
    }
}

    if (!all_components_initialized) {
        shutdown();
        return false;
    }

    // Call board-specific initialization
    if (!onInit()) {
        setError("Board-specific initialization failed");
        shutdown();
        return false;
    }

    is_initialized_ = true;
    clearError();
    return true;
}

void BoardBase::update() {
    if (!is_initialized_) {
        return;
    }

    // Feed watchdog
    feedWatchdog();

    // Update ROS
    ros_manager_.spinOnce();

    // Update components
    updateComponents();

    // Call board-specific update
    onUpdate();
}

void BoardBase::shutdown() {
    if (!is_initialized_) {
        return;
    }

    // Call board-specific shutdown
    onShutdown();

    // Shutdown all components
    for (auto& component : components_) {
        component->shutdown();
    }

    // Shutdown ROS manager
    ros_manager_.shutdown();

    // Clear watchdog
    if (watchdog_timer_) {
        timerEnd(watchdog_timer_);
        watchdog_timer_ = nullptr;
    }

    is_initialized_ = false;
}

bool BoardBase::addComponent(std::shared_ptr<ComponentBase> component) {
    if (!validateComponent(component)) {
        return false;
    }

    // Check for duplicate IDs
    auto it = std::find_if(components_.begin(), components_.end(),
        [&](const std::shared_ptr<ComponentBase>& existing) {
            return strcmp(existing->getId(), component->getId()) == 0;
        });

    if (it != components_.end()) {
        setError("Component with ID already exists");
        return false;
    }

    // Set board pointer and add to list
    component->board_ = this;
    components_.push_back(component);

    // Initialize component if board is already initialized
    if (is_initialized_) {
        if (!component->init()) {
            components_.pop_back();
            setError("Failed to initialize new component");
            return false;
        }
    }

    return true;
}

void BoardBase::removeComponent(const std::string& component_id) {
    auto it = std::find_if(components_.begin(), components_.end(),
        [&](const std::shared_ptr<ComponentBase>& component) {
            return component->getId() == component_id;
        });

    if (it != components_.end()) {
        (*it)->shutdown();
        components_.erase(it);
    }
}

std::shared_ptr<ComponentBase> BoardBase::getComponent(const std::string& component_id) {
    auto it = std::find_if(components_.begin(), components_.end(),
        [&](const std::shared_ptr<ComponentBase>& component) {
            return component->getId() == component_id;
        });

    return (it != components_.end()) ? *it : nullptr;
}

bool BoardBase::isHealthy() const {
    if (!is_initialized_ || !last_error_.empty()) {
        return false;
    }

    // Check ROS connection
    if (!ros_manager_.checkConnection()) {
        return false;
    }

    // Check component health
    for (const auto& component : components_) {
        if (component->getStatus().find("ERROR") != std::string::npos) {
            return false;
        }
    }

    return true;
}

String BoardBase::getStatus() const {
    if (!is_initialized_) {
        return "Not initialized";
    }
    return last_error_.empty() ? "OK" : last_error_.c_str();
}

void BoardBase::publishDiagnostics() {
    // Publish board-level diagnostics
    // TODO: Implement diagnostic message publishing

    // Publish component diagnostics
    for (auto& component : components_) {
        component->publishDiagnostics();
    }
}

void BoardBase::setError(const char* error) {
    last_error_ = error;
    Serial.printf("Board error: %s\n", error);
}

void BoardBase::clearError() {
    last_error_.clear();
}

void BoardBase::setupWatchdog() {
    if (config_.watchdog_timeout_ms > 0) {
        watchdog_timer_ = timerBegin(0, 80, true);  // Timer 0, prescaler 80, count up
        timerAttachInterrupt(watchdog_timer_, [](){
            ESP.restart();
        }, true);
        timerAlarmWrite(watchdog_timer_, config_.watchdog_timeout_ms * 1000, false);
        timerAlarmEnable(watchdog_timer_);
    }
}

void BoardBase::feedWatchdog() {
    if (watchdog_timer_) {
        timerWrite(watchdog_timer_, 0);
    }
}

bool BoardBase::validateComponent(const std::shared_ptr<ComponentBase>& component) {
    if (!component) {
        setError("Null component");
        return false;
    }

    if (!component->getId() || strlen(component->getId()) == 0) {
        setError("Invalid component ID");
        return false;
    }

    return true;
}

void BoardBase::updateComponents() {
    for (auto& component : components_) {
        component->update();
    }
}