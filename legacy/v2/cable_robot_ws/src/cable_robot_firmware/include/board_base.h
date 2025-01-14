// board_base.h
#pragma once

#include "ros_manager.h"
#include <vector>
#include <memory>

// Forward declarations
class ComponentBase;

class BoardBase {
public:
    struct Config {
        const char* board_name;
        ROSManager::Config ros_config;
        uint32_t watchdog_timeout_ms = 5000;
    };

    explicit BoardBase(const Config& config);
    virtual ~BoardBase();

    bool begin();
    void update();
    void shutdown();
    
    // Component management
    bool addComponent(std::shared_ptr<ComponentBase> component);
    void removeComponent(const std::string& component_id);
    std::shared_ptr<ComponentBase> getComponent(const std::string& component_id);
    
    // Status and diagnostics
    bool isHealthy() const;
    String getStatus() const;
    void publishDiagnostics();
    
    // Getters for managers
    ROSManager& getROSManager() { return ros_manager_; }

protected:
    // Virtual methods for derived boards
    virtual bool onInit() { return true; }
    virtual void onUpdate() {}
    virtual void onShutdown() {}
    
    // Error handling
    void setError(const char* error);
    void clearError();

private:
    Config config_;
    bool is_initialized_ = false;
    std::string last_error_;
    
    ROSManager ros_manager_;
    std::vector<std::shared_ptr<ComponentBase>> components_;
    
    // Watchdog and health monitoring
    hw_timer_t* watchdog_timer_ = nullptr;
    void setupWatchdog();
    void feedWatchdog();
    
    // Component management helpers
    bool validateComponent(const std::shared_ptr<ComponentBase>& component);
    void updateComponents();
};

