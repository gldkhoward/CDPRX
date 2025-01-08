#ifndef COMPONENT_BASE_H
#define COMPONENT_BASE_H

/**
 * @file component_base.h
 * @author Luke Howard (you@domain.com)
 * @brief This is the base class for all components (sensors/actuators)
 * @version 0.1
 * @date 2024-11-26
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include <vector>
#include <memory>
#include <string>

// Base class for all components (sensors/actuators)

// Forward declaration
class BoardBase;

class ComponentBase {
public:
    virtual ~ComponentBase() = default;
    
    struct ComponentConfig {
        const char* id;
        const char* type;
        uint32_t update_rate_hz = 10;
        bool publish_diagnostics = true;
        bool enabled = true;
    };

    virtual bool configure(const ComponentConfig& config) = 0;
    virtual bool init() = 0;
    virtual void update() = 0;
    virtual void shutdown() = 0;
    
    virtual const char* getId() const = 0;
    virtual std::string getStatus() const = 0;
    virtual void publishDiagnostics() = 0;
    virtual void getType() = 0;

protected:
    BoardBase* board_ = nullptr;
    friend class BoardBase;
    const char* type_ = nullptr;
    const char* id_ = nullptr;
};


#endif // COMPONENT_BASE_H