#ifndef __CONTROLLER_H
#define __CONTROLLER_H

//TODO: goal of refactor is to kick this out completely
extern "C" {
    #include "low_level.h"
}

class Controller {
public:
    struct ControllerConfig {
        float pos_gain = 20.0f; // [(counts/s) / counts]
        float vel_gain = 15.0f / 10000.0f; // [A/(counts/s)]
        float vel_integrator_gain = 10.0f / 10000.0f; // [A/(counts/s * s)]
        float vel_limit = 20000.0f; // [counts/s]
    };

    Controller(const ControllerConfig& config, Motor_t* legacy_motor_ref);
    void control_motor_loop();
private:
    float pos_setpoint_ = 0.0f;
    float pos_gain_ = 0.0f;
    float vel_setpoint_ = 0.0f;
    float vel_gain_ = 0.0f;
    float vel_integrator_gain_ = 0.0f;
    float vel_integrator_current_ = 0.0f;
    float vel_limit_ = 0.0f;
    float current_setpoint_ = 0.0f;
};

#endif /* __CONTROLLER_H */
