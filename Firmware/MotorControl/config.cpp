
/* Includes ------------------------------------------------------------------*/

#include "config.h"

#include <stdint.h>
#include <stdlib.h>

#include "nvm.h"
#include "crc.hpp"
#include "low_level.h"

/* Private defines -----------------------------------------------------------*/
#define CRC16_INIT 0xabcd

#if HW_VERSION_MAJOR == 3
#if HW_VERSION_MINOR <= 3
#define SHUNT_RESISTANCE (675e-6f)
#else
#define SHUNT_RESISTANCE (500e-6f)
#endif
#endif

/* Private macros ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/

typedef struct {
    Motor_control_mode_t control_mode;
    bool enable_step_dir;
    float counts_per_step;
    float pos_setpoint;
    float pos_gain;
    float vel_setpoint;
    float vel_gain;
    float vel_integrator_gain;
    float vel_integrator_current;
    float vel_limit;
    float current_setpoint;
    float calibration_current;
    float phase_inductance;
    float phase_resistance;
    float shunt_conductance;
    float phase_current_rev_gain; //Reverse gain for ADC to Amps
    Current_control_t current_control;
    Rotor_mode_t rotor_mode;
} MotorConfig_t;

/* Global constant data ------------------------------------------------------*/
/* Global variables ----------------------------------------------------------*/
/* Private constant data -----------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Function implementations --------------------------------------------------*/

template<typename ... Ts>
struct Config;

template<>
struct Config<> {
    static size_t get_size() {
        return 0;
    }
    static int load_config(size_t offset, uint16_t* crc16) {
        return 0;
    }
    static int store_config(size_t offset, uint16_t* crc16) {
        return 0;
    }
};

template<typename T, typename ... Ts>
struct Config<T, Ts...> {
    static size_t get_size() {
        return sizeof(T) + Config<Ts...>::get_size();
    }

    static int load_config(size_t offset, uint16_t* crc16, T* val0, Ts* ... vals) {
        size_t size = sizeof(T);
        // save current CRC (in case val0 and crc16 point to the same address)
        size_t previous_crc16 = *crc16;
        if (NVM_read(offset, (uint8_t *)val0, size))
            return -1;
        *crc16 = calc_crc16(previous_crc16, (uint8_t *)val0, size);
        if (Config<Ts...>::load_config(offset + size, crc16, vals...))
            return -1;
        return 0;
    }

    static int store_config(size_t offset, uint16_t* crc16, T* val0, Ts* ... vals) {
        size_t size = sizeof(T);
        if (NVM_write(offset, (uint8_t *)val0, size))
            return -1;
        // update CRC _after_ writing (in case val0 and crc16 point to the same address)
        if (crc16)
            *crc16 = calc_crc16(*crc16, (uint8_t *)val0, size);
        if (Config<Ts...>::store_config(offset + size, crc16, vals...))
            return -1;
        return 0;
    }

    static int load_config(T* val0, Ts* ... vals) {
        //printf("have %d bytes\r\n", NVM_get_max_read_length()); osDelay(5);
        if (Config<T, Ts..., uint16_t>::get_size() > NVM_get_max_read_length())
            return -1;
        uint16_t crc16 = CRC16_INIT;
        if (Config<T, Ts..., uint16_t>::load_config(0, &crc16, val0, vals..., &crc16))
            return -1;
        if (crc16)
            return -1;
        return 0;
    }

    static int store_config(T* val0, Ts* ... vals) {
        size_t size = Config<T, Ts...>::get_size() + 2;
        //printf("config is %d bytes\r\n", size); osDelay(5);
        if (size > NVM_get_max_write_length())
            return -1;
        if (NVM_start_write(size))
            return -1;
        uint16_t crc16 = CRC16_INIT;
        if (Config<T, Ts...>::store_config(0, &crc16, val0, vals...))
            return -1;
        if (Config<uint8_t, uint8_t>::store_config(size - 2, nullptr, (uint8_t *)&crc16 + 1, (uint8_t *)&crc16))
            return -1;
        if (NVM_commit())
            return -1;
        return 0;
    }
};


void load_default_motor_config(MotorConfig_t* config) {
    config->control_mode = CTRL_MODE_POSITION_CONTROL;  //see: Motor_control_mode_t
    config->enable_step_dir = false;                    //auto enabled after calibration
    config->counts_per_step = 2.0f;
    config->pos_setpoint = 0.0f;
    config->pos_gain = 20.0f;  // [(counts/s) / counts]
    config->vel_setpoint = 0.0f;
    //config->vel_setpoint = 800.0f; <sensorless example>
    config->vel_gain = 15.0f / 10000.0f;  // [A/(counts/s)]
    //config->vel_gain = 15.0f / 200.0f; // [A/(rad/s)] <sensorless example>
    config->vel_integrator_gain = 10.0f / 10000.0f;  // [A/(counts/s * s)]
    //config->vel_integrator_gain = 0.0f; // [A/(rad/s * s)] <sensorless example>
    config->vel_integrator_current = 0.0f;  // [A]
    config->vel_limit = 20000.0f;           // [counts/s]
    config->current_setpoint = 0.0f;        // [A]
    config->calibration_current = 10.0f;    // [A]
    config->phase_inductance = 0.0f;        // to be set by measure_phase_inductance
    config->phase_resistance = 0.0f;        // to be set by measure_phase_resistance
    config->shunt_conductance = 1.0f / SHUNT_RESISTANCE;  //[S]
    config->phase_current_rev_gain = 0.0f;                // to be set by DRV8301_setup
    // Read out max_allowed_current to see max supported value for current_lim.
    // You can change DRV8301_ShuntAmpGain to get a different range.
    //config->current_control.current_lim = 75.0f;  //[A]
    config->current_control.current_lim = 10.0f;  //[A]
    config->current_control.p_gain = 0.0f;        // [V/A] should be auto set after resistance and inductance measurement
    config->current_control.i_gain = 0.0f;        // [V/As] should be auto set after resistance and inductance measurement
    config->current_control.v_current_control_integral_d = 0.0f;
    config->current_control.v_current_control_integral_q = 0.0f;
    config->current_control.Ibus = 0.0f;
    config->current_control.final_v_alpha = 0.0f;
    config->current_control.final_v_beta = 0.0f;
    config->current_control.Iq = 0.0f;
    config->current_control.max_allowed_current = 0.0f;
    // config->rotor_mode = ROTOR_MODE_SENSORLESS;
    // config->rotor_mode = ROTOR_MODE_RUN_ENCODER_TEST_SENSORLESS;
    config->rotor_mode = ROTOR_MODE_ENCODER;
}

// This function is obviously stupid and should go away (make MotorConfig_t a member of Motor_t)
void set_motor_config(MotorConfig_t *config, Motor_t *motor) {
    motor->control_mode = config->control_mode;
    motor->enable_step_dir = config->enable_step_dir;
    motor->counts_per_step = config->counts_per_step;
    motor->pos_setpoint = config->pos_setpoint;
    motor->pos_gain = config->pos_gain;
    motor->vel_setpoint = config->vel_setpoint;
    motor->vel_gain = config->vel_gain;
    motor->vel_integrator_gain = config->vel_integrator_gain;
    motor->vel_integrator_current = config->vel_integrator_current;
    motor->vel_limit = config->vel_limit;
    motor->current_setpoint = config->current_setpoint;
    motor->calibration_current = config->calibration_current;
    motor->phase_inductance = config->phase_inductance;
    motor->phase_resistance = config->phase_resistance;
    motor->shunt_conductance = config->shunt_conductance;
    motor->phase_current_rev_gain = config->phase_current_rev_gain;
    motor->current_control = config->current_control;
    motor->rotor_mode = config->rotor_mode;
}

// This function is obviously stupid and should go away (make MotorConfig_t a member of Motor_t)
void get_motor_config(Motor_t *motor, MotorConfig_t *config) {
    config->control_mode = motor->control_mode;
    config->enable_step_dir = motor->enable_step_dir;
    config->counts_per_step = motor->counts_per_step;
    config->pos_setpoint = motor->pos_setpoint;
    config->pos_gain = motor->pos_gain;
    config->vel_setpoint = motor->vel_setpoint;
    config->vel_gain = motor->vel_gain;
    config->vel_integrator_gain = motor->vel_integrator_gain;
    config->vel_integrator_current = motor->vel_integrator_current;
    config->vel_limit = motor->vel_limit;
    config->current_setpoint = motor->current_setpoint;
    config->calibration_current = motor->calibration_current;
    config->phase_inductance = motor->phase_inductance;
    config->phase_resistance = motor->phase_resistance;
    config->shunt_conductance = motor->shunt_conductance;
    config->phase_current_rev_gain = motor->phase_current_rev_gain;
    config->current_control = motor->current_control;
    config->rotor_mode = motor->rotor_mode;
}


void init_configuration(void) {
    MotorConfig_t motor_config[2];
    if (NVM_init() || Config<MotorConfig_t, MotorConfig_t>::load_config(&motor_config[0], &motor_config[1])) {
        //printf("no config found\r\n"); osDelay(5);
        load_default_motor_config(&motor_config[0]);
        load_default_motor_config(&motor_config[1]);
    } else {
        //printf("load config successful\r\n"); osDelay(5);
    }
    
    set_motor_config(&motor_config[0], &motors[0]);
    set_motor_config(&motor_config[1], &motors[1]);
}

void clear_configuration(void) {
    NVM_erase();
}

void save_configuration(void) {
    MotorConfig_t motor_config[2];
    get_motor_config(&motors[0], &motor_config[0]);
    get_motor_config(&motors[1], &motor_config[1]);
    if (Config<MotorConfig_t, MotorConfig_t>::store_config(&motor_config[0], &motor_config[1])) {
        //printf("saving configuration failed\r\n"); osDelay(5);
    }
}
