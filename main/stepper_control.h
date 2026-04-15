#pragma once

#include <stdint.h>

#include "esp_err.h"

struct StepperStatus {
    long step_count;
    float position_deg;
    float target_deg;
    float kp;
    bool enabled;
    float min_delay_us;
    float max_delay_us;
};

esp_err_t stepper_init();
void stepper_set_target_deg(float target_deg);
void stepper_set_kp(float kp);
void stepper_set_enabled(bool enabled);
void stepper_stop();
void stepper_home();
void stepper_update_once();

StepperStatus stepper_get_status();
float stepper_get_position_deg();
int stepper_control_to_delay_us(float control_abs);
