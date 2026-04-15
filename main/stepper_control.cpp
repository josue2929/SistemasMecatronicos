#include "stepper_control.h"

#include <math.h>

#include "app_config.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include "io_map.h"

namespace {

long s_step_count = 0;
float s_target_deg = APP_STEPPER_DEFAULT_TARGET_DEG;
float s_kp = APP_STEPPER_DEFAULT_KP;
bool s_enabled = true;
float s_min_delay_us = APP_STEPPER_MIN_DELAY_US;
float s_max_delay_us = APP_STEPPER_MAX_DELAY_US;

constexpr float kDegPerStep = 360.0f / static_cast<float>(APP_STEPPER_STEPS_PER_REV);

int enable_level(bool enabled)
{
    if (APP_STEPPER_ENABLE_ACTIVE_LOW) {
        return enabled ? 0 : 1;
    }
    return enabled ? 1 : 0;
}

void pulse_step(int step_period_us)
{
    gpio_set_level(IO_GPIO_STEPPER_STEP, 1);
    esp_rom_delay_us(APP_STEPPER_STEP_PULSE_HIGH_US);
    gpio_set_level(IO_GPIO_STEPPER_STEP, 0);

    if (step_period_us > static_cast<int>(APP_STEPPER_STEP_PULSE_HIGH_US)) {
        esp_rom_delay_us(step_period_us - APP_STEPPER_STEP_PULSE_HIGH_US);
    }
}

void set_direction(bool dir_positive)
{
    gpio_set_level(IO_GPIO_STEPPER_DIR, dir_positive ? 1 : 0);
}

}  // namespace

esp_err_t stepper_init()
{
    gpio_config_t cfg = {};
    cfg.pin_bit_mask = (1ULL << IO_GPIO_STEPPER_STEP) |
                       (1ULL << IO_GPIO_STEPPER_DIR) |
                       (1ULL << IO_GPIO_STEPPER_ENABLE);
    cfg.mode = GPIO_MODE_OUTPUT;
    cfg.pull_up_en = GPIO_PULLUP_DISABLE;
    cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    cfg.intr_type = GPIO_INTR_DISABLE;

    esp_err_t err = gpio_config(&cfg);
    if (err != ESP_OK) {
        return err;
    }

    gpio_set_level(IO_GPIO_STEPPER_STEP, 0);
    gpio_set_level(IO_GPIO_STEPPER_DIR, 1);
    gpio_set_level(IO_GPIO_STEPPER_ENABLE, enable_level(s_enabled));
    esp_rom_delay_us(2000);
    return ESP_OK;
}

void stepper_set_target_deg(float target_deg)
{
    s_target_deg = target_deg;
    stepper_set_enabled(true);
}

void stepper_set_kp(float kp)
{
    if (kp > 0.0f && isfinite(kp)) {
        s_kp = kp;
    }
}

void stepper_set_enabled(bool enabled)
{
    s_enabled = enabled;
    gpio_set_level(IO_GPIO_STEPPER_ENABLE, enable_level(s_enabled));
}

void stepper_stop()
{
    stepper_set_enabled(false);
}

void stepper_home()
{
    s_step_count = 0;
    s_target_deg = APP_STEPPER_HOME_DEG;
}

float stepper_get_position_deg()
{
    return static_cast<float>(s_step_count) * kDegPerStep;
}

int stepper_control_to_delay_us(float control_abs)
{
    if (!isfinite(control_abs) || control_abs < 0.0f) {
        control_abs = 0.0f;
    }
    if (control_abs > 100.0f) {
        control_abs = 100.0f;
    }

    float delay = s_max_delay_us -
                  ((control_abs / 100.0f) * (s_max_delay_us - s_min_delay_us));

    if (delay < s_min_delay_us) {
        delay = s_min_delay_us;
    }
    if (delay > s_max_delay_us) {
        delay = s_max_delay_us;
    }

    return static_cast<int>(delay);
}

void stepper_update_once()
{
    if (!s_enabled) {
        return;
    }

    const float position_deg = stepper_get_position_deg();
    const float error = s_target_deg - position_deg;
    const float control = s_kp * error;

    if (fabsf(error) <= APP_STEPPER_POSITION_TOLERANCE_DEG) {
        return;
    }

    const bool dir_positive = control > 0.0f;
    set_direction(dir_positive);

    const int delay_us = stepper_control_to_delay_us(fabsf(control));
    pulse_step(delay_us);

    if (dir_positive) {
        s_step_count++;
    } else {
        s_step_count--;
    }
}

StepperStatus stepper_get_status()
{
    StepperStatus status = {};
    status.step_count = s_step_count;
    status.position_deg = stepper_get_position_deg();
    status.target_deg = s_target_deg;
    status.kp = s_kp;
    status.enabled = s_enabled;
    status.min_delay_us = s_min_delay_us;
    status.max_delay_us = s_max_delay_us;
    return status;
}
