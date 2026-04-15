#include "actuators.h"

#include <ctype.h>
#include <stdio.h>
#include <string.h>

#include "app_config.h"
#include "driver/gpio.h"
#include "io_map.h"

namespace {

struct HBridgeChannel {
    const char* name;
    gpio_num_t in_a;
    gpio_num_t in_b;
    bool on;
};

HBridgeChannel s_channels[] = {
    {"PUMP1", IO_GPIO_PUMP1_IN1, IO_GPIO_PUMP1_IN2, false},
    {"PUMP2", IO_GPIO_PUMP2_IN1, IO_GPIO_PUMP2_IN2, false},
    {"VALVE1", IO_GPIO_VALVE1_IN1, IO_GPIO_VALVE1_IN2, false},
    {"VALVE2", IO_GPIO_VALVE2_IN1, IO_GPIO_VALVE2_IN2, false},
};

int on_level_a()
{
    return APP_ACTUATOR_FORWARD_IN_A_HIGH ? 1 : 0;
}

int on_level_b()
{
    return APP_ACTUATOR_FORWARD_IN_A_HIGH ? 0 : 1;
}

HBridgeChannel& channel_for(ActuatorId id)
{
    return s_channels[static_cast<int>(id)];
}

bool equal_prefix_ignore_case(const char* text, const char* prefix)
{
    if (text == nullptr || prefix == nullptr) {
        return false;
    }

    while (*prefix != '\0') {
        if (*text == '\0') {
            return false;
        }
        if (toupper(static_cast<unsigned char>(*text)) !=
            toupper(static_cast<unsigned char>(*prefix))) {
            return false;
        }
        text++;
        prefix++;
    }

    return true;
}

void apply_channel(HBridgeChannel& channel)
{
    if (channel.on) {
        gpio_set_level(channel.in_a, on_level_a());
        gpio_set_level(channel.in_b, on_level_b());
    } else {
        gpio_set_level(channel.in_a, 0);
        gpio_set_level(channel.in_b, 0);
    }
}

}  // namespace

esp_err_t actuators_init()
{
    gpio_config_t cfg = {};
    cfg.pin_bit_mask = (1ULL << IO_GPIO_PUMP1_IN1) |
                       (1ULL << IO_GPIO_PUMP1_IN2) |
                       (1ULL << IO_GPIO_PUMP2_IN1) |
                       (1ULL << IO_GPIO_PUMP2_IN2) |
                       (1ULL << IO_GPIO_VALVE1_IN1) |
                       (1ULL << IO_GPIO_VALVE1_IN2) |
                       (1ULL << IO_GPIO_VALVE2_IN1) |
                       (1ULL << IO_GPIO_VALVE2_IN2);
    cfg.mode = GPIO_MODE_OUTPUT;
    cfg.pull_up_en = GPIO_PULLUP_DISABLE;
    cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    cfg.intr_type = GPIO_INTR_DISABLE;

    const esp_err_t err = gpio_config(&cfg);
    if (err != ESP_OK) {
        return err;
    }

    actuators_stop_all();
    return ESP_OK;
}

void actuators_set(ActuatorId id, bool on)
{
    HBridgeChannel& channel = channel_for(id);
    channel.on = on;
    apply_channel(channel);
}

void actuators_stop_all()
{
    for (HBridgeChannel& channel : s_channels) {
        channel.on = false;
        apply_channel(channel);
    }
}

ActuatorStatus actuators_get_status()
{
    ActuatorStatus status = {};
    status.pump1_on = s_channels[static_cast<int>(ActuatorId::Pump1)].on;
    status.pump2_on = s_channels[static_cast<int>(ActuatorId::Pump2)].on;
    status.valve1_on = s_channels[static_cast<int>(ActuatorId::Valve1)].on;
    status.valve2_on = s_channels[static_cast<int>(ActuatorId::Valve2)].on;
    return status;
}

const char* actuator_name(ActuatorId id)
{
    return channel_for(id).name;
}

bool actuator_id_from_command(const char* cmd, ActuatorId* out)
{
    if (cmd == nullptr || out == nullptr) {
        return false;
    }

    if (equal_prefix_ignore_case(cmd, "P1:") || equal_prefix_ignore_case(cmd, "PUMP1:")) {
        *out = ActuatorId::Pump1;
        return true;
    }
    if (equal_prefix_ignore_case(cmd, "P2:") || equal_prefix_ignore_case(cmd, "PUMP2:")) {
        *out = ActuatorId::Pump2;
        return true;
    }
    if (equal_prefix_ignore_case(cmd, "V1:") || equal_prefix_ignore_case(cmd, "VALVE1:")) {
        *out = ActuatorId::Valve1;
        return true;
    }
    if (equal_prefix_ignore_case(cmd, "V2:") || equal_prefix_ignore_case(cmd, "VALVE2:")) {
        *out = ActuatorId::Valve2;
        return true;
    }

    return false;
}

void actuators_format_csv_header(char* out, size_t out_len)
{
    snprintf(out, out_len, "pump1_on,pump2_on,valve1_on,valve2_on");
}

void actuators_format_csv(char* out, size_t out_len)
{
    const ActuatorStatus status = actuators_get_status();
    snprintf(out, out_len, "%d,%d,%d,%d",
             status.pump1_on ? 1 : 0,
             status.pump2_on ? 1 : 0,
             status.valve1_on ? 1 : 0,
             status.valve2_on ? 1 : 0);
}
