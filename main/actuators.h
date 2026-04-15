#pragma once

#include <stddef.h>

#include "esp_err.h"

enum class ActuatorId {
    Pump1,
    Pump2,
    Valve1,
    Valve2,
};

struct ActuatorStatus {
    bool pump1_on;
    bool pump2_on;
    bool valve1_on;
    bool valve2_on;
};

esp_err_t actuators_init();
void actuators_set(ActuatorId id, bool on);
void actuators_stop_all();
ActuatorStatus actuators_get_status();

const char* actuator_name(ActuatorId id);
bool actuator_id_from_command(const char* cmd, ActuatorId* out);
void actuators_format_csv_header(char* out, size_t out_len);
void actuators_format_csv(char* out, size_t out_len);
