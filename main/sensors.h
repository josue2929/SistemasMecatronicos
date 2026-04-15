#pragma once

#include <stddef.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "esp_err.h"

enum class SensorEdgeType {
    None,
    Rising,
    Falling,
};

struct LevelSensorReading {
    const char* name;
    gpio_num_t gpio;
    bool level;
    SensorEdgeType edge;
    uint32_t edge_count;
};

struct AdcSensorReading {
    int raw_adc;
    float voltage_v;
};

struct PressureReading {
    AdcSensorReading adc;
    float pressure_kpa;
};

struct TemperatureReading {
    AdcSensorReading adc;
    float temperature_c;
    float ntc_resistance_ohm;
};

struct FlowReading {
    uint32_t pulse_count;
    uint32_t delta_pulses;
    float pulses_per_second;
    float flow_l_min;
};

struct SensorSnapshot {
    uint64_t timestamp_ms;
    LevelSensorReading s1_level;
    LevelSensorReading s2_level;
    LevelSensorReading s3_level;
    LevelSensorReading vibration;
    PressureReading pressure;
    TemperatureReading temperature;
    FlowReading flow;
};

esp_err_t sensors_init();
SensorSnapshot sensors_sample_all();

const char* sensor_edge_to_string(SensorEdgeType edge);
void sensors_format_csv_header(char* out, size_t out_len);
void sensors_format_csv(const SensorSnapshot& snapshot, char* out, size_t out_len);
