#include "sensors.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "app_config.h"
#include "esp_attr.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "io_map.h"
#include "esp_adc/adc_oneshot.h"

namespace {

constexpr const char* TAG = "SENSORS";

struct LevelSensorState {
    const char* name;
    gpio_num_t gpio;
    bool initialized;
    bool last_level;
    uint32_t edge_count;
};

LevelSensorState s_level_states[] = {
    {"S1", IO_GPIO_S1_LEVEL, false, false, 0U},
    {"S2", IO_GPIO_S2_LEVEL, false, false, 0U},
    {"S3", IO_GPIO_S3_LEVEL, false, false, 0U},
};

adc_oneshot_unit_handle_t s_adc1_handle = nullptr;

portMUX_TYPE s_flow_mux = portMUX_INITIALIZER_UNLOCKED;
uint32_t s_flow_pulse_count = 0U;
uint32_t s_last_flow_pulse_count = 0U;
int64_t s_last_flow_sample_us = 0;

LevelSensorState s_vibration_state = {
    "VIB", IO_GPIO_VIBRATION, false, false, 0U,
};

bool gpio_to_bool(gpio_num_t gpio, bool active_high)
{
    const bool raw_high = gpio_get_level(gpio) != 0;
    return active_high ? raw_high : !raw_high;
}

LevelSensorReading sample_level(LevelSensorState& state, bool active_high)
{
    const bool level = gpio_to_bool(state.gpio, active_high);
    SensorEdgeType edge = SensorEdgeType::None;

    if (!state.initialized) {
        state.initialized = true;
    } else if (level != state.last_level) {
        edge = level ? SensorEdgeType::Rising : SensorEdgeType::Falling;
        state.edge_count++;
    }

    state.last_level = level;

    LevelSensorReading reading = {};
    reading.name = state.name;
    reading.gpio = state.gpio;
    reading.level = level;
    reading.edge = edge;
    reading.edge_count = state.edge_count;
    return reading;
}

float raw_to_voltage(int raw_adc)
{
    if (raw_adc < 0) {
        return NAN;
    }

    float voltage = (static_cast<float>(raw_adc) / CAL_ADC_RAW_MAX) * CAL_ADC_REFERENCE_VOLTAGE_V;
    voltage = (voltage * CAL_ADC_FRONTEND_GAIN) + CAL_ADC_FRONTEND_OFFSET_V;
    return voltage;
}

AdcSensorReading read_adc(adc_channel_t channel)
{
    AdcSensorReading reading = {};
    reading.raw_adc = -1;
    reading.voltage_v = NAN;

    int raw = 0;
    const esp_err_t err = adc_oneshot_read(s_adc1_handle, channel, &raw);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "adc_oneshot_read(channel=%d) failed: %s", static_cast<int>(channel), esp_err_to_name(err));
        return reading;
    }

    reading.raw_adc = raw;
    reading.voltage_v = raw_to_voltage(raw);
    return reading;
}

PressureReading sample_pressure()
{
    PressureReading reading = {};
    reading.adc = read_adc(IO_ADC_CHANNEL_PRESSURE);
    reading.pressure_kpa = (reading.adc.voltage_v * CAL_PRESSURE_KPA_PER_V) + CAL_PRESSURE_OFFSET_KPA;
    return reading;
}

TemperatureReading sample_temperature()
{
    TemperatureReading reading = {};
    reading.adc = read_adc(IO_ADC_CHANNEL_TEMPERATURE_T3);
    reading.temperature_c = NAN;
    reading.ntc_resistance_ohm = NAN;

    const float voltage = reading.adc.voltage_v;
    if (!isfinite(voltage) || voltage <= 0.001f || voltage >= (CAL_ADC_REFERENCE_VOLTAGE_V - 0.001f)) {
        return reading;
    }

    const float ntc_resistance = (CAL_TEMPERATURE_DIVIDER_R * voltage) /
                                 (CAL_ADC_REFERENCE_VOLTAGE_V - voltage);
    if (ntc_resistance <= 0.0f || !isfinite(ntc_resistance)) {
        return reading;
    }

    const float inv_t = (1.0f / CAL_TEMPERATURE_K0) +
                        (1.0f / CAL_TEMPERATURE_NTC_BETA) *
                        logf(ntc_resistance / CAL_TEMPERATURE_NTC_R25);
    if (inv_t <= 0.0f || !isfinite(inv_t)) {
        return reading;
    }

    reading.ntc_resistance_ohm = ntc_resistance;
    reading.temperature_c = (1.0f / inv_t) + CAL_TEMPERATURE_C0;
    return reading;
}

uint32_t flow_pulse_count()
{
    portENTER_CRITICAL(&s_flow_mux);
    const uint32_t count = s_flow_pulse_count;
    portEXIT_CRITICAL(&s_flow_mux);
    return count;
}

FlowReading sample_flow(int64_t now_us)
{
    FlowReading reading = {};
    reading.pulse_count = flow_pulse_count();

    if (s_last_flow_sample_us == 0) {
        s_last_flow_sample_us = now_us;
        s_last_flow_pulse_count = reading.pulse_count;
        return reading;
    }

    const uint32_t delta = reading.pulse_count - s_last_flow_pulse_count;
    const float elapsed_s = static_cast<float>(now_us - s_last_flow_sample_us) / 1000000.0f;

    if (elapsed_s > 0.0f) {
        reading.delta_pulses = delta;
        reading.pulses_per_second = static_cast<float>(delta) / elapsed_s;
        reading.flow_l_min = ((reading.pulses_per_second * 60.0f) /
                              CAL_FLOWMETER_PULSES_PER_LITER) +
                             CAL_FLOWMETER_FLOW_OFFSET_L_MIN;
    }

    s_last_flow_sample_us = now_us;
    s_last_flow_pulse_count = reading.pulse_count;
    return reading;
}

void IRAM_ATTR flowmeter_isr(void* arg)
{
    (void)arg;
    portENTER_CRITICAL_ISR(&s_flow_mux);
    s_flow_pulse_count++;
    portEXIT_CRITICAL_ISR(&s_flow_mux);
}

}  // namespace

esp_err_t sensors_init()
{
    gpio_config_t level_cfg = {};
    level_cfg.pin_bit_mask = (1ULL << IO_GPIO_S1_LEVEL) |
                             (1ULL << IO_GPIO_S2_LEVEL) |
                             (1ULL << IO_GPIO_S3_LEVEL);
    level_cfg.mode = GPIO_MODE_INPUT;
    level_cfg.pull_up_en = APP_LEVEL_SENSORS_ENABLE_PULLUP ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
    level_cfg.pull_down_en = APP_LEVEL_SENSORS_ENABLE_PULLDOWN ? GPIO_PULLDOWN_ENABLE : GPIO_PULLDOWN_DISABLE;
    level_cfg.intr_type = GPIO_INTR_DISABLE;
    ESP_RETURN_ON_ERROR(gpio_config(&level_cfg), TAG, "level gpio_config failed");

    gpio_config_t vibration_cfg = {};
    vibration_cfg.pin_bit_mask = (1ULL << IO_GPIO_VIBRATION);
    vibration_cfg.mode = GPIO_MODE_INPUT;
    vibration_cfg.pull_up_en = APP_VIBRATION_SENSOR_ENABLE_PULLUP ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
    vibration_cfg.pull_down_en = APP_VIBRATION_SENSOR_ENABLE_PULLDOWN ? GPIO_PULLDOWN_ENABLE : GPIO_PULLDOWN_DISABLE;
    vibration_cfg.intr_type = GPIO_INTR_DISABLE;
    ESP_RETURN_ON_ERROR(gpio_config(&vibration_cfg), TAG, "vibration gpio_config failed");

    gpio_config_t flow_cfg = {};
    flow_cfg.pin_bit_mask = (1ULL << IO_GPIO_FLOWMETER);
    flow_cfg.mode = GPIO_MODE_INPUT;
    flow_cfg.pull_up_en = APP_FLOWMETER_ENABLE_PULLUP ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
    flow_cfg.pull_down_en = APP_FLOWMETER_ENABLE_PULLDOWN ? GPIO_PULLDOWN_ENABLE : GPIO_PULLDOWN_DISABLE;
    flow_cfg.intr_type = GPIO_INTR_POSEDGE;
    ESP_RETURN_ON_ERROR(gpio_config(&flow_cfg), TAG, "flow gpio_config failed");

    esp_err_t err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_RETURN_ON_ERROR(err, TAG, "gpio_install_isr_service failed");
    }
    ESP_RETURN_ON_ERROR(gpio_isr_handler_add(IO_GPIO_FLOWMETER, flowmeter_isr, nullptr),
                        TAG, "gpio_isr_handler_add failed");

    adc_oneshot_unit_init_cfg_t adc_init_cfg = {};
    adc_init_cfg.unit_id = ADC_UNIT_1;
    adc_init_cfg.ulp_mode = ADC_ULP_MODE_DISABLE;
    ESP_RETURN_ON_ERROR(adc_oneshot_new_unit(&adc_init_cfg, &s_adc1_handle),
                        TAG, "adc_oneshot_new_unit failed");

    adc_oneshot_chan_cfg_t adc_chan_cfg = {};
    adc_chan_cfg.atten = APP_ADC_ATTENUATION;
    adc_chan_cfg.bitwidth = APP_ADC_BITWIDTH;
    ESP_RETURN_ON_ERROR(adc_oneshot_config_channel(s_adc1_handle, IO_ADC_CHANNEL_PRESSURE, &adc_chan_cfg),
                        TAG, "pressure adc config failed");
    ESP_RETURN_ON_ERROR(adc_oneshot_config_channel(s_adc1_handle, IO_ADC_CHANNEL_TEMPERATURE_T3, &adc_chan_cfg),
                        TAG, "temperature adc config failed");

    return ESP_OK;
}

SensorSnapshot sensors_sample_all()
{
    const int64_t now_us = esp_timer_get_time();

    SensorSnapshot snapshot = {};
    snapshot.timestamp_ms = static_cast<uint64_t>(now_us / 1000);
    snapshot.s1_level = sample_level(s_level_states[0], APP_LEVEL_SENSORS_ACTIVE_HIGH);
    snapshot.s2_level = sample_level(s_level_states[1], APP_LEVEL_SENSORS_ACTIVE_HIGH);
    snapshot.s3_level = sample_level(s_level_states[2], APP_LEVEL_SENSORS_ACTIVE_HIGH);
    snapshot.vibration = sample_level(s_vibration_state, APP_VIBRATION_SENSOR_ACTIVE_HIGH);
    snapshot.pressure = sample_pressure();
    snapshot.temperature = sample_temperature();
    snapshot.flow = sample_flow(now_us);
    return snapshot;
}

const char* sensor_edge_to_string(SensorEdgeType edge)
{
    switch (edge) {
    case SensorEdgeType::Rising:
        return "RISING";
    case SensorEdgeType::Falling:
        return "FALLING";
    case SensorEdgeType::None:
    default:
        return "NONE";
    }
}

void sensors_format_csv_header(char* out, size_t out_len)
{
    snprintf(out, out_len,
             "timestamp_ms,s1_level,s1_edge,s1_edges,s2_level,s2_edge,s2_edges,"
             "s3_level,s3_edge,s3_edges,pressure_raw,pressure_v,pressure_kpa,"
             "temperature_raw,temperature_v,temperature_c,flow_pulses,flow_delta,"
             "flow_pps,flow_l_min,vibration_level,vibration_edge,vibration_edges");
}

void sensors_format_csv(const SensorSnapshot& snapshot, char* out, size_t out_len)
{
    snprintf(out, out_len,
             "%llu,%d,%s,%lu,%d,%s,%lu,%d,%s,%lu,%d,%.3f,%.2f,%d,%.3f,%.2f,%lu,%lu,%.2f,%.3f,%d,%s,%lu",
             static_cast<unsigned long long>(snapshot.timestamp_ms),
             snapshot.s1_level.level ? 1 : 0,
             sensor_edge_to_string(snapshot.s1_level.edge),
             static_cast<unsigned long>(snapshot.s1_level.edge_count),
             snapshot.s2_level.level ? 1 : 0,
             sensor_edge_to_string(snapshot.s2_level.edge),
             static_cast<unsigned long>(snapshot.s2_level.edge_count),
             snapshot.s3_level.level ? 1 : 0,
             sensor_edge_to_string(snapshot.s3_level.edge),
             static_cast<unsigned long>(snapshot.s3_level.edge_count),
             snapshot.pressure.adc.raw_adc,
             snapshot.pressure.adc.voltage_v,
             snapshot.pressure.pressure_kpa,
             snapshot.temperature.adc.raw_adc,
             snapshot.temperature.adc.voltage_v,
             snapshot.temperature.temperature_c,
             static_cast<unsigned long>(snapshot.flow.pulse_count),
             static_cast<unsigned long>(snapshot.flow.delta_pulses),
             snapshot.flow.pulses_per_second,
             snapshot.flow.flow_l_min,
             snapshot.vibration.level ? 1 : 0,
             sensor_edge_to_string(snapshot.vibration.edge),
             static_cast<unsigned long>(snapshot.vibration.edge_count));
}
