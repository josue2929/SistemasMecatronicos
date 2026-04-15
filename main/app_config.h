#pragma once

#include "hal/adc_types.h"

// -------------------- Sample periods --------------------
#define APP_DIGITAL_SAMPLE_PERIOD_MS      500U
#define APP_ADC_SAMPLE_PERIOD_MS          1000U
#define APP_FLOW_SAMPLE_PERIOD_MS         1000U
#define APP_ALL_SENSORS_SAMPLE_PERIOD_MS  1000U
#define APP_CONTROL_LOOP_PERIOD_MS        1U

// -------------------- Level sensors --------------------
#define APP_LEVEL_SENSORS_ACTIVE_HIGH     true
#define APP_LEVEL_SENSORS_ENABLE_PULLUP   false
#define APP_LEVEL_SENSORS_ENABLE_PULLDOWN false

// -------------------- Flowmeter --------------------
#define APP_FLOWMETER_ENABLE_PULLUP       true
#define APP_FLOWMETER_ENABLE_PULLDOWN     false

// -------------------- Vibration sensor --------------------
#define APP_VIBRATION_SENSOR_ACTIVE_HIGH     true
#define APP_VIBRATION_SENSOR_ENABLE_PULLUP   false
#define APP_VIBRATION_SENSOR_ENABLE_PULLDOWN false

// -------------------- ADC --------------------
#define APP_ADC_ATTENUATION               ADC_ATTEN_DB_12
#define APP_ADC_BITWIDTH                  ADC_BITWIDTH_DEFAULT

// -------------------- Calibration --------------------
#define CAL_ADC_RAW_MAX                   4095.0f
#define CAL_ADC_REFERENCE_VOLTAGE_V       3.300f
#define CAL_ADC_FRONTEND_GAIN             1.000f
#define CAL_ADC_FRONTEND_OFFSET_V         0.000f

#define CAL_PRESSURE_KPA_PER_V            100.000f
#define CAL_PRESSURE_OFFSET_KPA           0.000f

#define CAL_TEMPERATURE_NTC_BETA          3950.0f
#define CAL_TEMPERATURE_NTC_R25           10000.0f
// Match this to the fixed resistor from 3.3V to GPIO39.
#define CAL_TEMPERATURE_DIVIDER_R         1000.0f
#define CAL_TEMPERATURE_K0                298.15f
#define CAL_TEMPERATURE_C0                -273.15f

#define CAL_FLOWMETER_PULSES_PER_LITER    450.000f
#define CAL_FLOWMETER_FLOW_OFFSET_L_MIN   0.000f

// -------------------- Stepper --------------------
#define APP_STEPPER_ENABLE_ACTIVE_LOW     true
#define APP_STEPPER_STEPS_PER_REV         200
#define APP_STEPPER_HOME_DEG              0.0f
#define APP_STEPPER_DEFAULT_TARGET_DEG    180.0f
#define APP_STEPPER_DEFAULT_KP            1.0f
#define APP_STEPPER_MIN_DELAY_US          1200.0f
#define APP_STEPPER_MAX_DELAY_US          12000.0f
#define APP_STEPPER_STEP_PULSE_HIGH_US    5U
#define APP_STEPPER_POSITION_TOLERANCE_DEG 1.8f

// -------------------- HW-095 actuators --------------------
// ON writes IN_A=1 and IN_B=0. Flip this only if you intentionally want the
// opposite polarity for every pump/valve channel.
#define APP_ACTUATOR_FORWARD_IN_A_HIGH  true

// -------------------- Process supervision --------------------
#define APP_SAFETY_PRESSURE_MAX_KPA       300.0f
#define APP_SAFETY_TEMPERATURE_MIN_C      -10.0f
#define APP_SAFETY_TEMPERATURE_MAX_C      80.0f
#define APP_SAFETY_STOP_ON_T3_FULL        true
#define APP_SAFETY_STOP_ON_SENSOR_FAULT   false

// -------------------- Reference automatic mode --------------------
#define APP_AUTO_OPEN_TARGET_DEG           180.0f
#define APP_AUTO_CLOSED_TARGET_DEG         0.0f
#define APP_AUTO_PRESSURE_SETPOINT_KPA     100.0f
#define APP_AUTO_PRESSURE_HYSTERESIS_KPA   5.0f

// -------------------- Bluetooth --------------------
#define BT_DEVICE_NAME                    "JosueServer"
#define SPP_SERVER_NAME                   "SPP_SERVER"
#define SPP_TAG                           "BT_PROCESS"
#define APP_BT_COMMAND_MAX_LEN            128
#define APP_BT_COMMAND_QUEUE_LENGTH       8

// -------------------- Telemetry --------------------
#define APP_TELEMETRY_DEFAULT_STREAM_BT   false
#define APP_ENABLE_STATUS_LED             false
