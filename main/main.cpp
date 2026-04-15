#include <ctype.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "actuators.h"
#include "app_config.h"
#include "bluetooth_spp.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sensors.h"
#include "stepper_control.h"

namespace {

constexpr const char* TAG = "APP";

struct AppState {
    bool stream_bt;
    bool auto_mode;
    bool safety_enabled;
    float pressure_setpoint_kpa;
    char safety_reason[64];
    bool safety_ok;
};

AppState s_app = {
    APP_TELEMETRY_DEFAULT_STREAM_BT,
    false,
    true,
    APP_AUTO_PRESSURE_SETPOINT_KPA,
    "OK",
    true,
};

uint64_t millis()
{
    return static_cast<uint64_t>(esp_timer_get_time() / 1000);
}

bool equals_ignore_case(const char* lhs, const char* rhs)
{
    if (lhs == nullptr || rhs == nullptr) {
        return false;
    }

    while (*lhs != '\0' && *rhs != '\0') {
        if (toupper(static_cast<unsigned char>(*lhs)) !=
            toupper(static_cast<unsigned char>(*rhs))) {
            return false;
        }
        lhs++;
        rhs++;
    }

    return *lhs == '\0' && *rhs == '\0';
}

bool starts_with_ignore_case(const char* text, const char* prefix)
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

char* trim(char* text)
{
    if (text == nullptr) {
        return text;
    }

    while (isspace(static_cast<unsigned char>(*text))) {
        text++;
    }

    char* end = text + strlen(text);
    while (end > text && isspace(static_cast<unsigned char>(*(end - 1)))) {
        end--;
    }
    *end = '\0';
    return text;
}

void reply(const char* fmt, ...)
{
    char msg[256] = {};
    va_list args;
    va_start(args, fmt);
    vsnprintf(msg, sizeof(msg), fmt, args);
    va_end(args);

    printf("%s\n", msg);
    bluetooth_spp_send_line(msg);
}

bool parse_bool_arg(const char* cmd, bool* out)
{
    const char* value = strchr(cmd, ':');
    if (value == nullptr || out == nullptr) {
        return false;
    }
    value++;

    if (equals_ignore_case(value, "1") ||
        equals_ignore_case(value, "ON") ||
        equals_ignore_case(value, "TRUE")) {
        *out = true;
        return true;
    }

    if (equals_ignore_case(value, "0") ||
        equals_ignore_case(value, "OFF") ||
        equals_ignore_case(value, "FALSE")) {
        *out = false;
        return true;
    }

    return false;
}

bool parse_float_after_colon(const char* cmd, float* out)
{
    const char* value = strchr(cmd, ':');
    if (value == nullptr || out == nullptr) {
        return false;
    }
    return sscanf(value + 1, "%f", out) == 1 && isfinite(*out);
}

void format_telemetry_header(char* out, size_t out_len)
{
    char sensor_header[512] = {};
    char actuator_header[128] = {};
    sensors_format_csv_header(sensor_header, sizeof(sensor_header));
    actuators_format_csv_header(actuator_header, sizeof(actuator_header));
    snprintf(out, out_len,
             "%s,%s,step_position_deg,step_target_deg,step_kp,step_count,motor_enabled,"
             "auto_mode,stream_bt,safety_ok,safety_reason",
             sensor_header,
             actuator_header);
}

void format_telemetry(const SensorSnapshot& snapshot, char* out, size_t out_len)
{
    char sensor_csv[512] = {};
    char actuator_csv[128] = {};
    sensors_format_csv(snapshot, sensor_csv, sizeof(sensor_csv));
    actuators_format_csv(actuator_csv, sizeof(actuator_csv));

    const StepperStatus stepper = stepper_get_status();
    snprintf(out, out_len,
             "%s,%s,%.2f,%.2f,%.3f,%ld,%d,%d,%d,%d,%s",
             sensor_csv,
             actuator_csv,
             stepper.position_deg,
             stepper.target_deg,
             stepper.kp,
             stepper.step_count,
             stepper.enabled ? 1 : 0,
             s_app.auto_mode ? 1 : 0,
             s_app.stream_bt ? 1 : 0,
             s_app.safety_ok ? 1 : 0,
             s_app.safety_reason);
}

void send_telemetry_header()
{
    char header[768] = {};
    format_telemetry_header(header, sizeof(header));
    printf("%s\n", header);
    bluetooth_spp_send_line(header);
}

void send_telemetry(const SensorSnapshot& snapshot)
{
    char line[768] = {};
    format_telemetry(snapshot, line, sizeof(line));
    printf("%s\n", line);
    if (s_app.stream_bt) {
        bluetooth_spp_send_line(line);
    }
}

void send_help()
{
    bluetooth_spp_send_line("Commands:");
    bluetooth_spp_send_line("T:<deg> | STEP:<deg>    set stepper target");
    bluetooth_spp_send_line("K:<kp>                 set stepper proportional gain");
    bluetooth_spp_send_line("P1:0|1 P2:0|1          pump control");
    bluetooth_spp_send_line("V1:0|1 V2:0|1          valve control");
    bluetooth_spp_send_line("ACT:0                  all pumps/valves off");
    bluetooth_spp_send_line("S | H | I              stop all, home counter, status");
    bluetooth_spp_send_line("R | READ | HEADER      read telemetry or print CSV header");
    bluetooth_spp_send_line("STREAM:0|1             Bluetooth telemetry stream");
    bluetooth_spp_send_line("AUTO:0|1               reference automatic stepper mode");
    bluetooth_spp_send_line("P:<kPa>                automatic pressure setpoint");
    bluetooth_spp_send_line("SAFE:0|1               safety interlocks");
}

void evaluate_safety(const SensorSnapshot& snapshot)
{
    s_app.safety_ok = true;
    snprintf(s_app.safety_reason, sizeof(s_app.safety_reason), "OK");

    if (APP_SAFETY_STOP_ON_T3_FULL && snapshot.s3_level.level) {
        s_app.safety_ok = false;
        snprintf(s_app.safety_reason, sizeof(s_app.safety_reason), "T3_FULL");
        return;
    }

    if (isfinite(snapshot.pressure.pressure_kpa) &&
        snapshot.pressure.pressure_kpa > APP_SAFETY_PRESSURE_MAX_KPA) {
        s_app.safety_ok = false;
        snprintf(s_app.safety_reason, sizeof(s_app.safety_reason), "PRESSURE_HIGH");
        return;
    }

    if (isfinite(snapshot.temperature.temperature_c) &&
        (snapshot.temperature.temperature_c < APP_SAFETY_TEMPERATURE_MIN_C ||
         snapshot.temperature.temperature_c > APP_SAFETY_TEMPERATURE_MAX_C)) {
        s_app.safety_ok = false;
        snprintf(s_app.safety_reason, sizeof(s_app.safety_reason), "TEMP_RANGE");
        return;
    }

    if (APP_SAFETY_STOP_ON_SENSOR_FAULT &&
        (!isfinite(snapshot.pressure.pressure_kpa) ||
         !isfinite(snapshot.temperature.temperature_c))) {
        s_app.safety_ok = false;
        snprintf(s_app.safety_reason, sizeof(s_app.safety_reason), "SENSOR_FAULT");
    }
}

void update_automatic_mode(const SensorSnapshot& snapshot)
{
    if (!s_app.auto_mode) {
        return;
    }

    if (s_app.safety_enabled && !s_app.safety_ok) {
        actuators_stop_all();
        stepper_set_target_deg(APP_AUTO_CLOSED_TARGET_DEG);
        return;
    }

    const bool source_available = snapshot.s1_level.level || snapshot.s2_level.level;
    const bool destination_full = snapshot.s3_level.level;
    if (!source_available || destination_full) {
        actuators_stop_all();
        stepper_set_target_deg(APP_AUTO_CLOSED_TARGET_DEG);
        return;
    }

    if (isfinite(snapshot.pressure.pressure_kpa)) {
        if (snapshot.pressure.pressure_kpa >
            (s_app.pressure_setpoint_kpa + APP_AUTO_PRESSURE_HYSTERESIS_KPA)) {
            actuators_stop_all();
            stepper_set_target_deg(APP_AUTO_CLOSED_TARGET_DEG);
            return;
        }

        if (snapshot.pressure.pressure_kpa <
            (s_app.pressure_setpoint_kpa - APP_AUTO_PRESSURE_HYSTERESIS_KPA)) {
            stepper_set_target_deg(APP_AUTO_OPEN_TARGET_DEG);
            return;
        }
    }

    stepper_set_target_deg(APP_AUTO_OPEN_TARGET_DEG);
}

void enforce_manual_safety()
{
    if (!s_app.safety_enabled || s_app.auto_mode || s_app.safety_ok) {
        return;
    }

    stepper_stop();
    actuators_stop_all();
}

void send_status()
{
    const StepperStatus stepper = stepper_get_status();
    const ActuatorStatus actuators = actuators_get_status();
    reply("STATUS POS=%.2f TARGET=%.2f KP=%.3f STEPS=%ld MOTOR=%s P1=%s P2=%s V1=%s V2=%s AUTO=%s STREAM=%s SAFE=%s REASON=%s PSET=%.2f",
          stepper.position_deg,
          stepper.target_deg,
          stepper.kp,
          stepper.step_count,
          stepper.enabled ? "ON" : "OFF",
          actuators.pump1_on ? "ON" : "OFF",
          actuators.pump2_on ? "ON" : "OFF",
          actuators.valve1_on ? "ON" : "OFF",
          actuators.valve2_on ? "ON" : "OFF",
          s_app.auto_mode ? "ON" : "OFF",
          s_app.stream_bt ? "ON" : "OFF",
          s_app.safety_enabled ? "ON" : "OFF",
          s_app.safety_reason,
          s_app.pressure_setpoint_kpa);
}

void process_command(char* raw_cmd, SensorSnapshot* snapshot, bool* have_snapshot)
{
    char* cmd = trim(raw_cmd);
    if (cmd == nullptr || cmd[0] == '\0') {
        return;
    }

    if (equals_ignore_case(cmd, "HELP") || equals_ignore_case(cmd, "?")) {
        send_help();
        return;
    }

    ActuatorId actuator_id = ActuatorId::Pump1;
    if (actuator_id_from_command(cmd, &actuator_id)) {
        bool enabled = false;
        if (parse_bool_arg(cmd, &enabled)) {
            s_app.auto_mode = false;
            if (s_app.safety_enabled && !s_app.safety_ok && enabled) {
                reply("ERR safety active: %s", s_app.safety_reason);
            } else {
                actuators_set(actuator_id, enabled);
                reply("OK %s=%s", actuator_name(actuator_id), enabled ? "ON" : "OFF");
            }
        } else {
            reply("ERR invalid actuator value");
        }
        return;
    }

    if (starts_with_ignore_case(cmd, "ACT:")) {
        bool enabled = false;
        if (parse_bool_arg(cmd, &enabled) && !enabled) {
            actuators_stop_all();
            reply("OK ACTUATORS=OFF");
        } else {
            reply("ERR use ACT:0 to turn all actuators off");
        }
        return;
    }

    if (starts_with_ignore_case(cmd, "T:") || starts_with_ignore_case(cmd, "STEP:")) {
        float target = 0.0f;
        if (parse_float_after_colon(cmd, &target)) {
            s_app.auto_mode = false;
            stepper_set_target_deg(target);
            reply("OK TARGET=%.2f", target);
        } else {
            reply("ERR invalid target");
        }
        return;
    }

    if (starts_with_ignore_case(cmd, "K:")) {
        float kp = 0.0f;
        if (parse_float_after_colon(cmd, &kp) && kp > 0.0f) {
            stepper_set_kp(kp);
            reply("OK KP=%.3f", kp);
        } else {
            reply("ERR invalid kp");
        }
        return;
    }

    if (equals_ignore_case(cmd, "S") || equals_ignore_case(cmd, "STOP")) {
        s_app.auto_mode = false;
        stepper_stop();
        actuators_stop_all();
        reply("OK STOP");
        return;
    }

    if (equals_ignore_case(cmd, "H") || equals_ignore_case(cmd, "HOME")) {
        stepper_home();
        reply("OK HOME");
        return;
    }

    if (equals_ignore_case(cmd, "I") || equals_ignore_case(cmd, "INFO") ||
        equals_ignore_case(cmd, "STATUS")) {
        send_status();
        return;
    }

    if (equals_ignore_case(cmd, "HEADER")) {
        send_telemetry_header();
        return;
    }

    if (equals_ignore_case(cmd, "R") || equals_ignore_case(cmd, "READ")) {
        if (!*have_snapshot) {
            *snapshot = sensors_sample_all();
            evaluate_safety(*snapshot);
            *have_snapshot = true;
        }
        char line[768] = {};
        format_telemetry(*snapshot, line, sizeof(line));
        bluetooth_spp_send_line(line);
        printf("%s\n", line);
        return;
    }

    if (starts_with_ignore_case(cmd, "STREAM:") || starts_with_ignore_case(cmd, "MON:")) {
        bool enabled = false;
        if (parse_bool_arg(cmd, &enabled)) {
            s_app.stream_bt = enabled;
            reply("OK STREAM=%s", enabled ? "ON" : "OFF");
        } else {
            reply("ERR invalid stream value");
        }
        return;
    }

    if (starts_with_ignore_case(cmd, "AUTO:")) {
        bool enabled = false;
        if (parse_bool_arg(cmd, &enabled)) {
            s_app.auto_mode = enabled;
            actuators_stop_all();
            if (!enabled) {
                stepper_stop();
            }
            reply("OK AUTO=%s", enabled ? "ON" : "OFF");
        } else {
            reply("ERR invalid auto value");
        }
        return;
    }

    if (starts_with_ignore_case(cmd, "SAFE:")) {
        bool enabled = false;
        if (parse_bool_arg(cmd, &enabled)) {
            s_app.safety_enabled = enabled;
            reply("OK SAFE=%s", enabled ? "ON" : "OFF");
        } else {
            reply("ERR invalid safe value");
        }
        return;
    }

    if (starts_with_ignore_case(cmd, "P:")) {
        float setpoint = 0.0f;
        if (parse_float_after_colon(cmd, &setpoint) && setpoint >= 0.0f) {
            s_app.pressure_setpoint_kpa = setpoint;
            reply("OK PSET=%.2f", setpoint);
        } else {
            reply("ERR invalid pressure setpoint");
        }
        return;
    }

    reply("UNKNOWN CMD: %s", cmd);
}

}  // namespace

extern "C" void app_main()
{
    ESP_LOGI(TAG, "Starting mechatronics controller");

    ESP_ERROR_CHECK(actuators_init());
    ESP_ERROR_CHECK(stepper_init());
    ESP_ERROR_CHECK(sensors_init());
    ESP_ERROR_CHECK(bluetooth_spp_init());

    SensorSnapshot latest_snapshot = {};
    bool have_snapshot = false;
    uint64_t last_sample_ms = 0;

    send_telemetry_header();

    while (true) {
        char cmd[APP_BT_COMMAND_MAX_LEN] = {};
        while (bluetooth_spp_receive_command(cmd, sizeof(cmd), 0)) {
            process_command(cmd, &latest_snapshot, &have_snapshot);
            memset(cmd, 0, sizeof(cmd));
        }

        const uint64_t now_ms = millis();
        if (!have_snapshot ||
            (now_ms - last_sample_ms) >= APP_ALL_SENSORS_SAMPLE_PERIOD_MS) {
            latest_snapshot = sensors_sample_all();
            have_snapshot = true;
            last_sample_ms = now_ms;

            evaluate_safety(latest_snapshot);
            update_automatic_mode(latest_snapshot);
            send_telemetry(latest_snapshot);
        }

        enforce_manual_safety();
        stepper_update_once();
        vTaskDelay(pdMS_TO_TICKS(APP_CONTROL_LOOP_PERIOD_MS));
    }
}
