#include "definitions.h"

long stepCount = 0;
float target_deg = 180.0f;
float kp = 1.0f;
bool motor_enabled = true;

float min_delay_us = 1200.0f;
float max_delay_us = 12000.0f;

char rx_line[128] = {0};
volatile bool cmd_ready = false;
int rx_index = 0;

static uint32_t spp_client_handle = 0;

void pulseStep(int step_period_us)
{
    gpio_set_level(STEP_PIN, 1);
    esp_rom_delay_us(5);
    gpio_set_level(STEP_PIN, 0);

    if (step_period_us > 5) {
        esp_rom_delay_us(step_period_us - 5);
    }
}

void setDirection(bool dir)
{
    gpio_set_level(DIR_PIN, dir ? 1 : 0);
}

float getPositionDeg()
{
    return stepCount * DEG_PER_STEP;
}

int controlToDelayUs(float control_abs)
{
    if (control_abs > 100.0f) control_abs = 100.0f;

    float delay = max_delay_us - ((control_abs / 100.0f) * (max_delay_us - min_delay_us));

    if (delay < min_delay_us) delay = min_delay_us;
    if (delay > max_delay_us) delay = max_delay_us;

    return (int)delay;
}

static void bt_send(const char* msg)
{
    if (spp_client_handle != 0 && msg != nullptr) {
        esp_spp_write(spp_client_handle, strlen(msg), (uint8_t*)msg);
    }
}

void processCommand(const char* cmd)
{
    if (cmd == nullptr) return;

    if (strncmp(cmd, "T:", 2) == 0) {
        float new_target = 0.0f;
        if (sscanf(cmd, "T:%f", &new_target) == 1) {
            target_deg = new_target;
            motor_enabled = true;

            char msg[64];
            snprintf(msg, sizeof(msg), "OK TARGET=%.2f\r\n", target_deg);
            bt_send(msg);
            printf("%s", msg);
        }
    }
    else if (strncmp(cmd, "K:", 2) == 0) {
        float new_kp = 0.0f;
        if (sscanf(cmd, "K:%f", &new_kp) == 1 && new_kp > 0.0f) {
            kp = new_kp;

            char msg[64];
            snprintf(msg, sizeof(msg), "OK KP=%.3f\r\n", kp);
            bt_send(msg);
            printf("%s", msg);
        }
    }
    else if (strcmp(cmd, "S") == 0) {
        motor_enabled = false;
        bt_send("OK STOP\r\n");
        printf("OK STOP\n");
    }
    else if (strcmp(cmd, "H") == 0) {
        stepCount = 0;
        bt_send("OK HOME\r\n");
        printf("OK HOME\n");
    }
    else if (strcmp(cmd, "I") == 0) {
        char msg[128];
        snprintf(msg, sizeof(msg),
                 "POS=%.2f deg | TARGET=%.2f | KP=%.3f | STEPS=%ld\r\n",
                 getPositionDeg(), target_deg, kp, stepCount);
        bt_send(msg);
        printf("%s", msg);
    }
    else {
        bt_send("UNKNOWN CMD\r\n");
        printf("UNKNOWN CMD: %s\n", cmd);
    }
}

static void spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
    switch (event) {
    case ESP_SPP_INIT_EVT:
        esp_bt_dev_set_device_name(BT_DEVICE_NAME);
        esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
        esp_spp_start_srv(ESP_SPP_SEC_NONE, ESP_SPP_ROLE_SLAVE, 0, SPP_SERVER_NAME);
        ESP_LOGI(SPP_TAG, "SPP init done");
        break;

    case ESP_SPP_START_EVT:
        ESP_LOGI(SPP_TAG, "SPP server started");
        break;

    case ESP_SPP_SRV_OPEN_EVT:
        spp_client_handle = param->srv_open.handle;
        ESP_LOGI(SPP_TAG, "Client connected, handle=%lu", (unsigned long)spp_client_handle);
        bt_send("Bluetooth connected\r\n");
        break;

    case ESP_SPP_CLOSE_EVT:
        spp_client_handle = 0;
        ESP_LOGI(SPP_TAG, "Client disconnected");
        break;

    case ESP_SPP_DATA_IND_EVT:
        for (int i = 0; i < param->data_ind.len; i++) {
            char c = (char)param->data_ind.data[i];

            if (c == '\r') continue;

            if (c == '\n') {
                if (rx_index > 0) {
                    rx_line[rx_index] = '\0';
                    cmd_ready = true;
                    rx_index = 0;
                }
            } else {
                if (rx_index < (int)sizeof(rx_line) - 1) {
                    rx_line[rx_index++] = c;
                }
            }
        }
        break;

    default:
        break;
    }
}

static void gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    (void)event;
    (void)param;
}

void init_bluetooth()
{
    esp_err_t ret;

    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    //ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BTDM));

    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    ESP_ERROR_CHECK(esp_bt_gap_register_callback(gap_cb));
    ESP_ERROR_CHECK(esp_spp_register_callback(spp_cb));
    ESP_ERROR_CHECK(esp_spp_init(ESP_SPP_MODE_CB));
}

extern "C" void app_main()
{
    gpio_reset_pin(STEP_PIN);
    gpio_reset_pin(DIR_PIN);
    gpio_reset_pin(EN_PIN);

    gpio_set_direction(STEP_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(DIR_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(EN_PIN, GPIO_MODE_OUTPUT);

    gpio_set_level(EN_PIN, 0);
    gpio_set_level(DIR_PIN, 1);

    esp_rom_delay_us(2000);

    init_bluetooth();

    while (1)
    {
        if (cmd_ready) {
            cmd_ready = false;
            processCommand(rx_line);
            memset(rx_line, 0, sizeof(rx_line));
        }

        if (!motor_enabled) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        float position_deg = getPositionDeg();
        float error = target_deg - position_deg;
        float u = kp * error;

        if (fabs(error) <= 1.8f) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        bool dir = (u > 0);
        setDirection(dir);

        int delay_us = controlToDelayUs(fabs(u));
        pulseStep(delay_us);

        if (dir) stepCount++;
        else     stepCount--;

        // Ceder CPU para que corran otras tareas y no se dispare el watchdog
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}