#include "bluetooth_spp.h"

#include <stdio.h>
#include <string.h>

#include "app_config.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_check.h"
#include "esp_gap_bt_api.h"
#include "esp_log.h"
#include "esp_spp_api.h"
#include "freertos/queue.h"
#include "nvs_flash.h"

namespace {

uint32_t s_spp_client_handle = 0;
QueueHandle_t s_command_queue = nullptr;
char s_rx_line[APP_BT_COMMAND_MAX_LEN] = {};
size_t s_rx_index = 0;

void enqueue_command()
{
    if (s_command_queue == nullptr || s_rx_index == 0) {
        return;
    }

    s_rx_line[s_rx_index] = '\0';
    (void)xQueueSend(s_command_queue, s_rx_line, 0);
    memset(s_rx_line, 0, sizeof(s_rx_line));
    s_rx_index = 0;
}

void spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t* param)
{
    switch (event) {
    case ESP_SPP_INIT_EVT: {
        esp_bt_gap_set_device_name(BT_DEVICE_NAME);
        esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);

        esp_spp_start_srv_cfg_t srv_cfg = {};
        srv_cfg.local_scn = 0;
        srv_cfg.create_spp_record = true;
        srv_cfg.sec_mask = ESP_SPP_SEC_NONE;
        srv_cfg.role = ESP_SPP_ROLE_SLAVE;
        srv_cfg.name = SPP_SERVER_NAME;
        ESP_ERROR_CHECK(esp_spp_start_srv_with_cfg(&srv_cfg));
        ESP_LOGI(SPP_TAG, "SPP init done");
        break;
    }

    case ESP_SPP_START_EVT:
        ESP_LOGI(SPP_TAG, "SPP server started");
        break;

    case ESP_SPP_SRV_OPEN_EVT:
        s_spp_client_handle = param->srv_open.handle;
        ESP_LOGI(SPP_TAG, "Client connected, handle=%lu", static_cast<unsigned long>(s_spp_client_handle));
        bluetooth_spp_send_line("Bluetooth connected. Send HELP for commands.");
        break;

    case ESP_SPP_CLOSE_EVT:
        s_spp_client_handle = 0;
        ESP_LOGI(SPP_TAG, "Client disconnected");
        break;

    case ESP_SPP_DATA_IND_EVT:
        for (int i = 0; i < param->data_ind.len; i++) {
            const char c = static_cast<char>(param->data_ind.data[i]);

            if (c == '\r') {
                continue;
            }

            if (c == '\n') {
                enqueue_command();
            } else if (s_rx_index < (sizeof(s_rx_line) - 1U)) {
                s_rx_line[s_rx_index++] = c;
            } else {
                s_rx_index = 0;
                memset(s_rx_line, 0, sizeof(s_rx_line));
                bluetooth_spp_send_line("ERR command too long");
            }
        }
        break;

    default:
        break;
    }
}

void gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t* param)
{
    (void)event;
    (void)param;
}

esp_err_t init_nvs()
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_RETURN_ON_ERROR(nvs_flash_erase(), SPP_TAG, "nvs_flash_erase failed");
        ret = nvs_flash_init();
    }
    return ret;
}

}  // namespace

esp_err_t bluetooth_spp_init()
{
    if (s_command_queue == nullptr) {
        s_command_queue = xQueueCreate(APP_BT_COMMAND_QUEUE_LENGTH, APP_BT_COMMAND_MAX_LEN);
        if (s_command_queue == nullptr) {
            return ESP_ERR_NO_MEM;
        }
    }

    ESP_RETURN_ON_ERROR(init_nvs(), SPP_TAG, "nvs init failed");

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_bt_controller_init(&bt_cfg), SPP_TAG, "bt controller init failed");
    ESP_RETURN_ON_ERROR(esp_bt_controller_enable(ESP_BT_MODE_BTDM), SPP_TAG, "bt controller enable failed");

    ESP_RETURN_ON_ERROR(esp_bluedroid_init(), SPP_TAG, "bluedroid init failed");
    ESP_RETURN_ON_ERROR(esp_bluedroid_enable(), SPP_TAG, "bluedroid enable failed");

    ESP_RETURN_ON_ERROR(esp_bt_gap_register_callback(gap_cb), SPP_TAG, "gap callback failed");
    ESP_RETURN_ON_ERROR(esp_spp_register_callback(spp_cb), SPP_TAG, "spp callback failed");

    esp_spp_cfg_t spp_cfg = {};
    spp_cfg.mode = ESP_SPP_MODE_CB;
    spp_cfg.enable_l2cap_ertm = true;
    spp_cfg.tx_buffer_size = 0;
    ESP_RETURN_ON_ERROR(esp_spp_enhanced_init(&spp_cfg), SPP_TAG, "spp init failed");
    return ESP_OK;
}

bool bluetooth_spp_receive_command(char* out, size_t out_len, TickType_t wait_ticks)
{
    if (s_command_queue == nullptr || out == nullptr || out_len == 0U) {
        return false;
    }

    char line[APP_BT_COMMAND_MAX_LEN] = {};
    if (xQueueReceive(s_command_queue, line, wait_ticks) != pdTRUE) {
        return false;
    }

    strncpy(out, line, out_len - 1U);
    out[out_len - 1U] = '\0';
    return true;
}

bool bluetooth_spp_is_connected()
{
    return s_spp_client_handle != 0;
}

void bluetooth_spp_send(const char* msg)
{
    if (s_spp_client_handle != 0 && msg != nullptr) {
        esp_spp_write(s_spp_client_handle, strlen(msg), reinterpret_cast<uint8_t*>(const_cast<char*>(msg)));
    }
}

void bluetooth_spp_send_line(const char* msg)
{
    if (msg == nullptr) {
        return;
    }

    bluetooth_spp_send(msg);
    bluetooth_spp_send("\r\n");
}
