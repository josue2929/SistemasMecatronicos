#pragma once

#include <stddef.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"

esp_err_t bluetooth_spp_init();
bool bluetooth_spp_receive_command(char* out, size_t out_len, TickType_t wait_ticks);
bool bluetooth_spp_is_connected();
void bluetooth_spp_send(const char* msg);
void bluetooth_spp_send_line(const char* msg);
