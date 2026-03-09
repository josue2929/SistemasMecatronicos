#pragma once

#include <stdio.h>
#include <string.h>
#include <math.h>

#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_spp_api.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// -------------------- Pines --------------------
static const gpio_num_t STEP_PIN = GPIO_NUM_21;
static const gpio_num_t DIR_PIN  = GPIO_NUM_22;
static const gpio_num_t EN_PIN   = GPIO_NUM_23;

// -------------------- Motor --------------------
static const int STEPS_PER_REV = 200;
static const float DEG_PER_STEP = 360.0f / STEPS_PER_REV;

// -------------------- Bluetooth --------------------
#define BT_DEVICE_NAME "JosueServer"
#define SPP_SERVER_NAME "SPP_SERVER"
#define SPP_TAG "BT_STEPPER"

// -------------------- Control --------------------
extern long stepCount;
extern float target_deg;
extern float kp;
extern bool motor_enabled;

extern float min_delay_us;
extern float max_delay_us;

// -------------------- RX Bluetooth --------------------
extern char rx_line[128];
extern volatile bool cmd_ready;
extern int rx_index;

// -------------------- Funciones --------------------
void pulseStep(int delay_us);
void setDirection(bool dir);
float getPositionDeg();
int controlToDelayUs(float control_abs);
void processCommand(const char* cmd);
void init_bluetooth();