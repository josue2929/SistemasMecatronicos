#pragma once

#include "driver/gpio.h"
#include "hal/adc_types.h"

// -------------------- Stepper driver --------------------
#define IO_GPIO_STEPPER_STEP          GPIO_NUM_4
#define IO_GPIO_STEPPER_DIR           GPIO_NUM_22
#define IO_GPIO_STEPPER_ENABLE        GPIO_NUM_23

// -------------------- Level/proximity sensors --------------------
#define IO_GPIO_S1_LEVEL              GPIO_NUM_32
#define IO_GPIO_S2_LEVEL              GPIO_NUM_33
#define IO_GPIO_S3_LEVEL              GPIO_NUM_34

// -------------------- Pressure ADC --------------------
#define IO_GPIO_PRESSURE_ADC          GPIO_NUM_36
#define IO_ADC_UNIT_PRESSURE          ADC_UNIT_1
#define IO_ADC_CHANNEL_PRESSURE       ADC_CHANNEL_0

// -------------------- Temperature ADC --------------------
#define IO_GPIO_TEMPERATURE_T3_ADC    GPIO_NUM_39
#define IO_ADC_UNIT_TEMPERATURE_T3    ADC_UNIT_1
#define IO_ADC_CHANNEL_TEMPERATURE_T3 ADC_CHANNEL_3

// -------------------- Flowmeter --------------------
#define IO_GPIO_FLOWMETER             GPIO_NUM_27

// -------------------- Vibration sensor --------------------
#define IO_GPIO_VIBRATION             GPIO_NUM_21

// -------------------- HW-095 #1: pumps --------------------
#define IO_GPIO_PUMP1_IN1             GPIO_NUM_16
#define IO_GPIO_PUMP1_IN2             GPIO_NUM_17
#define IO_GPIO_PUMP2_IN1             GPIO_NUM_18
#define IO_GPIO_PUMP2_IN2             GPIO_NUM_19

// -------------------- HW-095 #2: valves --------------------
#define IO_GPIO_VALVE1_IN1            GPIO_NUM_25
#define IO_GPIO_VALVE1_IN2            GPIO_NUM_26
#define IO_GPIO_VALVE2_IN1            GPIO_NUM_13
#define IO_GPIO_VALVE2_IN2            GPIO_NUM_14
