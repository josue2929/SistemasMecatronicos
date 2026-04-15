idf_component_register(
    SRCS
        "main.cpp"
        "actuators.cpp"
        "sensors.cpp"
        "stepper_control.cpp"
        "bluetooth_spp.cpp"
    INCLUDE_DIRS "."
    REQUIRES bt nvs_flash driver esp_adc esp_timer
)
