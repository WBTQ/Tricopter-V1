idf_component_register(SRCS "main.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES servo esp-dsp mpu6050 esp_timer esp_driver_uart esp_driver_ledc esp_driver_gpio)
