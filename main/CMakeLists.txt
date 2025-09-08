idf_component_register(SRCS "main.cpp" "simple_mpu6050.c" "simple_timer.c" "simple_servo_esc.c"
                    INCLUDE_DIRS "."
                    REQUIRES servo esp-dsp mpu6050 esp_timer esp_driver_uart esp_driver_ledc esp_driver_gpio)
