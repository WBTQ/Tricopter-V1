#ifndef COMPONENT_WRAPPER_H
#define COMPONENT_WRAPPER_H


#include "mpu6050.h"
#include "soc/gpio_num.h"


#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define PWM_IO_1          (5) // Define the output GPIO
#define PWM_IO_2          (18) // Define the output GPIO
#define PWM_IO_3          (19) // Define the output GPIO
#define PWM_IO_4          (17) // Define the output GPIO
#define LEDC_CHANNEL1            LEDC_CHANNEL_0
#define LEDC_CHANNEL2            LEDC_CHANNEL_1
#define LEDC_CHANNEL3            LEDC_CHANNEL_2
#define LEDC_CHANNEL4            LEDC_CHANNEL_3
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY_MAX               (1638) // Set duty to 50%. (2 ** 13) * 50% = 4096
#define LEDC_DUTY_MID               (1229) 
#define LEDC_DUTY_MIN               (819) 
#define LEDC_DUTY_RANGE             (819.0) 
#define LEDC_FREQUENCY          	(100) // Frequency in Hertz. Set frequency at 100 Hz
#define LEDC_DELAY_IN_US          	(2000)

extern void mpu6050_init();
extern void mpu6050_read();
static void motor_init();



#endif COMPONENT_WRAPPER_H
