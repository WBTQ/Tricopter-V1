#include "components_wrappers.h"

#include "ledc.h"
#include "iot_servo.h"


servo_config_t servo_cfg = {
    .max_angle = 180,
    .min_width_us = 500,
    .max_width_us = 2500,
    .freq = 400,
    .timer_number = LEDC_TIMER_0,
    .channels = {
        .servo_pin = {
            PWM_IO_1,
            PWM_IO_2,
            PWM_IO_3,
            PWM_IO_4,
        },
        .ch = {
            LEDC_CHANNEL1,
            LEDC_CHANNEL2,
            LEDC_CHANNEL3,
            LEDC_CHANNEL4,
        },
    },
    .channel_number = 4,
};


void motor_init(){

    // Initialize the servo
    iot_servo_init(LEDC_LOW_SPEED_MODE, &servo_cfg);
    
    //esc_ledc_init();

}

void set_servo_angle(uint16_t angle){

    iot_servo_write_angle(LEDC_LOW_SPEED_MODE, 0, angle);

}


static void esc_ledc_init(void)
{
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .duty_resolution  = LEDC_DUTY_RES,
        .timer_num        = LEDC_TIMER,
        .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 4 kHz
        .clk_cfg          = LEDC_AUTO_CLK,
        .deconfigure      = 0
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t pwm_channel1 = {
        .gpio_num       = PWM_IO_1,
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL1,
        .intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = LEDC_TIMER,
        .duty           = 819, // Set duty to 0%
        .hpoint         = 0,
        .flags          = 0
    };
    ledc_channel_config_t pwm_channel2 = {
        .gpio_num       = PWM_IO_2,
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL2,
        .intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = LEDC_TIMER,
        .duty           = 819, // Set duty to 0%
        .hpoint         = 0,
        .flags          = 0
    };
    ledc_channel_config_t pwm_channel3 = {
        .gpio_num       = PWM_IO_3,
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL3,
        .intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = LEDC_TIMER,
        .duty           = 819, // Set duty to 0%
        .hpoint         = 0,
        .flags          = 0
    };
    ledc_channel_config_t pwm_channel4 = {
        .gpio_num       = PWM_IO_4,
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL4,
        .intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = LEDC_TIMER,
        .duty           = 819, // Set duty to 0%
        .hpoint         = 0,
        .flags          = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&pwm_channel1));
    ESP_ERROR_CHECK(ledc_channel_config(&pwm_channel2));
    ESP_ERROR_CHECK(ledc_channel_config(&pwm_channel3));
    ESP_ERROR_CHECK(ledc_channel_config(&pwm_channel4));
}




void setPin1Duty(uint32_t duty){
	    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL1, duty));
	    // Update duty to apply the new value
	    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL1));	
}

void setAllPinDuty(uint32_t d1,uint32_t d2,uint32_t d3,uint32_t d4){
	    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL1, d1));
	    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL2, d2));
	    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL3, d3));
	    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL4, d4));
	    // Update duty to apply the new value
	    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL1));
	    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL2));
	    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL3));
	    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL4));	
}