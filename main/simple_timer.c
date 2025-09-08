#include "components_wrappers.h"

#include "esp_timer.h"

esp_timer_create_args_t delay_timer_config = {
    .callback = NULL,
    .arg = NULL,
    .dispatch_method = ESP_TIMER_TASK,
    .name = "Delay timer",
    .skip_unhandled_events = true};


void periodic_timer(int period_us, esp_timer_cb_t callbackFunction, esp_timer_handle_t *timerHandle){

    esp_timer_create_args_t cal_timer_config = {
        .callback = callbackFunction,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "Periodic timer",
        .skip_unhandled_events = true};
    esp_timer_create(&cal_timer_config, timerHandle);
    esp_timer_start_periodic(*timerHandle, period_us);
}

void delay_timer(int period_us, esp_timer_cb_t callbackFunction){

    delay_timer_config.callback=callbackFunction;
    esp_timer_handle_t cal_timer = NULL;
    esp_timer_create(&delay_timer_config, &cal_timer);
    
    esp_timer_start_once(cal_timer, period_us);
}

void simple_timer_init(){
    esp_timer_early_init();
    esp_timer_init();
}
