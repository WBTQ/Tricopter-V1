/* LEDC (LED Controller) basic example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/


#include <stdint.h>
#include <stdio.h>
#include "esp_dsp.h"
#include "ekf_imu13states.h"
//#include "arch/sys_arch.h"

extern "C" {
#include "driver/ledc.h"
#include "i2c_periph.c"
#include "driver/uart.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "hal/i2c_types.h"
#include "mpu6050.h"
#include "soc/gpio_num.h"

void app_main(void);
}

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
#define RELATIVE_DUTY(p)		(int)(( 100.0 + p ) * (LEDC_DUTY_RANGE)/100)

uint32_t dutycycles[]={LEDC_DUTY_MID,LEDC_DUTY_MAX,LEDC_DUTY_MID,LEDC_DUTY_MIN} ;
static mpu6050_handle_t mpu6050_dev = NULL;
static mpu6050_acce_value_t acce;
static mpu6050_gyro_value_t gyro;
static mpu6050_gyro_value_t gyro_calibration = {3.495, -6.095, -1.057};
static complimentary_angle_t complimentary_angle;
uint32_t goal_duty;

TaskHandle_t xTestTaskHandle = NULL;
TaskHandle_t xSensorTaskHandle = NULL;
TaskHandle_t xMotorTaskHandle = NULL;
TaskHandle_t xMotorFadeTaskHandle = NULL;
uint8_t newdata;

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

static void mpu6050_init()
{
    mpu6050_dev = mpu6050_create(I2C_NUM_0, MPU6050_I2C_ADDRESS);
    mpu6050_config(mpu6050_dev, ACCE_FS_4G, GYRO_FS_500DPS);
    mpu6050_wake_up(mpu6050_dev);
}
static void mpu6050_read()
{    
    mpu6050_get_acce(mpu6050_dev, &acce);
    mpu6050_get_gyro(mpu6050_dev, &gyro);
    
    gyro.gyro_x -= gyro_calibration.gyro_x;
    gyro.gyro_y -= gyro_calibration.gyro_y;
    gyro.gyro_z -= gyro_calibration.gyro_z;

    // mpu6050_complimentory_filter(mpu6050_dev, &acce, &gyro, &complimentary_angle);

}

static bool i2c_initialized = false;
esp_err_t i2c_init(void)
{
    /* I2C was initialized before */
    if (i2c_initialized) {
        return ESP_OK;
    }

    const i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = GPIO_NUM_6,
        .scl_io_num = GPIO_NUM_7,
        .sda_pullup_en = GPIO_PULLUP_DISABLE,
        .scl_pullup_en = GPIO_PULLUP_DISABLE,
        .master = {.clk_speed = 400000},
        .clk_flags = 0
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &i2c_conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, i2c_conf.mode,
    									 0, 0, 0));

    i2c_initialized = true;

    return ESP_OK;
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

void vTestTaskCode( void * pvParameters ){		

}
void timerCallbackSensor(void *pvParameters) {
    newdata = 1;
}
void vSensorTaskCode( void * pvParameters )
{
	// In order to get accurate calculation of complimentary angle we need fast reading (5ms)
    // FreeRTOS resolution is slow, so esp_timer is used
    const esp_timer_create_args_t cal_timer_config = {
        .callback = timerCallbackSensor,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "MPU6050 timer",
        .skip_unhandled_events = true};
    esp_timer_handle_t cal_timer = NULL;
    esp_timer_create(&cal_timer_config, &cal_timer);
    esp_timer_start_periodic(cal_timer, 1000000); // 5ms
    
    ekf_imu13states *ekf13 = new ekf_imu13states();
    ekf13->Init();
    // Set up some initial values to emulate and calculate system values
    int total_N = 3000;
    // Pi value
    float pi = std::atan(1) * 4;

    // gyroscope bias error
    float gyro_err_data[] = {0.1, 0.2, 0.3}; // static constatnt error
    dspm::Mat gyro_err(gyro_err_data, 3, 1);


    // Measurement noise covariance values for diagonal covariance matrix.
    // For the real system these values could be adjusted!
    // These calues depends on how noisy the measurement.
    //
    float R[10];
    for (size_t i = 0; i < 10; i++) {
        R[i] = 0.01;
    }

    // Reference vectors
    float accel0_data[] = {0, 0, 1};
    // In real system magnetometer vector will have different value and direction
    // The EKF will calculate them. This value is used as initial state.
    float magn0_data[] = {1, 0, 0};

    dspm::Mat accel0(accel0_data, 3, 1);
    dspm::Mat magn0(magn0_data, 3, 1);
    
    float dt = 0.01;

    dspm::Mat gyro_data(3, 1);
    int count = 0;

    // Initial rotation matrix
    dspm::Mat Rm = dspm::Mat::eye(3);
    dspm::Mat Re = dspm::Mat::eye(3);

    gyro_err *= 1;

    std::cout << "Gyro error: " << gyro_err.t() << std::endl;
    std::cout << "Calibration phase started: " << std::endl;


    printf("sensor timer setup done \n");	
    
    printf("Data: x,y,z\n");
	for( ;; )
	{
		
	 	// uint32_t angle_duty = RELATIVE_DUTY(complimentary_angle.pitch / 6.28 * 100);
		// printf("p %f \n", complimentary_angle.pitch);
		// printf("r %f \n", complimentary_angle.roll);
		// printf("d %d \n \n", (int)(angle_duty));

        float cal_x, cal_y, cal_z;
        for(int i=0;i<1000;i++){
            while (!newdata){}
            mpu6050_read();
            cal_x += gyro.gyro_x;
            cal_y += gyro.gyro_y;
            cal_z += gyro.gyro_z;
    }
        // printf("%f ", acce.acce_x);
		// printf("%f ", acce.acce_y);
		// printf("%f ", acce.acce_z);
		printf("%f ", gyro.gyro_x );
		printf("%f ", gyro.gyro_y);
		printf("%f ", gyro.gyro_z);
        
		// printf("%f ", gyro.gyro_x);
		// printf("%f ", gyro.gyro_y);
		// printf("%f ", gyro.gyro_z);
		
		printf("\n");
		
		// vTaskDelay(200 / portTICK_PERIOD_MS);
	}
 
}
void vMotorFadeTaskCode( void * pvParameters )
{	
	int i=0;
	for( ;; )
	{
	    while (i<=20) {
			goal_duty= RELATIVE_DUTY(i);
			setAllPinDuty(goal_duty,goal_duty,goal_duty,goal_duty);
			vTaskDelay(LEDC_DELAY_IN_US / portTICK_PERIOD_MS);
		    i+=5;
	    }
	    while (i>0) {
		    i-=5;
			goal_duty= RELATIVE_DUTY(i);
			setAllPinDuty(goal_duty,goal_duty,goal_duty,goal_duty);
			vTaskDelay(LEDC_DELAY_IN_US / portTICK_PERIOD_MS);
	    }
	    i=0;
 	}
}
void app_main(void)
{
	// Set the LEDC peripheral configuration
    esc_ledc_init();
	i2c_init();
    mpu6050_init();
    
    static uint8_t ucParameterToPass;
	
	printf("init finished \n");
	xTaskCreate(vSensorTaskCode,"SensorTask", 10000,
		 &ucParameterToPass, tskIDLE_PRIORITY, &xSensorTaskHandle );
    // configASSERT( xSensorTaskHandle ); // Use the handle to delete the task.

    //if( xSensorTaskHandle != NULL )     vTaskDelete( xSensorTaskHandle );
    
    
    
/*    xTaskCreate(vMotorFadeTaskCode,"MotorTestTask", 200,
    	 &ucParameterToPass, tskIDLE_PRIORITY, &xMotorFadeTaskHandle);
    configASSERT( xSensorTaskHandle ); // Use the handle to delete the task.
    if( xSensorTaskHandle != NULL )		vTaskDelete( xSensorTaskHandle );
*/
    
    while(1){

    }
}
