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

#include "mpu6050.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "components_wrappers.h"
}

#define RELATIVE_DUTY(p)		(int)(( 100.0 + p ) * (LEDC_DUTY_RANGE)/100)

static mpu6050_acce_value_t acce;
static mpu6050_gyro_value_t gyro;
static complimentary_angle_t complimentary_angle;

uint32_t dutycycles[]={LEDC_DUTY_MID,LEDC_DUTY_MAX,LEDC_DUTY_MID,LEDC_DUTY_MIN} ;
uint32_t goal_duty;

esp_timer_handle_t sensor_timer;
TaskHandle_t xTestTaskHandle = NULL;
TaskHandle_t xSensorTaskHandle = NULL;
TaskHandle_t xMotorTaskHandle = NULL;
TaskHandle_t xMotorFadeTaskHandle = NULL;
uint8_t newdata;


extern "C" void app_main(void);





void vTestTaskCode( void * pvParameters ){		

}
void timerCallbackSensor(void *arg) {
    newdata = 1;
    xTaskNotifyFromISR(xSensorTaskHandle,0,eNoAction,0);
}
void vSensorTaskCode( void * pvParameters )
{
	// In order to get accurate calculation of complimentary angle we need fast reading (5ms)
    // FreeRTOS resolution is slow, so esp_timer is used

    periodic_timer(CL_PERIOD, timerCallbackSensor, &sensor_timer);
    
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

        xTaskNotifyWait(0,ULONG_MAX,0,pdMS_TO_TICKS(1));
            mpu6050_read();

        printf("%f ", acce.acce_x);
        printf("%f ", acce.acce_y);
        printf("%f ", acce.acce_z);
		printf("%f ", gyro.gyro_x );
		printf("%f ", gyro.gyro_y);
		printf("%f ", gyro.gyro_z);		
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
    motor_init();
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
