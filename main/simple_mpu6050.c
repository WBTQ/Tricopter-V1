
#include "components_wrappers.h"

#include "hal/i2c_types.h"


static mpu6050_handle_t mpu6050_dev = NULL;
static mpu6050_acce_value_t acce;
static mpu6050_gyro_value_t gyro;
static mpu6050_gyro_value_t gyro_calibration = {3.495, -6.095, -1.057};
static complimentary_angle_t complimentary_angle;



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

void mpu6050_init()
{
	i2c_init();
    mpu6050_dev = mpu6050_create(I2C_NUM_0, MPU6050_I2C_ADDRESS);
    mpu6050_config(mpu6050_dev, ACCE_FS_4G, GYRO_FS_500DPS);
    mpu6050_wake_up(mpu6050_dev);
}

void mpu6050_read()
{    
    mpu6050_get_acce(mpu6050_dev, &acce);
    mpu6050_get_gyro(mpu6050_dev, &gyro);
    
    gyro.gyro_x -= gyro_calibration.gyro_x;
    gyro.gyro_y -= gyro_calibration.gyro_y;
    gyro.gyro_z -= gyro_calibration.gyro_z;

    // mpu6050_complimentory_filter(mpu6050_dev, &acce, &gyro, &complimentary_angle);

}