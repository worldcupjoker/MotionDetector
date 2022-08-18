/**
 * MPU6050.c
 */

#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_flash.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "MPU6050.h"

static const char *TAG = "i2c-mpu6050";

#define I2C_MASTER_SCL_IO           22                         /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           23                         /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          400000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

#define MPU6050_SENSOR_ADDR                 0x68        /*!< Slave address of the MPU9250 sensor */
#define MPU6050_WHO_AM_I_REG_ADDR           0x75        /*!< Register addresses of the "who am I" register */

#define MPU6050_PWR_MGMT_1_REG_ADDR         0x6B        /*!< Register addresses of the power managment register */
#define MPU6050_RESET_BIT                   7

const float STANDARD_GRAVITY = 9.80665;
const float ACCEL_SCALE = 16384;

// Function declarations
/**
 * @brief find the offset to calibrate the sensor.
 */
void calibrateSensor(reading *offSet, reading *realData) {
    printf("Calibrating the sensor. Please hold the sensor still for 5 seconds.\n");
    vTaskDelay(200);
    getRealData(realData);
    offSet->AcX = -realData->AcX;
    offSet->AcY = -realData->AcY;
    offSet->AcZ = -realData->AcZ;
    vTaskDelay(300);
    printf("Sensor calibration is done.\n");
}

/**
 * @brief get the actual reading after adjusting the data from the offset.
 */
void getActualReading(reading *offSet, reading *realData, reading *actualReading) {
    getRealData(realData);
    actualReading->AcX = realData->AcX + offSet->AcX;
    actualReading->AcY = realData->AcY + offSet->AcY;
    actualReading->AcZ = realData->AcZ + offSet->AcZ;
}

/**
 * @brief Read a sequence of bytes from a MPU6050 sensor registers
 */
esp_err_t getRawData(uint8_t *rawData, int rawDataLength)
{
    uint8_t dataAddr[1] = {0x3B};
    return i2c_master_write_read_device(I2C_MASTER_NUM, MPU6050_SENSOR_ADDR, dataAddr, sizeof(dataAddr), rawData, rawDataLength, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

/**
 * @brief get the averaged data from the raw data
 */

void getRealData(reading *realData) {
    int total = 100;
    float x = 0;
    float y = 0;
    float z = 0;
    for (int i = 0; i < total; i++) {
        uint8_t rawData[14];
        esp_err_t err = getRawData(&rawData, sizeof(rawData));
        if (err != ESP_OK) {
            printf("i2c error: %s",esp_err_to_name(err));
        }

        x += unsignedToSigned(rawData[0], rawData[1]) / ACCEL_SCALE * STANDARD_GRAVITY;
        y += unsignedToSigned(rawData[2], rawData[3]) / ACCEL_SCALE * STANDARD_GRAVITY;
        z += unsignedToSigned(rawData[4], rawData[5]) / ACCEL_SCALE * STANDARD_GRAVITY;
    }

    realData->AcX = x / total;
    realData->AcY = y / total;
    realData->AcZ = z / total;
}

/**
 * @brief i2c master initialization
 */
esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);
    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

/**
 * @brief unsigned 16 bits to signed 16 bits
 */
int16_t unsignedToSigned(uint8_t firstByte, uint8_t secondByte) {
    return (int16_t)(firstByte << 8 | secondByte);
}

/**
 * @brief change the sleep mode
 */
esp_err_t wakeUpSensor(void) {
    uint8_t writeBuf[2] = {MPU6050_PWR_MGMT_1_REG_ADDR, 0};
    return i2c_master_write_to_device(I2C_MASTER_NUM, MPU6050_SENSOR_ADDR, writeBuf, sizeof(writeBuf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}
