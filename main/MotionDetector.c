/**
 * MotionDetector.c
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_flash.h"
#include "MPU6050.h"

void app_main(void)
{
    esp_err_t err = i2c_master_init();
    esp_err_t err1 = wakeUpSensor();
    reading realData;
    reading offSet;
    reading actualReading;
    calibrateSensor(&offSet, &realData);
    while (true) {
        getActualReading(&offSet, &realData, &actualReading);
        printf("%.6f, %.6f, %.6f\n", actualReading.AcX, actualReading.AcY, actualReading.AcZ);
        vTaskDelay(100);
    }
}
