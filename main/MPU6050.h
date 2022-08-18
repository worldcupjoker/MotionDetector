/**
 * MPU6050.h
 */

#ifndef MPU6050_H_   /* Include guard */
#define MPU6050_H_

typedef struct reading {
    float AcX;
    float AcY;
    float AcZ;
} reading;

esp_err_t getRawData(uint8_t *, int);
void getRealData(reading *);
esp_err_t i2c_master_init(void);
int16_t unsignedToSigned(uint8_t, uint8_t);
esp_err_t wakeUpSensor(void);
void calibrateSensor(reading *, reading *);
void getActualReading(reading *, reading *, reading *);

#endif