#ifndef MPU6050_CONFIG_H
#define MPU6050_CONFIG_H
/* User-provided MPU6050 hardware offset register values, matching I2Cdevlib
 * set[XYZ]AccelOffset / set[XYZ]GyroOffset. These are not SI-unit subtractions. */
#define MPU6050_ACCEL_X_OFFSET (-2789)
#define MPU6050_ACCEL_Y_OFFSET (-4452)
#define MPU6050_ACCEL_Z_OFFSET (1391)
#define MPU6050_GYRO_X_OFFSET (97)
#define MPU6050_GYRO_Y_OFFSET (-40)
#define MPU6050_GYRO_Z_OFFSET (-51)
#endif
