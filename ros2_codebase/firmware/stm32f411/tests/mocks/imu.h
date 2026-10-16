#ifndef MOCK_IMU_H
#define MOCK_IMU_H
#include "board.h"
typedef struct { unsigned id; } I2C_HandleTypeDef;
typedef int HAL_StatusTypeDef;
#define HAL_OK 0
#define HAL_ERROR 1
#define I2C_MEMADD_SIZE_8BIT 1
typedef struct { float accel[3],gyro[3];uint32_t stamp_ms;bool valid; } imu_sample_t;
typedef struct { bool (*init)(void);bool (*read)(imu_sample_t *); } imu_driver_t;
extern const imu_driver_t board_imu;
HAL_StatusTypeDef imu_i2c_read(I2C_HandleTypeDef *,uint8_t,uint8_t,uint8_t *,uint16_t);
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *,uint16_t,uint16_t,uint16_t,uint8_t *,uint16_t,uint32_t);
#endif
