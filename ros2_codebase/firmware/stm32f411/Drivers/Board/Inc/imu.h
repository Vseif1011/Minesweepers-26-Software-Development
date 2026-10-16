#ifndef IMU_H
#define IMU_H
#include "board.h"
/* Bus binding alone is insufficient: a confirmed model supplies register and
 * unit conversion callbacks. No MPU6050 assumption is inherited from Pi code. */
typedef struct {
  bool (*init)(void);
  bool (*read)(imu_sample_t *sample); /* SI units, mounting documented separately */
} imu_driver_t;
extern const imu_driver_t board_imu;
HAL_StatusTypeDef imu_i2c_read(I2C_HandleTypeDef *bus, uint8_t address7,
                             uint8_t reg, uint8_t *out, uint16_t size);
HAL_StatusTypeDef imu_spi_read(SPI_HandleTypeDef *bus, gpio_signal_t cs,
                             uint8_t command, uint8_t *out, uint16_t size);
#endif
