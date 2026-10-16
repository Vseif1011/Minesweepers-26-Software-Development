#include "imu.h"
#include <math.h>
#include <string.h>
bool imu_init(void) { return board_imu.init && board_imu.read && board_imu.init(); }
bool imu_read(imu_sample_t *s) {
  memset(s,0,sizeof(*s));
  if (!board_imu.read || !board_imu.read(s)) return false;
  for (unsigned i=0;i<3;++i)
    if (!isfinite(s->accel[i]) || !isfinite(s->gyro[i])) return false;
  s->stamp_ms=HAL_GetTick(); s->valid=true; return true;
}
HAL_StatusTypeDef imu_i2c_read(I2C_HandleTypeDef *bus, uint8_t address7,
                             uint8_t reg, uint8_t *out, uint16_t size) {
  if (!bus || !out || !size || address7>127) return HAL_ERROR;
  return HAL_I2C_Mem_Read(bus,(uint16_t)address7<<1,reg,I2C_MEMADD_SIZE_8BIT,out,size,2);
}
HAL_StatusTypeDef imu_spi_read(SPI_HandleTypeDef *bus, gpio_signal_t cs,
                             uint8_t command, uint8_t *out, uint16_t size) {
  if (!bus || !cs.port || !out || !size) return HAL_ERROR;
  gpio_write(cs,true);
  HAL_StatusTypeDef status=HAL_SPI_Transmit(bus,&command,1,2);
  if (status==HAL_OK) status=HAL_SPI_Receive(bus,out,size,2);
  gpio_write(cs,false); return status;
}
