/* Register definitions/scales: InvenSense RM-MPU-6000A-00 Rev 4.2, in
 * rpi_ws/src/mpu6050/DataSheet/MPU-6000-Register-Map.pdf. */
#include "imu.h"
#include "peripherals.h"
#include "mpu6050_config.h"
#include "FreeRTOS.h"
#include "task.h"
#define MPU_ADDRESS 0x68u /* chosen wiring: AD0 -> GND */
static bool write_register(uint8_t reg,uint8_t value) {
  return HAL_I2C_Mem_Write(&hi2c1,MPU_ADDRESS<<1,reg,I2C_MEMADD_SIZE_8BIT,&value,1,2)==HAL_OK;
}
static bool write_offset(uint8_t reg,int16_t offset) {
  uint8_t bytes[2]={(uint8_t)((uint16_t)offset>>8),(uint8_t)offset},readback[2];
  if (HAL_I2C_Mem_Write(&hi2c1,MPU_ADDRESS<<1,reg,I2C_MEMADD_SIZE_8BIT,bytes,2,2)!=HAL_OK)
    return false;
  if (imu_i2c_read(&hi2c1,MPU_ADDRESS,reg,readback,2)!=HAL_OK) return false;
  return readback[0]==bytes[0] && readback[1]==bytes[1];
}
static bool init(void) {
  uint8_t id;
  if (imu_i2c_read(&hi2c1,MPU_ADDRESS,0x75,&id,1)!=HAL_OK || id!=0x68) return false;
  if (!write_register(0x6b,0x80)) return false;
  vTaskDelay(pdMS_TO_TICKS(100));
  const uint8_t setup[][2]={{0x6b,1},{0x6c,0},{0x1a,3},{0x19,19},{0x1b,0},{0x1c,0}};
  for (unsigned i=0;i<sizeof(setup)/sizeof(*setup);++i) {
    if (!write_register(setup[i][0],setup[i][1])) return false;
    uint8_t value;
    if (imu_i2c_read(&hi2c1,MPU_ADDRESS,setup[i][0],&value,1)!=HAL_OK || value!=setup[i][1]) return false;
  }
  /* Exact signed 16-bit writes, including low bits, as the user's setters do.
   * Registers are volatile; reapply after every sensor reset and verify them. */
  const struct { uint8_t reg;int16_t value; } offsets[]={
    {0x06,MPU6050_ACCEL_X_OFFSET},{0x08,MPU6050_ACCEL_Y_OFFSET},{0x0a,MPU6050_ACCEL_Z_OFFSET},
    {0x13,MPU6050_GYRO_X_OFFSET},{0x15,MPU6050_GYRO_Y_OFFSET},{0x17,MPU6050_GYRO_Z_OFFSET}
  };
  for (unsigned i=0;i<sizeof(offsets)/sizeof(*offsets);++i)
    if (!write_offset(offsets[i].reg,offsets[i].value)) return false;
  return true;
}
static int16_t signed_word(const uint8_t *b) { return (int16_t)((uint16_t)b[0]<<8|b[1]); }
static bool read_sample(imu_sample_t *s) {
  uint8_t status,raw[14];
  if (imu_i2c_read(&hi2c1,MPU_ADDRESS,0x3a,&status,1)!=HAL_OK || !(status&1)) return false;
  if (imu_i2c_read(&hi2c1,MPU_ADDRESS,0x3b,raw,sizeof(raw))!=HAL_OK) return false;
  for (unsigned i=0;i<3;++i) {
    int16_t a=signed_word(raw+2*i),g=signed_word(raw+8+2*i);
    if (a==INT16_MIN || a==INT16_MAX || g==INT16_MIN || g==INT16_MAX) return false;
    s->accel[i]=a*(9.80665f/16384.0f);
    s->gyro[i]=g*(0.017453292519943f/131.0f);
  }
  return true;
}
const imu_driver_t board_imu={init,read_sample};
