#include "imu.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
I2C_HandleTypeDef hi2c1;
static uint8_t registers[128];static bool bus_failure,bad_offset_readback;static uint32_t waited;
void vTaskDelay(uint32_t ticks) { waited+=ticks; }
HAL_StatusTypeDef imu_i2c_read(I2C_HandleTypeDef *bus,uint8_t address,uint8_t reg,uint8_t *out,uint16_t size) {
  assert(bus==&hi2c1 && address==0x68 && reg+size<=128);
  if(bus_failure) return HAL_ERROR;
  memcpy(out,registers+reg,size);
  if(bad_offset_readback && reg==0x06) out[0]^=1;
  return HAL_OK;
}
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *bus,uint16_t address,uint16_t reg,uint16_t width,uint8_t *data,uint16_t size,uint32_t timeout) {
  assert(bus==&hi2c1 && address==0xd0 && width==1 && (size==1 || size==2) && timeout==2);
  if(bus_failure) return HAL_ERROR;
  memcpy(registers+reg,data,size);return HAL_OK;
}
static void word(unsigned reg,int16_t value) { registers[reg]=(uint16_t)value>>8;registers[reg+1]=(uint8_t)value; }
int main(void) {
  assert(!board_imu.init()); /* wrong identity */
  registers[0x75]=0x68;assert(board_imu.init());assert(waited==100);
  assert(registers[0x19]==19 && registers[0x1a]==3 && registers[0x6b]==1);
  const uint8_t offset_registers[]={0x06,0x08,0x0a,0x13,0x15,0x17};
  const int16_t expected[]={-2789,-4452,1391,97,-40,-51};
  for(unsigned i=0;i<6;++i) {
    uint16_t actual=(uint16_t)registers[offset_registers[i]]<<8|registers[offset_registers[i]+1];
    assert(actual==(uint16_t)expected[i]);
  }
  bad_offset_readback=true;assert(!board_imu.init());bad_offset_readback=false;
  imu_sample_t s={0};assert(!board_imu.read(&s)); /* no new sample */
  registers[0x3a]=1;word(0x3b,16384);word(0x3d,-16384);word(0x3f,0);
  word(0x43,131);word(0x45,-131);word(0x47,0);
  assert(board_imu.read(&s));assert(fabsf(s.accel[0]-9.80665f)<.0001f);
  assert(fabsf(s.accel[1]+9.80665f)<.0001f && s.accel[2]==0);
  assert(fabsf(s.gyro[0]-.0174532925f)<.00001f && s.gyro[1]<0);
  word(0x43,INT16_MIN);assert(!board_imu.read(&s)); /* saturated sensor */
  bus_failure=true;assert(!board_imu.read(&s));assert(!board_imu.init());
  puts("MPU6050 tests: identity, configuration, exact signed user offsets/readback, freshness, SI conversion, saturation and bus failure passed");
}
