#ifndef APP_H
#define APP_H
#include "board.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
typedef struct {
  safety_t safety;
  float duty[MOTOR_COUNT], velocity[ENCODER_COUNT];
  int64_t ticks[ENCODER_COUNT];
  uint32_t sample_ms;
  bool hardware_ok, encoders_valid;
} control_snapshot_t;
typedef struct {
  imu_sample_t imu;
  float metal;
  uint32_t sample_ms;
  bool metal_valid;
} sensor_snapshot_t;
extern QueueHandle_t command_queue, control_queue, sensor_queue;
extern TaskHandle_t control_task_handle, comm_task_handle;
extern volatile uint32_t app_control_heartbeat;
void app_start(bool hardware_ok);
void app_link_update(bool connected, bool clock_ok, uint64_t epoch_ms);
void app_epoch(uint32_t at_ms, uint64_t *epoch_ms, bool *connected, bool *clock_ok);
void serial_task(void *argument);
#endif
