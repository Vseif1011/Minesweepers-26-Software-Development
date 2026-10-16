#include "app.h"
#include "serial_protocol.h"
#include "serial_transport.h"
#include <string.h>
/* Wire scalars are little endian IEEE754; memcpy avoids unaligned accesses. */
static uint8_t payload[SERIAL_MAX_PAYLOAD],frame[SERIAL_MAX_FRAME];
static size_t offset;
static uint32_t sync_token;
static bool sync_pending;
static void put(const void *value,size_t len) { memcpy(payload+offset,value,len);offset+=len; }
static void send(uint8_t type) {
  size_t len=serial_encode(type,payload,offset,frame);
  (void)serial_transport_write(frame,len);offset=0;
}
static void receive(uint8_t type,const uint8_t *data,size_t len,void *context) {
  (void)context;uint32_t now=HAL_GetTick();
  if (type==SERIAL_SYNC_RESPONSE && len==12 && sync_pending) {
    uint32_t token;uint64_t epoch;
    memcpy(&token,data,4);memcpy(&epoch,data+4,8);
    uint32_t elapsed=now-sync_token;
    if (token==sync_token && elapsed<=40 && epoch>=UINT64_C(1700000000000) &&
        epoch<UINT64_C(4102444800000)) {
      sync_pending=false;
      app_link_update(true,true,epoch+elapsed/2);
    }
  } else if (type==SERIAL_COMMAND && len==37) {
    motor_command_t cmd={0};
    memcpy(&cmd.session,data,4);memcpy(&cmd.sequence,data+4,4);
    memcpy(&cmd.issued_at_ms,data+8,8);
    if (data[16]>1) { app_link_update(false,false,0);return; }
    cmd.arm=data[16]!=0;memcpy(cmd.target,data+17,sizeof(cmd.target));
    xQueueOverwrite(command_queue,&cmd);
  }
}
static void publish_samples(unsigned cycle) {
  control_snapshot_t c;sensor_snapshot_t s;
  if (xQueuePeek(control_queue,&c,0)!=pdTRUE) return;
  uint32_t now=HAL_GetTick();uint64_t epoch;bool link,clock;
  app_epoch(now,&epoch,&link,&clock);
  if ((uint32_t)(now-c.sample_ms)>50) return;
  bool sensors=xQueuePeek(sensor_queue,&s,0)==pdTRUE && (uint32_t)(now-s.sample_ms)<100;
  if (cycle%5==0) {
    uint8_t flags=(c.safety.armed?1:0)|(c.safety.ready?2:0)|(c.hardware_ok?4:0)|
      (c.encoders_valid?8:0)|(sensors && s.imu.valid?16:0)|(sensors && s.metal_valid?32:0)|
      (clock?64:0)|(link?128:0);
    uint32_t control_free=uxTaskGetStackHighWaterMark(control_task_handle);
    uint32_t comm_free=uxTaskGetStackHighWaterMark(comm_task_handle);
    uint32_t heap=xPortGetFreeHeapSize(),heap_min=xPortGetMinimumEverFreeHeapSize();
    put(&c.safety.session,4);put(&c.safety.sequence,4);put(&c.safety.faults,4);put(&now,4);
    put(&flags,1);put(c.duty,sizeof(c.duty));put(&control_free,4);put(&comm_free,4);
    put(&heap,4);put(&heap_min,4);send(SERIAL_STATUS);
  }
  if (cycle%2!=0) return;
  if (clock && c.encoders_valid) {
    uint64_t stamp=epoch-(uint32_t)(now-c.sample_ms);
    put(&stamp,8);put(&c.sample_ms,4);put(c.ticks,sizeof(c.ticks));
    put(c.velocity,sizeof(c.velocity));send(SERIAL_ENCODERS);
  }
  if (!sensors) return;
  if (clock && s.imu.valid) {
    uint64_t stamp=epoch-(uint32_t)(now-s.imu.stamp_ms);
    put(&stamp,8);put(&s.imu.stamp_ms,4);put(s.imu.accel,sizeof(s.imu.accel));
    put(s.imu.gyro,sizeof(s.imu.gyro));send(SERIAL_IMU);
  }
  if (s.metal_valid) { put(&s.metal,4);send(SERIAL_METAL); }
}
void serial_task(void *argument) {
  (void)argument;serial_parser_t parser={0};uint8_t buffer[256];
  uint32_t sync_at=HAL_GetTick()-500,last_rx=HAL_GetTick();unsigned cycle=0;
  TickType_t wake=xTaskGetTickCount();
  serial_transport_reset();app_link_update(false,false,0);
  for (;;) {
    uint32_t now=HAL_GetTick();size_t count=0;
    if (!serial_transport_configured() || !serial_transport_read(buffer,sizeof(buffer),&count)) {
      parser.used=0;sync_pending=false;app_link_update(false,false,0);
      xQueueReset(command_queue);
    } else {
      if (parser.used && (uint32_t)(now-last_rx)>100) parser.used=0;
      if (count) { serial_feed(&parser,buffer,count,receive,NULL);last_rx=now; }
      if ((uint32_t)(now-sync_at)>=500) {
        sync_at=sync_token=now;sync_pending=true;
        put(&sync_token,4);send(SERIAL_SYNC_REQUEST);
      }
      publish_samples(cycle++);
    }
    if ((TickType_t)(xTaskGetTickCount()-wake)>=pdMS_TO_TICKS(10)) wake=xTaskGetTickCount();
    vTaskDelayUntil(&wake,pdMS_TO_TICKS(10));
  }
}
