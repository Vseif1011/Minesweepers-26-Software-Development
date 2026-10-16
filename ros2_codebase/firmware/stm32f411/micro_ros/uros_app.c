#include "app.h"
#include "rcl/rcl.h"
#include "rcl/error_handling.h"
#include "rclc/rclc.h"
#include "rclc/executor.h"
#include "rmw_microros/rmw_microros.h"
#include "minesweeper_interfaces/msg/motor_command.h"
#include "minesweeper_interfaces/msg/motor_status.h"
#include "minesweeper_interfaces/msg/encoder_ticks.h"
#include "sensor_msgs/msg/imu.h"
#include "std_msgs/msg/float32.h"
#include <string.h>
#include <stddef.h>
#ifndef FIRMWARE_ROS_DOMAIN_ID
#define FIRMWARE_ROS_DOMAIN_ID 0
#endif
bool cubemx_transport_open(struct uxrCustomTransport *);
bool cubemx_transport_close(struct uxrCustomTransport *);
size_t cubemx_transport_write(struct uxrCustomTransport *,const uint8_t *,size_t,uint8_t *);
size_t cubemx_transport_read(struct uxrCustomTransport *,uint8_t *,size_t,int,uint8_t *);
void *microros_allocate(size_t,void *);
void microros_deallocate(void *,void *);
void *microros_reallocate(void *,size_t,void *);
void *microros_zero_allocate(size_t,size_t,void *);
size_t xPortGetFreeHeapSizeMicroROS(void);
size_t xPortGetMinimumEverFreeHeapSizeMicroROS(void);
static rclc_support_t support;
static rcl_node_t node;
static rcl_subscription_t command_sub;
static rcl_publisher_t status_pub,encoder_pub,imu_pub,metal_pub;
static rclc_executor_t executor;
static minesweeper_interfaces__msg__MotorCommand command_msg;
static unsigned created;
/* Sole owner of micro-ROS entities and allocator is this task. */
static void on_command(const void *message) {
  const minesweeper_interfaces__msg__MotorCommand *msg=message;
  motor_command_t cmd={.session=msg->session,.sequence=msg->sequence,
    .issued_at_ms=msg->issued_at_ms,.arm=msg->arm};
  memcpy(cmd.target,msg->target,sizeof(cmd.target));
  xQueueOverwrite(command_queue,&cmd);
}
static void discard(rcl_ret_t result) { (void)result; rcl_reset_error(); }
static void destroy_entities(void) {
  app_link_update(false,false,0);
  /* Reverse order also handles partial initialization failures. */
  if (created>=9) discard(rclc_executor_fini(&executor));
  if (created>=8) discard(rcl_subscription_fini(&command_sub,&node));
  if (created>=7) discard(rcl_publisher_fini(&metal_pub,&node));
  if (created>=6) discard(rcl_publisher_fini(&imu_pub,&node));
  if (created>=5) discard(rcl_publisher_fini(&encoder_pub,&node));
  if (created>=4) discard(rcl_publisher_fini(&status_pub,&node));
  if (created>=3) discard(rcl_node_fini(&node));
  if (created>=2) discard(rclc_support_fini(&support));
  created=0;
}
static bool create_entities(rcl_allocator_t *allocator) {
  rcl_init_options_t options=rcl_get_zero_initialized_init_options();
  if (rcl_init_options_init(&options,*allocator)!=RCL_RET_OK) return false;
  created=1;
  rcl_ret_t result=rcl_init_options_set_domain_id(&options,FIRMWARE_ROS_DOMAIN_ID);
  if (result==RCL_RET_OK) {
    memset(&support,0,sizeof(support));
    created=2; /* clean up even if clock/context creation partially fails */
    result=rclc_support_init_with_options(&support,0,NULL,&options,allocator);
  }
  discard(rcl_init_options_fini(&options));
  if (result!=RCL_RET_OK) return false;
  created=2;
  node=rcl_get_zero_initialized_node();
  if (rclc_node_init_default(&node,"stm32_controller","",&support)!=RCL_RET_OK) return false;
  created=3;
  status_pub=rcl_get_zero_initialized_publisher();
  encoder_pub=rcl_get_zero_initialized_publisher();
  imu_pub=rcl_get_zero_initialized_publisher();
  metal_pub=rcl_get_zero_initialized_publisher();
  rmw_qos_profile_t qos=rmw_qos_profile_sensor_data;qos.depth=1;
  if (rclc_publisher_init(&status_pub,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(minesweeper_interfaces,msg,MotorStatus),"/motor/status",&qos)!=RCL_RET_OK) return false;
  created=4;
  if (rclc_publisher_init(&encoder_pub,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(minesweeper_interfaces,msg,EncoderTicks),"/encoders/ticks",&qos)!=RCL_RET_OK) return false;
  created=5;
  if (rclc_publisher_init(&imu_pub,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs,msg,Imu),"/imu/data_raw",&qos)!=RCL_RET_OK) return false;
  created=6;
  if (rclc_publisher_init(&metal_pub,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,msg,Float32),"/metal_detector/signal",&qos)!=RCL_RET_OK) return false;
  created=7;
  command_sub=rcl_get_zero_initialized_subscription();
  if (rclc_subscription_init(&command_sub,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(minesweeper_interfaces,msg,MotorCommand),"/motor/setpoints",&qos)!=RCL_RET_OK) return false;
  created=8;
  executor=rclc_executor_get_zero_initialized_executor();
  created=9;
  if (rclc_executor_init(&executor,&support.context,1,allocator)!=RCL_RET_OK) return false;
  if (rclc_executor_add_subscription(&executor,&command_sub,&command_msg,on_command,ON_NEW_DATA)!=RCL_RET_OK) return false;
  return true;
}
static builtin_interfaces__msg__Time stamp(uint32_t sample_ms,uint32_t now,uint64_t epoch) {
  uint64_t sample_epoch=epoch-(uint32_t)(now-sample_ms);
  builtin_interfaces__msg__Time result={.sec=(int32_t)(sample_epoch/1000),
    .nanosec=(uint32_t)(sample_epoch%1000)*1000000};return result;
}
static bool publish_samples(unsigned cycle) {
  control_snapshot_t c; sensor_snapshot_t s;
  if (xQueuePeek(control_queue,&c,0)!=pdTRUE) return true;
  uint32_t now=HAL_GetTick();uint64_t epoch;bool link,clock;
  app_epoch(now,&epoch,&link,&clock);
  if ((uint32_t)(now-c.sample_ms)>50) return false;
  if (cycle%5==0) {
    minesweeper_interfaces__msg__MotorStatus msg={0};
    msg.session=c.safety.session;msg.sequence=c.safety.sequence;msg.faults=c.safety.faults;
    msg.uptime_ms=now;msg.armed=c.safety.armed;msg.ready=c.safety.ready;
    msg.hardware_ok=c.hardware_ok;msg.encoders_valid=c.encoders_valid;
    bool sensors=xQueuePeek(sensor_queue,&s,0)==pdTRUE && (uint32_t)(now-s.sample_ms)<100;
    msg.imu_valid=sensors && s.imu.valid;msg.metal_valid=sensors && s.metal_valid;
    memcpy(msg.duty,c.duty,sizeof(msg.duty));
    msg.control_stack_free_words=uxTaskGetStackHighWaterMark(control_task_handle);
    msg.uros_stack_free_words=uxTaskGetStackHighWaterMark(uros_task_handle);
    msg.heap_free_bytes=xPortGetFreeHeapSizeMicroROS();
    msg.heap_min_free_bytes=xPortGetMinimumEverFreeHeapSizeMicroROS();
    if (rcl_publish(&status_pub,&msg,NULL)!=RCL_RET_OK) return false;
  }
  if (cycle%2!=0) return true;
  if (clock && c.encoders_valid) {
    minesweeper_interfaces__msg__EncoderTicks msg={0};
    msg.stamp=stamp(c.sample_ms,now,epoch);msg.sample_time_ms=c.sample_ms;
    memcpy(msg.ticks,c.ticks,sizeof(msg.ticks));memcpy(msg.velocity_rad_s,c.velocity,sizeof(msg.velocity_rad_s));
    if (rcl_publish(&encoder_pub,&msg,NULL)!=RCL_RET_OK) return false;
  }
  if (xQueuePeek(sensor_queue,&s,0)!=pdTRUE || (uint32_t)(now-s.sample_ms)>100) return true;
  if (clock && s.imu.valid) {
    sensor_msgs__msg__Imu msg={0};
    msg.header.stamp=stamp(s.imu.stamp_ms,now,epoch);
    char frame[]="imu_link";
    msg.header.frame_id.data=frame;msg.header.frame_id.size=sizeof(frame)-1;
    msg.header.frame_id.capacity=sizeof(frame);
    msg.orientation_covariance[0]=-1; /* no orientation estimate on MCU */
    msg.angular_velocity.x=s.imu.gyro[0];msg.angular_velocity.y=s.imu.gyro[1];msg.angular_velocity.z=s.imu.gyro[2];
    msg.linear_acceleration.x=s.imu.accel[0];msg.linear_acceleration.y=s.imu.accel[1];msg.linear_acceleration.z=s.imu.accel[2];
    if (rcl_publish(&imu_pub,&msg,NULL)!=RCL_RET_OK) return false;
  }
  if (s.metal_valid) {
    std_msgs__msg__Float32 msg={.data=s.metal};
    if (rcl_publish(&metal_pub,&msg,NULL)!=RCL_RET_OK) return false;
  }
  return true;
}
void uros_task(void *argument) {
  (void)argument;
  rcl_allocator_t allocator=rcutils_get_zero_initialized_allocator();
  allocator.allocate=microros_allocate;allocator.deallocate=microros_deallocate;
  allocator.reallocate=microros_reallocate;allocator.zero_allocate=microros_zero_allocate;
  if (!rcutils_set_default_allocator(&allocator)) board_panic();
  if (rmw_uros_set_custom_transport(true,NULL,cubemx_transport_open,cubemx_transport_close,
      cubemx_transport_write,cubemx_transport_read)!=RMW_RET_OK) board_panic();
  for (;;) {
    app_link_update(false,false,0);
    if (rmw_uros_ping_agent(100,1)!=RMW_RET_OK) { vTaskDelay(pdMS_TO_TICKS(500));continue; }
    if (!create_entities(&allocator)) {
      destroy_entities();vTaskDelay(pdMS_TO_TICKS(500));continue;
    }
    if (rmw_uros_sync_session(100)!=RMW_RET_OK || !rmw_uros_epoch_synchronized()) {
      destroy_entities();vTaskDelay(pdMS_TO_TICKS(500));continue;
    }
    app_link_update(true,true,(uint64_t)rmw_uros_epoch_millis());
    uint32_t check_at=HAL_GetTick();unsigned cycle=0;
    TickType_t wake=xTaskGetTickCount();
    for (;;) {
      rcl_ret_t result=rclc_executor_spin_some(&executor,RCL_MS_TO_NS(2));
      if (result!=RCL_RET_OK && result!=RCL_RET_TIMEOUT) break;
      if (!publish_samples(cycle++)) break;
      if ((uint32_t)(HAL_GetTick()-check_at)>=500) {
        if (rmw_uros_ping_agent(50,1)!=RMW_RET_OK || rmw_uros_sync_session(50)!=RMW_RET_OK) break;
        app_link_update(true,rmw_uros_epoch_synchronized(),(uint64_t)rmw_uros_epoch_millis());
        check_at=HAL_GetTick();
      }
      if ((TickType_t)(xTaskGetTickCount()-wake)>=pdMS_TO_TICKS(10)) wake=xTaskGetTickCount();
      vTaskDelayUntil(&wake,pdMS_TO_TICKS(10));
    }
    destroy_entities();
    xQueueReset(command_queue); /* discard commands belonging to the old connection */
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}
