#include "app.h"
#include <string.h>
extern IWDG_HandleTypeDef hiwdg;
QueueHandle_t command_queue,control_queue,sensor_queue;
TaskHandle_t control_task_handle,comm_task_handle;
volatile uint32_t app_control_heartbeat;
static volatile bool estop_irq;
static bool link_connected, link_clock;
static uint32_t link_at;
static uint64_t link_epoch;
static bool hardware_ready;
static StaticQueue_t command_q,control_q,sensor_q;
static uint8_t command_buf[sizeof(motor_command_t)];
static uint8_t control_buf[sizeof(control_snapshot_t)];
static uint8_t sensor_buf[sizeof(sensor_snapshot_t)];
static StaticTask_t control_tcb,sensor_tcb,comm_tcb,idle_tcb;
static StackType_t control_stack[768],sensor_stack[768],comm_stack[1024],idle_stack[256];
void app_link_update(bool connected,bool clock_ok,uint64_t epoch_ms) {
  taskENTER_CRITICAL();
  link_connected=connected;link_clock=clock_ok;link_epoch=epoch_ms;link_at=HAL_GetTick();
  taskEXIT_CRITICAL();
}
void app_epoch(uint32_t at,uint64_t *epoch,bool *connected,bool *clock_ok) {
  taskENTER_CRITICAL();
  uint32_t age=at-link_at;
  *connected=link_connected && age<1500;
  *clock_ok=link_clock && age<1500;
  *epoch=*clock_ok?link_epoch+age:0;
  taskEXIT_CRITICAL();
}
static void control_task(void *arg) {
  (void)arg;
  control_snapshot_t snap={0}; motor_pid_t pid[MOTOR_COUNT]={0};
  /* A boot-specific challenge prevents ordinary retained commands from replaying.
   * This is a safety freshness mechanism, not cryptographic authentication. */
  safety_init(&snap.safety,HAL_GetUIDw0()^HAL_GetUIDw1()^HAL_GetUIDw2()^DWT->CYCCNT);
  TickType_t wake=xTaskGetTickCount();
  for (;;) {
    uint32_t now=HAL_GetTick();
    uint64_t epoch; bool link,clock;
    app_epoch(now,&epoch,&link,&clock);
    snap.hardware_ok=hardware_ready;
    snap.encoders_valid=hardware_ready && encoder_sample(snap.velocity,snap.ticks);
    bool feedback_ok=!motor_feedback_required(board_motor_config) || snap.encoders_valid;
    safety_inputs_t inputs={hardware_ready,link,board_estop_active(),feedback_ok,clock};
    if (estop_irq) {
      /* PRIMASK, because a high-priority E-stop ISR is above BASEPRI. */
      uint32_t mask=__get_PRIMASK(); __disable_irq(); estop_irq=false;
      __set_PRIMASK(mask);
      safety_trip(&snap.safety,FAULT_ESTOP);
    }
    safety_update(&snap.safety,inputs,now);
    motor_command_t command;
    if (xQueueReceive(command_queue,&command,0)==pdTRUE)
      (void)safety_command(&snap.safety,board_motor_config,&command,inputs,now,epoch);
    if (!snap.safety.armed) motor_stop_all();
    for (unsigned i=0;i<MOTOR_COUNT;++i) {
      const motor_config_t *c=&board_motor_config[i];
      float duty=0;
      if (snap.safety.armed) {
        duty=c->mode==MOTOR_VELOCITY ?
          pid_step(&pid[i],c,snap.safety.target[i],snap.velocity[(unsigned)c->encoder],
                   CONTROL_PERIOD_MS/1000.0f) : snap.safety.target[i];
        /* Close the interrupt race between an E-stop and applying PWM. */
        uint32_t mask=__get_PRIMASK(); __disable_irq();
        bool ok=!estop_irq && !board_estop_active() && motor_write(i,duty,now);
        __set_PRIMASK(mask);
        if (!ok) {
          safety_trip(&snap.safety,FAULT_ESTOP|FAULT_HARDWARE);motor_stop_all();duty=0;
        }
      } else pid_reset(&pid[i]);
      snap.duty[i]=duty;
    }
    if (!snap.safety.armed) memset(snap.duty,0,sizeof(snap.duty));
    snap.sample_ms=now;
    xQueueOverwrite(control_queue,&snap);
    app_control_heartbeat=now;
    HAL_IWDG_Refresh(&hiwdg);
    if ((TickType_t)(xTaskGetTickCount()-wake)>=pdMS_TO_TICKS(CONTROL_PERIOD_MS)) {
      safety_trip(&snap.safety,FAULT_DEADLINE); motor_stop_all();
      wake=xTaskGetTickCount();
    }
    vTaskDelayUntil(&wake,pdMS_TO_TICKS(CONTROL_PERIOD_MS));
  }
}
static void sensor_task(void *arg) {
  (void)arg;
  bool imu_ready=imu_init();
  TickType_t wake=xTaskGetTickCount();
  for (;;) {
    sensor_snapshot_t s={0};
    if (imu_ready) (void)imu_read(&s.imu);
    s.sample_ms=HAL_GetTick(); s.metal_valid=metal_read(&s.metal,s.sample_ms);
    xQueueOverwrite(sensor_queue,&s);
    vTaskDelayUntil(&wake,pdMS_TO_TICKS(20));
  }
}
void app_start(bool hardware_ok) {
  hardware_ready=hardware_ok;
  CoreDebug->DEMCR|=CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL|=DWT_CTRL_CYCCNTENA_Msk;
  command_queue=xQueueCreateStatic(1,sizeof(motor_command_t),command_buf,&command_q);
  control_queue=xQueueCreateStatic(1,sizeof(control_snapshot_t),control_buf,&control_q);
  sensor_queue=xQueueCreateStatic(1,sizeof(sensor_snapshot_t),sensor_buf,&sensor_q);
  configASSERT(command_queue && control_queue && sensor_queue);
  control_task_handle=xTaskCreateStatic(control_task,"control",768,NULL,6,control_stack,&control_tcb);
  TaskHandle_t sensors=xTaskCreateStatic(sensor_task,"sensors",768,NULL,3,sensor_stack,&sensor_tcb);
  comm_task_handle=xTaskCreateStatic(serial_task,"serial",1024,NULL,2,comm_stack,&comm_tcb);
  configASSERT(control_task_handle && sensors && comm_task_handle);
}
void HAL_GPIO_EXTI_Callback(uint16_t pin) {
  if (board_estop.port && pin==board_estop.pin) { motor_stop_all();estop_irq=true; }
}
void vApplicationGetIdleTaskMemory(StaticTask_t **tcb,StackType_t **stack,uint32_t *size) {
  *tcb=&idle_tcb;*stack=idle_stack;*size=256;
}
void vApplicationStackOverflowHook(TaskHandle_t task,char *name) {
  (void)task;(void)name;board_panic();
}
void vApplicationMallocFailedHook(void) { board_panic(); }
