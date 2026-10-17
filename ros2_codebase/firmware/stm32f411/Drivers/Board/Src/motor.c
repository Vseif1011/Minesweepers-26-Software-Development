#include "board.h"
#include <math.h>
static int8_t signs[MOTOR_COUNT];
static uint32_t stopped_at[MOTOR_COUNT];
void motor_stop_all(void) {
  for (unsigned i=0; i<MOTOR_COUNT; ++i) {
    const motor_binding_t *b=&board_motors[i];
    gpio_write(b->enable,false);
    if (b->timer && b->timer->Instance) __HAL_TIM_SET_COMPARE(b->timer,b->channel,0);
    /* Keep last direction history to enforce deadtime after disarming too. */
  }
}
bool motor_write(unsigned i, float duty, uint32_t now) {
  if (i>=MOTOR_COUNT) return false;
  const motor_binding_t *b=&board_motors[i];
  if (!BOARD_HARDWARE_APPROVED || !b->timer || !b->direction.port || !b->pwm_active_high ||
      (b->enable_present && !b->enable.port) || !isfinite(duty)) return false;
  float limit=board_motor_config[i].duty_limit;
  duty=fmaxf(-limit,fminf(limit,duty));
  int8_t sign=duty > 0 ? 1 : duty < 0 ? -1 : 0;
  if (signs[i] && sign != signs[i]) {
    __HAL_TIM_SET_COMPARE(b->timer,b->channel,0); gpio_write(b->enable,false);
    stopped_at[i]=now; signs[i]=0; return true;
  }
  if ((uint32_t)(now-stopped_at[i]) < b->reversal_deadtime_ms) return true;
  if (!sign) {
    __HAL_TIM_SET_COMPARE(b->timer,b->channel,0); gpio_write(b->enable,false);
    return true;
  }
  bool high=(sign>0)==b->direction_positive_high;
  HAL_GPIO_WritePin(b->direction.port,b->direction.pin,high?GPIO_PIN_SET:GPIO_PIN_RESET);
  uint32_t period=__HAL_TIM_GET_AUTORELOAD(b->timer);
  __HAL_TIM_SET_COMPARE(b->timer,b->channel,(uint32_t)(fabsf(duty)*(float)period));
  gpio_write(b->enable,true); signs[i]=sign; return true;
}
