#include "board.h"
#include "peripherals.h"
#include <math.h>
/* UNKNOWN: no Arduino or Raspberry Pi pin number is translated to STM32. */
/* Only PWM timer/channel assignments are confirmed. Driver inputs remain unbound. */
const motor_binding_t board_motors[MOTOR_COUNT] = {
  {.timer=&htim2,.channel=TIM_CHANNEL_1}, {.timer=&htim2,.channel=TIM_CHANNEL_2},
  {.timer=&htim2,.channel=TIM_CHANNEL_3}, {.timer=&htim2,.channel=TIM_CHANNEL_4},
  {.timer=&htim1,.channel=TIM_CHANNEL_1}
};
const encoder_binding_t board_encoders[ENCODER_COUNT] = {0};
const motor_config_t board_motor_config[MOTOR_COUNT] = {
  { .mode=MOTOR_UNBOUND, .encoder=-1 }, { .mode=MOTOR_UNBOUND, .encoder=-1 },
  { .mode=MOTOR_UNBOUND, .encoder=-1 }, { .mode=MOTOR_UNBOUND, .encoder=-1 },
  { .mode=MOTOR_UNBOUND, .encoder=-1 }
};
/* Chosen NC auxiliary loop to GND; open/broken wire -> pulled high -> stop.
 * Separate hardwired E-stop contacts must interrupt motor energy independently. */
const gpio_signal_t board_estop = {GPIOA,GPIO_PIN_4,true};
const metal_binding_t board_metal = {0};
bool board_init(void) {
  sensor_peripherals_init();
  /* PWM output pins stay at reset input state in this commissioning profile. */
  motor_stop_all();
  if (!BOARD_HARDWARE_APPROVED || !board_estop.port) return false;
  unsigned feedback_mask=0;
  for (unsigned i=0; i<MOTOR_COUNT; ++i) {
    const motor_binding_t *b=&board_motors[i];
    const motor_config_t *c=&board_motor_config[i];
    if (!b->timer || !b->direction.port || !b->enable_present || !b->enable.port ||
        !b->pwm_active_high || b->reversal_deadtime_ms == 0 ||
        !isfinite(c->duty_limit) || c->duty_limit <= 0 || c->duty_limit > 1 ||
        !isfinite(c->target_limit) || c->target_limit <= 0) return false;
    /* This driver supports active-high PWM, DIR and an independent ENABLE.
     * Other electrical topologies require a reviewed driver implementation. */
    if (c->mode==MOTOR_DUTY) {
      if (c->encoder != -1 || c->target_limit > 1) return false;
    } else if (c->mode==MOTOR_VELOCITY) {
      if (c->encoder < 0 || c->encoder >= ENCODER_COUNT ||
          (feedback_mask & (1u<<c->encoder)) ||
          !isfinite(c->kp) || !isfinite(c->ki) || !isfinite(c->kd) ||
          !isfinite(c->integral_limit) || c->kp < 0 || c->ki < 0 || c->kd < 0 ||
          c->integral_limit < 0) return false;
      feedback_mask |= 1u<<c->encoder;
    } else return false;
  }
  /* All enables must become inactive before timer outputs are muxed. */
  for (unsigned i=0;i<MOTOR_COUNT;++i)
    if (!gpio_output_init(board_motors[i].enable)) return false;
  for (unsigned i=0;i<MOTOR_COUNT;++i)
    if (!gpio_output_init(board_motors[i].direction)) return false;
  if (!encoder_init() || !motor_pwm_init(BOARD_PWM_FREQUENCY_HZ)) return false;
  motor_stop_all();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  GPIO_InitTypeDef pins={0};pins.Pin=GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  pins.Mode=GPIO_MODE_AF_PP;pins.Pull=GPIO_NOPULL;pins.Speed=GPIO_SPEED_FREQ_LOW;
  pins.Alternate=GPIO_AF1_TIM2;HAL_GPIO_Init(GPIOA,&pins);
  pins.Pin=GPIO_PIN_8;pins.Alternate=GPIO_AF1_TIM1;HAL_GPIO_Init(GPIOA,&pins);
  for(unsigned i=0;i<MOTOR_COUNT;++i) {
    if(HAL_TIM_PWM_Start(board_motors[i].timer,board_motors[i].channel)!=HAL_OK) {
      motor_stop_all();return false;
    }
  }
  return true;
}
bool board_estop_active(void) { return !board_estop.port || gpio_read(board_estop); }
void board_panic(void) {
  __disable_irq(); motor_stop_all();
  for (;;) { __NOP(); } /* IWDG resets if already started; never feed it here. */
}
