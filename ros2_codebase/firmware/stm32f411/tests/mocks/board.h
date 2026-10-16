#ifndef MOCK_BOARD_H
#define MOCK_BOARD_H
#include "control.h"
#include <stddef.h>
#define BOARD_HARDWARE_APPROVED 1 /* test fixture ONLY; production is locked */
typedef struct { uint16_t value; } GPIO_TypeDef;
typedef struct { uint32_t ccr[4],arr; } timer_regs_t;
typedef struct { timer_regs_t *Instance; } TIM_HandleTypeDef;
typedef struct { GPIO_TypeDef *port; uint16_t pin; bool active_high; } gpio_signal_t;
typedef struct {
  TIM_HandleTypeDef *timer;uint32_t channel;gpio_signal_t direction,enable;
  bool enable_present,pwm_active_high,direction_positive_high;uint32_t reversal_deadtime_ms;
} motor_binding_t;
extern const motor_binding_t board_motors[MOTOR_COUNT];
extern const motor_config_t board_motor_config[MOTOR_COUNT];
#define GPIO_PIN_SET 1
#define GPIO_PIN_RESET 0
#define __HAL_TIM_SET_COMPARE(t,c,v) ((t)->Instance->ccr[(c)/4]=(v))
#define __HAL_TIM_GET_AUTORELOAD(t) ((t)->Instance->arr)
static inline void HAL_GPIO_WritePin(GPIO_TypeDef *p,uint16_t pin,int value) {
  if (value) p->value|=pin;else p->value&=(uint16_t)~pin;
}
void gpio_write(gpio_signal_t signal,bool active);
void motor_stop_all(void);
bool motor_write(unsigned i,float duty,uint32_t now);
void board_panic(void) __attribute__((noreturn));
#endif
