#include "board.h"
bool gpio_output_init(gpio_signal_t s) {
  if (!s.port || !s.pin || (s.pin & (s.pin-1))) return false;
  if (s.port==GPIOA) __HAL_RCC_GPIOA_CLK_ENABLE();
  else if (s.port==GPIOB) __HAL_RCC_GPIOB_CLK_ENABLE();
  else if (s.port==GPIOC) __HAL_RCC_GPIOC_CLK_ENABLE();
  else return false;
  gpio_write(s,false); /* preload output data BEFORE changing pin mode */
  GPIO_InitTypeDef cfg={0};cfg.Pin=s.pin;cfg.Mode=GPIO_MODE_OUTPUT_PP;
  cfg.Pull=GPIO_NOPULL;cfg.Speed=GPIO_SPEED_FREQ_LOW;HAL_GPIO_Init(s.port,&cfg);
  return true;
}
void gpio_write(gpio_signal_t s, bool active) {
  if (s.port && s.pin) HAL_GPIO_WritePin(s.port, s.pin,
    active == s.active_high ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
bool gpio_read(gpio_signal_t s) {
  return s.port && s.pin && ((HAL_GPIO_ReadPin(s.port,s.pin)==GPIO_PIN_SET)==s.active_high);
}
