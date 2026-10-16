#ifndef PERIPHERALS_H
#define PERIPHERALS_H
#include "board.h"
extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim1,htim2,htim3,htim4;
void sensor_peripherals_init(void);
/* Does not mux output pins or start PWM. Frequency requires driver data. */
bool motor_pwm_init(uint32_t frequency_hz);
#endif
