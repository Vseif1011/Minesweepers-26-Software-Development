#include "peripherals.h"
I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim1,htim2,htim3,htim4;
void HAL_I2C_MspInit(I2C_HandleTypeDef *i2c) {
  if (i2c->Instance!=I2C1) return;
  __HAL_RCC_GPIOB_CLK_ENABLE();__HAL_RCC_I2C1_CLK_ENABLE();
  GPIO_InitTypeDef pins={0};pins.Pin=GPIO_PIN_8|GPIO_PIN_9;
  pins.Mode=GPIO_MODE_AF_OD;pins.Pull=GPIO_NOPULL;
  pins.Speed=GPIO_SPEED_FREQ_VERY_HIGH;pins.Alternate=GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB,&pins); /* external pullups to 3.3 V required */
}
void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef *timer) {
  GPIO_InitTypeDef pins={0};pins.Mode=GPIO_MODE_AF_PP;pins.Pull=GPIO_NOPULL;
  pins.Speed=GPIO_SPEED_FREQ_LOW;
  if (timer->Instance==TIM3) {
    __HAL_RCC_TIM3_CLK_ENABLE();__HAL_RCC_GPIOA_CLK_ENABLE();
    pins.Pin=GPIO_PIN_6|GPIO_PIN_7;pins.Alternate=GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOA,&pins);
  } else if (timer->Instance==TIM4) {
    __HAL_RCC_TIM4_CLK_ENABLE();__HAL_RCC_GPIOB_CLK_ENABLE();
    pins.Pin=GPIO_PIN_6|GPIO_PIN_7;pins.Alternate=GPIO_AF2_TIM4;
    HAL_GPIO_Init(GPIOB,&pins);
  }
}
void sensor_peripherals_init(void) {
  __HAL_RCC_GPIOA_CLK_ENABLE();
  GPIO_InitTypeDef estop={0};estop.Pin=GPIO_PIN_4;
  estop.Mode=GPIO_MODE_IT_RISING_FALLING;estop.Pull=GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA,&estop);
  /* Above FreeRTOS BASEPRI. Callback calls no FreeRTOS API. */
  HAL_NVIC_SetPriority(EXTI4_IRQn,0,0);HAL_NVIC_EnableIRQ(EXTI4_IRQn);
  hi2c1.Instance=I2C1;hi2c1.Init.ClockSpeed=100000;
  hi2c1.Init.DutyCycle=I2C_DUTYCYCLE_2;hi2c1.Init.OwnAddress1=0;
  hi2c1.Init.AddressingMode=I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode=I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.GeneralCallMode=I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode=I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1)!=HAL_OK) board_panic();
  TIM_HandleTypeDef *timers[]={&htim3,&htim4};
  for (unsigned i=0;i<2;++i) {
    TIM_HandleTypeDef *t=timers[i];
    t->Instance=i==0?TIM3:TIM4;t->Init.Prescaler=0;
    t->Init.CounterMode=TIM_COUNTERMODE_UP;t->Init.Period=65535;
    t->Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;t->Init.AutoReloadPreload=TIM_AUTORELOAD_PRELOAD_DISABLE;
    TIM_Encoder_InitTypeDef cfg={0};cfg.EncoderMode=TIM_ENCODERMODE_TI12;
    cfg.IC1Polarity=TIM_ICPOLARITY_RISING;cfg.IC2Polarity=TIM_ICPOLARITY_RISING;
    cfg.IC1Selection=TIM_ICSELECTION_DIRECTTI;cfg.IC2Selection=TIM_ICSELECTION_DIRECTTI;
    cfg.IC1Prescaler=TIM_ICPSC_DIV1;cfg.IC2Prescaler=TIM_ICPSC_DIV1;
    cfg.IC1Filter=0;cfg.IC2Filter=0; /* conditioning/maximum edge rate to be measured */
    if (HAL_TIM_Encoder_Init(t,&cfg)!=HAL_OK) board_panic();
  }
}
bool motor_pwm_init(uint32_t frequency) {
  if (!BOARD_HARDWARE_APPROVED || !frequency || frequency>48000000 || 96000000%frequency) return false;
  __HAL_RCC_TIM1_CLK_ENABLE();__HAL_RCC_TIM2_CLK_ENABLE();
  TIM_HandleTypeDef *timers[]={&htim2,&htim1};
  for(unsigned i=0;i<2;++i) {
    TIM_HandleTypeDef *t=timers[i];t->Instance=i==0?TIM2:TIM1;
    t->Init.Prescaler=0;t->Init.CounterMode=TIM_COUNTERMODE_UP;
    t->Init.Period=96000000/frequency-1;t->Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
    t->Init.AutoReloadPreload=TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (i && t->Init.Period>65535) return false;
    if (HAL_TIM_PWM_Init(t)!=HAL_OK) return false;
    TIM_OC_InitTypeDef oc={0};oc.OCMode=TIM_OCMODE_PWM1;oc.Pulse=0;
    oc.OCPolarity=TIM_OCPOLARITY_HIGH;oc.OCFastMode=TIM_OCFAST_DISABLE;
    unsigned count=i==0?4:1;
    for(unsigned ch=0;ch<count;++ch)
      if(HAL_TIM_PWM_ConfigChannel(t,&oc,ch*4)!=HAL_OK) return false;
  }
  return true;
}
