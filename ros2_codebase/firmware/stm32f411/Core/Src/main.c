#include "main.h"
#include "app.h"
#include "usb_device.h"
IWDG_HandleTypeDef hiwdg;
int main(void) {
  HAL_Init();
  SystemClock_Config();
  bool hardware_ok=board_init();
  /* User confirmed 25 MHz HSE; PLLQ provides the USB 48 MHz clock. */
  MX_USB_DEVICE_Init();
  hiwdg.Instance=IWDG;
  hiwdg.Init.Prescaler=IWDG_PRESCALER_64;
  hiwdg.Init.Reload=499; /* nominal 1 s at LSI=32kHz, tolerance per datasheet */
  if (HAL_IWDG_Init(&hiwdg)!=HAL_OK) board_panic();
  app_start(hardware_ok);
  vTaskStartScheduler();
  board_panic();
}
void SystemClock_Config(void) {
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  RCC_OscInitTypeDef osc={0};
  osc.OscillatorType=RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  osc.HSEState=RCC_HSE_ON;
  osc.HSIState=RCC_HSI_ON;
  osc.HSICalibrationValue=RCC_HSICALIBRATION_DEFAULT;
  osc.PLL.PLLState=RCC_PLL_ON; osc.PLL.PLLSource=RCC_PLLSOURCE_HSE;
  osc.PLL.PLLM=25; osc.PLL.PLLN=192; osc.PLL.PLLP=RCC_PLLP_DIV2; osc.PLL.PLLQ=4;
  if (HAL_RCC_OscConfig(&osc)!=HAL_OK) board_panic();
  RCC_ClkInitTypeDef clk={0};
  clk.ClockType=RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  clk.SYSCLKSource=RCC_SYSCLKSOURCE_PLLCLK; clk.AHBCLKDivider=RCC_SYSCLK_DIV1;
  clk.APB1CLKDivider=RCC_HCLK_DIV2; clk.APB2CLKDivider=RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&clk,FLASH_LATENCY_3)!=HAL_OK) board_panic();
}
void HAL_MspInit(void) {
  __HAL_RCC_SYSCFG_CLK_ENABLE(); __HAL_RCC_PWR_CLK_ENABLE();
  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
}
void Error_Handler(void) { board_panic(); }
