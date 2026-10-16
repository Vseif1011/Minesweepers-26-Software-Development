#include "app.h"
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
void xPortSysTickHandler(void);
void SysTick_Handler(void) {
  HAL_IncTick();
  if (xTaskGetSchedulerState()!=taskSCHEDULER_NOT_STARTED) xPortSysTickHandler();
}
void OTG_FS_IRQHandler(void) { HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS); }
void NMI_Handler(void) { board_panic(); }
void HardFault_Handler(void) { board_panic(); }
void MemManage_Handler(void) { board_panic(); }
void BusFault_Handler(void) { board_panic(); }
void UsageFault_Handler(void) { board_panic(); }
void DebugMon_Handler(void) {}
/* Only a confirmed pin profile may enable an EXTI vector. */
void EXTI0_IRQHandler(void) { if (board_estop.pin==GPIO_PIN_0) HAL_GPIO_EXTI_IRQHandler(board_estop.pin); }
void EXTI1_IRQHandler(void) { if (board_estop.pin==GPIO_PIN_1) HAL_GPIO_EXTI_IRQHandler(board_estop.pin); }
void EXTI2_IRQHandler(void) { if (board_estop.pin==GPIO_PIN_2) HAL_GPIO_EXTI_IRQHandler(board_estop.pin); }
void EXTI3_IRQHandler(void) { if (board_estop.pin==GPIO_PIN_3) HAL_GPIO_EXTI_IRQHandler(board_estop.pin); }
void EXTI4_IRQHandler(void) { if (board_estop.pin==GPIO_PIN_4) HAL_GPIO_EXTI_IRQHandler(board_estop.pin); }
void EXTI9_5_IRQHandler(void) { HAL_GPIO_EXTI_IRQHandler(board_estop.pin & 0x03e0u); }
void EXTI15_10_IRQHandler(void) { HAL_GPIO_EXTI_IRQHandler(board_estop.pin & 0xfc00u); }
