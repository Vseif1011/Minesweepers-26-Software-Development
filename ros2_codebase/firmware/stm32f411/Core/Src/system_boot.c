#include "stm32f4xx.h"
extern void Vendor_SystemInit(void);
extern const uint32_t g_pfnVectors[];
/* Handles both ST-Link reset and WeAct HID application handoff, including
 * older bootloaders that leave SysTick running. No initialized data used. */
void SystemInit(void) {
  __disable_irq();
  SysTick->CTRL=0;
  for (unsigned i=0;i<8;++i) { NVIC->ICER[i]=UINT32_MAX;NVIC->ICPR[i]=UINT32_MAX; }
  SCB->ICSR=SCB_ICSR_PENDSTCLR_Msk|SCB_ICSR_PENDSVCLR_Msk;
  Vendor_SystemInit();
  SCB->VTOR=(uint32_t)g_pfnVectors;
  __DSB();__ISB();__enable_irq();
}
