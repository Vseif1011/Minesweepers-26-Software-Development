#include "usbd_core.h"
#include "usbd_cdc.h"
#include "board.h"
PCD_HandleTypeDef hpcd_USB_OTG_FS;
static uint32_t class_storage[(sizeof(USBD_CDC_HandleTypeDef)+3)/4];
void *USBD_static_malloc(uint32_t size) {
  return size<=sizeof(class_storage)?class_storage:NULL;
}
void USBD_static_free(void *pointer) { (void)pointer; }
void HAL_PCD_MspInit(PCD_HandleTypeDef *pcd) {
  if (pcd->Instance!=USB_OTG_FS) return;
  __HAL_RCC_GPIOA_CLK_ENABLE();
  GPIO_InitTypeDef pin={0};
  pin.Pin=GPIO_PIN_11|GPIO_PIN_12;pin.Mode=GPIO_MODE_AF_PP;
  pin.Pull=GPIO_NOPULL;pin.Speed=GPIO_SPEED_FREQ_VERY_HIGH;pin.Alternate=GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA,&pin);
  __HAL_RCC_USB_OTG_FS_CLK_ENABLE();
  HAL_NVIC_SetPriority(OTG_FS_IRQn,6,0);HAL_NVIC_EnableIRQ(OTG_FS_IRQn);
}
void HAL_PCD_MspDeInit(PCD_HandleTypeDef *pcd) {
  if (pcd->Instance!=USB_OTG_FS) return;
  HAL_NVIC_DisableIRQ(OTG_FS_IRQn);__HAL_RCC_USB_OTG_FS_CLK_DISABLE();
  HAL_GPIO_DeInit(GPIOA,GPIO_PIN_11|GPIO_PIN_12);
}
void HAL_PCD_SetupStageCallback(PCD_HandleTypeDef *h) { USBD_LL_SetupStage(h->pData,(uint8_t*)h->Setup); }
void HAL_PCD_DataOutStageCallback(PCD_HandleTypeDef *h,uint8_t ep) { USBD_LL_DataOutStage(h->pData,ep,h->OUT_ep[ep].xfer_buff); }
void HAL_PCD_DataInStageCallback(PCD_HandleTypeDef *h,uint8_t ep) { USBD_LL_DataInStage(h->pData,ep,h->IN_ep[ep].xfer_buff); }
void HAL_PCD_SOFCallback(PCD_HandleTypeDef *h) { USBD_LL_SOF(h->pData); }
void HAL_PCD_ResetCallback(PCD_HandleTypeDef *h) { USBD_LL_SetSpeed(h->pData,USBD_SPEED_FULL);USBD_LL_Reset(h->pData); }
void HAL_PCD_SuspendCallback(PCD_HandleTypeDef *h) { USBD_LL_Suspend(h->pData); }
void HAL_PCD_ResumeCallback(PCD_HandleTypeDef *h) { USBD_LL_Resume(h->pData); }
void HAL_PCD_ISOOUTIncompleteCallback(PCD_HandleTypeDef *h,uint8_t ep) { (void)h;(void)ep; }
void HAL_PCD_ISOINIncompleteCallback(PCD_HandleTypeDef *h,uint8_t ep) { (void)h;(void)ep; }
void HAL_PCD_ConnectCallback(PCD_HandleTypeDef *h) { USBD_LL_DevConnected(h->pData); }
void HAL_PCD_DisconnectCallback(PCD_HandleTypeDef *h) { USBD_LL_DevDisconnected(h->pData); }
static USBD_StatusTypeDef status(HAL_StatusTypeDef result) { return result==HAL_OK?USBD_OK:USBD_FAIL; }
USBD_StatusTypeDef USBD_LL_Init(USBD_HandleTypeDef *dev) {
  PCD_HandleTypeDef *h=&hpcd_USB_OTG_FS;
  h->Instance=USB_OTG_FS;h->Init.dev_endpoints=4;h->Init.speed=PCD_SPEED_FULL;
  h->Init.dma_enable=DISABLE;h->Init.phy_itface=PCD_PHY_EMBEDDED;
  h->Init.Sof_enable=DISABLE;h->Init.low_power_enable=DISABLE;
  h->Init.lpm_enable=DISABLE;h->Init.vbus_sensing_enable=DISABLE;
  h->Init.use_dedicated_ep1=DISABLE;h->pData=dev;dev->pData=h;
  if (HAL_PCD_Init(h)!=HAL_OK) return USBD_FAIL;
  HAL_PCDEx_SetRxFiFo(h,0x80);HAL_PCDEx_SetTxFiFo(h,0,0x40);
  HAL_PCDEx_SetTxFiFo(h,1,0x40);HAL_PCDEx_SetTxFiFo(h,2,0x20);
  return USBD_OK;
}
USBD_StatusTypeDef USBD_LL_DeInit(USBD_HandleTypeDef *d) { return status(HAL_PCD_DeInit(d->pData)); }
USBD_StatusTypeDef USBD_LL_Start(USBD_HandleTypeDef *d) { return status(HAL_PCD_Start(d->pData)); }
USBD_StatusTypeDef USBD_LL_Stop(USBD_HandleTypeDef *d) { return status(HAL_PCD_Stop(d->pData)); }
USBD_StatusTypeDef USBD_LL_OpenEP(USBD_HandleTypeDef *d,uint8_t ep,uint8_t type,uint16_t size) { return status(HAL_PCD_EP_Open(d->pData,ep,size,type)); }
USBD_StatusTypeDef USBD_LL_CloseEP(USBD_HandleTypeDef *d,uint8_t ep) { return status(HAL_PCD_EP_Close(d->pData,ep)); }
USBD_StatusTypeDef USBD_LL_FlushEP(USBD_HandleTypeDef *d,uint8_t ep) { return status(HAL_PCD_EP_Flush(d->pData,ep)); }
USBD_StatusTypeDef USBD_LL_StallEP(USBD_HandleTypeDef *d,uint8_t ep) { return status(HAL_PCD_EP_SetStall(d->pData,ep)); }
USBD_StatusTypeDef USBD_LL_ClearStallEP(USBD_HandleTypeDef *d,uint8_t ep) { return status(HAL_PCD_EP_ClrStall(d->pData,ep)); }
uint8_t USBD_LL_IsStallEP(USBD_HandleTypeDef *d,uint8_t ep) {
  PCD_HandleTypeDef *h=d->pData;return ep&0x80?h->IN_ep[ep&0x7f].is_stall:h->OUT_ep[ep].is_stall;
}
USBD_StatusTypeDef USBD_LL_SetUSBAddress(USBD_HandleTypeDef *d,uint8_t a) { return status(HAL_PCD_SetAddress(d->pData,a)); }
USBD_StatusTypeDef USBD_LL_Transmit(USBD_HandleTypeDef *d,uint8_t ep,uint8_t *buf,uint32_t size) { return status(HAL_PCD_EP_Transmit(d->pData,ep,buf,size)); }
USBD_StatusTypeDef USBD_LL_PrepareReceive(USBD_HandleTypeDef *d,uint8_t ep,uint8_t *buf,uint32_t size) { return status(HAL_PCD_EP_Receive(d->pData,ep,buf,size)); }
uint32_t USBD_LL_GetRxDataSize(USBD_HandleTypeDef *d,uint8_t ep) { return HAL_PCD_EP_GetRxCount(d->pData,ep); }
void USBD_LL_Delay(uint32_t ms) { HAL_Delay(ms); }
