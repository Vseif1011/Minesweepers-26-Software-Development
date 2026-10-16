#include "usb_device.h"
#include "usbd_core.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"
#include "board.h"
extern USBD_DescriptorsTypeDef FS_Desc;
USBD_HandleTypeDef hUsbDeviceFS;
void MX_USB_DEVICE_Init(void) {
  if (USBD_Init(&hUsbDeviceFS,&FS_Desc,DEVICE_FS)!=USBD_OK ||
      USBD_RegisterClass(&hUsbDeviceFS,&USBD_CDC)!=USBD_OK ||
      USBD_CDC_RegisterInterface(&hUsbDeviceFS,&USBD_Interface_fops_FS)!=USBD_OK ||
      USBD_Start(&hUsbDeviceFS)!=USBD_OK) board_panic();
}
