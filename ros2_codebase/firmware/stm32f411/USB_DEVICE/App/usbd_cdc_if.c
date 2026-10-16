#include "usbd_cdc_if.h"
#include "usb_device.h"
#include <string.h>
static uint8_t receive_buffer[64],initial_tx_buffer[64];
static uint8_t line_coding[7]={0x00,0xc2,0x01,0,0,0,8};
static int8_t init(void) {
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS,initial_tx_buffer,0);
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS,receive_buffer);return USBD_OK;
}
static int8_t deinit(void) { return USBD_OK; }
static int8_t control(uint8_t cmd,uint8_t *buf,uint16_t len) {
  if (len>=7) {
    if (cmd==CDC_SET_LINE_CODING) memcpy(line_coding,buf,7);
    if (cmd==CDC_GET_LINE_CODING) memcpy(buf,line_coding,7);
  }
  return USBD_OK;
}
static int8_t receive(uint8_t *buf,uint32_t *len) {
  transport_rx(buf,*len);
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS,receive_buffer);
  USBD_CDC_ReceivePacket(&hUsbDeviceFS);return USBD_OK;
}
static int8_t complete(uint8_t *buf,uint32_t *len,uint8_t ep) {
  (void)buf;(void)len;(void)ep;return USBD_OK;
}
USBD_CDC_ItfTypeDef USBD_Interface_fops_FS={init,deinit,control,receive,complete};
uint8_t CDC_Transmit_FS(uint8_t *buf,uint16_t len) {
  if (hUsbDeviceFS.dev_state!=USBD_STATE_CONFIGURED || !hUsbDeviceFS.pClassData)
    return USBD_FAIL;
  USBD_CDC_HandleTypeDef *cdc=hUsbDeviceFS.pClassData;
  if (cdc->TxState) return USBD_BUSY;
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS,buf,len);
  return USBD_CDC_TransmitPacket(&hUsbDeviceFS);
}
