/* ISR producer, single communication-task consumer; bounded USB transfers. */
#include "serial_transport.h"
#include "app.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include <string.h>
#define RX_CAPACITY 2048u
static uint8_t rx[RX_CAPACITY],tx[256];
static volatile uint32_t head,tail;
static volatile bool overflow;
void transport_rx(const uint8_t *buf,uint32_t len) {
  uint32_t h=head,t=tail;
  if (len>RX_CAPACITY-(t-h)) { overflow=true;return; }
  for (uint32_t i=0;i<len;++i) rx[(t+i)%RX_CAPACITY]=buf[i];
  __DMB();tail=t+len;
}
void serial_transport_reset(void) {
  uint32_t mask=__get_PRIMASK();__disable_irq();
  head=tail=0;overflow=false;__set_PRIMASK(mask);
}
bool serial_transport_configured(void) {
  return hUsbDeviceFS.dev_state==USBD_STATE_CONFIGURED;
}
bool serial_transport_read(uint8_t *buf,size_t capacity,size_t *length) {
  *length=0;
  if (overflow) { serial_transport_reset();return false; }
  uint32_t h=head,t=tail;__DMB();
  size_t count=t-h;if (count>capacity) count=capacity;
  for (size_t i=0;i<count;++i) buf[i]=rx[(h+i)%RX_CAPACITY];
  __DMB();head=h+count;*length=count;return true;
}
bool serial_transport_write(const uint8_t *buf,size_t length) {
  if (length>sizeof(tx) || !serial_transport_configured()) return false;
  uint32_t start=HAL_GetTick();
  while (hUsbDeviceFS.pClassData && ((USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData)->TxState) {
    if ((uint32_t)(HAL_GetTick()-start)>=5) return false;
    vTaskDelay(1);
  }
  memcpy(tx,buf,length);
  return CDC_Transmit_FS(tx,(uint16_t)length)==USBD_OK;
}
